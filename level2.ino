/* ============================================================================
   SOIL MOISTURE BASED IRRIGATION CONTROLLER
   SIH 2026 - Internal Practical Assessment
   VIVEKA M | Reg 411723106089 | PSVPEC | ECE | Year IV

   Platform : ESP32 (Wokwi Simulation)
   Purpose  : Reads soil moisture + temperature, applies plausibility &
              smoothing, makes a local irrigation decision, drives a pump
              relay, detects sensor faults, and stores-and-forwards
              readings when the "network" is unavailable.

   HOW TO RUN (Wokwi):
     1. Open https://wokwi.com/projects/new/esp32
     2. Replace the default sketch with this file (or upload the project
        folder which already contains diagram.json + wokwi.toml).
     3. Click the green Play button. Open the Serial Monitor (115200 baud).
     4. On boot the controller automatically runs the 5-case TEST SUITE
        (Task 4) and prints a labelled log for each case. After the suite
        finishes it switches to LIVE mode, reading the potentiometers.
     5. Turn "Soil Moisture" potentiometer to change the simulated
        moisture level. Press the NETWORK button to toggle the simulated
        network between ONLINE and OFFLINE (watch the store-and-forward
        buffer drain when it reconnects).
   ============================================================================ */

#include <Arduino.h>

/* ---------------------------------------------------------------------------
   TASK 1 : REQUIREMENT TABLE  (design decisions, fixed once, used everywhere)
   ---------------------------------------------------------------------------
   Quantity            | Value
   ---------------------|---------------------------------------------------
   Sampling interval    | 2000 ms  (non-blocking, millis()-based)
   Plausible range       | 0 - 100 %  (soil), -10 - 60 C (temperature)
   Safe range (soil)     | 30 % - 70 %
   Action threshold      | < 30 %  -> irrigation required
                          | > 70 %  -> irrigation must stay OFF (over-wet)
   Consecutive readings  | 3   (must persist for 3 readings before acting)
   Smoothing window      | 5 samples, simple moving average
   Stuck-value window     | 5 consecutive identical raw readings = fault
   Store-and-forward cap | 50 records (oldest dropped if buffer overflows)
   --------------------------------------------------------------------------- */

// ------------------------------ PIN MAP -------------------------------------
#define SOIL_POT_PIN     34   // Potentiometer simulating the soil moisture sensor
#define TEMP_POT_PIN     35   // Potentiometer simulating the temperature sensor
#define PUMP_RELAY_PIN   25   // Relay module -> water pump
#define ALERT_LED_PIN    26   // Red LED   -> "irrigation alert / pump ON" indicator
#define FAULT_LED_PIN    27   // Yellow LED-> sensor fault indicator
#define NETWORK_BTN_PIN  32   // Push button -> toggles simulated network link
#define NETWORK_LED_PIN  33   // Green LED -> network ONLINE indicator

// ------------------------------ CONSTANTS (Task 1 table, in code) -----------
const unsigned long SAMPLE_INTERVAL_MS  = 2000;
const int SOIL_SAFE_MIN                 = 30;     // %
const int SOIL_SAFE_MAX                 = 70;     // %
const int SOIL_ACTION_THRESHOLD         = 45;     // %
const int CONSECUTIVE_READINGS_REQUIRED = 4;
const int PLAUSIBLE_SOIL_MIN            = 0;
const int PLAUSIBLE_SOIL_MAX            = 100;
const int PLAUSIBLE_TEMP_MIN            = -10;
const int PLAUSIBLE_TEMP_MAX            = 60;
const int SMOOTHING_WINDOW              = 5;
const int STUCK_WINDOW                  = 5;
const int SF_BUFFER_CAPACITY            = 50;     // store-and-forward capacity

/* =============================================================================
   TASK 1 (cont.) : THE THREE TEST SIGNALS + 2 EXTRA CASES = 5 TEST CASES
   Used by the automatic test suite that runs at boot (Task 4 requirement).
   Each array represents raw, un-smoothed soil-moisture readings (%).
   ============================================================================= */
const int SIG_LEN = 12;

// Case 1: NORMAL - gentle drift inside the safe band, nothing should trigger
const int caseNormal[SIG_LEN]      = {55,54,56,55,53,54,56,57,55,54,53,55};

// Case 2: GENUINE EXCURSION - moisture drops below threshold and stays there
const int caseExcursion[SIG_LEN]   = {50,45,38,25,22,20,19,18,17,16,15,14};

// Case 3: NOISY - normal values with isolated spikes (must be rejected/smoothed)
const int caseNoisy[SIG_LEN]       = {55,54,200,56,55,-5,54,53,150,55,54,56};

// Case 4: STUCK SENSOR - value freezes (classic sensor failure)
const int caseStuck[SIG_LEN]       = {50,48,47,47,47,47,47,47,47,47,47,47};

// Case 5: NETWORK OUTAGE THEN RECONNECT - moisture normal, network flaps
//         (values below are just normal readings; the "outage" is simulated
//          in the runTestSuite() logic by forcing g_networkOnline = false
//          for the first half of the case, then true for the second half)
const int caseNetwork[SIG_LEN]     = {60,59,58,58,57,58,59,60,59,58,57,58};

/* =============================================================================
   STATE
   ============================================================================= */

// -- smoothing (moving average) --
int soilHistory[SMOOTHING_WINDOW];
int soilHistoryIdx = 0;
bool soilHistoryFull = false;

// -- stuck-sensor detection --
int rawHistory[STUCK_WINDOW];
int rawHistoryIdx = 0;
bool rawHistoryFull = false;

// -- consecutive-reading counter for the local decision --
int consecutiveLowCount = 0;

// -- fault state --
bool sensorFaulted = false;
String faultReason = "";

// -- network simulation --
bool g_networkOnline = true;
bool lastBtnState = HIGH;

// -- pump / alert state --
bool pumpOn = false;
bool alertActive = false;

// -- non-blocking timing --
unsigned long lastSampleTime = 0;

// -- store-and-forward record --
struct Record {
  unsigned long timestampMs;   // time the reading was TAKEN (not sent)
  int soilSmoothed;
  int tempValue;
  bool alert;
};
Record sfBuffer[SF_BUFFER_CAPACITY];
int sfHead = 0;      // index of oldest record
int sfCount = 0;      // number of records currently buffered

// -- test-suite control --
bool testSuiteRunning = true;
int testCaseIndex = 0;   // 0..4
int testSampleIndex = 0; // 0..SIG_LEN-1

/* =============================================================================
   HELPERS
   ============================================================================= */

// ---- plausibility check (Task 2) --------------------------------------------
bool isPlausibleSoil(int v) {
  return (v >= PLAUSIBLE_SOIL_MIN && v <= PLAUSIBLE_SOIL_MAX);
}
bool isPlausibleTemp(int v) {
  return (v >= PLAUSIBLE_TEMP_MIN && v <= PLAUSIBLE_TEMP_MAX);
}

// ---- smoothing: simple moving average (Task 2) -------------------------------
int pushAndSmooth(int newVal) {
  soilHistory[soilHistoryIdx] = newVal;
  soilHistoryIdx = (soilHistoryIdx + 1) % SMOOTHING_WINDOW;
  if (soilHistoryIdx == 0) soilHistoryFull = true;

  int count = soilHistoryFull ? SMOOTHING_WINDOW : soilHistoryIdx;
  if (count == 0) return newVal;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += soilHistory[i];
  return (int)(sum / count);
}

// ---- stuck-value detection (Task 4) ------------------------------------------
bool pushAndCheckStuck(int rawVal) {
  rawHistory[rawHistoryIdx] = rawVal;
  rawHistoryIdx = (rawHistoryIdx + 1) % STUCK_WINDOW;
  if (rawHistoryIdx == 0) rawHistoryFull = true;

  if (!rawHistoryFull) return false; // not enough history yet
  for (int i = 1; i < STUCK_WINDOW; i++) {
    if (rawHistory[i] != rawHistory[0]) return false; // not all identical
  }
  return true; // every recent raw reading identical -> stuck
}

// ---- store-and-forward: enqueue a record (Task 3) -----------------------------
void sfEnqueue(unsigned long ts, int soil, int temp, bool alert) {
  if (sfCount >= SF_BUFFER_CAPACITY) {
    // buffer full: drop the oldest to make room (documented limitation)
    sfHead = (sfHead + 1) % SF_BUFFER_CAPACITY;
    sfCount--;
  }
  int writeIdx = (sfHead + sfCount) % SF_BUFFER_CAPACITY;
  sfBuffer[writeIdx].timestampMs = ts;
  sfBuffer[writeIdx].soilSmoothed = soil;
  sfBuffer[writeIdx].tempValue = temp;
  sfBuffer[writeIdx].alert = alert;
  sfCount++;
}

// ---- store-and-forward: publish oldest-first when network is back (Task 3) ----
void sfFlushIfOnline() {
  if (!g_networkOnline) return;
  while (sfCount > 0) {
    Record r = sfBuffer[sfHead];
    Serial.print(F("[PUBLISH] {\"ts_ms\":"));
    Serial.print(r.timestampMs);
    Serial.print(F(",\"soil\":"));
    Serial.print(r.soilSmoothed);
    Serial.print(F(",\"temp\":"));
    Serial.print(r.tempValue);
    Serial.print(F(",\"alert\":"));
    Serial.print(r.alert ? "true" : "false");
    Serial.println(F("}"));
    sfHead = (sfHead + 1) % SF_BUFFER_CAPACITY;
    sfCount--;
  }
}

// ---- actuator control ---------------------------------------------------------
void setPump(bool on) {
  pumpOn = on;
  digitalWrite(PUMP_RELAY_PIN, on ? HIGH : LOW);
}
void setAlertLed(bool on) {
  alertActive = on;
  digitalWrite(ALERT_LED_PIN, on ? HIGH : LOW);
}
void setFaultLed(bool on) {
  digitalWrite(FAULT_LED_PIN, on ? HIGH : LOW);
}
void setNetworkLed(bool on) {
  digitalWrite(NETWORK_LED_PIN, on ? HIGH : LOW);
}

/* =============================================================================
   CORE PIPELINE : one call = one "sample cycle"
   raw soil / raw temp come either from the test suite or from live sensors.
   This is the single function both TEST mode and LIVE mode funnel through, so
   the decision logic is only ever written once and tested by the suite before
   it ever touches a real (simulated) sensor.
   ============================================================================= */
void processSample(int rawSoil, int rawTemp, const char* label) {
  unsigned long ts = millis();

  Serial.print(F("[SAMPLE] "));
  Serial.print(label);
  Serial.print(F(" | raw_soil=")); Serial.print(rawSoil);
  Serial.print(F(" raw_temp=")); Serial.print(rawTemp);

  // ---- Task 4: fault detection happens BEFORE anything acts on the value ----
  bool stuck = pushAndCheckStuck(rawSoil);
  bool implausible = !isPlausibleSoil(rawSoil) || !isPlausibleTemp(rawTemp);

  if (stuck || implausible) {
    sensorFaulted = true;
    faultReason = stuck ? "STUCK_VALUE" : "OUT_OF_PLAUSIBLE_RANGE";
    setFaultLed(true);
    setPump(false);          // never act on a reading we don't trust
    setAlertLed(false);
    consecutiveLowCount = 0; // fault breaks the consecutive-good-reading streak
    Serial.print(F(" -> FAULT ("));
    Serial.print(faultReason);
    Serial.println(F(") - reading discarded, pump forced OFF"));
    // A fault is still logged (with a fault flag) so the outage is visible in
    // the recovered history, but it is not used for the irrigation decision.
    sfEnqueue(ts, rawSoil, rawTemp, false);
    sfFlushIfOnline();
    return;
  }

  // reading is plausible and not stuck -> sensor considered healthy again
  if (sensorFaulted) {
    Serial.println(F(" -> sensor recovered, resuming normal operation"));
  }
  sensorFaulted = false;
  faultReason = "";
  setFaultLed(false);

  // ---- Task 2: smoothing (isolated spikes cannot flip the decision) ----------
  int smoothed = pushAndSmooth(rawSoil);
  Serial.print(F(" smoothed_soil=")); Serial.print(smoothed);

  // ---- Task 3: local decision requiring N consecutive bad readings ----------
  bool belowThreshold = (smoothed < SOIL_ACTION_THRESHOLD);
  if (belowThreshold) {
    consecutiveLowCount++;
  } else {
    consecutiveLowCount = 0;
  }

  bool shouldIrrigate = (consecutiveLowCount >= CONSECUTIVE_READINGS_REQUIRED);

  // Safety: never irrigate if smoothed value is already above the wet ceiling
  if (smoothed > SOIL_SAFE_MAX) {
    shouldIrrigate = false;
  }

  setPump(shouldIrrigate);
  setAlertLed(shouldIrrigate);

  Serial.print(F(" consec_low=")); Serial.print(consecutiveLowCount);
  Serial.print(F(" pump=")); Serial.print(shouldIrrigate ? "ON" : "OFF");
  Serial.println();

  // ---- Task 3: store-and-forward, timestamped at time of READING -------------
  sfEnqueue(ts, smoothed, rawTemp, shouldIrrigate);
  sfFlushIfOnline();
}

/* =============================================================================
   TASK 4 : AUTOMATIC 5-CASE TEST SUITE (runs once at boot, prints a labelled
   log for each case so the log can be captured from the Serial Monitor and
   pasted into test_logs/ as evidence).
   ============================================================================= */
const char* caseNames[5] = {
  "CASE 1 - NORMAL",
  "CASE 2 - GENUINE EXCURSION",
  "CASE 3 - NOISY (spikes + implausible)",
  "CASE 4 - STUCK SENSOR",
  "CASE 5 - NETWORK OUTAGE THEN RECONNECT"
};

bool runTestSuiteStep() {
  if (testCaseIndex >= 5) return false; // suite finished

  if (testSampleIndex == 0) {
    Serial.println();
    Serial.print(F("===== "));
    Serial.print(caseNames[testCaseIndex]);
    Serial.println(F(" ====="));
    // reset per-case state so cases don't bleed into each other
    consecutiveLowCount = 0;
    soilHistoryIdx = 0; soilHistoryFull = false;
    rawHistoryIdx = 0; rawHistoryFull = false;
  }

  // Case 5 forces the network offline for the first half, online for the 2nd
  if (testCaseIndex == 4) {
    g_networkOnline = (testSampleIndex >= SIG_LEN / 2);
    setNetworkLed(g_networkOnline);
    if (testSampleIndex == 0) Serial.println(F("[NETWORK] forced OFFLINE for this case"));
    if (testSampleIndex == SIG_LEN / 2) Serial.println(F("[NETWORK] reconnected -> flushing buffer oldest-first"));
  }

  int rawSoil, rawTemp = 27; // fixed plausible temperature for test cases
  switch (testCaseIndex) {
    case 0: rawSoil = caseNormal[testSampleIndex]; break;
    case 1: rawSoil = caseExcursion[testSampleIndex]; break;
    case 2: rawSoil = caseNoisy[testSampleIndex]; break;
    case 3: rawSoil = caseStuck[testSampleIndex]; break;
    case 4: rawSoil = caseNetwork[testSampleIndex]; break;
    default: rawSoil = 50;
  }

  processSample(rawSoil, rawTemp, caseNames[testCaseIndex]);

  testSampleIndex++;
  if (testSampleIndex >= SIG_LEN) {
    testSampleIndex = 0;
    testCaseIndex++;
  }
  return true;
}

/* =============================================================================
   ARDUINO ENTRY POINTS
   ============================================================================= */
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(NETWORK_LED_PIN, OUTPUT);
  pinMode(NETWORK_BTN_PIN, INPUT_PULLUP);

  setPump(false);
  setAlertLed(false);
  setFaultLed(false);
  setNetworkLed(true);

  Serial.println(F("============================================================"));
  Serial.println(F(" SOIL MOISTURE IRRIGATION CONTROLLER - SIH 2026"));
  Serial.println(F(" Running automatic 5-case test suite (Task 4)..."));
  Serial.println(F("============================================================"));
}

void loop() {
  unsigned long now = millis();

  // ---- run the boot-time test suite first, one sample per interval ----------
  if (testSuiteRunning) {
    if (now - lastSampleTime >= 150) { // fast pace for the automated suite
      lastSampleTime = now;
      bool more = runTestSuiteStep();
      if (!more) {
        testSuiteRunning = false;
        g_networkOnline = true;
        setNetworkLed(true);
        Serial.println();
        Serial.println(F("===== TEST SUITE COMPLETE - switching to LIVE mode ====="));
        Serial.println(F("Turn the Soil Moisture potentiometer and press the"));
        Serial.println(F("NETWORK button to explore live behaviour."));
        Serial.println();
        // reset state cleanly before live operation
        consecutiveLowCount = 0;
        soilHistoryIdx = 0; soilHistoryFull = false;
        rawHistoryIdx = 0; rawHistoryFull = false;
      }
    }
    return;
  }

  // ---- LIVE MODE ---------------------------------------------------------------

  // network toggle button (active LOW, debounced by simple edge check)
  bool btnState = digitalRead(NETWORK_BTN_PIN);
  if (btnState == LOW && lastBtnState == HIGH) {
    g_networkOnline = !g_networkOnline;
    setNetworkLed(g_networkOnline);
    Serial.print(F("[NETWORK] toggled -> "));
    Serial.println(g_networkOnline ? "ONLINE" : "OFFLINE");
    if (g_networkOnline) sfFlushIfOnline();
  }
  lastBtnState = btnState;

  // non-blocking sampling on the fixed interval from Task 1
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    int rawSoilAdc = analogRead(SOIL_POT_PIN);           // 0-4095
    int rawTempAdc = analogRead(TEMP_POT_PIN);            // 0-4095
    int rawSoil = map(rawSoilAdc, 0, 4095, 0, 100);        // -> %
    int rawTemp = map(rawTempAdc, 0, 4095, -10, 60);       // -> deg C

    processSample(rawSoil, rawTemp, "LIVE");
  }

  // any other non-blocking work (e.g. a display refresh) would go here,
  // never inside a delay() - this is what keeps the loop responsive.
}
