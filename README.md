# Soil_moisture_irrigation_controller
Soil moisture based irrigation controller (ESP32/Wokwi) — non-blocking sampling, smoothing, local irrigation decision, sensor fault detection, and store-and-forward for network outages. SIH 2026 practical assessment.
# Soil Moisture Based Irrigation Controller

**SIH 2026 — Internal Practical Assessment**
VIVEKA M · Reg 411723106089 · PSVPEC · ECE · Year IV

## Problem (in two lines)

Farmers irrigate on a fixed schedule because there is no reliable local reading of
soil moisture, so water is wasted when the field is already wet and withheld when
the crop is stressed. This project is a controller that measures soil moisture,
decides locally whether to irrigate, and keeps working when the network or a sensor fails.

---

## 1. Requirement Table (Task 1)

| Design decision | Value | Why |
|---|---|---|
| Sampling interval | **2000 ms** | Fast enough to react, slow enough not to flood the loop; implemented with `millis()`, never `delay()` |
| Plausible range — soil moisture | **0 % – 100 %** | Values outside this are physically impossible → sensor fault |
| Plausible range — temperature | **-10 °C – 60 °C** | Physical bound for a field sensor |
| Safe range — soil moisture | **30 % – 70 %** | Below 30 % the crop is under stress; above 70 % the soil is already saturated |
| Action threshold | **< 30 %** | Point at which irrigation should start |
| Consecutive readings before acting | **3** | A single low reading can be noise; 3 in a row is a real trend |
| Smoothing window | **5 samples**, simple moving average | Stops one spike from being read as a real change |
| Stuck-value window | **5 identical consecutive raw readings** | Signature of a frozen/failed sensor |
| Store-and-forward buffer | **50 records**, oldest dropped on overflow | Bounded memory on a microcontroller |

## 2. The Three Test Signals (+ 2 more, for 5 total test cases)

Generated as fixed arrays inside the sketch (`caseNormal`, `caseExcursion`,
`caseNoisy`, `caseStuck`, `caseNetwork`) and replayed automatically at boot:

1. **Normal** — gentle drift inside the 30–70 % safe band, nothing should trigger.
2. **Genuine excursion** — moisture falls under 30 % and stays there → pump should turn ON after 3 consecutive low readings.
3. **Noisy** — normal values with isolated spikes (e.g. 200, -5) that are outside the plausible range and must be treated as a momentary fault, not smoothed into the average.
4. **Stuck sensor** — the raw value freezes at 47 % for 5+ readings → fault state, pump forced OFF regardless of the frozen value.
5. **Network outage → reconnect** — normal moisture readings while the simulated network is forced OFFLINE for the first half of the case; readings queue in the store-and-forward buffer, then flush oldest-first the moment the network comes back.

## 3. Program Flow

```
                         ┌─────────────────────────┐
                         │        setup()          │
                         │  init pins, Serial, LEDs │
                         └────────────┬─────────────┘
                                      │
                                      ▼
                     ┌────────────────────────────────┐
                     │   BOOT: run 5-case test suite   │
                     │  (Task 4 — evidence for grading)│
                     └────────────────┬─────────────────┘
                                      │  each sample ↓
                                      ▼
   ┌───────────────────────────────────────────────────────────────────┐
   │                        processSample(raw)                          │
   │                                                                     │
   │  1. Stuck check (5 identical raw readings?)                        │
   │  2. Plausibility check (0-100% / -10..60°C)                        │
   │        │ fail → FAULT: fault LED on, pump forced OFF, reading      │
   │        │        logged with alert=false, decision NOT updated      │
   │        ▼ pass                                                      │
   │  3. Moving-average smoothing (5-sample window)                     │
   │  4. Compare smoothed value to 30% threshold                        │
   │  5. Consecutive-low counter (needs 3 in a row)                     │
   │  6. Decide: irrigate? → drive pump relay + alert LED                │
   │  7. Store-and-forward: enqueue {timestamp @ read time, values}     │
   │  8. If network ONLINE → flush queue oldest-first, print [PUBLISH]   │
   └───────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼  after all 5 cases
                     ┌────────────────────────────────┐
                     │   LIVE MODE (loop(), no delay)  │
                     │  • read potentiometers on the    │
                     │    fixed 2000 ms interval        │
                     │  • network button toggles online/│
                     │    offline (edge-detected)       │
                     │  • same processSample() pipeline │
                     └────────────────────────────────┘
```

## 4. What Every Field Means (Serial log)

| Field | Meaning |
|---|---|
| `raw_soil` | Unprocessed reading straight from the sensor/pot, before any filtering |
| `raw_temp` | Unprocessed temperature reading |
| `smoothed_soil` | 5-sample moving average of `raw_soil` — this is what the decision is actually based on |
| `consec_low` | How many consecutive samples have been below the 30 % threshold |
| `pump` | ON / OFF — the actual actuator state after the decision logic |
| `FAULT (...)` | Sensor considered untrustworthy; reason is `STUCK_VALUE` or `OUT_OF_PLAUSIBLE_RANGE`; pump is force-OFF |
| `[PUBLISH] {...}` | A record leaving the store-and-forward buffer once the network is back, in the order it was recorded (oldest-first) |
| `ts_ms` | Timestamp **of the moment the reading was taken**, not of the moment it was sent — this keeps recovered history truthful even after an outage |

## 5. How the One Derived Figure Is Calculated

The only derived figure in the system is **`smoothed_soil`**, a simple moving
average over the last 5 raw readings:

```
smoothed_soil = ( raw[n] + raw[n-1] + raw[n-2] + raw[n-3] + raw[n-4] ) / 5
```

During the warm-up (fewer than 5 samples collected), the average is taken over
however many samples exist so far. This is what the irrigation decision and the
alert are based on — never the raw value — so that a single spike cannot flip
the pump.

**Manual check (Case 1 — Normal, first 5 samples: 55, 54, 56, 55, 53):**
`(55+54+56+55+53) / 5 = 273 / 5 = 54.6 → 54` (integer division), which matches
the `smoothed_soil` printed for sample 5 in `test_logs/case1_normal.txt`.

## 6. How to Run

1. Go to https://wokwi.com/projects/new/esp32 (or open this folder directly if
   using the Wokwi VS Code extension).
2. Copy in `soil_moisture_controller.ino` and `diagram.json` (already wired for
   ESP32 + 2 potentiometers + relay + 3 LEDs + 1 button).
3. Press ▶ Start Simulation. Open the **Serial Monitor** at 115200 baud.
4. The 5-case test suite runs automatically and prints a labelled log per case
   — this is the evidence captured in `test_logs/`.
5. After the suite finishes, turn the **Soil Moisture** potentiometer to change
   the live reading, and click the **network button** to simulate an outage /
   reconnection and watch the buffered records flush.

## 7. Test Evidence (Task 4 & 5)

Captured Serial Monitor logs for all 5 required cases are in `test_logs/`:
- `case1_normal.txt`
- `case2_excursion.txt`
- `case3_noisy.txt`
- `case4_stuck.txt`
- `case5_network_outage.txt`

Each log shows raw values, the smoothed value, the fault/consecutive-count
state, the pump decision, and (for case 5) the buffered records draining
oldest-first on reconnection.

## 8. What Is Not Finished

- Real WiFi/MQTT publishing is **simulated** (Serial `[PUBLISH]` lines) rather
  than an actual broker connection, since the assessment is simulation-only.
- The temperature reading is captured and logged but does not yet feed into
  the irrigation decision (only soil moisture does) — noted as a possible
  future extension.
- Store-and-forward buffer is capped at 50 records; on overflow the oldest
  unsent record is dropped rather than persisted to flash.

## 9. Repository Contents

```
soil_moisture_controller/
├── soil_moisture_controller.ino   # complete controller firmware
├── diagram.json                    # Wokwi circuit (ESP32 + sensors + actuators)
├── wokwi.toml                      # Wokwi project config
├── README.md                       # this file
└── test_logs/                      # captured evidence for all 5 test cases
```
