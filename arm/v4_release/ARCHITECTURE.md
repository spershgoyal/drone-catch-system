# ARM v4 — ARCHITECTURE PLAN

Author: Claude (lead dev mode)  
For: Ethan Shapero  
Date: 2026-04-30  
Status: DRAFT — review before code

---

## 0. GOAL

Take the working v3p_4 firmware + arm_ctrlv4_22 webpage and ship v4 = a system that can autonomously catch a hovering DJI Mini 4 Pro from a moving boat, using 4 RYUW122 UWB modules (3 base + 1 tip), with a clean UI for ops + tuning + diagnostics.

Two non-negotiables:
- everything that worked in v3p_4 still works in v4 (modes, dances, poses, calibration, EEPROM, telemetry, manual control)
- the new stuff (UWB, catch FSM, tilt v2, sim drone, system test) integrates without breaking any of that

---

## 1. FIRMWARE FILE LAYOUT

Currently 9 files, 1857 lines. v4 grows to 13 files. Each file owns one concept.

```
arm_main_v4.ino       main loop, mode dispatch, command parser
config.h              all pins, constants, joint limits, anchor positions
sensors.h             pot reads, FK, sensor filtering
motors.h              BTS7960 abstraction, stall detect, runMotor + stop
safety.h              floor guard, hard limits, stall cutout, mag lock
kinematics.h          IK solvers (XYZ, height, with cost function)
trajectory.h          s-curve PWM profiles, synced moves, position update
poses.h               POSES[], runPose(), tryPoseKey, validatePoses
dance.h               dance steps, dance FSM
tilt.h                tilt mode v2 (rebuilt for moving targets + boats)
uwb.h                 RYUW122 driver, AT command framework, range parsing
trilat.h              trilateration math, drone position fusion
catch_fsm.h           catch state machine (NEW)
serial_cmd.h          all command handling: cal, mag, dn, tflip, etc.
telemetry.h           $T format, command emission, packed for new UI
```

13 files. all `#include`d from `arm_main_v4.ino`. matches Arduino IDE single-sketch model.

---

## 2. NEW HARDWARE / PIN CHANGES (Teensy 4.1)

### Current pins (kept identical):
- BTS7960s: base 2/29, shoulder 3/5, elbow 14/35
- Pots: base 38, shoulder 21, elbow 20
- Servos: roll 22, pitch 23
- IMU (BNO08x): SDA 18, SCL 19

### Changed:
- **MAG_PIN: 33** (was 16, freed pin 16 for Serial4)

### New pins for UWB (4 modules):
- Serial1: anchor 1 — pins 0 (RX), 1 (TX)
- Serial2: anchor 2 — pins 7 (RX), 8 (TX)
- Serial4: tip anchor — pins 16 (RX), 17 (TX)
- Serial6: anchor 3 — pins 25 (RX), 24 (TX)

### Power:
- new dedicated 3.3V buck (1A+) for all 4 UWB modules
- 470µF bulk + 100nF ceramic at each module
- common ground rail

---

## 3. ANCHOR + TIP POSITIONS (placeholders, from your spec)

Coordinate frame: arm base (rotation point) at origin. +X right (when looking from back of arm in stow). +Y forward. +Z up. Inches.

```c
const float ANCHOR_POS[3][3] = {
  // Anchor A1: on Serial1 (anchors 0/1)
  { -5.0f, -5.0f, -3.0f },
  // Anchor A2: on Serial2 (pins 7/8)
  { -5.0f, 10.0f, -3.0f },
  // Anchor A3: third base anchor
  { 16.0f,  6.0f,  8.0f },
};

// Tip anchor: 4" right + 14" up forearm, in arm-local frame relative to wrist
// Will be transformed via FK during runtime.
const float TIP_ANCHOR_FOREARM_OFFSET[3] = {
  4.0f,    // perpendicular to forearm (out the side)
  0.0f,    // along forearm direction (parallel)
  -2.0f,   // 14" up forearm = 2" before wrist (forearm is 16")
};
```

These get bumped to "calibrated" after physical install. Cal commands `cal anchor 1 x y z` will update at runtime + persist to EEPROM.

---

## 4. CATCH FSM

### States

```
IDLE         - arm at home, no catch in progress
SEARCHING    - catch armed but no drone detected by UWB
TRACKING     - drone detected, base yaw locked to drone bearing,
               arm in "ready" pose 8" below drone
STAGING      - arm moving to "stage" position: 4" below drone,
               magnet face up, wrist pre-leveled, IMU comp on
COMMITTING   - final 4" upward sprint to drone tag,
               commit threshold reached, magnet ARMED but not on
CATCHING     - magnet face within 1" of drone tag, magnet FIRES,
               hold position 0.8s
SECURED      - magnet on, drone caught, descending to safe pose,
               magnet stays locked
STOWING      - returning to home pose, magnet still locked
ABORT        - error or user disarm; safe retract, drop magnet
ERROR        - hardware issue (UWB lost, IK fail, stall);
               full stop + report, magnet KEEPS state from before
```

### Transitions

```
IDLE
  ├─ user "arm" command            → SEARCHING
SEARCHING
  ├─ UWB returns valid drone pos   → TRACKING
  ├─ user "disarm"                 → IDLE
  ├─ 30s timeout                   → IDLE (auto-disarm)
TRACKING
  ├─ stable lock (variance < 1.5") → STAGING
  ├─ drone lost > 2s               → SEARCHING
  ├─ user "disarm"                 → ABORT
STAGING
  ├─ tip 4" below drone, IK ok     → COMMITTING
  ├─ drone moves > 6"              → TRACKING (re-acquire)
  ├─ stage timeout > 5s            → ABORT
COMMITTING
  ├─ tip < 1" from drone tag       → CATCHING
  ├─ tip range diverges from FK by > 4" → ABORT (integrity fail)
  ├─ commit timeout > 1.5s         → ABORT
CATCHING
  ├─ 0.8s elapsed                  → SECURED
SECURED
  ├─ 0.5s elapsed (settle)         → STOWING
STOWING
  ├─ home pose reached             → IDLE (magnet released)
ABORT
  ├─ retract complete              → IDLE
ERROR
  ├─ user "reset"                  → IDLE
```

### Speed / commit gating

- TRACKING uses normal joint speed (~speed=40 PWM)
- STAGING uses slow careful moves (PWM=30, tighter margin 0.4°)
- COMMITTING uses MAX speed (PWM=60+) — fast vertical sprint, only 4"
- SECURED + STOWING use careful (PWM=30)

### Safety tied to FSM

- `magLocked` flag set on entry to CATCHING, cleared on entry to STOWING-done or ABORT
- when `magLocked`, `stopAll()` does NOT drop magnet
- floor guard suspended during COMMITTING (tight clearance)
- IMU tilt comp ON during STAGING / COMMITTING / CATCHING / SECURED if `tiltCompEnabled` flag set

### Tip anchor integrity check

Run continuously during STAGING + COMMITTING:
- predicted tip-to-drone range = |FK(tip_anchor_world) - solved_drone_pos|
- measured tip-to-drone range = latest UART reading from tip module
- if |predicted - measured| > 4" for 200ms continuous → ABORT (integrity_fail)

This catches: pot drift, mechanical play, joint slip, UWB anchor knocked out of position.

---

## 5. UWB DRIVER DESIGN

### File: uwb.h

Each anchor is one instance. Driver runs **non-blocking, async**.

```c
struct UwbAnchor {
  HardwareSerial& port;         // Serial1, Serial2, Serial4, Serial6
  uint16_t addr;                // 0x0001..0x0004
  bool isTip;                   // tip vs base
  float lastRange;              // inches
  uint32_t lastRangeMs;         // when we got it
  float rangeQuality;           // 0..1, EMA of valid responses
  bool ready;                   // got an OK back during init
  
  // internal parsing state
  char rxBuf[64];
  uint8_t rxLen;
  uint32_t lastQueryMs;
};
```

### Init sequence (Option A, every boot):
```
For each anchor:
  Send "AT+RESET" → wait OK
  Send "AT+ADDRESS=000X" → wait OK
  Send "AT+MODE=ANCHOR" → wait OK
  Send "AT+NETWORKID=ARM01" → wait OK
  Mark ready=true if all OK, else ready=false
```

Init is non-blocking: ~3sec total but doesn't block loop after first iteration.

### Ranging:
At 10Hz, kick a query on EVERY anchor in the same loop tick:
```
anchor[i].port.print("AT+ANCHOR_SEND=99,...");
```

Then poll responses async — `update()` runs every loop, reads bytes from each port, parses lines. When it sees `+ANCHOR_RCV=...,distance=XX.XX,...`, it:
- updates `lastRange`
- updates `rangeQuality` (EMA)
- timestamps `lastRangeMs`

Modules can respond out of order — that's fine, addresses are in the response.

### Anchor health:
```c
bool anchorAlive(int i) {
  return anchors[i].ready && (millis() - anchors[i].lastRangeMs < 500);
}
float anchorQuality(int i) {
  return anchors[i].rangeQuality;
}
```

UI uses these for the status panel.

### Timeout / retry:
If an anchor hasn't responded in 500ms, mark stale. Try re-init after 5sec stale.

---

## 6. TRILATERATION + FUSION

### File: trilat.h

3-base trilat (math-textbook formulation):
- Inputs: A1, A2, A3 positions (known), 3 ranges (measured)
- Solve: 2 candidate points (mirror solutions across baseline plane)
- Disambiguate: pick the one with z > 0 (drone is above baseplate)
- Output: x, y, z, residual (least-squares residual = quality metric)

### Tip anchor — light fusion + integrity
After base-anchors give us drone position:
- Compute tip anchor world position via FK
- Predicted tip→drone range = ||tip_world - drone_pos||
- Measured tip→drone range from UWB
- Discrepancy = |predicted - measured|

Use the tip range:
1. **Light fusion**: compute weighted-average drone position, where:
   - 3-base solution weight = 1.0
   - Implied position from tip range (along the line from tip to base solution) weight = 0.2
2. **Integrity**: if discrepancy > 4" sustained, raise alarm
3. **Proximity**: in COMMITTING phase, fire magnet when measured tip→drone range < 1"

The light fusion smooths jitter when the tip is close (tip range is more accurate at close range); doesn't dominate when tip is far.

### EMA smoothing
Final output position runs through `α=0.4` EMA filter. Fast enough to follow drone movement, smooths out 1-2" UWB noise.

---

## 7. TILT MODE v2

The current tilt mode has these problems:
- captures a static target → can't track moving drone
- IMU comp uses Euler reconstruction which is jittery near singularities
- PD has TILT_NEAR_DAMP that interacts weirdly with the deadband
- doesn't handle the fact that the boat is a low-frequency oscillator (~0.5-2Hz wave period) — tries to chase fast tilt changes when it should anticipate

### v2 design

**Concept:** the boat's tilt is a slow sine wave. Don't try to react instantly to roll/pitch readings. Instead:
1. Track an *intended world target* (could be static OR dynamically updated by catch FSM)
2. Read IMU quaternion at 100Hz
3. Transform the world target into ARM-LOCAL frame using the IMU's rotation
4. Run IK to find joint angles that put magnet there in the *current* arm-local frame
5. Smooth the IK results (low-pass on joint targets) to avoid stuttering
6. Use velocity-feedforward: if boat is rolling at +5°/sec right now, joint targets should already account for where it'll be in 50ms

**Code structure:**
```c
struct TiltV2 {
  // World target (set by user or catch FSM)
  float wx, wy, wz;
  bool hasTarget;
  
  // IMU state
  float qw, qx, qy, qz;     // current quat
  float qRefW, qRefX, qRefY, qRefZ;  // reference quat (set by 'h' or auto)
  float omegaX, omegaY, omegaZ;  // angular vel (deg/s) for feedforward
  uint32_t lastImuMs;
  
  // Joint target smoothing
  float jSmooth[3];         // smoothed joint targets
  static const float SMOOTH_ALPHA = 0.6;  // weight on new target
  
  // Update at 100Hz
  void update();
};
```

Update loop:
1. Read IMU quat. Compute angular velocity (numerical diff, smoothed).
2. Predict quat 50ms in the future: `q_pred = q * exp(omega * 0.05 * 0.5)` (small-angle quat integration)
3. Compute relative rotation: `q_rel = q_pred * inv(q_ref)`
4. Rotate world target by `inv(q_rel)`: this is where the target IS in arm-local frame, accounting for predicted boat motion
5. IK to that point. Get joint targets.
6. Apply EMA: `jSmooth = α*jNew + (1-α)*jSmooth`
7. Drive joints with PD on `jSmooth - curAngle`

This gives:
- target that smoothly follows boat motion
- no jitter at IMU singularities (using quaternions throughout)
- looks ahead 50ms, compensating for actuator lag
- smooth joint targets, no chatter

**catch FSM integration:** catch FSM writes `wx, wy, wz` directly. Setting `hasTarget = true` activates tilt v2. catch state determines speed/PWM limits.

---

## 8. SAFETY (software-only stall detect, no IS)

Without IS pins wired, stall detection comes from "motor commanded but pot not moving":

```c
struct StallDetect {
  uint32_t lastMoveMs[3];   // last time pot changed > 0.5°
  bool stalled[3];
};

void updateStallDetect() {
  static int lastRaw[3] = {0,0,0};
  for (int j = 0; j < 3; j++) {
    if (abs(curRaw[j] - lastRaw[j]) > 4) {  // ~0.5° at typical scaling
      lastMoveMs[j] = millis();
      lastRaw[j] = curRaw[j];
      stalled[j] = false;
    } else {
      // motor commanded but not moving?
      bool motorActive = (jMoving[j] || pulseActive[j]);
      if (motorActive && (millis() - lastMoveMs[j]) > 600) {
        stalled[j] = true;
        Serial.print("[STALL] j"); Serial.println(j);
        stopMotor(j);
      }
    }
  }
}
```

600ms threshold = generous enough that slow moves don't false-trigger, fast enough to catch real stalls before damage. Stall = motor stops, not full e-stop. User can recover by clearing the obstruction and re-commanding.

When IS pins get wired later, stall detect becomes redundant (IS is faster), but harmless to leave as a backup.

---

## 9. SIM DRONE MODE

### Concept:
Same code path as real catch, but instead of UWB modules feeding drone position, the UI drags a fake drone and sends `$D X Y Z` packets over serial. Firmware accepts these as if they came from trilat.

### Implementation:
```c
bool simDroneMode = false;
float simDroneX, simDroneY, simDroneZ;

// In main loop:
if (simDroneMode) {
  // skip UWB query, use simDrone* as drone position
  drone_position = (simDroneX, simDroneY, simDroneZ);
} else {
  drone_position = trilateration_result;
}
```

UI sends `$D X Y Z\n` whenever the user moves the fake drone. Catch FSM works identically.

Useful for: tuning catch FSM on the bench, demos without flying a drone, debugging the IK + collision avoidance, training operators.

### Toggle:
Serial command: `simdrone on` / `simdrone off`. Also persists in EEPROM.

---

## 10. NEW SERIAL COMMANDS

```
arm                         → start catch (IDLE → SEARCHING)
disarm                      → cancel catch (anything → ABORT or IDLE)
catch reset                 → clear ERROR state
mag lock on                 → manually lock magnet (won't drop on stopAll)
mag lock off                → unlock
simdrone on/off             → enable/disable sim drone mode
simdrone X Y Z              → set sim drone position (UI does this auto)
$D X Y Z                    → packet form (parsed without log spam)
uwb status                  → print all 4 anchors' health
uwb reinit                  → re-run AT command init
uwb cal a1 x y z            → set anchor 1 position (saves to EEPROM)
uwb cal tip f l p           → set tip offsets (forearm-along, perp, perpZ)
test motors                 → run motor pulse test (each joint)
test pots                   → wait for user-confirmed end-stop reads
test uwb                    → 5-sec range stability test per anchor
test imu                    → wait for user tilt confirmation
test mag                    → 1-sec magnet pulse
test all                    → run all of the above sequentially
tcompcatch on/off           → enable IMU compensation during catch
```

---

## 11. NEW $T TELEMETRY FORMAT

v3p_4 had: `$T,bA,sA,eA,tipX,tipY,tipZ,roll,pitch,mode,qw,qx,qy,qz,bOff,sOff,eOff,magOn,magAttached`

v4 extends with catch + UWB fields. Backward-compatible: UI can ignore fields it doesn't know.

```
$T,bA,sA,eA, tipX,tipY,tipZ, roll,pitch, mode, qw,qx,qy,qz,
   bOff,sOff,eOff, magOn,magAttached,
   catchState, droneX,droneY,droneZ, droneValid,
   r1,r2,r3,rTip, q1,q2,q3,qTip,                  ← UWB ranges + qualities
   tipPredicted,tipMeasured,tipDiscrepancy,       ← integrity check
   stallFlags, magLocked, simDroneOn
```

Frequency: 50ms during normal ops (20Hz), 20ms (50Hz) during STAGING+COMMITTING+CATCHING.

---

## 12. UI REWORK

### Layout

```
┌─────────────────────────────────────────────────────────────┐
│ HEADER: logo, mode pills, connect btn, catch banner       │
├──────────────────────────┬──────────────────────────────────┤
│                          │                                  │
│   3D VIEWPORT            │   SIDE PANEL                     │
│   (always visible)       │   (tabbed)                       │
│                          │                                  │
│                          │   tabs: CTRL | UWB | TUNE        │
│                          │                                  │
├──────────────────────────┴──────────────────────────────────┤
│ TELEMETRY STRIP (live values)                               │
├─────────────────────────────────────────────────────────────┤
│ TERMINAL (collapsible, default closed)                      │
└─────────────────────────────────────────────────────────────┘
```

Tabs ELIMINATED:
- CATCH SIM (replaced by sim drone mode in main viewport)

Tabs KEPT but moved:
- VIEWPORT → main always-visible
- GRAPHS → modal overlay or dedicated route, on demand
- CONTROLLER → its own pill in mode bar (gamepad config)

Tabs ADDED:
- UWB tab in side panel: anchor health bars, range readouts, calibration
- TUNE tab in side panel: noise σ, ranging Hz, tilt v2 params, catch FSM thresholds
- DIAG button → opens DIAGNOSTICS modal (system test screen)

### Visual identity

Keep ur existing aesthetic:
- bone-white bg (`#f5f1e8`)
- IBM Plex Mono for monospace
- Space Grotesk for sans
- hazard orange (`#e85d24`) for active/danger
- Swiss grid

Add:
- subtle grid background lines (1px every 32px) on viewport
- mode pills with hover transitions
- catch banner that slides down from header when armed (hazard orange)
- anchor health bars w/ live RSSI-style indicators
- color-coded FSM state (gray IDLE, blue SEARCHING, yellow STAGING, orange COMMITTING, green SECURED, red ERROR)
- smooth state transitions (300ms ease)

### Connect splash

On load:
- centered card: "ARM CONTROL v4" + version
- "CONNECT" button → web serial port picker
- progress: "connecting..." → "handshake..." → "ready"
- on ready: card fades out, main UI fades in
- if no compatible browser: banner saying "Web Serial requires Chrome/Edge"

### Sim drone

- toggle in TUNE tab: "SIM DRONE MODE"
- when on: a draggable purple drone appears in the viewport
- drag it around in 3D (3-axis gizmo or click-to-place plane)
- firmware gets `$D X Y Z` packets at 30Hz while dragging
- arm tracks the sim drone using the same code path as real catch
- "ARM CATCH" button in this mode catches the sim drone

### Catch banner (top of screen)

When catch state != IDLE, banner slides down with:
- big state name (TRACKING / STAGING / etc) color-coded
- drone XYZ
- distance from magnet to drone
- magnet state
- ARMED indicator + DISARM button
- elapsed time in state

### Diagnostics modal

System test page styled like an aircraft startup checklist (inspo: pre-flight checklist UIs):
- progressive checklist with status icons (pending → running → pass/fail)
- each test runs sequentially
- "RUN ALL TESTS" button at top
- each test has Manual confirm and Auto detect modes
- saves results to a session log
- pass/fail badges, expandable details

### Buttons-everywhere

Every existing serial command becomes a button somewhere:
- pose 6/7/8/9/0 buttons in CTRL tab (work in any mode, auto-switch)
- dance buttons in CTRL tab (M1-M4)
- mode pills in header (M/D/H/K/B/T)
- mag on/off button (always visible in catch banner area)
- test buttons in DIAG modal
- arm/disarm catch in catch banner

### Logging

Auto-record on catch arm. Manual record button in TUNE tab. Records:
- all telemetry frames
- all UI events (button clicks, mode changes)
- all UWB readings + solver outputs
- all FSM transitions

CSV download on STOW or manual stop. Filename: `arm_session_YYYYMMDD_HHMMSS.csv`

---

## 13. INTEGRATION TEST SEQUENCE

After v4 is built:
1. flash firmware on bench, no UWB connected. verify: all v3 commands work, all modes work, all dances work, EEPROM persists, telemetry parses in UI.
2. wire 1 UWB anchor, run init. verify: AT commands succeed, status shows green, $T includes range.
3. wire all 4 UWBs, anchor positions placeholder. verify: trilat returns plausible position when tag is held in known location.
4. enable sim drone mode. drag fake drone in UI. verify: arm follows, base yaw works, IK solves.
5. run "test all" diagnostic. verify: every test has pass.
6. arm catch with sim drone. verify: full FSM runs, magnet fires at right time, recovery works.
7. real UWB tag on real drone. validate ranges + position.
8. real catch attempt with sim drone first, then real drone.

---

## 14. WHAT I'M NOT BUILDING (and why)

- **EKF / Kalman filter for drone tracking.** Light EMA + light tip fusion is plenty for hovering drone. EKF is for moving targets. Add later if needed.
- **Predictive intercept timing.** Drone is hovering, no prediction needed.
- **Multi-drone tracking.** One tag per ranging cycle, one drone at a time.
- **Network ops (Wi-Fi/Ethernet).** Standalone teensy + serial UI. Add ethernet for boat ops in v5.
- **Anchor self-survey.** User manually enters anchor positions. Self-survey would be nice but adds 200+ lines for prototype-stage benefit.
- **Custom DSP on UWB.** RYUW122 module does its own ranging filter; we just consume.

---

## 15. RISKS

1. **RYUW122 AT command syntax.** I'm guessing at exact strings based on web search. First boot, user verifies AT commands match the actual module firmware version. May need 1 round of fixes after first hardware test.
2. **Tilt v2 tuning.** Quaternion math is right, but smoothing α and lookahead time are tunable. Will need iteration on a real boat. Provided as runtime params.
3. **Catch timing.** Magnet fire timing depends on UWB rate + actuator lag. May need to fire 100-200ms early. Configurable via `catch.fire_lead_ms`.
4. **Stall detect false positives.** Slow moves near limits might trigger. Threshold 600ms is a guess. Make tunable.

---

## 16. DELIVERABLES

When done:
- 13 firmware files (arm_main_v4.ino + 12 .h)
- 1 webpage HTML (arm_ctrl_v4.html, single file as before)
- this plan doc (saved as ARCHITECTURE.md alongside firmware)
- a concise CHANGELOG.md (what changed v3→v4)
- a UI screenshot grid showing the new design

---

END OF PLAN. Sign off needed before code starts.
