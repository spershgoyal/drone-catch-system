# ARM v4 — Drone Catcher Control System

Author: Ethan Shapero  
Hardware: 5-DOF robotic arm (Teensy 4.1)  
Goal: autonomously catch DJI Mini 4 Pro from a moving boat using UWB tag tracking

## Repository layout

```
arm_v4/                 firmware (Arduino sketch + headers)
  arm_v4.ino            main entry
  config.h              pin assignments, constants
  state.h               shared global state (extern declarations)
  sensors.h             pot reads, FK, slip detection
  motors.h              BTS7960 abstraction + stall detection
  safety.h              floor guard + magnet (with magLocked)
  kinematics.h          IK with traj-aware cost
  trajectory.h          s-curve, synced moves, position update
  poses.h               preset poses
  dance.h               dance sequences
  wrist.h               servo control + auto-level
  tilt.h                tilt mode v2 (quat + lookahead, optimized for boats)
  uwb.h                 RYUW122 driver (3 base + 1 tip)
  trilat.h              trilateration + tip fusion + integrity check
  catch_fsm.h           autonomous catch state machine
  eeprom_persist.h      EEPROM save/load
  telemetry.h           $T format
  serial_cmd.h          line-based command parser
webpage/
  arm_ctrl_v4.html      single-file UI (React-free, vanilla + Three.js)
ARCHITECTURE.md         full design doc
CHANGELOG.md            v3 -> v4 diff
README.md               this file
```

## Hardware

- **Teensy 4.1** (NOT 3.5 — pin assignments differ in v4)
- **3 base UWB anchors** on Serial1, Serial2, Serial6 (RYUW122 modules)
- **1 tip UWB anchor** on Serial4 (mounted on forearm boom)
- **1 drone tag** (RYUW122 in tag mode on the drone)
- **BNO08x IMU** on I2C (Wire), pins 18/19
- **3x BTS7960** motor drivers (base, shoulder, elbow)
- **3x potentiometers** (analog feedback)
- **2x servos** (wrist roll/pitch, pins 22/23)
- **Electromagnet** on pin 33 (NPN low-side switch)

### Pin map

| Pin | Function |
|-----|----------|
| 0/1 | Serial1 RX/TX (anchor A1) |
| 2 | base RPWM |
| 3 | shoulder RPWM |
| 5 | shoulder LPWM |
| 7/8 | Serial2 RX/TX (anchor A2) |
| 14 | elbow RPWM |
| 16/17 | Serial4 RX/TX (tip anchor) |
| 18/19 | I2C BNO08x |
| 20 | elbow pot |
| 21 | shoulder pot |
| 22 | wrist roll servo |
| 23 | wrist pitch servo |
| 24/25 | Serial6 TX/RX (anchor A3) |
| 29 | base LPWM |
| 33 | magnet drive (was 16 in v3) |
| 35 | elbow LPWM |
| 38 | base pot |

## Build & flash

### Arduino IDE
1. Open `arm_v4/arm_v4.ino` in Arduino IDE.
2. Install Teensyduino if not already.
3. Install dependencies (Arduino Library Manager):
   - `SparkFun BNO080 Cortex Based IMU`
4. Select board: Teensy 4.1
5. Compile + upload.

### CLI / VS Code
- Same code layout. Open the `arm_v4/` folder.
- `arduino-cli` works: `arduino-cli compile -b teensy:avr:teensy41 arm_v4/`

## UI

Open `webpage/arm_ctrl_v4.html` in Chrome, Edge, or Opera (Web Serial required). Click CONNECT, pick the Teensy serial port. Aesthetic: bone-white, IBM Plex Mono, hazard orange.

## Quickstart

1. Flash firmware
2. Open UI, connect serial
3. Run **Diagnostics** → "RUN ALL TESTS"
4. If all pass, calibrate anchor positions (UWB tab → APPLY + SAVE)
5. To test catch logic without a drone:
   - Toggle "SIM DRONE" in TUNE tab
   - Drag the purple drone in 3D view
   - Click "ARM CATCH" — watch FSM run
6. Real catch: position drone (DJI Mini 4 Pro with UWB tag) in front of arm, click ARM CATCH

## Catch FSM states

```
IDLE → SEARCHING → TRACKING → STAGING → COMMITTING → CATCHING → SECURED → STOWING → IDLE
                                                          ↓
                                                      ABORT/ERROR
```

Approach is from below: arm hovers 8" below drone in TRACKING, drops to 4" in STAGING, sprints upward in COMMITTING, magnet fires when tip anchor reports < 1" from drone tag.

## Known TODOs

1. **RYUW122 AT command syntax** — verify against your module's datasheet on first run. See top of `uwb.h`. May require minor edits to AT commands strings.
2. **Anchor positions** — placeholders in `config.h`. Calibrate after physical install.
3. **Tip anchor offset** — placeholder in `config.h`. Calibrate after physical install.
4. **MAG_PITCH_BIAS** — currently -30° based on previous bracket. Verify after assembly.

## Architecture doc

See `ARCHITECTURE.md` for full design rationale, FSM diagrams, UWB protocol notes, tilt v2 derivation, and integration test sequence.
