# CHANGELOG

## v4.0.0 — 2026-04-30

Major rework. Splits monolithic v3p_4.ino into 17-file modular structure. Adds UWB drone tracking, autonomous catch FSM, rebuilt tilt mode, software stall detection.

### Added
- **UWB drone tracking** — 4 RYUW122 modules (3 base anchors + 1 tip anchor)
  - `uwb.h` non-blocking driver with AT init state machine
  - `trilat.h` 3-anchor trilateration with mirror disambiguation
  - Light tip-anchor fusion (weight 0.2) for close-range refinement
  - Tip integrity check: aborts catch if FK-vs-UWB diverge > 4" for 200ms
  - EMA smoothing (α=0.4) on solved drone position
- **Catch FSM** (`catch_fsm.h`) — 10 states, autonomous from arm to stow
  - Approaches drone from below; magnet face up
  - Stages 4" below, sprints upward in COMMITTING phase
  - Tight position margins (0.3°) during final approach
  - Magnet auto-locks during CATCH states (stopAll won't drop)
- **Tilt mode v2** (`tilt.h`) — rebuilt for boats
  - Quaternion-based throughout (no Euler reconstruction jitter)
  - 50ms motion lookahead using angular velocity
  - EMA-smoothed joint targets prevent stuttering
  - Tracks moving targets (catch FSM writes drone position into world target)
- **Software stall detection** (`motors.h`) — motor commanded but pot not moving for 600ms cuts that joint
- **Slip detection** (`sensors.h`) — flags uncommanded joint motion (e.g. shoulder slipping past balance point) without aborting moves; position loop self-corrects
- **Sim drone mode** — UI sends `$D X Y Z` packets, firmware tracks fake drone with same code path
- **Diagnostics modal** (UI) — checklist-style pre-flight test sequence (motors, pots, IMU, UWB, magnet)
- **Anchor health bars** (UI) — per-anchor quality + range visualization
- **Auto CSV logging** — records all telemetry on catch arm; downloadable on stop
- **Catch banner** (UI) — slides down from header when catch armed; live state, drone pos, mag status, elapsed
- **Buttons-everywhere** (UI) — every command surfaced as a button; auto-switches mode when needed (e.g. clicking pose '8' from MANUAL auto-switches to XYZ)
- **Connect splash** (UI) — fade-in transition on serial pair
- **3-tab side panel** (UI) — CTRL / UWB / TUNE (replaced old multi-tab layout)
- **Anchor calibration commands** — `cal anchor N x y z`, `cal tip p l z`, persisted to EEPROM
- **catch reset / mag lock / tcompcatch** commands
- IK trajectory cost penalty for paths through SHLD_FORBID zone

### Changed
- **Target Teensy 3.5 → 4.1** — pin assignments updated, takes advantage of Serial4/Serial6
- **Magnet pin 16 → 33** — frees Serial4 for tip anchor UART
- **Pot read 16-sample boxcar → 4-sample + α=0.4 EMA** — cleaner derivative for tilt PD
- Modes enum extended with `CATCH_MODE`
- Telemetry `$T` extended with catch state, drone XYZ, 4 ranges, 4 qualities, stall flags, magLocked, simDroneOn, catchArmed (back-compat: UI ignores fields it doesn't know)
- Tilt mode telemetry rate boosted to 50Hz during catch states
- EEPROM magic `JOF3` → `JOF4` (old saves invalidated; layout includes anchors)
- Floor guard suspended during COMMITTING/CATCHING (allows brief Z dips for final approach)

### Removed (UI)
- CATCH SIM tab (replaced by sim drone mode in main viewport)
- The everything-jammed-into-tabs layout in favor of clean 3-tab side panel + modals
- All "must use terminal commands" workflows — every operation is a button now

### Notes / TODOs
- RYUW122 AT command syntax in `uwb.h` is best-guess from REYAX docs. Verify after first hardware test.
- Anchor positions in `config.h` are placeholders from your spec; calibrate after physical install.
- Tilt v2 tuning params (α, lookahead) are starting points; expect iteration on a real boat.

## v3.x — 2026-04 (previous)
- v3p_4: cal commands, EEPROM persistence, magnet bias, basic tilt mode
- v3: introductory split between modes, dances, IK
- v2 / v1: bring-up
