# Drone Catching Arm

5-DOF robotic arm that uses an ESP32-CAM to track a marker on a drone's belly
and an electromagnet to catch it.

## What's in here

```
firmware/
  arm_main_v3p/         <- main arm controller, runs on Teensy 3.5
    arm_main_v3p.ino
  cam_test_v5/          <- vision system, runs on ESP32-CAM AI-Thinker
    cam_test_v5.ino
web/
  arm_ctrlv4.html       <- single-file React app, drives both via Web Serial
markers/
  drone_marker_6in.pdf  <- 6" black circle, prints across 2 letter pages
  drone_marker_2in.pdf  <- 2" black circle, prints on one page
```

## Hardware

### Arm (Teensy 3.5)

- **Motors:** 3x 600JSX brushed DC w/ worm gearbox, driven by BTS7960 H-bridges
  - Base: 80 kg/cm
  - Shoulder + elbow: 200 kg/cm each
- **Wrist:** 2x standard hobby servos (roll + pitch), 7V from buck regulator
- **End effector:** 12V 25kg electromagnet on the wrist, with a 60x60x1.5mm mild
  steel plate stuck to the drone's belly
- **Sensors:** 10-turn pots on each of the 3 main joints, BNO08x IMU
- **Power:** Meanwell LRS-350-24 (24V/14A) for motors, 24-to-7V buck for servos
- **Structure:** 3D printed ABS, ~40% gyroid infill

Pin assignments are at the top of `arm_main_v3p.ino`.

### Vision (ESP32-CAM AI-Thinker)

- Standard AI-Thinker board with OV2640 camera
- Mounted 15" to the right of the arm base, lens pointing UP
- USB-C side faces the arm
- Communicates over USB serial (separate from the Teensy)

### Drone

- DJI Mini 4 Pro (249g)
- 6" printed marker taped flat to the belly
- Steel catch plate on the belly under or beside the marker

## Setup

### 1. Flash the firmware

Both .ino files compile in Arduino IDE.

**Teensy 3.5 board** for `arm_main_v3p.ino`. Needs Teensyduino installed.
Required libraries:
  - `PWMServo` (built into Teensyduino)
  - `Wire` (built in)
  - `SparkFun_BNO080_Arduino_Library` (install via Library Manager)
  - `EEPROM` (built into Teensyduino)

**AI Thinker ESP32-CAM** for `cam_test_v5.ino`. Tools menu settings:
  - Board: AI Thinker ESP32-CAM
  - Partition: Huge APP (3MB No OTA / 1MB SPIFFS)
  - PSRAM: Enabled (REQUIRED — won't fit in regular RAM)
  - CPU Frequency: 240MHz

You'll need an FTDI/USB-TTL programmer to flash the ESP32-CAM (the AI-Thinker
board has no built-in USB). Hold GPIO0 to GND while pressing reset to enter
bootloader.

### 2. Open the web app

`web/arm_ctrlv4.html` is a single self-contained file. Open in Chrome (must be
Chrome or Edge — Web Serial isn't in Firefox/Safari).

The page expects to be served over HTTP/HTTPS (Web Serial requires a secure
context). Easiest way: drag the file into a Chrome tab. If that doesn't work
because of the file:// scheme, run a quick local server:

```
python -m http.server 8000
# then open http://localhost:8000/arm_ctrlv4.html
```

Click **CONNECT TEENSY** in the top bar, pick the Teensy's serial port. Then
click **CONNECT CAM** in section G to connect the ESP32-CAM port too.

### 3. Calibrate

**Joint offsets** (Section D — CALIBRATION):
The pots are not perfectly aligned with the joints, so each joint needs an
offset to make reported angles match physical reality. Move each joint to a
known position (e.g. arm vertical = shoulder at 0°), look at the reported
angle, and use the +/- buttons to bring it to the correct value.

When done, hit **SAVE**. Persists across boots.

**Cam intrinsics:**
1. Print the 6" marker, tape it together carefully, verify the printed scale
   bar reads exactly 100mm with a ruler
2. Tape the marker flat to the drone's belly
3. In the cam terminal (section G), `D152.4` to tell the firmware your marker
   is 152.4mm
4. Aim cam at well-lit scene, wait 2 seconds for AE to settle
5. Send `X` to lock exposure
6. Hold marker at exactly 1000mm from the lens, send `C1000` to back-solve
   the focal length
7. Send `save` to persist

## How it works

### Arm

The Teensy reads pots, runs IK to convert XYZ targets to joint angles, drives
motors with synced-arrival S-curve profiles. Modes:
- M = manual jog
- K = XYZ target
- H = height-only target
- D = dance routines (mostly for showing off)
- T = IMU tilt-compensation (holds tip in world frame even as base tilts)
- B = debug
- 6/7/8/9/0 = preset poses

### Cam

ESP32-CAM grabs SVGA grayscale frames, runs connected-components labeling to
find dark blobs, scores each by circularity + fill ratio + bounding box aspect,
and reports the most circle-like blob's center and area. Distance is computed
from the apparent diameter (area-from-pixels → distance via focal length).

Outputs `FOUND XYZ=(x, y, z) mm area=... circ=... ...` lines over serial.

### Web app

React + Three.js. Connects to both the Teensy and ESP32-CAM via Web Serial.
Renders a 3D model of the arm, parses telemetry from both devices, exposes
calibration UI, and runs the catch sequence:

1. Wrist pre-positions to face the drone (roll=90, pitch=120)
2. Tip moves to 5" below the drone's reported XYZ
3. Hold for 1.5s to let everything settle
4. Tip rises into contact
5. (Future: trigger electromagnet, retract)

## Catch sequence

In Section G (CAM TRACK), after the cam is reporting valid drone positions,
hit **CATCH DRONE**. The arm goes through the sequence above. **ABORT** stops
it.

For testing without a real drone, you can spin up the simulator on the Catch
Sim tab — drag a virtual drone around, watch the arm track it.

## Known weirdness

- ESP32-CAM stays in bootloader if RTS is held — the web app pulses DTR/RTS
  on connect to escape this. If the cam isn't responding, hit reset on the
  AI-Thinker board.
- Web Serial doesn't always release ports cleanly. If a connect fails after
  a previous session, refresh the page.
- The Teensy sends telemetry at 20Hz. If you see laggy updates, check serial
  buffer health.
- Temperature affects the BTS7960 a lot. Long runs of high PWM can heat-throttle.
  We have a brief slow-zone around joint limits to mitigate.

## Original author

Ethan Shapero. DM with questions.
