# Drone Catch System

Monorepo for the drone-catching project, combining:

- host-side computer vision and UWB tracking
- ESP32-CAM vision firmware
- Teensy-based robotic arm control
- browser-based arm control UI
- printable visual markers
- placeholder space for the future hand-servo controller

## Repository Layout

```text
drone-catch-system/
  vision/
    black-vision-tracker/   # Python/OpenCV/UWB tracker that runs on the host computer
    esp32_cam_firmware/     # ESP32-CAM firmware from the attached arm project
    markers/                # printable drone markers
  arm/
    teensy_firmware/        # robotic arm controller firmware
    web_control/            # browser UI for arm + camera control over Web Serial
    v4_release/             # newer Teensy + multi-anchor UWB release that now drives the host tracker design
  hand/
    servo_controller/       # reserved for the future hand/end-effector servo code
```

## What Runs Where

- `vision/black-vision-tracker` runs on the arm-side computer or SBC.
- `vision/esp32_cam_firmware` runs on the ESP32-CAM.
- `arm/teensy_firmware` runs on the Teensy that drives the robotic arm.
- `arm/v4_release` contains the newer arm firmware with the 3-base-anchor plus tip-anchor UWB flow.
- `arm/web_control` runs in Chrome or Edge using Web Serial.
- `hand/servo_controller` is reserved for the future gripper/hand servo component.

## Quick Start

### Vision tracker

```bash
cd /Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
cp ./tracker_config.example.json ./tracker_config.json
black-vision track --tracker-config ./tracker_config.json --print-json
```

### Arm firmware

Open `arm/teensy_firmware/arm_main_v3p.ino` in Arduino IDE with Teensyduino.

### ESP32-CAM firmware

Open `vision/esp32_cam_firmware/cam_test_v5.ino` in Arduino IDE and flash it to the AI-Thinker ESP32-CAM.

### Web control

Open `arm/web_control/arm_ctrlv4.html` in Chrome/Edge, or serve the folder locally:

```bash
cd /Users/spershgoyal/Documents/Playground/drone-catch-system/arm/web_control
python3 -m http.server 8000
```

Then open `http://localhost:8000/arm_ctrlv4.html`.

## Notes

- The original attached robotic-arm files were reorganized into subsystem folders, but the code itself was preserved.
- The host tracker now mirrors the newer arm-side UWB flow: fresh multi-anchor UWB is preferred first, then hybrid UWB plus vision, then camera-only fallback.
- The newer OpenCV/UWB tracker lives alongside the older ESP32-CAM firmware so you can compare both approaches in one repo.
- The future hand-servo code should go under `hand/servo_controller`.
