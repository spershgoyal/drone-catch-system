# Arm

This folder contains the robotic-arm control stack imported from the attached `drone_catch_arm.zip`.

## Contents

```text
arm/
  teensy_firmware/
    arm_main_v3p.ino
  web_control/
    arm_ctrlv4.html
  v4_release/
    arm_v4/
    webpage/
```

## Hardware Summary

- Teensy 3.5 for the main arm controller
- 3 brushed DC joints with BTS7960 H-bridges
- 2 hobby servos at the wrist
- electromagnet end effector
- BNO08x IMU and multi-turn pots for sensing

## How It Fits Into This Repo

- `teensy_firmware/` controls the arm motion system.
- `v4_release/` contains the newer multi-anchor UWB arm release that the host tracker now mirrors.
- `vision/esp32_cam_firmware/` contains the original ESP32-CAM firmware that paired with this arm project.
- `web_control/` is the browser UI that talks to both the Teensy and ESP32-CAM over Web Serial.
- `hand/servo_controller/` is where the future hand-specific servo control should live.
