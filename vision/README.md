# Vision

This folder contains the camera and tracking pieces for the drone-catching system.

## Contents

- `black-vision-tracker/`
  - Host-side Python project using OpenCV for black-target detection and Reyax UWB fusion.
  - Best fit when you want higher-level tracking logic to run on a laptop, mini PC, or SBC.
- `esp32_cam_firmware/`
  - Existing ESP32-CAM firmware from the attached arm project.
  - Useful as the on-device camera path and a reference implementation.
- `markers/`
  - Printable black circular targets for the drone belly.

## Recommended Workflow

Use the ESP32-CAM as the image source, but keep heavy tracking and sensor fusion in `black-vision-tracker/` on the host computer. That gives you more room for OpenCV, logging, calibration, and future fusion with extra UWB anchors.
