# Drone Catch System Flowcharts

These flowcharts are derived from the current code in:

- [hand/servo_controller/arduino_firmware/hand_servo_controller.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/hand/servo_controller/arduino_firmware/hand_servo_controller.ino:1)
- [vision/black-vision-tracker/black_vision/cli.py](/Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker/black_vision/cli.py:1)
- [vision/black-vision-tracker/black_vision/tracking.py](/Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker/black_vision/tracking.py:1)
- [vision/black-vision-tracker/black_vision/uwb.py](/Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker/black_vision/uwb.py:1)
- [vision/black-vision-tracker/black_vision/detector.py](/Users/spershgoyal/Documents/Playground/drone-catch-system/vision/black-vision-tracker/black_vision/detector.py:1)
- [vision/esp32_cam_firmware/cam_test_v5.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/vision/esp32_cam_firmware/cam_test_v5.ino:1)
- [arm/v4_release/arm_v4/arm_v4.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/arm/v4_release/arm_v4/arm_v4.ino:1)
- [arm/v4_release/arm_v4/catch_fsm.h](/Users/spershgoyal/Documents/Playground/drone-catch-system/arm/v4_release/arm_v4/catch_fsm.h:1)
- [arm/v4_release/arm_v4/serial_cmd.h](/Users/spershgoyal/Documents/Playground/drone-catch-system/arm/v4_release/arm_v4/serial_cmd.h:1)
- [arm/web_control/arm_ctrlv4.html](/Users/spershgoyal/Documents/Playground/drone-catch-system/arm/web_control/arm_ctrlv4.html:1)

## 1. Repository Runtime Topology

```mermaid
flowchart LR
    OP["Operator"] --> UI["Web Control UI"]
    UI --> ARM["Teensy Arm Firmware v4"]
    ARM --> MOTORS["Base / Shoulder / Elbow / Wrist / Magnet"]
    ARM --> UWB["3 Base Anchors + 1 Tip Anchor"]

    CAM["ESP32-CAM Firmware"] --> HOST["Host Vision Tracker"]
    UWB --> HOST
    HOST --> ARM

    HOST --> HAND["Arduino Hand Controller"]
    HAND --> FINGERS["5 Servo Hand"]
```

## 2. Hand Firmware Flow

This is the actual control loop in `hand_servo_controller.ino`.

```mermaid
flowchart TD
    A["setup()"] --> B["initializeState()"]
    B --> C["attachAll()"]
    C --> D["print ready / wiring / help"]
    D --> E["loop()"]

    E --> F["readSerialLines()"]
    F --> G{"newline complete?"}
    G -- "no" --> H["append byte to line buffer"]
    H --> I["updateMotion()"]
    G -- "yes" --> J["handleLine(command)"]

    J --> K{"command type"}
    K -- "ping/status/map/help" --> L["print response"]
    K -- "attach/detach" --> M["attachAll() or detachAll()"]
    K -- "home/pose/grasp" --> N["set target angles"]
    K -- "set/setall" --> O["parse target(s) and speed"]
    K -- "trim" --> P["update trim and rewrite servo"]
    K -- "test" --> Q["run open -> close -> home sequence"]
    K -- "unknown" --> R["print error"]

    L --> I
    M --> I
    N --> I
    O --> I
    P --> I
    Q --> I
    R --> I

    I --> S{"20 ms elapsed?"}
    S -- "no" --> E
    S -- "yes" --> T["for each attached servo: step current toward target"]
    T --> U["write physical angle with trim/invert/clamp"]
    U --> E
```

## 3. Vision Tracker CLI Flow

This is the entrypoint structure in `cli.py`.

```mermaid
flowchart TD
    A["black-vision main()"] --> B["argparse subcommand selection"]
    B --> C{"detect / calibrate / track"}

    C -- "detect" --> D["resolve config"]
    D --> E["open camera"]
    E --> F["frame loop"]
    F --> G["BlackColorModel.detect()"]
    G --> H["annotate and display"]
    H --> I{"q pressed?"}
    I -- "no" --> F
    I -- "yes" --> Z["release camera / destroy windows"]

    C -- "calibrate" --> J["resolve config"]
    J --> K["open camera + create trackbars"]
    K --> L["frame loop"]
    L --> M["read trackbar values into live config"]
    M --> N["detect + annotate"]
    N --> O{"s / q / continue"}
    O -- "s" --> P["save_config()"]
    O -- "continue" --> L
    O -- "q" --> Z

    C -- "track" --> Q["load tracker config"]
    Q --> R["override CLI camera settings"]
    R --> S["open camera"]
    S --> T["open/configure UWB anchors unless --no-uwb"]
    T --> U["frame loop"]
    U --> V["poll anchors whose poll interval has elapsed"]
    V --> W["DroneTrackingSystem.process()"]
    W --> X["annotate frame + optional JSON output"]
    X --> Y{"q pressed?"}
    Y -- "no" --> U
    Y -- "yes" --> Z
```

## 4. Per-Frame Detection and Fusion Flow

This is the runtime path inside `DroneTrackingSystem.process()` and `PositionFusionEngine.estimate()`.

```mermaid
flowchart TD
    A["new frame + new anchor measurements"] --> B["detector.detect(frame)"]
    B --> C["camera_model.bearing_from_detection()"]
    C --> D["update latest measurement cache by anchor_id"]
    D --> E["fusion.current_measurements() keeps only fresh anchors"]
    E --> F["fusion.estimate(fresh_ranges, bearing)"]

    F --> G{"prefer_uwb?"}

    G -- "yes" --> H["try pure UWB trilateration"]
    H --> I{"valid?"}
    I -- "yes" --> J["build estimate source=uwb"]
    I -- "no" --> K["try hybrid UWB + camera solve"]
    K --> L{"valid?"}
    L -- "yes" --> M["build estimate source=hybrid"]
    L -- "no" --> N["fallback to vision-only ray + assumed distance"]

    G -- "no" --> O["try hybrid first"]
    O --> P{"valid?"}
    P -- "yes" --> M
    P -- "no" --> Q["try pure UWB"]
    Q --> R{"valid?"}
    R -- "yes" --> J
    R -- "no" --> N

    J --> S["smooth against last position"]
    M --> S
    N --> S
    S --> T["compute distance_to_gripper and capture_ready"]
    T --> U["TrackingFrameResult"]
```

## 5. UWB Serial Anchor Flow

This is the live driver behavior in `uwb.py`.

```mermaid
flowchart TD
    A["ReyaxSerialAnchor.open()"] --> B["open serial port"]
    B --> C["flush input/output"]

    C --> D{"auto configure?"}
    D -- "yes" --> E["send AT / MODE / ADDRESS / NETWORKID / CPIN / CHANNEL / BANDWIDTH / RSSI"]
    D -- "no" --> F["ready for polling"]
    E --> F

    F --> G["poll_distance()"]
    G --> H{"send mode"}
    H -- "address_only" --> I["AT+ANCHOR_SEND=tag"]
    H -- "payload" --> J["AT+ANCHOR_SEND=tag,len,payload"]
    I --> K["read serial lines until timeout"]
    J --> K
    K --> L{"line starts with +ANCHOR_RCV?"}
    L -- "yes" --> M["parse distance / RSSI / timestamp"]
    L -- "no" --> N{"line starts with +ERR?"}
    N -- "yes" --> O["raise error"]
    N -- "no" --> K
    M --> P["return AnchorMeasurement"]
```

## 6. ESP32-CAM Firmware Flow

This is the logic in `cam_test_v5.ino`.

```mermaid
flowchart TD
    A["setup()"] --> B["begin Serial + EEPROM"]
    B --> C["allocate PSRAM buffers for labels / parents / stats"]
    C --> D["configure camera pins and SVGA mode"]
    D --> E["esp_camera_init()"]
    E --> F["load persisted thresholds / exposure / shape settings"]
    F --> G["loop()"]

    G --> H["handle_serial()"]
    H --> I["esp_camera_fb_get()"]
    I --> J{"frame valid?"}
    J -- "no" --> K["print capture error"]
    K --> G
    J -- "yes" --> L["two-pass connected-components scan over dark pixels"]
    L --> M["compute per-component area / bbox / perimeter / centroid"]
    M --> N["reject blobs by area, fill, circularity, aspect"]
    N --> O{"best component found?"}
    O -- "yes" --> P["estimate XYZ from marker size + image center"]
    P --> Q["print FOUND line"]
    O -- "no" --> R["print LOST line"]
    Q --> S["update LED pulse / fps / last detection cache"]
    R --> S
    S --> T["return frame buffer"]
    T --> G
```

## 7. Arm v4 Main Loop

This is the actual ordering in `arm_v4.ino`.

```mermaid
flowchart TD
    A["loop()"] --> B["buildJointMotorActive()"]
    B --> C["updateSensors(active)"]
    C --> D["updateImu()"]
    D --> E["uwb_update()"]

    E --> F{"simDroneOn?"}
    F -- "yes" --> G["copy sim drone XYZ into dronePos"]
    F -- "no" --> H["trilatSolveDrone()"]
    H --> I{"solve succeeded?"}
    I -- "yes" --> J["update dronePos / droneResidual / tip integrity"]
    I -- "no" --> K["droneValid = false"]

    G --> L["hardLimitCheck()"]
    J --> L
    K --> L

    L --> M["floorGuard(allow commit/catching exception)"]
    M --> N["updateStallDetect(active)"]
    N --> O["updateWristAutoLevel()"]

    O --> P{"mode needs tilt update?"}
    P -- "yes" --> Q["updateTiltMode_v2()"]
    P -- "no" --> R["skip tilt"]
    Q --> S{"mode != TILT?"}
    R --> S

    S -- "yes" --> T["positionUpdate(tightMargin, disableWrongWay)"]
    T --> U{"MANUAL or DEBUG?"}
    U -- "yes" --> V["updateManualPulses()"]
    U -- "no" --> W["continue"]
    S -- "no" --> W

    V --> X{"DANCE mode?"}
    W --> X
    X -- "yes" --> Y["updateDance()"]
    X -- "no" --> Z{"CATCH mode?"}
    Y --> Z
    Z -- "yes" --> AA["updateCatchFsm()"]
    Z -- "no" --> AB["skip"]

    AA --> AC["read serial chars and line commands"]
    AB --> AC
    AC --> AD["emit telemetry on schedule"]
    AD --> A
```

## 8. Catch FSM

This is the autonomous catch logic in `catch_fsm.h`.

```mermaid
flowchart TD
    A["IDLE"] -->|arm command| B["SEARCHING"]
    B -->|droneValid| C["TRACKING"]
    B -->|timeout or disarm| A

    C -->|stable lock window| D["STAGING"]
    C -->|drone lost| B
    C -->|disarm| H["ABORT"]

    D -->|reached stage band| E["COMMITTING"]
    D -->|drone moved too far| C
    D -->|timeout| H

    E -->|tip anchor near drone OR FK fallback threshold| F["CATCHING"]
    E -->|integrity fail| H
    E -->|timeout| H

    F -->|hold elapsed| G["SECURED"]
    G -->|settle elapsed| I["STOWING"]
    I -->|home / retract reached| A

    H -->|safe retract finished| A
```

## 9. Web Control UI Flow

This is the main app-level control/data path in `arm_ctrlv4.html`.

```mermaid
flowchart TD
    A["App() mount"] --> B["build serial hooks for arm and camera"]
    B --> C["build 3D scene + telemetry history + UI state"]

    C --> D{"user action"}
    D -- "connect arm" --> E["navigator.serial.requestPort()"]
    E --> F["start reader loop"]
    F --> G["onSerialLine() / onTelemetry()"]
    G --> H["update angles, tip pose, offsets, mode, UWB, catch state"]
    H --> I["rerender controls, graphs, 3D scene"]

    D -- "connect camera" --> J["second serial hook for ESP32-CAM"]
    J --> K["onCamLine() / onCamRaw()"]
    K --> L["parse FOUND / LOST / raw camera XYZ"]
    L --> I

    D -- "button, key, slider, gamepad" --> M["sendChar() or sendLine()"]
    M --> N["firmware command over Web Serial"]
    N --> G

    D -- "set 3D target or catch action" --> O["convert UI target to XYZ command sequence"]
    O --> N
```

## 10. Legacy Arm v3

The older `arm/teensy_firmware/arm_main_v3p.ino` is a single-file predecessor to v4. Its control structure is simpler:

```mermaid
flowchart TD
    A["setup()"] --> B["load EEPROM offsets"]
    B --> C["init pins, servos, IMU"]
    C --> D["loop()"]
    D --> E["read pots / FK / IMU"]
    E --> F["enforce limits / floor guard"]
    F --> G["run mode logic: manual / xyz / height / dance / tilt"]
    G --> H["read serial commands"]
    H --> I["emit telemetry"]
    I --> D
```
