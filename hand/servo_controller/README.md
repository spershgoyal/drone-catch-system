# Hand Servo Controller

Arduino-only starter stack for a 5-servo hand or gripper.

## Recommended Single-Controller Choice

Use the Arduino as the hand controller.

Why this is the better one-box choice for 5 hobby servos:

- it generates stable servo pulses directly
- it handles current-spike timing better than a general-purpose Linux computer
- it keeps moving even if the main vision computer is busy
- it is much easier to bench-test from the Arduino Serial Monitor

If you ever decide to use only a Raspberry Pi later, the right path is usually a PCA9685 servo driver board. For now, the repo is set up for Arduino-only control.

## Folder Layout

```text
hand/servo_controller/
  arduino_firmware/
    hand_servo_controller.ino
  protocol.md
```

## Wiring

Recommended bench wiring for the hand controller:

- Arduino USB to your computer for flashing and Serial Monitor commands
- 5 servo signal wires to Arduino pins:
  - `thumb` -> pin `3`
  - `index` -> pin `5`
  - `middle` -> pin `6`
  - `ring` -> pin `9`
  - `pinky` -> pin `10`
- all servo red wires to an external `5V` to `6V` servo supply
- all servo black or brown wires to servo power ground
- Arduino `GND` tied to the servo power ground

Do not power the servo rail from the Arduino `5V` pin.

## Default Servo Mapping

The starter firmware assumes five finger channels:

- `thumb`
- `index`
- `middle`
- `ring`
- `pinky`

Change the pin numbers, pulse ranges, and open/closed angles in [hand_servo_controller.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/hand/servo_controller/arduino_firmware/hand_servo_controller.ino:1) to match your hand geometry.

Each channel has:

- `pin`
- `min_us` and `max_us`
- `home_deg`
- `open_deg`
- `closed_deg`
- `min_deg` and `max_deg`
- `trim_deg`
- `invert`

## Commands

The Arduino firmware accepts line-based commands over USB serial:

- `ping`
- `status`
- `map`
- `attach`
- `detach`
- `home`
- `pose open`
- `pose pregrasp`
- `pose close`
- `grasp 0.65`
- `set thumb 110`
- `setall 90 90 90 90 90`
- `test thumb`
- `test all`
- `stop`

Full details are in [protocol.md](/Users/spershgoyal/Documents/Playground/drone-catch-system/hand/servo_controller/protocol.md:1).

## How To Use It

1. Open [hand_servo_controller.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/hand/servo_controller/arduino_firmware/hand_servo_controller.ino:1) in Arduino IDE.
2. Select your board and serial port.
3. Upload the sketch.
4. Open Serial Monitor.
5. Set line ending to `Newline`.
6. Set baud to `115200`.
7. Try:

```text
ping
map
status
pose open
test thumb
pose pregrasp
grasp 0.50
grasp 1.00
```

## Integration Hook

The intended control sequence during a catch is:

1. tracker says target is valid and arm is approaching
2. operator or higher-level controller sends `pose pregrasp`
3. arm enters final commit
4. controller sends `grasp 1.00` once the hand is inside the capture window
5. after secure hold, controller keeps the hold pose or returns to a custom safe pose

## Tuning Notes

- Start with low motion speed.
- Bench-test one servo at a time.
- Use `test thumb`, `test index`, and so on before `test all`.
- If a servo overtravels electrically, reduce `min_us` and `max_us`.
- Reduce `closed_deg` on any finger that hits a hard stop.
- Flip `invert` on any finger that closes the wrong direction.
- If a finger needs a small neutral correction, use `trim_deg`.

## Next Upgrade Options

- store trims in EEPROM
- add current or force sensing
- add tendon-synergy mapping instead of one-servo-per-finger
- add a shared command channel from the arm Teensy so the hand can be driven automatically during final capture
