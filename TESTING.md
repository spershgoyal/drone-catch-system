# System Test Plan

Use this sequence so you validate one failure mode at a time instead of discovering everything during a live catch.

## Phase 0: Safety and Power

- Remove props from the drone for all indoor and bench tests.
- Put the arm on a stand or clamp it so it cannot tip.
- Power servos and motors from their own supply rails. Do not power the hand servos from the Arduino `5V` pin.
- Confirm all grounds are common between the host computer, arm controller, hand controller, UWB modules, and power supplies.
- Add an easy kill path:
  - motor power switch
  - servo power switch
  - magnet disable switch
  - software `disarm` command

Exit criteria:
- No component resets when a servo or motor starts moving.
- UWB modules stay online during motion.

## Phase 1: Hand Bench Test Only

Run the hand by itself before it ever touches the arm.

1. Flash [hand_servo_controller.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/hand/servo_controller/arduino_firmware/hand_servo_controller.ino:1).
2. Connect one servo at a time.
3. Open Arduino Serial Monitor at `115200` baud with `Newline` line ending, then run:

```text
ping
map
status
test thumb
pose open
pose close
```

4. Confirm each finger moves in the correct direction.
5. If a servo moves backward, set `invert` for that channel in the Arduino config.
6. If a servo hits a hard stop, reduce its `min_deg`, `max_deg`, `open_deg`, or `closed_deg`.

Exit criteria:
- Each servo moves smoothly.
- No binding, chatter, or brown-outs.
- `open`, `pregrasp`, and `close` poses are repeatable.

## Phase 2: UWB Only

Test ranging before involving the camera.

1. Flash the arm v4 firmware in [arm_v4.ino](/Users/spershgoyal/Documents/Playground/drone-catch-system/arm/v4_release/arm_v4/arm_v4.ino:1).
2. Use the arm UI or serial console to run `test uwb`.
3. Place the drone tag in several known positions and record the solved position.
4. Confirm three base anchors stay alive while the tip anchor is optional.

Exit criteria:
- UWB solves are stable at rest.
- Losing one anchor does not crash the controller.
- Base-anchor geometry is good enough that the position estimate does not jump wildly.

## Phase 3: Vision Only

Test the black-target detector separately.

1. Start the tracker with `--no-uwb`.
2. Use a black target on the drone belly or a dummy target.
3. Move the target through:
   - bright light
   - dim light
   - cluttered dark background
   - partial occlusion

Exit criteria:
- The target is detected when centered, off-axis, and near the edges.
- False positives are low enough that the arm will not chase the wrong thing.

## Phase 4: UWB-First Fallback Test

Now validate the new logic order.

1. Run the tracker normally with UWB and camera both connected.
2. Confirm the JSON or overlay reports `source=uwb` when three fresh anchors are present.
3. Disable or block one anchor and confirm it falls back to `source=hybrid`.
4. Block or unplug UWB and confirm it falls back to `source=vision`.

Exit criteria:
- Source switching matches the intended order:
  - `uwb`
  - `hybrid`
  - `vision`
- The tracker does not keep using stale ranges forever.

## Phase 5: Arm + Hand Dry Run

Do this with a dummy drone or foam block first.

1. Mount the hand on the arm.
2. Power the hand separately.
3. Put the arm in a safe staging pose.
4. Send:

```text
pose pregrasp
grasp 0.50
grasp 1.00
```

5. Verify the hand closes around the target without colliding with the arm structure.

Exit criteria:
- The hand stays inside the arm workspace.
- Closing force is enough to hold a dummy payload.
- No finger collides with props, landing legs, or the magnet fixture.

## Phase 6: Full Closed-Loop Dry Run

Use the full stack without spinning props.

1. Vision tracker on host or Pi.
2. Arm controller on Teensy.
3. Hand controller on Arduino.
4. Simulated or hand-carried target.
5. When `capture_ready=true`, command:
   - hand `pose pregrasp`
   - arm commit
   - hand `grasp 1.0`

Exit criteria:
- Serial links remain stable.
- The hand closes only after the arm is in the final capture window.
- Abort paths work.

## Phase 7: Live Drone Test

Only after every earlier phase passes.

1. Start with the drone tethered or in a cage.
2. Use soft finger pads or foam on contact surfaces.
3. Keep manual takeover available at all times.
4. Log:
   - UWB source state
   - vision confidence
   - arm FSM state
   - hand pose/grasp command

Exit criteria:
- Clean capture with no brown-out or unsafe motion.
- Reliable abort when the drone drifts out of bounds.
