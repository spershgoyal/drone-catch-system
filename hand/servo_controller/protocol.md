# Hand Serial Protocol

Line-based UTF-8 or ASCII protocol over Arduino USB serial.

Recommended serial settings:

- baud: `115200`
- newline: `\n`

## Responses

The controller responds with single-line messages:

- `ok ...` for success
- `err ...` for invalid commands or values
- `status ...` for the current servo state
- `pong` for connectivity tests

## Commands

### `ping`

Connectivity check.

Example:

```text
ping
```

Response:

```text
pong
```

### `status`

Returns current servo positions, targets, and whether any channel is moving.

### `map`

Prints the configured pin and range map for each servo channel.

### `attach`

Attaches all servos and starts holding position.

### `detach`

Detaches all servos so they stop actively holding torque.

### `home`

Moves all servos to their configured home angles.

### `stop`

Sets each target to the current position and stops motion.

### `pose <name>`

Supported starter poses:

- `open`
- `pregrasp`
- `close`
- `home`

Example:

```text
pose pregrasp
```

### `grasp <0.0..1.0>`

Interpolates between each servo's `open_deg` and `closed_deg`.

- `0.0` = fully open
- `1.0` = fully closed

Example:

```text
grasp 0.65
```

### `set <servo> <deg> [speed_deg_s]`

Moves one servo to a target angle.

Examples:

```text
set thumb 105
set index 92 45
```

### `setall <d0> <d1> <d2> <d3> <d4> [speed_deg_s]`

Moves all five channels at once.

Example:

```text
setall 60 80 85 80 75 50
```

### `trim <servo> <deg>`

Applies a small offset to a servo's physical output angle.

Example:

```text
trim thumb -4
```

### `test <servo|all>`

Moves one servo, or all servos, through a safe open/close/home bench test.

Examples:

```text
test thumb
test all
```

### `pulse <on|off|status>`

Controls the built-in repetitive bench pulse mode.

Examples:

```text
pulse status
pulse off
pulse on
```

### `pulse set <contract_deg> <splay_deg> [hold_ms] [speed_deg_s]`

Updates the repetitive pulse mode settings. When pulse mode is already on, the
new settings take effect immediately and restart at the contract position.

Example:

```text
pulse set 10 145 5000 90
```

## Servo Names

The starter firmware accepts:

- `thumb`
- `index`
- `middle`
- `ring`
- `pinky`
- `s0` through `s4`

## Typical Catch Flow

```text
attach
map
status
test thumb
pulse status
pulse set 10 145 5000 90
pulse off
test all
pulse on
```
