# Arduino Robot Arm

6-DOF robot arm driven by 6× MG995 servos. Supports **Arduino UNO** and **ESP32** targets via PlatformIO (default: ESP32).

## Hardware

### Pin Assignments

Servos are wired through an **Arduino Sensor Shield** (D-header labels).

| Shield | Joint       | Axis  | UNO Pin | ESP32 GPIO |
|--------|-------------|-------|---------|------------|
| D3     | Base        | Yaw   | 3       | 25         |
| D5     | Shoulder    | Pitch | 5       | 16         |
| D6     | Elbow       | Pitch | 6       | 27         |
| D9     | Wrist Pitch | Pitch | 9       | 13         |
| D10    | Wrist Roll  | Roll  | 10      | 5          |
| D11    | Gripper     | —     | 11      | 23         |

**Power:** Servos must be powered from a dedicated 5–6 V supply (≥6 A). Share GND with the Arduino. Do **not** power servos from the Arduino 5 V rail.

## Getting Started

### Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) or the VSCode PlatformIO extension
- **ESP32** (default): connected via USB (`/dev/ttyUSB0` on Linux, `COMx` on Windows)
- **Arduino UNO** (alternate): connected via USB (`/dev/ttyACM0` on Linux, `COMx` on Windows)

### Build & Flash

```bash
# Build default target (ESP32)
pio run

# Upload to ESP32 (default)
pio run --target upload

# Build / upload Arduino UNO explicitly
pio run -e uno
pio run -e uno --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

## Controlling the Robot

All control happens over a plain-text serial interface at **115200 baud**. Commands are **case-insensitive** and terminated with a newline (`\n`). One response line is returned per command.

### Connect

```bash
# PlatformIO built-in monitor
pio device monitor

# picocom — ESP32
picocom -b 115200 /dev/ttyUSB0

# picocom — Arduino UNO
picocom -b 115200 /dev/ttyACM0

# screen — ESP32
screen /dev/ttyUSB0 115200
```

On Windows use the COM port shown in Device Manager (e.g. `COM3`) in place of the `/dev/tty*` path.

### Command Reference

| Command | Arguments | Description | Response |
|---------|-----------|-------------|----------|
| `BASE` | `<angle>` | Move base (yaw) to angle in degrees | `OK BASE <angle>` |
| `SHOULDER` | `<angle>` | Move shoulder (pitch) | `OK SHOULDER <angle>` |
| `ELBOW` | `<angle>` | Move elbow (pitch) | `OK ELBOW <angle>` |
| `WRIST_PITCH` | `<angle>` | Move wrist pitch | `OK WRIST_PITCH <angle>` |
| `WRIST_ROLL` | `<angle>` | Move wrist roll | `OK WRIST_ROLL <angle>` |
| `GRIPPER` | `<angle>` | Move gripper (open/close) | `OK GRIPPER <angle>` |
| `MOVE` | `<base> <shoulder> <elbow> <wristPitch> <wristRoll> <gripper>` | Move all 6 joints simultaneously, synchronized | `OK MOVE` |
| `HOME` | — | Return all joints to 90° (safe rest position) | `OK HOME` |
| `STOP` | — | Emergency halt — freeze all joints immediately | `OK STOP` |
| `POS` | — | Report current logical position of all joints | `OK POS <f0> <f1> <f2> <f3> <f4> <f5>` |

#### Angle limits (hardware + software enforced)

| Joint | Min | Max | Notes |
|-------|-----|-----|-------|
| BASE | 0° | 180° | Full rotation |
| SHOULDER | 15° | 165° | Mechanically constrained to protect gears |
| ELBOW | 0° | 180° | |
| WRIST_PITCH | 0° | 180° | |
| WRIST_ROLL | 0° | 180° | |
| GRIPPER | 10° | 170° | 10° = fully open, 170° = fully closed |

#### Error responses

| Response | Cause |
|----------|-------|
| `ERR UNKNOWN_JOINT` | Joint name not recognised |
| `ERR MISSING_ANGLE` | Joint command sent without an angle argument |
| `ERR MISSING_ARGS` | `MOVE` sent with no arguments |
| `ERR TOO_FEW_ARGS` | `MOVE` sent with fewer than 6 angles |
| `ERR UNKNOWN_CMD` | Command not recognised |

### Examples

#### Move a single joint

```
→ BASE 90
← OK BASE 90.0

→ shoulder 45
← OK SHOULDER 45.0

→ GRIPPER 170
← OK GRIPPER 170.0
```

#### Query and reset position

```
→ POS
← OK POS 90.0 90.0 90.0 90.0 90.0 90.0

→ HOME
← OK HOME
```

#### Synchronized multi-joint move

Move all joints to a pose in a single command. All joints start and finish together using a trapezoidal motion profile:

```
→ MOVE 90 60 120 90 45 30
← OK MOVE
```

Order: `base shoulder elbow wristPitch wristRoll gripper`

#### Emergency stop

```
→ STOP
← OK STOP
```

#### Pick-and-place sequence

A minimal sequence to reach over, grip an object, and return home:

```
→ HOME
← OK HOME
→ MOVE 45 60 90 90 90 10
← OK MOVE
→ GRIPPER 150
← OK GRIPPER 150.0
→ MOVE 135 60 90 90 90 150
← OK MOVE
→ GRIPPER 10
← OK GRIPPER 10.0
→ HOME
← OK HOME
```

## Repository Layout

```
arduino_robot_arm/
├── AGENTS.md           ← hardware specs & agent skills
├── CONTRIBUTING.md     ← git workflow & contribution rules
├── platformio.ini      ← board & environment config
├── src/
│   └── main.cpp        ← firmware entry point
├── include/            ← shared project headers
├── lib/                ← local libraries (one folder per skill)
│   ├── SerialProtocol/
│   └── TCMProfile/
└── test/               ← unit tests (native PlatformIO env)
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for branch naming, commit conventions, and the code review process.

## License

Specify your license here (e.g., MIT, Apache 2.0, proprietary).
