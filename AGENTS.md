# ArduinoRobotArm — Agent Skills & Project Context

## Project Overview
6-DOF robot arm driven by **6× MG995 servos**, running on **Arduino UNO** via PlatformIO.
All six hardware PWM outputs of the UNO are used — one servo per joint.

## Hardware

### Arduino UNO PWM Pin Assignments
| Pin | Joint          | Axis  | Notes                        |
|-----|----------------|-------|------------------------------|
|  3  | Base           | Yaw   | Full 0–180° rotation         |
|  5  | Shoulder       | Pitch | Heaviest load joint          |
|  6  | Elbow          | Pitch | Second link                  |
|  9  | Wrist Pitch    | Pitch | Wrist up/down                |
| 10  | Wrist Roll     | Roll  | Wrist rotation               |
| 11  | Gripper        | —     | Open/close end-effector      |

### MG995 Servo Specifications
- **Operating voltage**: 4.8 V – 7.2 V (use dedicated 5–6 V supply, NOT Arduino 5 V rail)
- **Stall torque**: ~13 kg·cm @ 6 V
- **Speed**: ~0.20 s / 60° @ 6 V
- **PWM frequency**: 50 Hz (period = 20 ms)
- **Pulse width**: 500 µs – 2400 µs (full range); 1000–2000 µs (safe default)
- **Angle range**: 0° – 180°

### Power Budget
Six MG995 servos can draw up to **~6 × 1 A = 6 A** stall current.
Always power servos from a separate regulated supply; share GND with the Arduino.

## Toolchain
- **Framework**: Arduino (PlatformIO)
- **Board**: `uno` (ATmega328P, 16 MHz, 32 KB flash, 2 KB SRAM)
- **Platform**: `atmelavr`
- **IDE**: VSCode + PlatformIO extension

## Coding Conventions
- **Angle limits for every joint MUST be defined as `#define` constants in `main.cpp`** (essential rule — always present, always editable by hand).
- Use the **Arduino `Servo` library** (`#include <Servo.h>`) for PWM generation.
- Declare one `Servo` object per joint; attach in `setup()`.
- Keep `loop()` non-blocking — use `millis()` timers, not `delay()`.
- Name joints consistently: `base`, `shoulder`, `elbow`, `wristPitch`, `wristRoll`, `gripper`.
- Angles in **degrees (0–180)**; convert to microseconds only when calling `writeMicroseconds()`.
- Serial baud rate: **115200**.

## Skills to Import (per session)

### 1. servo_control
Manage individual servo movement:
- `setAngle(joint, degrees)` — write target angle, clamped to [0, 180].
- `moveSmooth(joint, from, to, stepMs)` — non-blocking incremental move.
- `detachAll()` — detach all servos to save power.

### 2. kinematics
Forward/inverse kinematics helpers for a 6-DOF arm:
- Compute joint angles from Cartesian (x, y, z, roll, pitch, yaw) target.
- Validate reachability before commanding servos.

### 3. serial_protocol
Simple text-based serial command parser:
- Format: `<JOINT> <ANGLE>\n` (e.g., `BASE 90\n`).
- Echo acknowledgment: `OK <JOINT> <ANGLE>` or `ERR <reason>`.
- Supports `HOME` command to return all joints to safe rest position.

### 4. sequence_player
Store and replay motion sequences from PROGMEM to save SRAM:
- `recordFrame(angles[6])` — append frame.
- `playSequence()` — non-blocking playback.
- `stopSequence()` — halt and hold current position.

### 5. safety_limits
Per-joint software angle limits and speed caps:
- Define `MIN_ANGLE[]` and `MAX_ANGLE[]` arrays per joint.
- Enforce maximum slew rate to protect gears under load.

## Repository Layout
```
ArduinoRobotArm/
├── AGENTS.md           ← this file
├── platformio.ini      ← board & env config
├── src/
│   └── main.cpp        ← entry point
├── include/            ← project headers
├── lib/                ← local libraries / skills
└── test/               ← unit tests (native env)
```

## Development Workflow
1. Implement a skill as a library under `lib/<skill_name>/`.
2. Add a header in `lib/<skill_name>/src/<skill_name>.h`.
3. Include the skill in `src/main.cpp`: `#include <skill_name.h>`.
4. Test on `native` env where possible; flash to UNO for hardware validation.
5. Document pin usage and any calibration constants in this file.

## Calibration Notes
*(Fill in measured pulse-width offsets per servo after physical calibration.)*
| Joint       | Min µs | Center µs | Max µs |
|-------------|--------|-----------|--------|
| Base        | 500    | 1500      | 2400   |
| Shoulder    | 500    | 1500      | 2400   |
| Elbow       | 500    | 1500      | 2400   |
| Wrist Pitch | 500    | 1500      | 2400   |
| Wrist Roll  | 500    | 1500      | 2400   |
| Gripper     | 700    | 1500      | 2300   |
