# Servo Calibration — Field Experience

Collected during hands-on calibration of the 6-DOF robot arm (ESP32 + MG-996R servos).

---

## Hardware

| Joint       | Pin (ESP32) | GPIO  |
|-------------|-------------|-------|
| Base        | D3          | GPIO25 |
| Shoulder    | D5          | GPIO16 |
| Elbow       | D6          | GPIO27 |
| Wrist Pitch | D9          | GPIO13 |
| Wrist Roll  | D10         | GPIO5  |
| Gripper     | D11         | GPIO23 |

- **Servo type**: MG-996R (standard 50 Hz, 500–2500 µs range)
- **Library**: `madhephaestus/ESP32Servo@^3.0.5`
- **Upload port**: COM15 (Windows) — always pass `--upload-port COM15`

---

## Calibration Results

| Joint       | Physical Center | Offset (°) | Notes                          |
|-------------|-----------------|------------|--------------------------------|
| Base        | 1500 µs         | 0          | Perfect at nominal center      |
| Shoulder    | 1000 µs         | −45        | Center shifted 500 µs low      |
| Elbow       | 1500 µs         | 0          | Perfect at nominal center      |
| Wrist Pitch | 1500 µs         | 0          | Perfect at nominal center      |
| Wrist Roll  | 1500 µs         | 0          | Perfect at nominal center      |
| Gripper     | TBD             | TBD        | Not yet confirmed              |

### How CENTER_OFFSET is applied (TCMController)

```
physical_angle = commanded_angle + centerOffset
pulse_us = minUs + (physical_angle / 180) × (maxUs − minUs)
```

So if physical center is at 1000 µs (= 45°) but commanded center is 90°:
→ `90 + (−45) = 45°` → `500 + (45/180 × 2000) = 1000 µs` ✓

---

## Calibration Method

### Step 1 — Continuous sweep test per joint

Each joint gets its own PlatformIO environment with a define flag:

```ini
build_flags = -D TARGET_ESP32 -D SERVO_CENTER_TEST -D SERVO_<JOINT>_INTERACTIVE
```

In `setup()`: allocate timers, attach servo.
In `loop()`: sweep between two pulse values, 2 s each, repeating.

```cpp
// setup()
ESP32PWM::allocateTimer(0);
ESP32PWM::allocateTimer(1);
ESP32PWM::allocateTimer(2);
ESP32PWM::allocateTimer(3);
servo.setPeriodHertz(50);
servo.attach(PIN, 500, 2500);

// loop()
servo.writeMicroseconds(LOW_US);
delay(2000);
servo.writeMicroseconds(HIGH_US);
delay(2000);
```

### Step 2 — Narrow the sweep to bracket center

Start wide (e.g., 1000–2000 µs), then narrow around where center appears.
Example for shoulder: `1000↔2000` → `1000↔1500` → `500↔1000` → confirmed 1000 µs.

### Step 3 — Record the offset

Once physical 90° is confirmed:
```
offset = physical_center_us_as_degrees − 90
```
where `us_as_degrees = (us − 500) / (2500 − 500) × 180`

Set `#define <JOINT>_CENTER_OFFSET <value>` in `src/main.cpp`.

### Step 4 — All-servo smoke test

After all joints are calibrated, run `SERVO_ALL_SWEEP` (env `esp32-test10`) to confirm all servos move together with smooth transitions.

---

## Smooth Sweep Pattern

For smooth motion, step in small increments (5 µs) with a short delay (20 ms) between steps instead of jumping directly to the target:

```cpp
for (int us = LOW; us <= HIGH; us += 5) {
  servo.writeMicroseconds(us);
  delay(20);
}
for (int us = HIGH; us >= LOW; us -= 5) {
  servo.writeMicroseconds(us);
  delay(20);
}
```

5 µs × 20 ms → ~800 ms per direction for a 200 µs range. Adjust step size or delay to taste.

---

## Troubleshooting

### Servo does not move at all
- Check `ESP32PWM::allocateTimer(0..3)` is called **before** any `attach()`.
- Without timer allocation, `attach()` silently fails and no PWM is generated.
- Each servo needs one timer; 4 timers available on ESP32 (0–3).

### Servo always drives to one mechanical end regardless of pulse width
- Arm moves freely when unpowered → goes to end when powered on.
- **Root cause**: defective internal feedback potentiometer.
- The servo's control IC always reads "wrong position" and drives the motor to compensate.
- **Fix**: replace the servo. The pin and signal are fine.

### Serial monitor blocks upload
- On Windows, COM15 is held open by the serial monitor.
- Close the serial monitor before every upload attempt.
- Error message: `could not open port 'COMx': Access is denied`.

### Wrong COM port
- `pio run -e <env> -t upload` uses `/dev/ttyUSB0` by default (Linux path).
- On Windows always pass: `--upload-port COM15` (or whichever COM port the ESP32 is on).

### Servo goes limp during single-joint test
- Expected — only the servo under test is attached and driven.
- Other joints have no PWM and go slack. This is normal for single-joint calibration.

### Horn re-seating
- If a horn is re-seated while powered, the servo will snap to the currently commanded position.
- Power off, position the arm manually, re-seat the horn, then power on.

---

## PlatformIO Environments (test suite)

| Env            | Define                       | Purpose                              |
|----------------|------------------------------|--------------------------------------|
| esp32-test     | SERVO_CENTER_TEST            | All 6 servos → 1500 µs              |
| esp32-test1    | SERVO_SINGLE_TEST            | Base only → 1500 µs                 |
| esp32-test2    | SERVO_BARE_TEST              | Base sweep (no allocateTimer)        |
| esp32-test3    | SERVO_SHOULDER_TEST          | Shoulder only → 1500 µs             |
| esp32-test4    | SERVO_SHOULDER_INTERACTIVE   | Shoulder continuous sweep            |
| esp32-test5    | SERVO_BASE_INTERACTIVE       | Base continuous sweep                |
| esp32-test6    | SERVO_ELBOW_INTERACTIVE      | Elbow continuous sweep               |
| esp32-test7    | SERVO_WRIST_PITCH_INTERACTIVE| Wrist pitch continuous sweep         |
| esp32-test8    | SERVO_WRIST_ROLL_INTERACTIVE | Wrist roll continuous sweep          |
| esp32-test9    | SERVO_GRIPPER_INTERACTIVE    | Gripper continuous sweep             |
| esp32-test10   | SERVO_ALL_SWEEP              | All 6 servos smooth sweep 500↔700 µs|
