---
name: send-command
description: Send a serial command to the robot arm over /dev/ttyUSB0 (115200 baud) and print the response. Pass the command as the argument, e.g. /send-command HOME or /send-command BASE 95
disable-model-invocation: true
allowed-tools: Bash
---

Send `$ARGUMENTS` to the robot arm serial port and print the response.

Run this with a 10-second timeout:

```bash
python3 - <<'EOF'
import serial, time
cmd = "$ARGUMENTS"
port = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(0.1)
port.write((cmd + '\n').encode())
time.sleep(0.4)
resp = port.read_all().decode().strip()
port.close()
print(f">> {cmd}")
print(resp if resp else "(no response)")
EOF
```

If `serial` is not available, install it first:
```bash
pip3 install pyserial --quiet
```

## Available commands

| Command | Description |
|---------|-------------|
| `HOME` | Move all joints to 90° (center) |
| `STOP` | Halt all joints at current position |
| `POS`  | Report current position of all joints |
| `MOVE <base> <shoulder> <elbow> <wrist_pitch> <wrist_roll> <gripper>` | Synchronized move, all joints arrive together |
| `BASE <angle>` | Move base joint (0–180°) |
| `SHOULDER <angle>` | Move shoulder joint (15–165°) |
| `ELBOW <angle>` | Move elbow joint (0–180°) |
| `WRIST_PITCH <angle>` | Move wrist pitch joint (0–180°) |
| `WRIST_ROLL <angle>` | Move wrist roll joint (0–180°) |
| `GRIPPER <angle>` | Move gripper joint (10–170°) |
