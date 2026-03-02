---
name: rebuild-upload
description: Clean the PlatformIO build cache, rebuild the firmware from scratch, then upload to the robot over /dev/ttyUSB0.
disable-model-invocation: true
allowed-tools: Bash
---

Run these three PlatformIO commands in sequence using `~/.platformio/penv/bin/pio`.
Stop immediately and report the error if any step fails.

1. **Clean** — remove all cached build artefacts:
   ```
   ~/.platformio/penv/bin/pio run --target clean
   ```

2. **Build** — compile the firmware from scratch:
   ```
   ~/.platformio/penv/bin/pio run
   ```

3. **Upload** — flash the firmware to the robot:
   ```
   ~/.platformio/penv/bin/pio run --target upload
   ```

After each step print a short one-line status (CLEAN OK / BUILD OK / UPLOAD OK or the error).
Use a 120-second timeout for each command.
