# Line Following + Barcode Navigation System

## Overview
This system enables your robotic car to autonomously navigate a figure-8 track by:
1. **Following a black line** using an analog IR sensor
2. **Reading Code-39 barcodes** at junctions using a digital IR sensor
3. **Executing turns** based on decoded letters

## Hardware Setup

### IR Sensors
- **Line Sensor (Analog)**: GP26 (ADC0) - MH-series IR for black line detection
- **Barcode Scanner (Digital)**: GP7 - MH-series IR for barcode reading

### Barcode Turn Rules
- **Turn RIGHT**: A, C, E, G, I, K, M, O, Q, S, U, W, Y
- **Turn LEFT**: B, D, F, H, J, L, N, P, R, T, V, X, Z

## Building the Firmware

### Option 1: Build Line Following Version
```bash
cd build
cmake ..
cmake --build . --target LINE-FOLLOW-BARCODE
```
Flash: `build/LINE-FOLLOW-BARCODE.uf2`

### Option 2: Build Original PID Version
```bash
cd build
cmake ..
cmake --build . --target INF2004-TEAM-PROJECT
```
Flash: `build/INF2004-TEAM-PROJECT.uf2`

## Calibration Steps

### 1. Line Sensor Threshold
The default threshold is **280** (in `line_sensor.h`):

```c
#define LINE_THRESHOLD  280  // Adjust based on your track
```

**To calibrate:**
1. Place sensor over white surface → read raw value (e.g., 150)
2. Place sensor over black line → read raw value (e.g., 400)
3. Set threshold to midpoint: `(150 + 400) / 2 = 275`

### 2. Turn Duration
The default turn time is **1000ms** (in `main_line_follow.c`):

```c
#define TURN_DURATION_MS  1000  // Adjust for your motors
```

**To tune:**
- Too short = robot doesn't complete 90° turn
- Too long = robot overshoots turn
- Test on your track and adjust in 100ms increments

### 3. PID Line Following Gains
Default gains (in `main_line_follow.c`):

```c
#define LINE_KP  0.8f
#define LINE_KI  0.0f
#define LINE_KD  0.2f
```

**Symptoms:**
- **Oscillating/zigzagging**: Reduce `LINE_KP` to 0.5
- **Slow to correct**: Increase `LINE_KP` to 1.0
- **Drifting off line**: Add `LINE_KI` (start with 0.01)

## State Machine Behavior

```
IDLE → LINE_FOLLOW → BARCODE_DETECTED → TURN_LEFT/RIGHT → RESUME_LINE → LINE_FOLLOW
```

### State Descriptions
1. **IDLE**: Motors off, waiting to start
2. **LINE_FOLLOW**: Following black line at `BASE_SPEED` (60)
3. **BARCODE_DETECTED**: Stopped, determining turn direction
4. **TURN_LEFT/RIGHT**: Executing 90° turn
5. **RESUME_LINE**: PID reset, returning to line following

## Serial Output (Telemetry)

Connect at **115200 baud** to see:

```json
[LINE] Line following task started
[BARCODE] Barcode scanning task started
[TELEM] Telemetry task started
[LINE] Following | Raw=320 | L=60.0 R=60.0
[BARCODE] *** DETECTED: 'A' ***
[LINE] BARCODE STOP - Barcode: 'A'
[LINE] -> Turning RIGHT
[LINE] Right turn complete
[LINE] Resuming line following
[TELEM] {"state":"LINE_FOLLOW","line_raw":315,"on_line":1,"barcode":"A","rpm_l":45.2,"rpm_r":44.8,"dist":1.35,"ticks_l":2340,"ticks_r":2298}
```

## LED Indicators

- **GP24 (Encoder LED)**: ON = Line detected, OFF = Line lost
- **GP13 (Debug LED)**: Blinks during turns
- **GP25 (Onboard LED)**: Heartbeat (every 500ms)

## Troubleshooting

### Issue: Line sensor always shows "line lost"
**Solution**: Check threshold calibration. Print `line_sensor_read_raw()` values.

### Issue: Barcode not detected
**Solution**: 
- Verify GP7 wiring
- Check barcode is Code-39 format
- Ensure sensor is 2-10mm from barcode
- Robot must be moving slowly over barcode

### Issue: Robot turns wrong direction
**Solution**: Check barcode letter classification (A,C,E...=right vs B,D,F...=left)

### Issue: Robot loses line after turn
**Solution**: 
- Reduce `TURN_DURATION_MS` (turn less than 90°)
- Increase `POST_TURN_DELAY_MS` (more stabilization time)
- Check motor speeds are balanced

### Issue: Robot spins in circles
**Solution**: 
- Check motor wiring (left vs right reversed?)
- Verify encoder pins match actual hardware
- Test motors individually: `motor_set_speed(50, 0)` should turn right

## Performance Tuning

### For Faster Line Following
```c
#define BASE_SPEED  80.0f  // Increase from 60
```

### For Smoother Turns
```c
#define TURN_SPEED  40.0f  // Decrease from 50
#define TURN_DURATION_MS  1200  // Increase time
```

### For Better Line Tracking (Multiple Sensors)
If you upgrade to a sensor array (3-5 sensors):
1. Modify `line_sensor.c` to read multiple ADC channels
2. Calculate weighted error: `error = (left*(-1) + center*0 + right*(+1))`
3. Use this error in `line_pid_compute()`

## File Structure

```
main_line_follow.c       - Main program with state machine
drivers/line_sensor/     - ADC-based line detection
drivers/barcode/         - Code-39 barcode decoder
drivers/motor/           - PWM motor control (reused)
drivers/encoder/         - Tick counting (reused)
drivers/debug_led/       - LED indicators (reused)
```

## Next Steps

1. **Flash firmware**: Upload `LINE-FOLLOW-BARCODE.uf2`
2. **Calibrate sensors**: Adjust threshold in `line_sensor.h`
3. **Test on track**: Place robot on line, power on
4. **Monitor serial**: Check telemetry for debugging
5. **Tune turns**: Adjust `TURN_DURATION_MS` for accurate 90° turns
6. **Deploy**: Navigate full figure-8 track with barcode junctions!

## Integration with Original System

To switch back to PID heading control + MQTT:
- Flash `INF2004-TEAM-PROJECT.uf2` instead
- Both versions share the same drivers (motor, encoder, IMU, etc.)
- You can combine features by merging code from both `main.c` files

---
**Good luck with your autonomous navigation!** 🚗💨
