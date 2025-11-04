# Troubleshooting Guide

## Issues Fixed in This Build

### 1. Encoder Not Detecting Ticks

**Problem**: Both encoders showing 0 ticks, no interrupts firing.

**Fixes Applied**:
- Changed interrupt trigger from `GPIO_IRQ_EDGE_FALL` only to `GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL`
- This detects both rising and falling edges, doubling sensitivity
- Some encoders output different signal types (active high vs active low)

**Testing**:
1. Flash the new firmware
2. Manually spin each wheel slowly
3. Watch the telemetry for `ticks_l` and `ticks_r` to increment
4. If still 0, check physical connections:
   - Left encoder → GP3
   - Right encoder → GP6
   - Ground and VCC properly connected

**Hardware Checks**:
- Verify encoder power (usually 3.3V or 5V)
- Check for loose connections
- Ensure encoder outputs are connected to correct GPIO pins
- Try measuring encoder output with multimeter/oscilloscope

---

### 2. Heading Fluctuating Wildly

**Problem**: Raw heading jumping between 20°, 43°, 132°, 136° - variations of 100°+ between readings.

**Root Causes**:
1. **Magnetic interference** from motors and power lines
2. **No magnetometer calibration** - hard-iron and soft-iron offsets not set
3. **Poor filtering** - simple EMA doesn't handle outliers well
4. **Wraparound issues** - 360°/0° boundary causing filter problems

**Fixes Applied**:

#### A. Improved EMA Filter (in `drivers/imu/imu.c`)
- Added wraparound handling (360°/0° boundary)
- Added outlier rejection (>90° changes rejected as interference)
- Increased EMA alpha from 0.15 to 0.30 for faster response

#### B. Filter Logic
```c
// Handles 360°/0° wraparound properly
float diff = heading_deg - ema_heading_deg;
if (diff > 180.0f) diff -= 360.0f;
else if (diff < -180.0f) diff += 360.0f;

// Rejects sudden jumps > 90° (likely motor interference)
if (fabsf(diff) > 90.0f) {
    heading_deg = ema_heading_deg;  // Use previous value
}
```

**Expected Improvement**:
- Filtered heading should now be smooth (variations < 5° normally)
- Raw heading may still jump but won't affect filtered value
- System rejects interference spikes automatically

---

## Magnetometer Calibration (RECOMMENDED)

Your magnetometer is currently **uncalibrated**. This causes heading drift and inaccuracy.

### Quick Calibration Procedure:

1. **Collect Calibration Data**:
   ```
   - Power on robot
   - Slowly rotate 360° in place (take 30-60 seconds)
   - Record min/max values for mx, my, mz from debug output
   ```

2. **Calculate Offsets** (hard-iron correction):
   ```
   X_offset = (X_max + X_min) / 2
   Y_offset = (Y_max + Y_min) / 2
   Z_offset = (Z_max + Z_min) / 2
   ```

3. **Update `robot_config.h`**:
   ```c
   #define MAG_X_OFFSET    <calculated_value>
   #define MAG_Y_OFFSET    <calculated_value>
   #define MAG_Z_OFFSET    <calculated_value>
   ```

4. **Apply in Code** (modify `drivers/imu/imu.c` in `imu_read_mag()`):
   ```c
   if (mx_g) *mx_g = (x * sx) - MAG_X_OFFSET;
   if (my_g) *my_g = (y * sy) - MAG_Y_OFFSET;
   if (mz_g) *mz_g = (z * sz) - MAG_Z_OFFSET;
   ```

### Environmental Factors Affecting Heading:

⚠️ **Magnetic Interference Sources**:
- DC motors (especially when running)
- Power wires with high current
- Steel/iron structures nearby
- Batteries (especially LiPo)
- Other electronic devices

💡 **Best Practices**:
- Keep magnetometer away from motors (physical distance)
- Use twisted pair wiring to reduce magnetic fields
- Shield motor drivers if possible
- Calibrate in the same environment where robot will operate
- Re-calibrate if you change battery or motor positions

---

## Current Configuration Summary

### GPIO Pin Assignments:
| Component | Function | GPIO |
|-----------|----------|------|
| Left Motor | PWM | GP10 |
| Left Motor | DIR | GP11 |
| Right Motor | PWM | GP8 |
| Right Motor | DIR | GP9 |
| Left Encoder | Signal | GP3 |
| Right Encoder | Signal | GP6 |
| IMU | SDA | GP16 |
| IMU | SCL | GP17 |

### Encoder Settings:
- **Trigger**: Both RISE and FALL edges ✓
- **Pull-up**: Enabled
- **Ticks per revolution**: 360
- **Wheel circumference**: 0.21m

### IMU Settings:
- **I2C Speed**: 400kHz
- **EMA Alpha**: 0.30 (more responsive)
- **Outlier threshold**: 90° (rejects spikes)
- **Sample rate**: 75 Hz (magnetometer)

---

## Testing Checklist

After flashing new firmware:

- [ ] **Encoders**: Manually spin wheels, verify ticks increment
- [ ] **Heading stability**: Should vary < 5° when stationary
- [ ] **Motor control**: Verify correct direction and speed
- [ ] **IMU**: Check if heading changes when rotating robot
- [ ] **Telemetry**: Confirm MQTT messages publishing

---

## Next Steps if Issues Persist

### If encoders still show 0 ticks:
1. Check encoder type (hall effect vs optical)
2. Verify encoder output voltage levels (3.3V compatible?)
3. Test with oscilloscope to see if signals are present
4. Try different pull resistor configuration (pull-down instead of pull-up)

### If heading still fluctuates:
1. Perform full magnetometer calibration (see above)
2. Increase outlier threshold to 60° for very noisy environments
3. Move IMU away from motors and power lines
4. Add ferrite beads to motor power wires
5. Consider using a 9-axis IMU with gyroscope for sensor fusion

### For additional help:
- Check serial output for `[ENC]` and `[IMU]` initialization messages
- Monitor raw magnetometer values for interference patterns
- Use `DEBUG_IMU_VERBOSE 1` in robot_config.h for detailed logging
