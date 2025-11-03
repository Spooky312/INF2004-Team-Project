# Quick Reference - robot_config.h

## 🎯 Quick Pin Reference

| Component | GPIO | Variable Name |
|-----------|------|---------------|
| **Encoders** |
| Left Encoder | GP3 | `ENCODER_LEFT_PIN` |
| Right Encoder | GP6 | `ENCODER_RIGHT_PIN` |
| **Motors** |
| Left Motor PWM | GP10 | `MOTOR_L_PWM` |
| Left Motor DIR | GP11 | `MOTOR_L_DIR` |
| Right Motor PWM | GP8 | `MOTOR_R_PWM` |
| Right Motor DIR | GP9 | `MOTOR_R_DIR` |
| **IMU (LSM303DLHC)** |
| I2C SDA | GP16 | `IMU_I2C_SDA` |
| I2C SCL | GP17 | `IMU_I2C_SCL` |
| **Line Sensor (Left IR)** |
| Digital Out | GP26 | `LINE_IR_DO_PIN` |
| Analog Out | GP27 | `LINE_IR_AO_PIN` |
| **Barcode (Right IR)** |
| Digital Out | GP7 | `BARCODE_IR_DO_PIN` |
| **System** |
| Emergency Stop | GP20 | `EMERGENCY_STOP_PIN` |
| **Ultrasonic** (future) |
| Trigger | GP5 | `ULTRASONIC_TRIG` |
| Echo | GP4 | `ULTRASONIC_ECHO` |

## ⚙️ Common Tuning Parameters

### Speed Control
```c
#define BASE_SPEED          0.5f    // Line following speed (0.0-1.0)
#define TURN_SPEED          0.20f   // Turn speed (0.0-1.0)
```

### PID Tuning - Speed
```c
#define PID_KP_SPEED        0.45f   // Proportional
#define PID_KI_SPEED        0.05f   // Integral
#define PID_KD_SPEED        0.02f   // Derivative
```

### PID Tuning - Heading
```c
#define PID_KP_HEADING      0.8f    // Proportional (↑ = faster response)
#define PID_KI_HEADING      0.05f   // Integral (eliminate drift)
#define PID_KD_HEADING      0.25f   // Derivative (damping)
```

### Turn Parameters
```c
#define TURN_ANGLE_DEG      90.0f   // Turn angle
#define TURN_TOLERANCE_DEG  5.0f    // Acceptable error
#define TURN_TIMEOUT_MS     5000    // Max turn time
```

### Line Following Behavior
```c
#define SEARCH_INTENSITY    0.4f    // Search turn strength
#define RECOVERY_CYCLES     15      // Stabilization cycles
```

### Line Sensor Calibration
```c
#define LINE_THRESHOLD      2000    // ADC threshold
                                    // (white: 0-100, black: 4000-4095)
```

## 📊 Encoder Calibration
```c
#define TICKS_PER_REV       360.0f  // Encoder ticks/revolution
#define WHEEL_CIRCUM_M      0.21f   // Wheel circumference (meters)
```

## 🔧 Quick Troubleshooting

### Robot Not Following Line
1. Check `LINE_THRESHOLD` - calibrate with `line_sensor_read_raw()`
2. Adjust `PID_KP_HEADING` - increase for sharper corrections
3. Verify `LINE_IR_AO_PIN` (GP27) wiring

### Robot Oscillating on Line
1. Reduce `PID_KP_HEADING` - lower for smoother control
2. Increase `PID_KD_HEADING` - better damping
3. Lower `BASE_SPEED` - slower = more stable

### Turns Overshooting
1. Reduce `TURN_SPEED` - slower turns
2. Reduce `TURN_TOLERANCE_DEG` - tighter precision
3. Check IMU calibration

### Barcode Not Detecting
1. Verify `BARCODE_IR_DO_PIN` (GP7) wiring
2. Check `BARCODE_BAR_IS_LOW` setting (0 or 1)
3. Adjust `BARCODE_SAMPLE_MS` timing

## 📝 How to Make Changes

1. **Open** `robot_config.h`
2. **Find** the parameter you want to change
3. **Edit** the value
4. **Save** the file
5. **Rebuild** your project

All drivers automatically use the new values!

## 🚀 Build Commands

```powershell
# From project root
cd build
cmake --build .
```

## 📍 File Location
```
c:\Projects\inf2004-team\robot_config.h
```
