# Configuration Migration Summary

## Overview
All GPIO pins and configurable parameters have been extracted from individual driver files and consolidated into a single centralized configuration file: `robot_config.h`

This makes it much easier to update pin assignments and tune parameters without having to hunt through multiple files.

## Changes Made

### ✅ New File Created
- **`robot_config.h`** - Centralized configuration header containing:
  - All GPIO pin assignments
  - Encoder parameters (ticks per rev, wheel circumference)
  - Line sensor settings (threshold, ADC channels)
  - Barcode scanner configuration
  - IMU I2C settings
  - PID controller gains (speed & heading)
  - Robot behavior parameters (speeds, turn angles)
  - FreeRTOS task priorities and stack sizes
  - Timing constants

### 📝 Files Modified

#### Driver Headers
1. **`drivers/encoder/encoder.h`**
   - Removed: `ENCODER_LEFT_PIN`, `ENCODER_RIGHT_PIN` definitions
   - Added: `#include "robot_config.h"`

2. **`drivers/imu/imu.h`**
   - Removed: `IMU_I2C`, `IMU_I2C_SDA`, `IMU_I2C_SCL`, `IMU_I2C_BAUD_HZ`
   - Removed: `IMU_DECLINATION_RAD`, `IMU_HEADING_EMA_ALPHA`
   - Added: `#include "robot_config.h"`

3. **`drivers/line_sensor/line_sensor.h`**
   - Removed: `LINE_ADC_PIN`, `LINE_ADC_INPUT`, `LINE_THRESHOLD`, `LINE_SAMPLE_COUNT`
   - Added: `#include "robot_config.h"`

4. **`drivers/barcode/barcode.h`**
   - Removed: All `BARCODE_*` configuration defines
   - Added: `#include "robot_config.h"`

#### Driver Implementation Files
5. **`drivers/encoder/encoder.c`**
   - Removed: `TICKS_PER_REV`, `WHEEL_CIRCUM_M`, `RPM_TIMEOUT_US`
   - Uses values from `robot_config.h`

6. **`drivers/motor/motor.c`**
   - Removed: `MOTOR_L_PWM`, `MOTOR_L_DIR`, `MOTOR_R_PWM`, `MOTOR_R_DIR`
   - Added: `#include "robot_config.h"`

7. **`drivers/imu/imu.c`**
   - Updated all `IMU_I2C` references to `IMU_I2C_INST`
   - Uses config values for pins and settings

8. **`drivers/line_sensor/line_sensor.c`**
   - Updated `LINE_ADC_PIN` to `LINE_IR_AO_PIN`
   - Uses config values for threshold and ADC input

9. **`drivers/barcode/barcode.c`**
   - Updated `BARCODE_IR_PIN` to `BARCODE_IR_DO_PIN`
   - Uses config values for all timing parameters

10. **`drivers/pid/pid.c`**
    - Removed all local PID gain definitions
    - Updated to use `PID_KP_SPEED`, `PID_KI_SPEED`, etc. from config
    - Updated anti-windup limits to use config constants
    - Updated `pid_get_heading_gains()` to return config values

#### Main Application
11. **`main.c`**
    - Added: `#include "robot_config.h"`
    - Removed: All local `#define` configuration constants
    - Updated task stack sizes to use config values
    - Updated timing constants to use config values
    - Updated behavior parameters to use config values

## Configuration Categories in robot_config.h

### 1. GPIO Pin Assignments
- Motor control pins (PWM & direction)
- Encoder pins (left & right)
- IMU I2C pins (SDA & SCL)
- Ultrasonic sensor pins (documented for future use)
- IR sensor pins (line following & barcode)
- Emergency stop button

### 2. Hardware Parameters
- Encoder: ticks per revolution, wheel circumference
- Line sensor: ADC threshold, sample count
- Barcode: sampling rates, verification delays
- IMU: I2C speed, declination, filter alpha

### 3. Control Parameters
- PID gains for speed control
- PID gains for heading control
- Anti-windup limits

### 4. Robot Behavior
- Movement speeds (base, turn)
- Turn parameters (angle, tolerance, timeout)
- Line following behavior (search intensity, recovery)
- Barcode detection timing

### 5. FreeRTOS Configuration
- Task priorities
- Stack sizes
- Update rates

## Benefits

✅ **Single Source of Truth**: All configuration in one place  
✅ **Easy Pin Remapping**: Change GPIO pins in one file  
✅ **Simplified Tuning**: Adjust PID gains centrally  
✅ **Better Documentation**: All settings clearly organized  
✅ **Reduced Errors**: No duplicate definitions across files  
✅ **Version Control Friendly**: Configuration changes tracked in one file  

## Usage

To modify any configuration:
1. Open `robot_config.h`
2. Locate the relevant section (GPIO, PID, Timing, etc.)
3. Update the value
4. Rebuild the project

All drivers will automatically use the updated values.

## Example: Changing a Pin Assignment

**Before** (Required editing multiple files):
```c
// In encoder.h
#define ENCODER_LEFT_PIN   3

// In main.c
gpio_set_irq_enabled(3, ...);  // Had to update here too!
```

**After** (Edit once in robot_config.h):
```c
// In robot_config.h
#define ENCODER_LEFT_PIN    3   // Change this one value

// All files automatically updated!
```

## Example: Tuning PID

**Before**:
```c
// Had to edit pid.c
static float kp_heading = 0.8f;
```

**After**:
```c
// Edit in robot_config.h
#define PID_KP_HEADING      0.8f
```

## Next Steps

Consider adding to `robot_config.h`:
- Network configuration (Wi-Fi credentials, MQTT settings)
- Ultrasonic sensor parameters (when implemented)
- Additional calibration constants
- Debug/logging levels
