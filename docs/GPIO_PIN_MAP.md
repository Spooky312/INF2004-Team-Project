# GPIO Pin Mapping - Robo Pico Board

## Updated Pin Assignments (as of latest configuration)

### Encoders
- **Left Encoder**: GP3 (ENCODER_LEFT_PIN)
- **Right Encoder**: GP6 (ENCODER_RIGHT_PIN)

### IMU (LSM303DLHC)
- **SDA**: GP16 (IMU_I2C_SDA)
- **SCL**: GP17 (IMU_I2C_SCL)
- **I2C Bus**: i2c0 @ 400kHz

### Ultrasonic Sensor (HC-SR04)
- **Trigger**: GP5 (not yet implemented in code)
- **Echo**: GP4 (not yet implemented in code)

### IR Sensors (Line Detection & Barcode)
#### Left IR Sensor
- **Digital Output (DO)**: GP26 (LINE_ADC_PIN for line following)
- **Analog Output (AO)**: GP27 (LINE_ADC_INPUT = ADC1)

#### Right IR Sensor
- **Digital Output (DO)**: GP7 (BARCODE_IR_PIN for barcode scanning)
- **Analog Output (AO)**: GP28 (not currently used in code)

### Motors (already configured)
#### Motor A (Left)
- **PWM**: GP10 (MOTOR_L_PWM)
- **Direction**: GP11 (MOTOR_L_DIR)

#### Motor B (Right)
- **PWM**: GP8 (MOTOR_R_PWM)
- **Direction**: GP9 (MOTOR_R_DIR)

### System Controls
- **Emergency Stop Button**: GP20 (EMERGENCY_STOP_PIN)

## Pin Summary Table

| Function | GPIO Pin | Type | Notes |
|----------|----------|------|-------|
| Left Encoder | GP3 | Digital Input | Interrupt-driven |
| Right Encoder | GP6 | Digital Input | Interrupt-driven |
| IMU SDA | GP16 | I2C Data | i2c0 |
| IMU SCL | GP17 | I2C Clock | i2c0 |
| Ultrasonic Trigger | GP5 | Digital Output | Not yet implemented |
| Ultrasonic Echo | GP4 | Digital Input | Not yet implemented |
| Left IR DO | GP26 | Digital Input | Line sensor |
| Left IR AO | GP27 | Analog Input | ADC1 for line detection |
| Right IR DO | GP7 | Digital Input | Barcode scanner |
| Right IR AO | GP28 | Analog Input | Available (not used) |
| Motor A PWM | GP10 | PWM Output | Left motor speed |
| Motor A DIR | GP11 | Digital Output | Left motor direction |
| Motor B PWM | GP8 | PWM Output | Right motor speed |
| Motor B DIR | GP9 | Digital Output | Right motor direction |
| Emergency Stop | GP20 | Digital Input | Pull-up, falling edge |

## Files Modified
- `drivers/encoder/encoder.h` - Updated ENCODER_RIGHT_PIN from 26 to 6
- `drivers/imu/imu.h` - Updated IMU_I2C_SDA from 4 to 16, IMU_I2C_SCL from 5 to 17
- `drivers/barcode/barcode.h` - Updated BARCODE_IR_PIN from 28 to 7
- `main.c` - Updated GPIO interrupt routing for new encoder pins

## Notes
- Line sensor uses ADC (analog) input on GP27
- Barcode scanner uses digital input on GP7
- All encoder pins configured with rising/falling edge interrupts
- Emergency stop uses falling edge interrupt with pull-up resistor
- Ultrasonic sensor pins are defined in this document but not yet implemented in the driver code
