# GPIO Pin Mapping - Robo Pico Board

This document lists all GPIO pin assignments for the INF2004 Robotic Car project using the Robo Pico board (with Pico W).

## Current Pin Assignments

### Encoders
- **Left Encoder**: GP3
- **Right Encoder**: GP6

### Motors (Current Configuration)
- **Left Motor PWM**: GP8
- **Left Motor DIR**: GP9
- **Right Motor PWM**: GP10
- **Right Motor DIR**: GP11

### IMU (LSM303DLHC)
- **SDA**: GP16
- **SCL**: GP17
- **I2C**: i2c0

### Debug LEDs
- **WiFi Status LED**: GP13
- **IMU Status LED**: GP19
- **Encoder Status LED**: GP24
- **Heartbeat LED**: GP25

### Change Direction Button
- Check `drivers/chg_direction/chg_direction.h` for current pin

## Future Pin Assignments (Not Yet Implemented)

### Ultrasonic Sensor
- **Trigger**: GP5
- **Echo**: GP4

### Left IR Sensor (Line Detection/Barcode)
- **Digital Out (DO)**: GP26
- **Analog Out (AO)**: GP27

### Right IR Sensor (Line Detection/Barcode)
- **Digital Out (DO)**: GP7
- **Analog Out (AO)**: GP28

## Notes
- The Robo Pico board has a Pico W mounted on it
- All pin assignments are compatible with the Robo Pico board layout
- When implementing new drivers, use these pin definitions
- Update this document when adding or changing pin assignments
