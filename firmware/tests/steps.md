# MPU6050 Orientation Test

This folder contains the files required to test the MPU6050 sensor and visualize its orientation in real time.

## Files

- `mpu-orientation-test.ino` → Upload this to the ESP32.
- `processing-code.pde` → Open and run this in Processing.

## Wiring

| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO18 |
| SCL | GPIO19 |

## Steps

1. Connect the MPU6050 to the ESP32.
2. Upload `mpu-orientation-test.ino` using the Arduino IDE.
3. Keep the sensor flat and still during startup for calibration.
4. Close the Arduino Serial Monitor.
5. Open `processing-code.pde` in Processing.
6. Select the correct serial port if required.
7. Run the Processing sketch.

The 3D model will now follow the orientation of the MPU6050 in real time.