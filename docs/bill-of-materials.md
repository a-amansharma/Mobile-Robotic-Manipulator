# Bill of Materials (BOM)

# Mobile Robotic Manipulator

This document lists all the hardware components used in the Mobile Robotic Manipulator project.

---

## 1. Microcontrollers

| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP32 DevKit V1 | 2 | Transmitter and Receiver |
| STM32F103C8T6 (Blue Pill) | 1 | Reads master arm potentiometers |

---

## 2. Motor Drivers

| Component | Quantity | Purpose |
|-----------|----------|---------|
| IBT-2 BTS7960 Motor Driver | 2 | Controls left and right drive motors |

---

## 3. Drive System

| Component | Quantity | Purpose |
|-----------|----------|---------|
| DC Geared Motors | 6 | Six-wheel rover drive |
| Wheels | 6 | Rover movement |
| Metal Chassis | 1 | Robot base |

---

## 4. Robotic Arm

| Component | Quantity | Purpose |
|-----------|----------|---------|
| MG996R Servo | 4 | Base, Shoulder, Elbow and Wrist Pitch |
| MG90S Servo | 2 | Wrist Roll and Gripper |
| 6-DOF Robotic Arm Frame | 1 | Manipulator mechanism |

---

## 5. Servo Driver

| Component | Quantity | Purpose |
|-----------|----------|---------|
| PCA9685 16-Channel Servo Driver | 1 | Controls robotic arm servos |

---

## 6. Master Controller

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Linear Potentiometer | 6 | Joint position sensing |
| Joystick Module | 1 | Rover movement |
| Rotary Potentiometer | 1 | Speed control |
| Push Button | 2 | Headlight and Buzzer control |

---

## 7. Sensors

| Component | Quantity | Purpose |
|-----------|----------|---------|
| MPU6050 IMU | 1 | Tilt detection and safety monitoring |
| HC-SR04 Ultrasonic Sensor | 1 | Rear obstacle detection |

---

## 8. Display

| Component | Quantity | Purpose |
|-----------|----------|---------|
| 0.96" OLED SSD1306 (I2C) | 1 | System status display |

---

## 9. Indicators

| Component | Quantity | Purpose |
|-----------|----------|---------|
| LED Headlight | 1 | Robot illumination |
| Active Buzzer | 1 | Audio feedback |

---

## 10. Power System

| Component | Quantity | Purpose |
|-----------|----------|---------|
| 7.4V 2S Li-ion Battery Pack | 1 | Main power source |
| LM2596 Buck Converter | 1 | 6V supply for servos |
| 5V Voltage Regulator | 1 | Powers MG90S servos and accessories |
| Power Switch | 1 | Main ON/OFF switch |

---

## 11. Communication

| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP-NOW Wireless Protocol | 1 | Communication between transmitter and receiver |
| UART Communication | 1 | STM32 to ESP32 data transfer |

---

## 12. Wiring & Accessories

- Jumper wires
- Silicone wires
- Servo extension cables
- Heat shrink tubing
- Zip ties
- Connectors
- Mounting screws
- Servo horns
- Standoffs
- Nuts and bolts

---

## 13. Development Tools

### Software

- Arduino IDE
- VS Code
- Git
- GitHub
- Processing IDE (3D Visualizer)

### Libraries

- WiFi
- ESP-NOW
- Wire
- Adafruit PWM Servo Driver
- Adafruit SSD1306
- Adafruit GFX

---

## Estimated Component Count

| Category | Quantity |
|----------|---------:|
| Microcontrollers | 3 |
| Motor Drivers | 2 |
| DC Motors | 6 |
| Servo Motors | 6 |
| Sensors | 2 |
| Displays | 1 |
| Buttons | 2 |
| Potentiometers | 7 |
| Driver Boards | 1 |
| Batteries | 1 |

---

## Notes

- The robotic arm uses a combination of MG996R and MG90S servos.
- ESP-NOW provides low-latency wireless communication between the transmitter and receiver.
- The STM32 reads the six master-arm potentiometers and sends joint data to the transmitter via UART.
- The receiver controls both the mobile rover and the robotic arm simultaneously.