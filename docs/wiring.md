# Wiring

# Mobile Robotic Manipulator

This document contains the wiring details for both the transmitter and receiver units.

---

# Receiver (ESP32)

## IBT-2 Motor Drivers

### Left Driver

| IBT-2 | ESP32 |
|--------|-------|
| RPWM | GPIO25 |
| LPWM | GPIO26 |
| R_EN | 5V |
| L_EN | 5V |
| VCC | 5V |
| GND | GND |

### Right Driver

| IBT-2 | ESP32 |
|--------|-------|
| RPWM | GPIO27 |
| LPWM | GPIO14 |
| R_EN | 5V |
| L_EN | 5V |
| VCC | 5V |
| GND | GND |

---

## PCA9685 Servo Driver

| PCA9685 | ESP32 |
|----------|-------|
| SDA | GPIO21 |
| SCL | GPIO22 |
| VCC | 3.3V |
| GND | GND |
| V+ | External 6V Servo Supply |

### Servo Channels

| Joint | Channel |
|--------|---------|
| Gripper | GPIO32 (Direct) |
| Wrist Roll | GPIO33 (Direct) |
| Wrist Pitch | CH4 |
| Elbow | CH6 |
| Shoulder | CH8 |
| Base Rotation | CH10 |

---

## MPU6050

| MPU6050 | ESP32 |
|----------|-------|
| SDA | GPIO18 |
| SCL | GPIO19 |
| VCC | 3.3V |
| GND | GND |

## Ultrasonic Sensor (HC-SR04)

| HC-SR04 | ESP32 |
|----------|-------|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO4 |
| ECHO | GPIO5 (via voltage divider) |

### Echo Voltage Divider

Since the HC-SR04 Echo pin outputs **5V** and the ESP32 GPIOs are **3.3V tolerant**, a voltage divider is used.

```
HC-SR04 Echo
      |
    1kΩ
      |
      +------> ESP32 GPIO5
      |
    2kΩ
      |
     GND
```

> **Note:** Always use a voltage divider (or a logic level shifter) between the HC-SR04 Echo pin and the ESP32 to protect the GPIO from 5V signals.

## Other Components

| Component | ESP32 |
|-----------|-------|
| Buzzer | GPIO13 |
| Headlight LED | GPIO23 |
| Onboard LED | GPIO2 |

---

# Transmitter (ESP32)

## Joystick

| Joystick | ESP32 |
|-----------|-------|
| VRX | GPIO34 |
| VRY | GPIO35 |
| VCC | 5V |
| GND | GND |

---

## Speed Potentiometer

| Potentiometer | ESP32 |
|---------------|-------|
| Signal | GPIO32 |
| VCC | 3.3V |
| GND | GND |

---

## Buttons

| Button | ESP32 |
|---------|-------|
| Headlight | GPIO25 |
| Buzzer | GPIO33 |

---

## OLED Display (SSD1306)

| OLED | ESP32 |
|------|-------|
| SDA | GPIO21 |
| SCL | GPIO22 |
| VCC | 3.3V |
| GND | GND |

---

## STM32 UART

| STM32 | ESP32 |
|--------|-------|
| PA9 (TX) | GPIO16 (RX2) |
| GND | GND |

---

# STM32 (Master Arm)

## Potentiometers

| Joint | STM32 Pin |
|--------|-----------|
| Gripper | PA0 |
| Wrist Roll | PA1 |
| Wrist Pitch | PA2 |
| Elbow | PA3 |
| Shoulder | PA4 |
| Base Rotation | PA5 |

---

# Power Connections

- 2S Li-ion Battery → IBT-2 Motor Drivers
- Buck Converter (6V) → MG996R Servos
- 1S Battery or Regulated 5V → MG90S Servos
- ESP32 and STM32 → Regulated 5V
- Connect **all grounds (GND)** together.

---

# Notes

- Verify all wiring before powering the system.
- Never power MG90S servos directly from a 2S or 3S battery.
- Use a regulated supply for all control electronics.
- Ensure common ground between all modules.