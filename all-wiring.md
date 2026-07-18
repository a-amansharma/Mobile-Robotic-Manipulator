# Wiring

Complete wiring reference for the **Mobile Robotic Manipulator**.

The system consists of three major control units:

| 🟦 Receiver Unit | 🟩 Transmitter Unit |
|------------------|--------------------|
| ESP32 mounted on the rover. Controls motors, robotic arm, sensors and actuators. | ESP32 handheld controller. Reads joystick, buttons, OLED display and receives robotic arm commands from the STM32. |

---

# Hardware Architecture

```
             STM32 Master Arm
                    │ UART
                    ▼
          Transmitter ESP32
                    │
              ESP-NOW Wireless
                    │
                    ▼
            Receiver ESP32
       ┌──────────┼──────────┐
       │          │          │
    Motors     Servos     Sensors
```

---

# 🟦 Receiver Unit (ESP32)

Controls the complete rover, robotic arm and onboard sensors.

---

## Motor Drivers (IBT-2)

### Left Driver

| IBT-2 Pin | ESP32 |
|-----------|-------|
| RPWM | GPIO25 |
| LPWM | GPIO26 |
| R_EN | 5V |
| L_EN | 5V |
| VCC | 5V |
| GND | GND |

### Right Driver

| IBT-2 Pin | ESP32 |
|-----------|-------|
| RPWM | GPIO27 |
| LPWM | GPIO14 |
| R_EN | 5V |
| L_EN | 5V |
| VCC | 5V |
| GND | GND |

---

## Servo Driver (PCA9685)

### I2C Connection

| PCA9685 | ESP32 |
|----------|-------|
| SDA | GPIO21 |
| SCL | GPIO22 |
| VCC | 3.3V |
| GND | GND |
| V+ | External 6V Servo Supply |

### Servo Outputs

| Joint | Connection |
|--------|------------|
| Gripper | GPIO32 (Direct) |
| Wrist Roll | GPIO33 (Direct) |
| Wrist Pitch | Channel 4 |
| Elbow | Channel 6 |
| Shoulder | Channel 8 |
| Base Rotation | Channel 10 |

---

## MPU6050 IMU

| MPU6050 | ESP32 |
|----------|-------|
| SDA | GPIO18 |
| SCL | GPIO19 |
| VCC | 3.3V |
| GND | GND |

---

## Ultrasonic Sensor (HC-SR04)

| HC-SR04 | ESP32 |
|----------|-------|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO4 |
| ECHO | GPIO5 *(through voltage divider)* |

### Echo Voltage Divider

```
HC-SR04 Echo
      │
     1kΩ
      │
      ├────────► GPIO5
      │
     2kΩ
      │
     GND
```

> **Important:** Never connect the Echo pin directly to the ESP32. Always use a voltage divider or logic level shifter.

---

## Other Components

| Component | ESP32 |
|-----------|-------|
| Buzzer | GPIO13 |
| Headlight LED | GPIO23 |
| Onboard LED | GPIO2 |

---

# 🟩 Transmitter Unit (ESP32)

Reads user inputs and sends commands wirelessly to the receiver.

---

## Analog Inputs

### Joystick

| Pin | ESP32 |
|------|-------|
| VRX | GPIO34 |
| VRY | GPIO35 |
| VCC | 5V |
| GND | GND |

---

### Speed Potentiometer

| Pin | ESP32 |
|------|-------|
| Signal | GPIO32 |
| VCC | 3.3V |
| GND | GND |

---

## Push Buttons

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

## UART Communication

| STM32 | ESP32 |
|--------|-------|
| PA9 (TX) | GPIO16 (RX2) |
| GND | GND |

---

# 🟨 STM32 Master Arm

Reads the potentiometers attached to the master robotic arm.

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

# 🔋 Power Distribution

| Device | Power Source |
|---------|--------------|
| IBT-2 Motor Drivers | 2S Li-ion Battery |
| MG996R Servos | Buck Converter (6V) |
| MG90S Servos | Regulated 5V / 1S Battery |
| ESP32 Boards | Regulated 5V |
| STM32 | Regulated 5V |

> **All modules must share a common ground (GND).**

---

# ⚠️ Important Notes

- Verify all wiring before powering the system.
- Never power MG90S servos directly from a 2S or 3S battery.
- Use a dedicated regulated supply for all control electronics.
- Keep motor power and logic power properly isolated where possible.
- Ensure a common ground between the transmitter, receiver and STM32.
- Always use a voltage divider on the HC-SR04 Echo pin.

---

## Wiring Summary

| Unit | Purpose |
|------|---------|
| 🟦 Receiver ESP32 | Controls motors, robotic arm, sensors and actuators |
| 🟩 Transmitter ESP32 | Reads joystick, buttons and sends commands via ESP-NOW |
| 🟨 STM32 | Reads the master robotic arm potentiometers |