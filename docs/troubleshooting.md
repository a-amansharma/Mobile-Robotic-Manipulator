# Troubleshooting

# Mobile Robotic Manipulator

## ESP-NOW Not Connecting

- Verify both ESP32 boards are powered.
- Check that the correct MAC address is used.
- Restart both transmitter and receiver.

---

## Servo Not Moving

- Check the servo wiring.
- Verify the power supply.
- Ensure the correct GPIO/PCA9685 channel is assigned.
- Do not power MG90S servos directly from a 2S or 3S battery.

---

## Rover Not Moving

- Check the battery voltage.
- Verify IBT-2 motor driver connections.
- Ensure all grounds are connected together.

---

## OLED Not Displaying

- Verify SDA and SCL connections.
- Check the I2C address.
- Confirm the display is receiving power.

---

## MPU6050 Not Working

- Verify SDA and SCL wiring.
- Check the I2C address.
- Ensure the module is powered correctly.

---

## Ultrasonic Sensor Not Detecting

- Verify Trig and Echo connections.
- Ensure the sensor has a clear line of sight.
- Check the voltage divider on the Echo pin if required.

---

## Servo Jitter

- Ensure all grounds are common.
- Use a dedicated regulated power supply for servos.
- Avoid powering servos from the ESP32.

---

## Common Mistakes

- Incorrect battery polarity.
- Loose jumper wires.
- Missing common ground.
- Wrong GPIO assignments.
- Incorrect servo power supply.
- Not checking wiring before powering the system.

Most issues can be resolved by verifying the wiring, power supply, and GPIO connections before troubleshooting the firmware.