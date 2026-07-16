# Power System

# Mobile Robotic Manipulator

The robot is powered using a **2S (7.4V) Li-ion battery**. Separate regulated power supplies are used for the drive motors, servos, and control electronics to ensure stable operation.

## Power Distribution

| Component | Power Source |
|----------|--------------|
| DC Drive Motors | 2S Li-ion Battery (via IBT-2) |
| MG996R Servos | Regulated 6V (Buck Converter) |
| MG90S Servos | 1S Li-ion Battery or Regulated 5V |
| ESP32 & STM32 | Regulated 5V / USB |
| Sensors & OLED | Regulated 3.3V/5V |

## Important Notes

- **Do not power MG90S servos directly from a 2S or 3S battery.**
- During development, two MG90S servos were damaged due to incorrect power supply and had to be replaced.
- Always use a buck converter or a dedicated 1S battery for MG90S servos.
- Connect the **ground (GND)** of all batteries, regulators, motor drivers, sensors, and controllers together.
- Verify all wiring and output voltages with a multimeter before powering the system.
- Disconnect the battery before making any wiring changes.

Following these guidelines ensures reliable operation and helps prevent damage to electronic components.