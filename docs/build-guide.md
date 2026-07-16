# Safety Notes

Before powering the system, carefully verify every wiring connection. Double-check the polarity, voltage levels, and common ground connections to avoid damaging components.

## Servo Power Recommendations

### MG996R Servo Motors (High Torque)

- Recommended Supply: 2S Li-ion Battery (7.4V) or regulated 6V supply
- These servos can safely handle higher current requirements.

### MG90S Metal Gear Servo Motors

- Recommended Supply: 1S Li-ion Battery (3.7V) or a regulated 5V supply using a buck converter.
- **Do NOT power MG90S servos directly from a 2S or 3S Li-ion battery.**

> During development, two MG90S servos were permanently damaged because they were accidentally powered directly from a 2S Li-ion battery. Always use a regulated low-voltage supply for these servos.

## Common Ground

All power supplies and electronic modules must share a common ground.

Ensure the following grounds are connected together:

- ESP32 Receiver
- ESP32 Transmitter
- STM32
- PCA9685 Servo Driver
- IBT-2 Motor Drivers
- Buck Converter
- Voltage Regulator
- Sensors
- Battery Ground

Failure to connect a common ground may result in unstable communication, servo jitter, or incorrect sensor readings.

## Before Applying Power

Always perform the following checks:

- Verify all wiring connections.
- Check battery polarity.
- Measure regulator output voltages using a multimeter.
- Ensure there are no short circuits.
- Confirm servo connectors are inserted correctly.
- Verify motor driver wiring.
- Confirm the ESP32 is powered with the correct voltage.

## Initial Power-Up

For the first power-on:

1. Power the electronics without connecting the robotic arm servos.
2. Verify the ESP32 boots correctly.
3. Check ESP-NOW communication.
4. Test the OLED display.
5. Verify sensor readings.
6. Connect the servos only after all voltages have been confirmed.

## Mechanical Safety

- Never force a servo beyond its mechanical limits.
- Ensure servo horns are centered before final assembly.
- Keep hands clear while testing the robotic arm.
- Disconnect the battery before modifying any wiring.