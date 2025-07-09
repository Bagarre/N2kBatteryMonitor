# NMEA 2000 Shunt & Sensor Module

![License: Apache 2.0](https://img.shields.io/badge/license-Apache%202.0-blue.svg)

## Overview

This project implements a DIY NMEA 2000 battery shunt and sensor module using an Adafruit Feather MCU with built‑in CAN support (via MCP2515 or SAMD CAN). It reads:

- **Current & voltage** via a 50 mV shunt (e.g., INA219 or external shunt)  
- **Battery temperature** (AGM) via 2‑wire thermistor  
- **Bow thruster temperature** via 2‑wire thermistor

It also provides:

- **State‑of‑Charge (SOC)** calculation using a configurable resting-voltage map plus basic coulomb counting
- **Under‑Voltage Lock‑Out (UVLO)** relay control
- **Over‑Temperature (thruster)** relay control
- **NMEA 2000 output** of:
  - PGN 127508 Battery Status (voltage, current, SOC, time‑to‑go)
  - PGN 127489 Battery Parameters (capacity, voltage thresholds)
  - PGN 130312 Temperature (battery & thruster)

All hardware configuration and calibration parameters live in `config.h` for easy customization.

---

## Features

- **Autonomous monitoring** of battery bank and thruster conditions
- **Robust safety**: hardware relays for UVLO and over‑temperature shutdown
- **NMEA 2000‐native**: no gateway required, just drop onto the marine CAN backbone
- **Configurable**: change shunt resistance, voltage thresholds, SOC map, temperature limits, and identifiers

---

## Pinout & Wiring

| Function                      | Feather Pin      | Notes                                   |
|-------------------------------|------------------|-----------------------------------------|
| Shunt sense + (INA219 SCL)    | A0               | Connect to INA219 shunt voltage output  |
| Shunt sense – (INA219 SDA)    | A1               | Connect to INA219 ground reference     |
| Battery thermistor voltage    | A2               | 10 kΩ NTC divider to 3.3 V reference    |
| Thruster thermistor voltage   | A3               | 10 kΩ NTC divider                       |
| UVLO relay control            | D5               | Drives normally‑open relay coil         |
| Over‑temp relay control       | D6               | Drives normally‑open relay coil         |
| NMEA 2000 CAN High/Low        | CAN1 pins        | Connect to backbone with 120 Ω terminations |
| VIN (power)                   | Battery positive | 9–16 V input (powered from bow battery) |
| GND                           | Battery negative | Common ground with N2K backbone         |

> **Note:** Ensure proper TVS and reverse‑polarity protection on VIN, and purge/shield CAN lines for reliability.

---

## Configuration (`config.h`)

- **Shunt parameters**: resistance (Ω), zero offset (mV)
- **Battery parameters**: capacity (Ah), voltage → SOC lookup table
- **Voltage thresholds**: UVLO (under‑voltage lock‑out), OVLO (full battery)
- **Temperature thresholds**: battery and thruster over‑temp shutdown (°C)
- **Device & battery instances**: unique NMEA 2000 instance IDs
- **Pin assignments**: changeable to suit alternate Feather boards

---

## Build & Flash

1. Install the [NMEA2000 Arduino library](https://github.com/ttlappalainen/NMEA2000)
2. Install [Adafruit INA219 library](https://github.com/adafruit/Adafruit_INA219)
3. Open `main.ino` in the Arduino IDE
4. Select your Feather board and COM port
5. Compile & upload

---

## License

This project is licensed under the **Apache License 2.0**. See [LICENSE](LICENSE) for details.

