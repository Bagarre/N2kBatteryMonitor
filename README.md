# Bow Battery / Thruster Monitor — NMEA2000 Sensor Node

Sensor-only node for the bow thruster battery area.

It reads a local shunt with an INA226, reads two DS18B20 temperature sensors, tracks coulomb-counted SOC, reports values over NMEA2000, and uses the Feather M4 CAN Express onboard NeoPixel for status. It does not control relays, solenoids, charging, or wireless/web UI.

## Hardware

```text
Board:      Adafruit Feather M4 CAN Express
Current:    INA226 + 500A/50mV shunt
Temp:       2x DS18B20 digital temperature probes
Network:    NMEA2000 over onboard CAN
Status LED: Feather onboard NeoPixel
Power:      N2K power or local 12V through 5V buck/regulator
```

## Arduino libraries

Install these in Arduino IDE Library Manager:

```text
OneWire
DallasTemperature
Adafruit NeoPixel
NMEA2000
NMEA2000_CAN
```

Board selection:

```text
Tools → Board → Adafruit SAMD → Adafruit Feather M4 CAN Express
```

## Wiring

### Power

Preferred for a battery monitor is local battery power through a buck converter, but N2K power is also acceptable for this low-current node if you understand it will die when the N2K backbone is off.

```text
12V or N2K +12V → small fuse → 5V buck/regulator → Feather USB/VIN
Ground          → buck ground → Feather GND
```

Use a small fuse, around 0.25A–0.5A if powered from N2K, or 0.5A–1A if powered locally.

Do not feed 12V directly to the Feather. Do not feed 5V into the 3.3V pin.

### NMEA2000 / CAN

The Feather M4 CAN Express has the CAN controller/transceiver onboard.

```text
Feather CAN-H → N2K CAN-H
Feather CAN-L → N2K CAN-L
Feather GND   → N2K ground/common
```

The code treats any received N2K/CAN message as proof that the bus is alive. It does not depend on a specific PGN from the Cerbo or Zeus.

### INA226 / shunt

I2C:

```text
Feather 3.3V → INA226 VCC
Feather GND  → INA226 GND
Feather SDA  → INA226 SDA
Feather SCL  → INA226 SCL
```

Shunt sense:

```text
Battery negative ----[ 500A / 50mV SHUNT ]---- System negative / thruster negative

INA226 VIN+ → battery side of shunt
INA226 VIN- → load/system side of shunt
```

Use short, dedicated Kelvin sense wires from the shunt screws. Twist VIN+ and VIN-. With 6-inch leads, shielded cable is not necessary.

Default shunt value in `config.h`:

```cpp
#define SHUNT_RESISTANCE_OHM 0.0001f   // 500A / 50mV
```

If your shunt is 500A / 25mV, change it to:

```cpp
#define SHUNT_RESISTANCE_OHM 0.00005f
```

### DS18B20 temperature sensors

Both DS18B20 sensors share one data pin.

Default pin:

```cpp
#define ONE_WIRE_PIN 5
```

Wiring:

```text
Feather 3.3V → DS18B20 red
Feather GND  → DS18B20 black
Feather D5   → DS18B20 yellow/data

4.7k resistor from DATA to 3.3V
```

Both sensors can be wired in parallel on the same OneWire bus.

Default index mapping:

```cpp
#define DS18B20_BATT_INDEX   0
#define DS18B20_THRUST_INDEX 1
```

If the battery and thruster readings are swapped, either swap the physical plugs or swap those two index values.

Recommended mounting:

```text
Battery temp:  probe against battery case or under/near negative terminal hardware, insulated from ambient air
Thruster temp: strapped to motor frame or metal mounting area
```

## Onboard NeoPixel status LED

The code uses the Feather's onboard NeoPixel instead of separate Power/OK/Error LEDs.

Startup sequence:

```text
Red   for 0.5 sec
Green for 0.5 sec
Blue  for 0.5 sec
White for 0.5 sec
```

Operational states:

```text
Blue solid       initializing / waiting during boot grace period
Green solid      normal operation
Red fast flash   hard error
Red slow flash   warning
```

Hard errors:

```text
No NMEA2000/CAN traffic seen after boot grace
INA226/shunt sensor missing or invalid
```

Warnings:

```text
Battery DS18B20 missing/invalid
Thruster DS18B20 missing/invalid
```

Configuration:

```cpp
#define N2K_BOOT_GRACE_MS       8000UL
#define N2K_ACTIVITY_TIMEOUT_MS 5000UL
#define LED_FAST_FLASH_MS       150UL
#define LED_SLOW_FLASH_MS       700UL
#define STATUS_LED_BRIGHTNESS   32
```

## Serial diagnostics and verbose mode

Diagnostics are always printed to USB Serial at 115200 baud.

Use Arduino IDE Serial Monitor:

```text
Tools → Serial Monitor → 115200 baud
```

Every active/clear diagnostic prints like:

```text
DIAG 1001 ACTIVE: INA226 missing/invalid
DIAG 1001 CLEAR: INA226 OK
```

Verbose value logging can be enabled in `config.h`:

```cpp
#define SERIAL_VERBOSE true
```

When enabled, the node periodically prints voltage, current, shunt mV, SOC, temps, INA status, and N2K status.

## NMEA2000 messages emitted

### Normal numeric reports

The sketch emits these periodically:

#### PGN 127508 — Battery Status

Contains:

```text
Battery instance
Battery voltage
Battery current
Battery temperature
```

Instance:

```cpp
#define N2K_BATTERY_INSTANCE 1
```

Current sign convention is controlled by:

```cpp
#define CURRENT_POSITIVE_IS_CHARGE false
```

The code is written so reported current should be positive when charging and negative when discharging. Flip this setting if the sign is backwards after installation.

#### Temperature PGN via `SetN2kTemperature`

The NMEA2000 library does not include a true `BatteryTemperature` enum, so both temperature reports use `N2kts_EngineRoomTemperature` and are distinguished by instance.

```text
Instance 1 = battery temperature
Instance 2 = thruster temperature
```

Config:

```cpp
#define N2K_BATT_TEMP_INSTANCE   1
#define N2K_THRUST_TEMP_INSTANCE 2
```

Depending on the installed NMEA2000 library version, `SetN2kTemperature()` may emit PGN 130312 or the library's supported temperature PGN format.

### SOC

SOC is tracked internally by coulomb counting:

```text
remainingAh += currentA * elapsedHours
SOC = remainingAh / batteryCapacityAh
```

The sketch includes a commented `SetN2kDCStatus()` block because not every installed version of the NMEA2000 library exposes the same helper for SOC/status PGNs. If your library supports it, uncomment that block in `sendSocStatus()`.

Battery capacity is set here:

```cpp
#define BATTERY_CAPACITY_AH 200.0f
```

Initial SOC on boot:

```cpp
#define INITIAL_SOC_PERCENT 100.0f
```

Long term, this should be persisted to nonvolatile memory, but that is intentionally not added yet.

## Diagnostic text over NMEA2000

Diagnostics are always sent to Serial. If N2K traffic has recently been seen, the same diagnostics are also sent as a project-specific proprietary text PGN.

Proprietary PGN:

```text
130900
```

Payload:

```text
byte 0:      diagnostic source instance
bytes 1-2:   diagnostic code
byte 3:      active flag, 1 = active/problem, 0 = clear/info
string:      diagnostic text, max 32 chars
```

Diagnostic codes:

```text
9000  Boot/info
1000  No N2K traffic / N2K traffic detected
1001  INA226 missing/invalid / INA226 OK
1002  Battery DS18B20 missing / Battery temp OK
1003  Thruster DS18B20 missing / Thruster temp OK
```

These are not meant to replace real operating alarms. The Zeus, Cerbo, SignalK, or Node-RED should decide what is an alarm based on the reported values. These diagnostics are just to avoid plugging in USB to learn why the LED is flashing.

## Quick bench test

1. Install libraries.
2. Select `Adafruit Feather M4 CAN Express`.
3. Upload sketch.
4. Open Serial Monitor at 115200.
5. Confirm boot message.
6. Confirm NeoPixel startup sequence.
7. With no N2K traffic after the boot grace period, LED should fast flash red and Serial should report `No N2K traffic`.
8. Connect N2K/CAN bus with active traffic. LED should stop fast flashing if INA226 is present.
9. Disconnect one DS18B20. LED should slow flash red and Serial should report the missing temp sensor.
10. Disconnect INA226. LED should fast flash red and Serial should report INA226 missing/invalid.

## Notes

- DS18B20 sensor index order can change if wiring changes. For final production, hardcoding ROM IDs would be cleaner.
- INA226 sense wires should be short, twisted, and directly connected to the shunt Kelvin points.
- N2K physical bus must be properly terminated at both ends.
- If powered from N2K, this node only monitors while the N2K backbone is powered.
