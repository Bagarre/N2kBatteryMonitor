// config.h
// Bow Battery / Thruster Monitor - sensor-only NMEA2000 node
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Device identity
// -----------------------------------------------------------------------------
#define DEVICE_NAME                 "Bow Battery Monitor"
#define DEVICE_SERIAL               "BOWBAT-001"
#define DEVICE_SOFTWARE_VERSION     "2.1.0"
#define DEVICE_MODEL_VERSION        "2.1"
#define N2K_DEVICE_UNIQUE_NUMBER    101
#define N2K_MANUFACTURER_CODE       2046      // 2046 = experimental
#define N2K_PRODUCT_CODE            101
#define N2K_LOAD_EQ                 4         // 4 * 50mA

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------
#define SENSOR_READ_INTERVAL_MS     250
#define N2K_SEND_INTERVAL_MS        1000
#define DIAG_SEND_INTERVAL_MS       5000
#define LED_UPDATE_INTERVAL_MS      100
#define SOC_SAVE_INTERVAL_MS        60000UL   // if EEPROM support is added later
#define N2K_ACTIVITY_TIMEOUT_MS      5000UL    // no received CAN/N2K traffic after this = N2K fault
#define N2K_BOOT_GRACE_MS            8000UL    // allow bus time to appear after boot
#define SERIAL_VERBOSE               false     // true = print periodic sensor values for testing
#define VERBOSE_PRINT_INTERVAL_MS    2000UL

// -----------------------------------------------------------------------------
// INA226 / shunt
// -----------------------------------------------------------------------------
#define INA226_I2C_ADDR             0x40

// Typical 500A / 50mV shunt = 0.0001 ohm.
// If your shunt is 500A / 25mV, use 0.00005 instead.
#define SHUNT_RESISTANCE_OHM        0.0001f

// Offset in amps. Start at 0, then tune after observing zero-current reading.
#define CURRENT_CALIBRATION_OFFSET_A 0.0f

// Treat tiny measured current as zero to prevent SOC drift.
#define CURRENT_ZERO_DEADBAND_A     0.30f

// Positive current convention:
//   true  = positive current means charging battery
//   false = positive current means discharging battery
#define CURRENT_POSITIVE_IS_CHARGE  false

// -----------------------------------------------------------------------------
// Battery / SOC
// -----------------------------------------------------------------------------
#define BATTERY_CAPACITY_AH         200.0f
#define INITIAL_SOC_PERCENT         100.0f

// Coulomb-counting guardrails
#define SOC_MIN_PERCENT             0.0f
#define SOC_MAX_PERCENT             100.0f

// Optional voltage correction/sanity thresholds. These are NOT control outputs.
#define BATT_FULL_VOLTAGE_V         13.20f
#define BATT_LOW_VOLTAGE_V          11.50f
#define BATT_HIGH_VOLTAGE_V         14.80f

// -----------------------------------------------------------------------------
// DS18B20 temperature sensors
// -----------------------------------------------------------------------------
#define ONE_WIRE_PIN                5

// Both DS18B20s can share this one data pin with one 4.7k pullup to 3.3V.
// Index order is the DallasTemperature discovery order. If the two readings are
// swapped, either swap the sensor plugs or swap these two index values.
#define DS18B20_BATT_INDEX          0
#define DS18B20_THRUST_INDEX        1

// 10-bit is fast enough for status monitoring and keeps conversion time short.
// 9-bit: ~94ms, 10-bit: ~188ms, 11-bit: ~375ms, 12-bit: ~750ms.
#define DS18B20_RESOLUTION_BITS     10
#define DS18B20_FIRST_CONVERSION_DELAY_MS 250

// Plausibility checks. Used only for diagnostics.
#define TEMP_MIN_VALID_C            -40.0f
#define TEMP_MAX_VALID_C            125.0f

// -----------------------------------------------------------------------------
// Onboard status LED
// Feather M4 CAN Express has an onboard NeoPixel. Most Adafruit cores define
// PIN_NEOPIXEL. If not, this fallback is usually correct for Feather M4 boards.
// -----------------------------------------------------------------------------
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL                8
#endif
#define STATUS_NEOPIXEL_PIN         PIN_NEOPIXEL
#define STATUS_NEOPIXEL_COUNT       1
#define STATUS_LED_BRIGHTNESS       32        // 0-255; keep modest at night

// LED behavior:
//   boot: R, G, B, white, 0.5s each
//   blue: initializing / waiting during boot grace
//   green: normal operation
//   fast red flash: hard error, including missing shunt/INA226 or no N2K traffic
//   slow red flash: warning, such as missing DS18B20 temp sensor
#define LED_FAST_FLASH_MS           150UL
#define LED_SLOW_FLASH_MS           700UL

// -----------------------------------------------------------------------------
// NMEA2000 instances
// -----------------------------------------------------------------------------
#define N2K_BATTERY_INSTANCE        1
#define N2K_BATT_TEMP_INSTANCE      1
#define N2K_THRUST_TEMP_INSTANCE    2

// -----------------------------------------------------------------------------
// Diagnostic / alert-like messages
// These are intentionally limited to sensor/node problems, not operational alarms.
// Zeus/Cerbo/SignalK can decide what is an actual alarm from the reported values.
// -----------------------------------------------------------------------------
#define SEND_DIAGNOSTIC_TEXT_PGN    true
#define DIAG_SOURCE_INSTANCE        1

#endif // CONFIG_H
