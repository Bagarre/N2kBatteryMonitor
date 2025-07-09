// config.h
// Put all your hardware and calibration settings here
#ifndef CONFIG_H
#define CONFIG_H

// Shunt parameters
#define SHUNT_RESISTANCE_OHM     0.00005    // 50 µΩ
#define SHUNT_CALIBRATION_OFFSET 0.0        // mV zero offset

// Battery parameters
#define BATTERY_CAPACITY_AH      200.0      // AGM capacity in Ah
// Voltage to SOC map (resting voltage -> SOC %)
// Format: { voltage_mv, soc_percent }
const uint16_t socMap[][2] = {
  {11500,  0},   // 11.5 V -> 0%
  {12200, 20},   // 12.2 V -> 20%
  {12600, 40},
  {12800, 60},
  {13000, 80},
  {13200,100}    // 13.2 V -> 100%
};
const uint8_t socMapSize = sizeof(socMap) / sizeof(socMap[0]);

// Voltage thresholds (millivolts)
#define UVLO_MV   11000   // Under-voltage lock-out at 11.0 V
#define OVLO_MV   14400   // Over-voltage full battery at 14.4 V

// Temperature thresholds (°C)
#define BATT_TEMP_SHUTDOWN_C   60   // Battery over-temp
#define THRUST_TEMP_SHUTDOWN_C 75   // Thruster over-temp

// Sensor pins
#define SHUNT_SENSE_PIN_A  A0    // INA219 or external ADC mV sense
#define SHUNT_SENSE_PIN_B  A1
#define THERM_PIN_BATT     A2    // Analog thermistor divider
#define THERM_PIN_THRUST   A3

// Relay outputs
#define RELAY_PIN_UVLO     5     // Low-voltage relay control
#define RELAY_PIN_TEMP     6     // High-temp relay control

#endif // CONFIG_H
