// main.ino

#include <Arduino.h>
#include "config.h"
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
// NMEA2000 battery instance
const uint8_t batteryInstance = 1;

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  // Initialize INA219
  ina219.begin();
  ina219.setCalibration_32V_2A();

  // Configure relays
  pinMode(RELAY_PIN_UVLO, OUTPUT);
  pinMode(RELAY_PIN_TEMP, OUTPUT);
  digitalWrite(RELAY_PIN_UVLO, HIGH);
  digitalWrite(RELAY_PIN_TEMP, HIGH);

  // Initialize NMEA2000
  NMEA2000.Open();
  // Optionally: NMEA2000.SetProductInformation(...)
}

uint8_t voltageToSOC(uint16_t mv) {
  for (uint8_t i = 1; i < socMapSize; i++) {
    if (mv <= socMap[i][0]) {
      uint16_t v0 = socMap[i-1][0], v1 = socMap[i][0];
      uint8_t  s0 = socMap[i-1][1], s1 = socMap[i][1];
      return s0 + uint16_t(s1 - s0) * (mv - v0) / (v1 - v0);
    }
  }
  return socMap[socMapSize-1][1];
}

void loop() {
  // Read bus voltage (mV) and current (mA)
  float busVoltage = ina219.getBusVoltage_V() * 1000.0;
  float current_mA = ina219.getCurrent_mA();

  // (Optional) simple coulomb counting
  static float accumAh = 0;
  accumAh += current_mA * (millis() - lastSend) / 3600000.0;

  uint8_t soc = voltageToSOC((uint16_t)busVoltage);

  // Read temperatures from NTC dividers
  auto readTempC = [](uint8_t pin) {
    const float R1 = 10000.0;
    int adc = analogRead(pin);
    float v = adc * (3.3f / 1023.0f);
    float r = R1 * (3.3f / v - 1);
    float tempK = 1.0f / (0.001129148f
                     + 0.000234125f * log(r)
                     + 0.0000000876741f * pow(log(r), 3));
    return tempK - 273.15f;
  };
  float tempBatt   = readTempC(THERM_PIN_BATT);
  float tempThrust = readTempC(THERM_PIN_THRUST);

  // Relay control
  digitalWrite(RELAY_PIN_UVLO, (busVoltage < UVLO_MV) ? LOW : HIGH);
  digitalWrite(RELAY_PIN_TEMP, (tempThrust > THRUST_TEMP_SHUTDOWN_C) ? LOW : HIGH);

  // Send messages every second
  if (millis() - lastSend >= 1000) {
    lastSend = millis();
    tN2kMsg msg;

    // DC Battery Status (PGN 127508)
    SetN2kDCBatStatus(
      msg,
      batteryInstance,
      busVoltage / 1000.0,
      current_mA / 1000.0,
      tempBatt
    );
    NMEA2000.SendMsg(msg);

    // Thruster temperature (PGN 130312)
        // Thruster temperature (PGN 130312) - external sensor
        // Battery temperature (PGN 130312) - external sensor
    SetN2kTemperature(
      msg,
      0,
      2,
      N2kts_EngineRoomTemperature,  // External sensor
      tempThrust,
      0.0
    );
    NMEA2000.SendMsg(msg);
  }

  NMEA2000.ParseMessages();
}
