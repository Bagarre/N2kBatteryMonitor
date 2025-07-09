
// main.ino
#include <Arduino.h>
#include "config.h"
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// NMEA2000 instance for this shunt device
const uint8_t deviceInstance = 1;    // choose unique >0
const uint8_t batteryInstance = 1;   // matches above

void setup() {
  // Serial for debug
  Serial.begin(115200);

  // Initialize INA219
  ina219.begin();
  ina219.setCalibration_32V_2A();

  // Relays
  pinMode(RELAY_PIN_UVLO, OUTPUT);
  pinMode(RELAY_PIN_TEMP, OUTPUT);
  digitalWrite(RELAY_PIN_UVLO, HIGH);   // normally open
  digitalWrite(RELAY_PIN_TEMP, HIGH);

  // NMEA2000 setup
  NMEA2000.SetN2kCANMsgBufSize(250);
  NMEA2000.SetN2kCANReceiveFrameBufSize(10);
  NMEA2000.SetForwardStream(Serial);
  NMEA2000.Open();
  NMEA2000.ExtendInfo.SetProductInformation("FeatherN2KShunt","v1.0","FeatherShunt",100);
  NMEA2000.ExtendInfo.AddManufacturerInformation("DIYCorp","1.0");
}

// Map resting voltage to SOC via lookup table
uint8_t voltageToSOC(uint16_t mv) {
  for (uint8_t i = 1; i < socMapSize; i++) {
    if (mv <= socMap[i][0]) {
      // linear interp
      uint16_t v0 = socMap[i-1][0], v1 = socMap[i][0];
      uint8_t s0 = socMap[i-1][1], s1 = socMap[i][1];
      return s0 + (uint16_t)(s1 - s0) * (mv - v0) / (v1 - v0);
    }
  }
  return socMap[socMapSize-1][1];
}

unsigned long lastSend = 0;
void loop() {
  // Read shunt voltage & current
  float busVoltage = ina219.getBusVoltage_V() * 1000.0;   // mV
  float shuntVoltage = ina219.getShuntVoltage_mV() + SHUNT_CALIBRATION_OFFSET;
  float current_mA = ina219.getCurrent_mA();

  // Compute SOC (use coulomb count + ocv if desired; here simple OCV when idle)
  static float accumAh = 0;
  accumAh += current_mA * (millis() - lastSend) / 3600000.0;
  uint8_t soc = voltageToSOC((uint16_t)busVoltage);

  // Thermistors (simple 10k NTC dividers)
  auto readTempC = [](uint8_t pin){
    const float R1 = 10000.0;
    int adc = analogRead(pin);
    float v = (float)adc * 3.3 / 1023.0;
    float r = R1 * (3.3/v - 1);
    // Steinhart-Hart coefficients for 10k NTC
    float t = 1.0/(0.001129148 + 0.000234125*log(r) + 0.0000000876741*pow(log(r),3));
    return t - 273.15;
  };
  float tempBatt = readTempC(THERM_PIN_BATT);
  float tempThrust = readTempC(THERM_PIN_THRUST);

  // Relay logic
  digitalWrite(RELAY_PIN_UVLO, busVoltage < UVLO_MV ? LOW : HIGH);
  digitalWrite(RELAY_PIN_TEMP, tempThrust > THRUST_TEMP_SHUTDOWN_C ? LOW : HIGH);

  // Send NMEA2000 every 1s
  if (millis() - lastSend > 1000) {
    lastSend = millis();
    tN2kMsg N2kMsg;
    // Battery Status PGN127508
    SetN2kBatteryStatus(N2kMsg, deviceInstance, batteryInstance, busVoltage/1000.0, current_mA/1000.0, soc, 255);
    NMEA2000.SendMsg(N2kMsg);

    // Temperature PGN130312 thruster
    SetN2kTemperature(N2kMsg, 0, 1, tempThrust);
    NMEA2000.SendMsg(N2kMsg);

    // Temperature PGN130312 battery
    SetN2kTemperature(N2kMsg, 0, 2, tempBatt);
    NMEA2000.SendMsg(N2kMsg);
  }

  NMEA2000.ParseMessages();
}
