// main.ino
// Bow Battery / Thruster Monitor
// Sensor-only NMEA2000 node:
//   - INA226 reads 500A shunt + battery voltage
//   - DS18B20 sensors read battery and thruster temperatures
//   - Coulomb-counted SOC is tracked/reported where supported
//   - Onboard Feather NeoPixel shows boot / N2K / warning / error status
//   - Diagnostic text goes to Serial always, and to N2K when the bus is alive
//   - No relays, no solenoids, no web UI, no wireless

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_NeoPixel.h>

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

// -----------------------------------------------------------------------------
// Small INA226 driver. Avoids dependency on a specific INA226 library API.
// INA226 datasheet constants:
//   shunt voltage LSB = 2.5 uV
//   bus voltage LSB   = 1.25 mV
// -----------------------------------------------------------------------------
class INA226Simple {
public:
  explicit INA226Simple(uint8_t address) : _addr(address) {}

  bool begin() {
    Wire.begin();

    // Config register:
    // AVG=16 samples, VBUSCT=1.1ms, VSHCT=1.1ms, mode=shunt+bus continuous.
    writeRegister16(0x00, 0x4527);
    delay(5);

    uint16_t bus = 0;
    return readRegister16(0x02, bus);
  }

  bool read(float &busVoltageV, float &shuntVoltageMV, float &currentA) {
    uint16_t rawBus = 0;
    uint16_t rawShuntU = 0;

    if (!readRegister16(0x02, rawBus)) return false;
    if (!readRegister16(0x01, rawShuntU)) return false;

    int16_t rawShunt = (int16_t)rawShuntU;

    busVoltageV = rawBus * 0.00125f;
    shuntVoltageMV = rawShunt * 0.0025f;
    currentA = (shuntVoltageMV / 1000.0f) / SHUNT_RESISTANCE_OHM;

#if CURRENT_POSITIVE_IS_CHARGE
    // Leave sign as-is.
#else
    // Invert so positive reported current means charging, negative means discharge.
    currentA = -currentA;
#endif

    currentA += CURRENT_CALIBRATION_OFFSET_A;

    if (fabs(currentA) < CURRENT_ZERO_DEADBAND_A) {
      currentA = 0.0f;
    }

    return true;
  }

private:
  uint8_t _addr;

  bool writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission() == 0;
  }

  bool readRegister16(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t n = Wire.requestFrom((int)_addr, 2);
    if (n != 2) return false;

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    value = ((uint16_t)msb << 8) | lsb;
    return true;
  }
};

INA226Simple ina226(INA226_I2C_ADDR);
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
Adafruit_NeoPixel statusPixel(STATUS_NEOPIXEL_COUNT, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct SensorState {
  bool inaOk = false;
  bool battTempOk = false;
  bool thrustTempOk = false;

  float batteryVoltageV = 0.0f;
  float shuntVoltageMV = 0.0f;
  float batteryCurrentA = 0.0f;   // positive = charging, negative = discharging
  float batteryTempC = NAN;
  float thrustTempC = NAN;

  float socPercent = INITIAL_SOC_PERCENT;
  float remainingAh = BATTERY_CAPACITY_AH * (INITIAL_SOC_PERCENT / 100.0f);
};

SensorState state;

unsigned long bootMs = 0;
unsigned long lastSensorReadMs = 0;
unsigned long lastN2kSendMs = 0;
unsigned long lastDiagSendMs = 0;
unsigned long lastLedUpdateMs = 0;
unsigned long lastSocUpdateMs = 0;
unsigned long lastN2kRxMs = 0;
unsigned long lastVerbosePrintMs = 0;

// Diagnostic edge tracking so repeated text does not spam the bus.
bool prevInaOk = true;
bool prevBattTempOk = true;
bool prevThrustTempOk = true;
bool prevN2kOk = true;

// -----------------------------------------------------------------------------
// Status LED helpers
// -----------------------------------------------------------------------------
static void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0, statusPixel.Color(r, g, b));
  statusPixel.show();
}

static void bootLedSequence() {
  setStatusLed(255, 0, 0);     delay(500);  // red
  setStatusLed(0, 255, 0);     delay(500);  // green
  setStatusLed(0, 0, 255);     delay(500);  // blue
  setStatusLed(255, 255, 255); delay(500);  // white-ish
  setStatusLed(0, 0, 255);                 // blue while initializing/checking
}

static bool n2kAlive() {
  // During boot, stay blue instead of immediately declaring no N2K.
  if ((millis() - bootMs) < N2K_BOOT_GRACE_MS) return true;
  return lastN2kRxMs != 0 && ((millis() - lastN2kRxMs) < N2K_ACTIVITY_TIMEOUT_MS);
}

static bool hardErrorActive() {
  return !state.inaOk || !n2kAlive();
}

static bool warningActive() {
  return !state.battTempOk || !state.thrustTempOk;
}

static void updateStatusLed() {
  static unsigned long lastBlinkMs = 0;
  static bool blinkOn = false;
  unsigned long now = millis();

  if ((now - bootMs) < N2K_BOOT_GRACE_MS && lastN2kRxMs == 0) {
    setStatusLed(0, 0, 255); // boot grace / waiting for bus traffic
    return;
  }

  if (hardErrorActive()) {
    if (now - lastBlinkMs >= LED_FAST_FLASH_MS) {
      blinkOn = !blinkOn;
      lastBlinkMs = now;
      setStatusLed(blinkOn ? 255 : 0, 0, 0);
    }
    return;
  }

  if (warningActive()) {
    if (now - lastBlinkMs >= LED_SLOW_FLASH_MS) {
      blinkOn = !blinkOn;
      lastBlinkMs = now;
      setStatusLed(blinkOn ? 255 : 0, 0, 0);
    }
    return;
  }

  setStatusLed(0, 255, 0); // normal operation
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static bool validTemp(float t) {
  return isfinite(t) && t >= TEMP_MIN_VALID_C && t <= TEMP_MAX_VALID_C;
}

static float readDs18b20C(uint8_t index, bool &ok) {
  float tempC = ds18b20.getTempCByIndex(index);

  if (tempC == DEVICE_DISCONNECTED_C || !validTemp(tempC)) {
    ok = false;
    return NAN;
  }

  ok = true;
  return tempC;
}

static void updateSoc(unsigned long nowMs) {
  if (lastSocUpdateMs == 0) {
    lastSocUpdateMs = nowMs;
    return;
  }

  float dtHours = (nowMs - lastSocUpdateMs) / 3600000.0f;
  lastSocUpdateMs = nowMs;

  if (!state.inaOk || dtHours <= 0.0f) return;

  // Positive current charges the battery. Negative current discharges it.
  state.remainingAh += state.batteryCurrentA * dtHours;

  if (state.remainingAh < 0.0f) state.remainingAh = 0.0f;
  if (state.remainingAh > BATTERY_CAPACITY_AH) state.remainingAh = BATTERY_CAPACITY_AH;

  state.socPercent = (state.remainingAh / BATTERY_CAPACITY_AH) * 100.0f;
  if (state.socPercent < SOC_MIN_PERCENT) state.socPercent = SOC_MIN_PERCENT;
  if (state.socPercent > SOC_MAX_PERCENT) state.socPercent = SOC_MAX_PERCENT;
}

static void readSensors() {
  float busV = 0.0f;
  float shuntMV = 0.0f;
  float currentA = 0.0f;

  state.inaOk = ina226.read(busV, shuntMV, currentA);
  if (state.inaOk) {
    state.batteryVoltageV = busV;
    state.shuntVoltageMV = shuntMV;
    state.batteryCurrentA = currentA;
  } else {
    state.batteryVoltageV = 0.0f;
    state.shuntVoltageMV = 0.0f;
    state.batteryCurrentA = 0.0f;
  }

  state.batteryTempC = readDs18b20C(DS18B20_BATT_INDEX, state.battTempOk);
  state.thrustTempC = readDs18b20C(DS18B20_THRUST_INDEX, state.thrustTempOk);

  // Start next DS18B20 conversion. With wait-for-conversion disabled, this returns
  // immediately and the next read gets the completed result.
  ds18b20.requestTemperatures();

  updateSoc(millis());
}

// -----------------------------------------------------------------------------
// Serial + N2K diagnostics
// -----------------------------------------------------------------------------
static void logDiagnosticSerial(uint16_t code, const char *text, bool active) {
  Serial.print(F("DIAG "));
  Serial.print(code);
  Serial.print(active ? F(" ACTIVE: ") : F(" CLEAR: "));
  Serial.println(text);
}

static bool canSendDiagnosticOverN2k() {
#if SEND_DIAGNOSTIC_TEXT_PGN
  return n2kAlive();
#else
  return false;
#endif
}

// Diagnostic text helper.
// Serial output is always sent. N2K proprietary diagnostic text is sent only when
// incoming N2K/CAN traffic has recently been seen, so we do not pretend the bus is OK.
static void sendDiagnosticText(uint16_t code, const char *text, bool active = true) {
  logDiagnosticSerial(code, text, active);

#if SEND_DIAGNOSTIC_TEXT_PGN
  if (!canSendDiagnosticOverN2k()) return;

  tN2kMsg msg;
  msg.SetPGN(130900UL);        // Proprietary diagnostic text PGN for this node/project
  msg.Priority = 6;
  msg.AddByte(DIAG_SOURCE_INSTANCE);
  msg.Add2ByteUInt(code);
  msg.AddByte(active ? 1 : 0); // 1=active/problem, 0=clear/info
  msg.AddStr(text, 32);
  NMEA2000.SendMsg(msg);
#endif
}

static void sendDiagnosticTransitions() {
  bool currentN2kOk = n2kAlive();

  if (currentN2kOk != prevN2kOk) {
    sendDiagnosticText(1000, currentN2kOk ? "N2K traffic detected" : "No N2K traffic", !currentN2kOk);
    prevN2kOk = currentN2kOk;
  }

  if (state.inaOk != prevInaOk) {
    sendDiagnosticText(1001, state.inaOk ? "INA226 OK" : "INA226 missing/invalid", !state.inaOk);
    prevInaOk = state.inaOk;
  }

  if (state.battTempOk != prevBattTempOk) {
    sendDiagnosticText(1002, state.battTempOk ? "Battery temp OK" : "Battery DS18B20 missing", !state.battTempOk);
    prevBattTempOk = state.battTempOk;
  }

  if (state.thrustTempOk != prevThrustTempOk) {
    sendDiagnosticText(1003, state.thrustTempOk ? "Thruster temp OK" : "Thruster DS18B20 missing", !state.thrustTempOk);
    prevThrustTempOk = state.thrustTempOk;
  }
}

static void sendPeriodicDiagnosticSummary() {
  if (!n2kAlive()) {
    sendDiagnosticText(1000, "No N2K traffic", true);
  }
  if (!state.inaOk) {
    sendDiagnosticText(1001, "INA226 missing/invalid", true);
  }
  if (!state.battTempOk) {
    sendDiagnosticText(1002, "Battery DS18B20 missing", true);
  }
  if (!state.thrustTempOk) {
    sendDiagnosticText(1003, "Thruster DS18B20 missing", true);
  }
}

static void printVerboseValues() {
#if SERIAL_VERBOSE
  unsigned long now = millis();
  if (now - lastVerbosePrintMs < VERBOSE_PRINT_INTERVAL_MS) return;
  lastVerbosePrintMs = now;

  Serial.print(F("V=")); Serial.print(state.batteryVoltageV, 3);
  Serial.print(F("V I=")); Serial.print(state.batteryCurrentA, 2);
  Serial.print(F("A shunt=")); Serial.print(state.shuntVoltageMV, 4);
  Serial.print(F("mV SOC=")); Serial.print(state.socPercent, 1);
  Serial.print(F("% battT="));
  if (state.battTempOk) Serial.print(state.batteryTempC, 1); else Serial.print(F("NA"));
  Serial.print(F("C thrustT="));
  if (state.thrustTempOk) Serial.print(state.thrustTempC, 1); else Serial.print(F("NA"));
  Serial.print(F("C INA=")); Serial.print(state.inaOk ? F("OK") : F("BAD"));
  Serial.print(F(" N2K=")); Serial.println(n2kAlive() ? F("OK") : F("BAD"));
#endif
}

// -----------------------------------------------------------------------------
// NMEA2000 reporting
// -----------------------------------------------------------------------------
static void sendBatteryStatus() {
  tN2kMsg msg;

  // PGN 127508 - Battery Status: voltage, current, temperature
  double tempK = state.battTempOk ? (state.batteryTempC + 273.15) : N2kDoubleNA;
  double voltage = state.inaOk ? state.batteryVoltageV : N2kDoubleNA;
  double current = state.inaOk ? state.batteryCurrentA : N2kDoubleNA;

  SetN2kDCBatStatus(
    msg,
    N2K_BATTERY_INSTANCE,
    voltage,
    current,
    tempK
  );
  NMEA2000.SendMsg(msg);
}

static void sendTemperatures() {
  tN2kMsg msg;

  // PGN 130312 / 130316 depending on library version implementation.
  // Battery temp. NMEA2000 library has no BatteryTemperature enum,
  // so use EngineRoomTemperature and distinguish by instance.
  // Instance N2K_BATT_TEMP_INSTANCE = battery temperature.
  SetN2kTemperature(
    msg,
    0,
    N2K_BATT_TEMP_INSTANCE,
    N2kts_EngineRoomTemperature,
    state.battTempOk ? (state.batteryTempC + 273.15) : N2kDoubleNA,
    N2kDoubleNA
  );
  NMEA2000.SendMsg(msg);

  // Thruster motor area temp. Engine room is not semantically perfect, but widely understood.
  SetN2kTemperature(
    msg,
    0,
    N2K_THRUST_TEMP_INSTANCE,
    N2kts_EngineRoomTemperature,
    state.thrustTempOk ? (state.thrustTempC + 273.15) : N2kDoubleNA,
    N2kDoubleNA
  );
  NMEA2000.SendMsg(msg);
}

static void sendSocStatus() {
  // Some versions of the NMEA2000 library include SetN2kDCStatus and some do not.
  // If your installed library supports it, uncomment this block.
  //
  // tN2kMsg msg;
  // SetN2kDCStatus(
  //   msg,
  //   0,                         // SID
  //   N2K_BATTERY_INSTANCE,
  //   N2kDCt_Battery,
  //   state.socPercent,           // State of Charge %
  //   N2kUInt8NA,                 // State of Health % unknown
  //   N2kDoubleNA,                // Time remaining
  //   N2kDoubleNA,                // Ripple voltage
  //   BATTERY_CAPACITY_AH         // Capacity Ah
  // );
  // NMEA2000.SendMsg(msg);

  // Fallback: SOC is kept internally even if not sent by this library build.
}

static void sendAllN2kValues() {
  sendBatteryStatus();
  sendTemperatures();
  sendSocStatus();
}

// Any incoming NMEA2000 message proves the CAN/N2K physical/logical path is alive.
static void handleN2kMessage(const tN2kMsg &msg) {
  (void)msg;
  lastN2kRxMs = millis();
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  bootMs = millis();

  statusPixel.begin();
  statusPixel.setBrightness(STATUS_LED_BRIGHTNESS);
  statusPixel.clear();
  statusPixel.show();
  bootLedSequence();

  Serial.println(F("Bow Battery Monitor boot"));

  Wire.begin();
  state.inaOk = ina226.begin();

  ds18b20.begin();
  ds18b20.setResolution(DS18B20_RESOLUTION_BITS);
  ds18b20.setWaitForConversion(false);
  ds18b20.requestTemperatures();
  delay(DS18B20_FIRST_CONVERSION_DELAY_MS);

  // NMEA2000 setup
  NMEA2000.SetProductInformation(
    DEVICE_SERIAL,
    N2K_PRODUCT_CODE,
    DEVICE_NAME,
    DEVICE_SOFTWARE_VERSION,
    DEVICE_MODEL_VERSION,
    N2K_LOAD_EQ,
    2001
  );

  NMEA2000.SetDeviceInformation(
    N2K_DEVICE_UNIQUE_NUMBER,
    140,                       // Device Function: Sensor / monitor-ish. Adjust if desired.
    75,                        // Device Class: Electrical generation / electrical system-ish
    N2K_MANUFACTURER_CODE
  );

  NMEA2000.SetMsgHandler(handleN2kMessage);
  NMEA2000.Open();

  // Prime readings and diagnostics.
  readSensors();
  prevInaOk = state.inaOk;
  prevBattTempOk = state.battTempOk;
  prevThrustTempOk = state.thrustTempOk;
  prevN2kOk = n2kAlive();

  sendDiagnosticText(9000, "Bow battery monitor boot", false);

  if (!state.inaOk) sendDiagnosticText(1001, "INA226 missing/invalid", true);
  if (!state.battTempOk) sendDiagnosticText(1002, "Battery DS18B20 missing", true);
  if (!state.thrustTempOk) sendDiagnosticText(1003, "Thruster DS18B20 missing", true);
}

void loop() {
  unsigned long now = millis();

  NMEA2000.ParseMessages();

  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;
    readSensors();
    sendDiagnosticTransitions();
  }

  if (now - lastN2kSendMs >= N2K_SEND_INTERVAL_MS) {
    lastN2kSendMs = now;
    sendAllN2kValues();
  }

  if (now - lastDiagSendMs >= DIAG_SEND_INTERVAL_MS) {
    lastDiagSendMs = now;
    sendPeriodicDiagnosticSummary();
  }

  if (now - lastLedUpdateMs >= LED_UPDATE_INTERVAL_MS) {
    lastLedUpdateMs = now;
    updateStatusLed();
  }

  printVerboseValues();
}
