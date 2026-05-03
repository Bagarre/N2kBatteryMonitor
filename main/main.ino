#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_NeoPixel.h>

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

// ================= USER CONFIG =================

#define N2K_OPEN_ENABLED false   // false for bench testing, true on boat

#define ONE_WIRE_PIN 5
#define STATUS_LED_PIN 8

// Maintenance / verbose logging switch.
// Wire GPIO to GND through DIP switch.
// INPUT_PULLUP means:
//   open = normal quiet mode
//   closed to GND = verbose maintenance mode
#define MAINTENANCE_MODE_PIN 12

#define INA226_ADDR 0x40

// 500A / 50mV shunt = 0.0001 ohm
#define SHUNT_RESISTANCE_OHM 0.0001f

#define BATTERY_CAPACITY_AH 200.0f
#define INITIAL_SOC_PERCENT 100.0f

#define N2K_BOOT_GRACE_MS 3000
#define N2K_ACTIVITY_TIMEOUT_MS 3000

#define LED_FAST_FLASH_MS 150
#define LED_SLOW_FLASH_MS 600

#define SENSOR_READ_INTERVAL_MS 1000
#define VERBOSE_PRINT_INTERVAL_MS 2000

#define STATUS_LED_BRIGHTNESS 40

// ================= INA226 DRIVER =================

class INA226Simple {
public:
  INA226Simple(uint8_t addr) : _addr(addr) {}

  bool begin() {
    Wire.begin();

    // AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, shunt+bus continuous
    write16(0x00, 0x4527);
    delay(5);

    uint16_t dummy;
    return read16(0x02, dummy);
  }

  bool read(float &busV, float &shuntMV, float &currentA) {
    uint16_t rawBus = 0;
    uint16_t rawShuntU = 0;

    if (!read16(0x02, rawBus)) return false;
    if (!read16(0x01, rawShuntU)) return false;

    int16_t rawShunt = (int16_t)rawShuntU;

    busV = rawBus * 0.00125f;        // INA226 bus voltage LSB
    shuntMV = rawShunt * 0.0025f;    // INA226 shunt voltage LSB

    currentA = (shuntMV / 1000.0f) / SHUNT_RESISTANCE_OHM;

    // Deadband tiny noise
    if (fabs(currentA) < 0.05f) currentA = 0.0f;

    return true;
  }

private:
  uint8_t _addr;

  bool write16(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
  }

  bool read16(uint8_t reg, uint16_t &val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((int)_addr, 2) != 2) return false;

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    val = ((uint16_t)msb << 8) | lsb;
    return true;
  }
};

// ================= GLOBALS =================

INA226Simple ina(INA226_ADDR);

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature temps(&oneWire);

Adafruit_NeoPixel led(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

enum SystemState {
  SYS_BOOTING,
  SYS_CHECKING,
  SYS_RUNNING,
  SYS_FAILED
};

SystemState sysState = SYS_BOOTING;

unsigned long bootMs = 0;
unsigned long n2kGraceStartMs = 0;
unsigned long lastCAN = 0;
unsigned long lastSensorReadMs = 0;
unsigned long lastVerbosePrintMs = 0;
unsigned long lastSocUpdateMs = 0;

bool initialCheckPrinted = false;

bool inaOk = false;
bool battTempOk = false;
bool thrTempOk = false;

float batteryVoltageV = 0.0f;
float shuntVoltageMV = 0.0f;
float batteryCurrentA = 0.0f;

float battTempC = NAN;
float thrTempC = NAN;

float remainingAh = BATTERY_CAPACITY_AH * (INITIAL_SOC_PERCENT / 100.0f);
float socPercent = INITIAL_SOC_PERCENT;

// ================= LED =================

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void bootSequence() {
  setLED(255, 0, 0);     delay(500);
  setLED(0, 255, 0);     delay(500);
  setLED(0, 0, 255);     delay(500);
  setLED(255, 255, 255); delay(500);
  setLED(0, 0, 255);
}

void updateLED() {
  static bool blink = false;
  static unsigned long lastBlinkMs = 0;

  unsigned long now = millis();

  // Blue only during boot/check grace
  if ((now - n2kGraceStartMs) < N2K_BOOT_GRACE_MS) {
    setLED(0, 0, 255);
    return;
  }

  bool n2kBad = !n2kAlive();
  bool hardError = n2kBad || !inaOk;
  bool warning = !battTempOk || !thrTempOk;

  if (hardError) {
    if (now - lastBlinkMs >= LED_FAST_FLASH_MS) {
      blink = !blink;
      lastBlinkMs = now;
      setLED(blink ? 255 : 0, 0, 0);
    }
    return;
  }

  if (warning) {
    if (now - lastBlinkMs >= LED_SLOW_FLASH_MS) {
      blink = !blink;
      lastBlinkMs = now;
      setLED(blink ? 255 : 0, 0, 0);
    }
    return;
  }

  setLED(0, 255, 0);
}

// ================= HELPERS =================

bool maintenanceMode() {
  return digitalRead(MAINTENANCE_MODE_PIN) == LOW;
}

bool validTemp(float t) {
  // DS18B20 missing commonly returns -127C.
  // 85C is the power-on/default conversion value and should not count as valid.
  return isfinite(t) && t != DEVICE_DISCONNECTED_C && t != 85.0f && t > -55.0f && t < 125.0f;
}

bool n2kAlive() {
#if N2K_OPEN_ENABLED
  return lastCAN != 0 && ((millis() - lastCAN) < N2K_ACTIVITY_TIMEOUT_MS);
#else
  return false;
#endif
}

void handleN2k(const tN2kMsg &msg) {
  (void)msg;
  lastCAN = millis();
}

void updateSoc(unsigned long nowMs) {
  if (lastSocUpdateMs == 0) {
    lastSocUpdateMs = nowMs;
    return;
  }

  float dtHours = (nowMs - lastSocUpdateMs) / 3600000.0f;
  lastSocUpdateMs = nowMs;

  if (!inaOk || dtHours <= 0.0f) return;

  // Positive current charges, negative current discharges.
  remainingAh += batteryCurrentA * dtHours;

  if (remainingAh < 0.0f) remainingAh = 0.0f;
  if (remainingAh > BATTERY_CAPACITY_AH) remainingAh = BATTERY_CAPACITY_AH;

  socPercent = (remainingAh / BATTERY_CAPACITY_AH) * 100.0f;
}

void readSensors() {
  inaOk = ina.read(batteryVoltageV, shuntVoltageMV, batteryCurrentA);

  if (!inaOk) {
    batteryVoltageV = 0.0f;
    shuntVoltageMV = 0.0f;
    batteryCurrentA = 0.0f;
  }

  battTempC = temps.getTempCByIndex(0);
  thrTempC  = temps.getTempCByIndex(1);

  battTempOk = validTemp(battTempC);
  thrTempOk  = validTemp(thrTempC);

  temps.requestTemperatures();

  updateSoc(millis());
}

// ================= SERIAL STATUS =================

void printCheckLine(const char *name, bool ok) {
  Serial.print("Checking ");
  Serial.print(name);
  Serial.print("... ");
  Serial.println(ok ? "OK" : "ERROR / not present");
}

void printSystemCheck() {
  Serial.println();
  Serial.println("=== Bow Battery Monitor System Check ===");

  printCheckLine("INA226 shunt monitor", inaOk);
  printCheckLine("battery DS18B20 temperature sensor", battTempOk);
  printCheckLine("thruster DS18B20 temperature sensor", thrTempOk);

#if N2K_OPEN_ENABLED
  printCheckLine("NMEA2000 traffic", n2kAlive());
#else
  Serial.println("Checking NMEA2000 traffic... SKIPPED - N2K_OPEN_ENABLED=false");
#endif

  Serial.println("-----------------------------------------");

  if (!inaOk || !battTempOk || !thrTempOk || !n2kAlive()) {
    Serial.println("System check complete: unable to continue normal operation.");
    Serial.println("Status LED will indicate fault/warning state.");
  } else {
    Serial.println("System check complete: OK.");
  }

  if (maintenanceMode()) {
    Serial.println("Maintenance mode: ENABLED - verbose logging active.");
  } else {
    Serial.println("Maintenance mode: disabled.");
  }

  Serial.println("=========================================");
  Serial.println();
}

void printVerboseStatus() {
  if (!maintenanceMode()) return;

  unsigned long now = millis();
  if (now - lastVerbosePrintMs < VERBOSE_PRINT_INTERVAL_MS) return;
  lastVerbosePrintMs = now;

  Serial.print("STATUS ");
  Serial.print("INA=");
  Serial.print(inaOk ? "OK" : "FAIL");

  Serial.print(" N2K=");
  Serial.print(n2kAlive() ? "OK" : "FAIL");

  Serial.print(" V=");
  Serial.print(batteryVoltageV, 3);

  Serial.print(" I=");
  Serial.print(batteryCurrentA, 2);

  Serial.print(" shunt_mV=");
  Serial.print(shuntVoltageMV, 4);

  Serial.print(" SOC=");
  Serial.print(socPercent, 1);
  Serial.print("%");

  Serial.print(" BattTemp=");
  if (battTempOk) Serial.print(battTempC, 1);
  else Serial.print("FAIL");

  Serial.print(" ThrTemp=");
  if (thrTempOk) Serial.print(thrTempC, 1);
  else Serial.print("FAIL");

  Serial.println();
}

// ================= N2K SEND =================

void sendN2kValues() {
#if N2K_OPEN_ENABLED
  tN2kMsg msg;

  double tempK = battTempOk ? (battTempC + 273.15) : N2kDoubleNA;
  double voltage = inaOk ? batteryVoltageV : N2kDoubleNA;
  double current = inaOk ? batteryCurrentA : N2kDoubleNA;

  SetN2kDCBatStatus(
    msg,
    0,        // battery instance
    voltage,
    current,
    tempK
  );
  NMEA2000.SendMsg(msg);

  // Battery temp as generic engine-room temp, instance 0
  SetN2kTemperature(
    msg,
    0,
    0,
    N2kts_EngineRoomTemperature,
    battTempOk ? (battTempC + 273.15) : N2kDoubleNA,
    N2kDoubleNA
  );
  NMEA2000.SendMsg(msg);

  // Thruster temp as generic engine-room temp, instance 1
  SetN2kTemperature(
    msg,
    0,
    1,
    N2kts_EngineRoomTemperature,
    thrTempOk ? (thrTempC + 273.15) : N2kDoubleNA,
    N2kDoubleNA
  );
  NMEA2000.SendMsg(msg);
#endif
}

// ================= SETUP / LOOP =================

void setup() {
  Serial.begin(115200);
  delay(100);

  bootMs = millis();

  pinMode(MAINTENANCE_MODE_PIN, INPUT_PULLUP);

  led.begin();
  led.setBrightness(STATUS_LED_BRIGHTNESS);
  led.clear();
  led.show();

  bootSequence();

  Serial.println("Bow Battery Monitor boot");

  Wire.begin();
  Serial.println("Checking I2C bus... OK");

  Serial.print("Checking INA226 shunt monitor... ");
  inaOk = ina.begin();
  Serial.println(inaOk ? "OK" : "ERROR / not present");

  Serial.print("Checking DS18B20 bus... ");
  temps.begin();
  temps.setWaitForConversion(false);
  temps.requestTemperatures();
  delay(750);
  Serial.println("OK");

#if N2K_OPEN_ENABLED
  Serial.println("Opening NMEA2000...");
  NMEA2000.SetMsgHandler(handleN2k);
  NMEA2000.Open();
  Serial.println("NMEA2000 open returned");
#else
  Serial.println("NMEA2000 open skipped for bench testing");
#endif

  n2kGraceStartMs = millis();

  readSensors();
  printSystemCheck();

  initialCheckPrinted = true;

  if (!inaOk || !battTempOk || !thrTempOk || !n2kAlive()) {
    sysState = SYS_FAILED;
  } else {
    sysState = SYS_RUNNING;
  }
}

void loop() {
  unsigned long now = millis();

#if N2K_OPEN_ENABLED
  NMEA2000.ParseMessages();
#endif

  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;
    readSensors();

    // If something recovers later, allow transition into running.
    if (inaOk && battTempOk && thrTempOk && n2kAlive()) {
      sysState = SYS_RUNNING;
    } else {
      sysState = SYS_FAILED;
    }
  }

  updateLED();
  printVerboseStatus();

#if N2K_OPEN_ENABLED
  static unsigned long lastN2kSendMs = 0;
  if (now - lastN2kSendMs >= 1000) {
    lastN2kSendMs = now;
    sendN2kValues();
  }
#endif
}