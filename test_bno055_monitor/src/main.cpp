#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace {

constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t READ_INTERVAL_MS = 200;
constexpr uint8_t BNO055_I2C_ADDRESS = 0x28;

// Laisse -1 / -1 pour utiliser les pins I2C par defaut de la carte.
// Si le BNO055 n'est pas detecte, remplace par les bonnes broches du Waveshare.
constexpr int BNO055_SDA_PIN = 9;
constexpr int BNO055_SCL_PIN = 8;

Adafruit_BNO055 bno(55, BNO055_I2C_ADDRESS, &Wire);
unsigned long lastReadMs = 0;
bool bnoReady = false;

void beginI2c() {
  if (BNO055_SDA_PIN >= 0 && BNO055_SCL_PIN >= 0) {
    Wire.begin(BNO055_SDA_PIN, BNO055_SCL_PIN);
    Serial.printf("I2C configured on SDA=%d SCL=%d\n", BNO055_SDA_PIN, BNO055_SCL_PIN);
    return;
  }

  Wire.begin();
  Serial.println("I2C configured on board default pins");
}

void printCalibration() {
  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("calib sys=%u gyro=%u accel=%u mag=%u\n", sys, gyro, accel, mag);
}

bool beginBno055() {
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring, power and I2C address.");
    return false;
  }

  delay(50);
  bno.setExtCrystalUse(true);

  Adafruit_BNO055::adafruit_bno055_rev_info_t revInfo;
  bno.getRevInfo(&revInfo);

  Serial.println("BNO055 detected");
  Serial.printf("sw_rev=%u bl_rev=%u accel_id=0x%02X mag_id=0x%02X gyro_id=0x%02X\n",
                revInfo.sw_rev, revInfo.bl_rev, revInfo.accel_rev, revInfo.mag_rev, revInfo.gyro_rev);
  printCalibration();
  return true;
}

void printOrientationSample() {
  sensors_event_t orientationEvent;
  bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);

  const float headingDeg = orientationEvent.orientation.x;
  const float rollDeg = orientationEvent.orientation.z;
  const float pitchDeg = orientationEvent.orientation.y;

  uint8_t systemStatus = 0;
  uint8_t selfTestResult = 0;
  uint8_t systemError = 0;
  bno.getSystemStatus(&systemStatus, &selfTestResult, &systemError);

  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.printf(
    "heading=%.2f deg | roll=%.2f deg | pitch=%.2f deg | calib=%u/%u/%u/%u | status=%u selftest=0x%02X err=%u\n",
    headingDeg, rollDeg, pitchDeg, sys, gyro, accel, mag, systemStatus, selfTestResult, systemError);
}

}  // namespace

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("BNO055 serial monitor");

  beginI2c();
  bnoReady = beginBno055();
  lastReadMs = millis();
}

void loop() {
  if (!bnoReady) {
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs >= 2000) {
      lastRetryMs = millis();
      Serial.println("Retrying BNO055 init...");
      bnoReady = beginBno055();
    }
    delay(20);
    return;
  }

  const unsigned long now = millis();
  if (now - lastReadMs < READ_INTERVAL_MS) {
    delay(5);
    return;
  }

  lastReadMs = now;
  printOrientationSample();
}
