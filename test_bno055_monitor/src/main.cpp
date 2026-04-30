#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <utility/imumaths.h>
#include "remote_secrets.h"

namespace {

constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t READ_INTERVAL_MS = 200;
constexpr uint32_t MQTT_PUBLISH_INTERVAL_MS = 500;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 2000;
constexpr uint32_t MQTT_RETRY_INTERVAL_MS = 3000;
constexpr uint8_t BNO055_I2C_ADDRESS = 0x28;

// Laisse -1 / -1 pour utiliser les pins I2C par defaut de la carte.
// Si le BNO055 n'est pas detecte, remplace par les bonnes broches du Waveshare.
constexpr int BNO055_SDA_PIN = 9;
constexpr int BNO055_SCL_PIN = 8;

Adafruit_BNO055 bno(55, BNO055_I2C_ADDRESS, &Wire);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
unsigned long lastReadMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;
unsigned long lastMqttPublishMs = 0;
bool bnoReady = false;
float lastHeadingDeg = NAN;
float lastRollDeg = NAN;
float lastPitchDeg = NAN;
uint8_t lastSystemStatus = 0;
uint8_t lastSelfTestResult = 0;
uint8_t lastSystemError = 0;
uint8_t lastCalibSys = 0;
uint8_t lastCalibGyro = 0;
uint8_t lastCalibAccel = 0;
uint8_t lastCalibMag = 0;

String mqttClientId() {
  uint64_t chipId = ESP.getEfuseMac();
  char buffer[40];
  snprintf(buffer, sizeof(buffer), "heliostat-bno055-%08lx", static_cast<unsigned long>(chipId & 0xFFFFFFFFu));
  return String(buffer);
}

String mqttStateTopic() {
  return String(REMOTE_MQTT_BASE_TOPIC) + "/bno055/state";
}

String mqttAvailabilityTopic() {
  return String(REMOTE_MQTT_BASE_TOPIC) + "/bno055/availability";
}

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
  bno.getCalibration(&lastCalibSys, &lastCalibGyro, &lastCalibAccel, &lastCalibMag);
  Serial.printf("calib sys=%u gyro=%u accel=%u mag=%u\n",
                lastCalibSys, lastCalibGyro, lastCalibAccel, lastCalibMag);
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

void updateOrientationSample() {
  sensors_event_t orientationEvent;
  bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);

  lastHeadingDeg = orientationEvent.orientation.x;
  lastRollDeg = orientationEvent.orientation.z;
  lastPitchDeg = orientationEvent.orientation.y;

  bno.getSystemStatus(&lastSystemStatus, &lastSelfTestResult, &lastSystemError);
  bno.getCalibration(&lastCalibSys, &lastCalibGyro, &lastCalibAccel, &lastCalibMag);
}

void printOrientationSample() {
  Serial.printf(
    "heading=%.2f deg | roll=%.2f deg | pitch=%.2f deg | calib=%u/%u/%u/%u | status=%u selftest=0x%02X err=%u\n",
    lastHeadingDeg, lastRollDeg, lastPitchDeg,
    lastCalibSys, lastCalibGyro, lastCalibAccel, lastCalibMag,
    lastSystemStatus, lastSelfTestResult, lastSystemError);
}

void beginWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD);
  Serial.printf("WiFi connecting to %s\n", REMOTE_WIFI_SSID);
}

void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (millis() - lastWifiRetryMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWifiRetryMs = millis();
  Serial.println("WiFi reconnect...");
  WiFi.disconnect();
  WiFi.begin(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD);
}

void ensureMqttConnected() {
  if (mqttClient.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (millis() - lastMqttRetryMs < MQTT_RETRY_INTERVAL_MS) {
    return;
  }

  lastMqttRetryMs = millis();
  const String clientId = mqttClientId();
  const String availabilityTopic = mqttAvailabilityTopic();
  Serial.printf("MQTT connect: %s:%u client=%s\n", REMOTE_MQTT_HOST, REMOTE_MQTT_PORT, clientId.c_str());

  const bool connected = mqttClient.connect(
    clientId.c_str(),
    availabilityTopic.c_str(),
    0,
    true,
    "offline");

  if (!connected) {
    Serial.printf("MQTT connect failed, state=%d\n", mqttClient.state());
    return;
  }

  mqttClient.publish(availabilityTopic.c_str(), "online", true);
  Serial.println("MQTT connected");
}

void publishStateIfDue() {
  if (!mqttClient.connected()) {
    return;
  }
  if (millis() - lastMqttPublishMs < MQTT_PUBLISH_INTERVAL_MS) {
    return;
  }
  lastMqttPublishMs = millis();

  JsonDocument doc;
  doc["sensor"] = "bno055";
  doc["heading_deg"] = lastHeadingDeg;
  doc["roll_deg"] = lastRollDeg;
  doc["pitch_deg"] = lastPitchDeg;
  doc["calib_sys"] = lastCalibSys;
  doc["calib_gyro"] = lastCalibGyro;
  doc["calib_accel"] = lastCalibAccel;
  doc["calib_mag"] = lastCalibMag;
  doc["system_status"] = lastSystemStatus;
  doc["self_test_result"] = lastSelfTestResult;
  doc["system_error"] = lastSystemError;
  doc["uptime_ms"] = millis();
  doc["wifi_rssi_dbm"] = WiFi.RSSI();

  char payload[384];
  const size_t len = serializeJson(doc, payload, sizeof(payload));
  mqttClient.publish(mqttStateTopic().c_str(), reinterpret_cast<const uint8_t*>(payload), len, false);
}

}  // namespace

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("BNO055 serial monitor");

  beginI2c();
  mqttClient.setServer(REMOTE_MQTT_HOST, REMOTE_MQTT_PORT);
  mqttClient.setBufferSize(512);
  beginWifi();
  bnoReady = beginBno055();
  lastReadMs = millis();
}

void loop() {
  ensureWifiConnected();
  ensureMqttConnected();
  mqttClient.loop();

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
  updateOrientationSample();
  printOrientationSample();
  publishStateIfDue();
}
