#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <BLEController.h>
#include <BLEControllerRegistry.h>
#include <ESP32Servo.h>
#include <SCServo.h>
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 512
#endif
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>

#if __has_include("remote_secrets.h")
#include "remote_secrets.h"
#endif

// le miroir doit etre oriente SUD et a l'horizontal pour que les angles soient corrects, sinon il faudra faire des ajustements dans les calculs d'angles


namespace {

#if defined(DEVICE_ROLE_CONTROLLER) && defined(DEVICE_ROLE_REMOTE)
#error "Choose only one role: DEVICE_ROLE_CONTROLLER or DEVICE_ROLE_REMOTE"
#endif

#if !defined(DEVICE_ROLE_CONTROLLER) && !defined(DEVICE_ROLE_REMOTE)
#error "Define DEVICE_ROLE_CONTROLLER or DEVICE_ROLE_REMOTE in platformio.ini"
#endif

constexpr bool STATUS_LED_ENABLED = false;
constexpr int STATUS_LED_PIN = 38;
constexpr int STATUS_LED_COUNT = 1;
constexpr uint8_t LED_BRIGHTNESS = 32;

constexpr float PAN_MIN_DEG = 0.0f;
constexpr float PAN_MAX_DEG = 180.0f;
constexpr float PAN_START_DEG = 90.0f;
constexpr float PAN_MODEL_SIGN = -1.0f;

constexpr float TILT_MIN_DEG = 0.0f;
constexpr float TILT_MAX_DEG = 180.0f;
constexpr float TILT_START_DEG = 180.0f;
constexpr float TILT_MODEL_SIGN = -1.0f;
constexpr float TILT_SERVO_OFFSET_DEG = 0.0f;
constexpr float TILT_SERVO_MIN_DEG = 00.0f;
constexpr float TILT_SERVO_MAX_DEG = 180.0f;
constexpr float TILT_SERVO_AT_MODEL_HORIZON_DEG = TILT_START_DEG + TILT_SERVO_OFFSET_DEG;

constexpr float STICK_DEADZONE = 0.12f;
constexpr float MAX_TARGET_SPEED_DEG_PER_SEC = 90.0f;
constexpr float MANUAL_PRECISION_SPEED_DEG_PER_SEC = 8.0f;
constexpr float SERVO_MAX_SLEW_DEG_PER_SEC = 500.0f;
constexpr uint32_t CONTROL_UPDATE_MS = 5;
constexpr uint32_t STATUS_PRINT_MS = 2000;
constexpr uint32_t CONTROL_LINK_TIMEOUT_MS = 250;
constexpr bool ENABLE_RUNTIME_STATUS_LOGS = true;
constexpr bool WAIT_FOR_SERIAL = false;
constexpr uint32_t WAIT_FOR_SERIAL_TIMEOUT_MS = 15000;
constexpr bool CLEAR_XBOX_BONDS_ON_BOOT = true;
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t REMOTE_ANNOUNCE_MS = 1000;
constexpr uint32_t BLINK_PERIOD_MS = 300;
constexpr uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr int PAN_SERVO_PIN = 5;
constexpr int TILT_SERVO_PIN = 6;
constexpr int PAN_SERVO_MIN_PULSE_US = 566;
constexpr int PAN_SERVO_MAX_PULSE_US = 2416;
constexpr int TILT_SERVO_MIN_PULSE_US = 460;
constexpr int TILT_SERVO_MAX_PULSE_US = 2362;
constexpr int SERVO_COMMAND_STEP_US = 1;
constexpr bool SHOW_SERVO_PULSE_US_IN_LOGS = false;
constexpr int SERVO_FREQUENCY_HZ = 350;
constexpr uint8_t REMOTE_ACTUATOR_BACKEND_PWM = 0;
constexpr uint8_t REMOTE_ACTUATOR_BACKEND_ST3020 = 1;
constexpr uint8_t REMOTE_ACTUATOR_BACKEND = REMOTE_ACTUATOR_BACKEND_ST3020;
constexpr int ST3020_UART_RX_PIN = 18;
constexpr int ST3020_UART_TX_PIN = 19;
constexpr uint32_t ST3020_UART_BAUD = 1000000;
constexpr uint8_t ST3020_PAN_ID = 1;
constexpr uint8_t ST3020_TILT_ID = 2;
constexpr uint16_t ST3020_DEFAULT_SPEED = 4000;
constexpr uint8_t ST3020_DEFAULT_ACC = 50;
constexpr float ST3020_PAN_SIGN = -1.0f;
constexpr float ST3020_TILT_SIGN = -1.0f;
constexpr int ST3020_POS_AT_0_DEG = 1024;
constexpr int ST3020_POS_AT_90_DEG = 2048;
constexpr int ST3020_POS_AT_180_DEG = 3072;
constexpr int ST3020_PAN_POS_MIN = ST3020_POS_AT_0_DEG;
constexpr int ST3020_PAN_POS_MAX = ST3020_POS_AT_180_DEG;
constexpr int ST3020_TILT_POS_MIN = 0;
constexpr int ST3020_TILT_POS_MAX = 4095;
constexpr float ST3020_PAN_MIN_DEG = 0.0f;
constexpr float ST3020_PAN_MAX_DEG = 180.0f;
constexpr float ST3020_TILT_MIN_DEG = 0.0f;
constexpr float ST3020_TILT_MAX_DEG = 210.0f;
constexpr float ST3020_TILT_EXTERNAL_MIN_DEG = -30.0f;
constexpr float ST3020_TILT_EXTERNAL_MAX_DEG = 180.0f;
constexpr float SPEED_CURVE_EXPONENT = 1.8f;
constexpr float HELIOSTAT_LATITUDE_DEG = 46.20027148248908f;
constexpr float HELIOSTAT_LONGITUDE_DEG = 6.139431924071319f;
constexpr time_t HELIOSTAT_START_UNIX_TIME_UTC = 0;
constexpr float HELIOSTAT_TIME_SCALE = 1.0f;
constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 64;
constexpr uint32_t REMOTE_STATE_PUBLISH_MS = 150;
constexpr uint32_t REMOTE_WIFI_RETRY_MS = 10000;
constexpr uint32_t REMOTE_MQTT_RETRY_MS = 5000;
constexpr uint32_t REMOTE_NTP_RETRY_MS = 15000;
constexpr uint32_t REMOTE_NTP_RESYNC_MS = 60000;
constexpr time_t MIN_VALID_UNIX_TIME_UTC = 1704067200;
constexpr uint16_t REMOTE_MQTT_BUFFER_SIZE = 1024;
constexpr float SCAN_SETTLE_TOLERANCE_DEG = 0.35f;
constexpr uint32_t SCAN_COARSE_DWELL_MS = 180;
constexpr uint32_t SCAN_FINE_DWELL_MS = 220;
constexpr uint32_t SCAN_MICRO_DWELL_MS = 260;
constexpr float SCAN_COARSE_RANGE_PAN_DEG = 6.0f;
constexpr float SCAN_COARSE_RANGE_TILT_DEG = 6.0f;
constexpr float SCAN_COARSE_STEP_DEG = 1.0f;
constexpr float SCAN_FINE_RANGE_PAN_DEG = 1.5f;
constexpr float SCAN_FINE_RANGE_TILT_DEG = 1.5f;
constexpr float SCAN_FINE_STEP_DEG = 0.25f;
constexpr float SCAN_MICRO_RANGE_PAN_DEG = 0.5f;
constexpr float SCAN_MICRO_RANGE_TILT_DEG = 0.5f;
constexpr float SCAN_MICRO_STEP_DEG = 0.1f;
constexpr float SCAN_MOVE_SPEED_DEFAULT_DEG_PER_SEC = 120.0f;
#ifndef REMOTE_WIFI_SSID
#define REMOTE_WIFI_SSID "TODO_WIFI_SSID"
#endif
#ifndef REMOTE_WIFI_PASSWORD
#define REMOTE_WIFI_PASSWORD "TODO_WIFI_PASSWORD"
#endif
#ifndef REMOTE_MQTT_HOST
#define REMOTE_MQTT_HOST "example.com"
#endif
#ifndef REMOTE_MQTT_PORT
#define REMOTE_MQTT_PORT 1883
#endif
#ifndef REMOTE_MQTT_BASE_TOPIC
#define REMOTE_MQTT_BASE_TOPIC "heliostat/remote1"
#endif
constexpr char REMOTE_WIFI_SSID_VALUE[] = REMOTE_WIFI_SSID;
constexpr char REMOTE_WIFI_PASSWORD_VALUE[] = REMOTE_WIFI_PASSWORD;
constexpr char REMOTE_MQTT_HOST_VALUE[] = REMOTE_MQTT_HOST;
constexpr uint16_t REMOTE_MQTT_PORT_VALUE = REMOTE_MQTT_PORT;
constexpr char REMOTE_MQTT_BASE_TOPIC_VALUE[] = REMOTE_MQTT_BASE_TOPIC;
constexpr char REMOTE_NTP_SERVER_1[] = "pool.ntp.org";
constexpr char REMOTE_NTP_SERVER_2[] = "time.nist.gov";
constexpr char REMOTE_NTP_SERVER_3[] = "time.google.com";

#if defined(DEVICE_ROLE_CONTROLLER)
constexpr const char* ROLE_NAME = "controller";
#else
constexpr const char* ROLE_NAME = "remote";
#endif

enum PacketType : uint8_t {
  PACKET_TYPE_CONTROL = 1,
  PACKET_TYPE_ANNOUNCE = 2,
};

constexpr uint8_t PACKET_FLAG_RECENTER = 0x01;
constexpr uint8_t PACKET_FLAG_CAPTURE_TARGET = 0x02;
constexpr uint8_t PACKET_FLAG_AUTO_TRACK = 0x04;
constexpr uint8_t PACKET_FLAG_TIME_UPDATE = 0x08;
constexpr uint8_t PACKET_FLAG_SCALE_UPDATE = 0x10;
constexpr uint8_t PACKET_FLAG_PRECISION = 0x20;

enum ControlMode : uint8_t {
  CONTROL_MODE_MANUAL = 0,
  CONTROL_MODE_TARGET_CAPTURED = 1,
  CONTROL_MODE_AUTO_TRACK = 2,
};

enum ScanStage : uint8_t {
  SCAN_STAGE_IDLE = 0,
  SCAN_STAGE_COARSE = 1,
  SCAN_STAGE_FINE = 2,
  SCAN_STAGE_MICRO = 3,
};

const char* controlModeName(ControlMode mode) {
  switch (mode) {
    case CONTROL_MODE_MANUAL:
      return "manual";
    case CONTROL_MODE_TARGET_CAPTURED:
      return "captured";
    case CONTROL_MODE_AUTO_TRACK:
      return "auto";
    default:
      return "unknown";
  }
}

const char* scanStageName(ScanStage stage) {
  switch (stage) {
    case SCAN_STAGE_IDLE:
      return "idle";
    case SCAN_STAGE_COARSE:
      return "coarse";
    case SCAN_STAGE_FINE:
      return "fine";
    case SCAN_STAGE_MICRO:
      return "micro";
    default:
      return "unknown";
  }
}

struct Vec3 {
  float x;
  float y;
  float z;
};

struct EspNowPacket {
  uint32_t magic;
  uint8_t type;
  uint8_t reserved[3];
  uint32_t seq;
  float panInput;
  float tiltInput;
  float panAngleDeg;
  float tiltAngleDeg;
  float panTargetDeg;
  float tiltTargetDeg;
  float capturedPanAngleDeg;
  float capturedTiltAngleDeg;
  float timeScale;
  float autoReferencePanDeg;
  float autoReferenceTiltDeg;
  int64_t unixTimeUtc;
  int64_t autoTrackStartUnixTimeUtc;
  int32_t panPulseUs;
  int32_t tiltPulseUs;
};

constexpr uint32_t LED_PACKET_MAGIC = 0x48454C31;  // "HEL1"
BLEController controller;
Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);
#if defined(DEVICE_ROLE_REMOTE)
Servo panServo;
Servo tiltServo;
SMS_STS st3020Bus;
WiFiClient remoteMqttNetClient;
PubSubClient remoteMqttClient(remoteMqttNetClient);
#endif

float panAngleDeg = PAN_START_DEG;
float tiltAngleDeg = TILT_START_DEG;
float panTargetDeg = PAN_START_DEG;
float tiltTargetDeg = TILT_START_DEG;
float remotePanInput = 0.0f;
float remoteTiltInput = 0.0f;
uint8_t baseRed = 0;
uint8_t baseGreen = 0;
uint8_t baseBlue = 0;
uint8_t currentRed = 0;
uint8_t currentGreen = 0;
uint8_t currentBlue = 0;

uint32_t lastControlUpdateMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t bootMs = 0;
uint32_t packetSequence = 0;
uint32_t lastRxMs = 0;
uint32_t lastAnnounceMs = 0;

bool xboxConnected = false;
bool espNowPeerReady = false;
bool lastSendOk = false;
bool packetReceived = false;
bool broadcastPeerReady = false;
bool blinkActive = false;
bool recenterRequested = false;
bool captureTargetRequested = false;
bool autoTrackEnabled = false;
bool precisionManualMode = false;
bool sendPending = false;
bool timeUpdatePending = false;
bool timeScaleUpdatePending = false;
ControlMode controlMode = CONTROL_MODE_MANUAL;
bool targetDirectionValid = false;
bool sunTimeValid = false;
Vec3 targetDirection = {0.0f, 0.0f, 0.0f};
time_t heliostatStartUnixTimeUtc = HELIOSTAT_START_UNIX_TIME_UTC;
uint32_t heliostatTimeBaseMillis = 0;
float capturedPanAngleDeg = PAN_START_DEG;
float capturedTiltAngleDeg = TILT_START_DEG;
float heliostatTimeScale = HELIOSTAT_TIME_SCALE;
float heliostatLatitudeDeg = HELIOSTAT_LATITUDE_DEG;
float heliostatLongitudeDeg = HELIOSTAT_LONGITUDE_DEG;
float lastAutoReferencePanDeg = PAN_START_DEG;
float lastAutoReferenceTiltDeg = TILT_START_DEG;
time_t autoTrackStartUnixTimeUtc = 0;
bool wifiLinkOk = false;
bool mqttLinkOk = false;
uint32_t lastWifiConnectAttemptMs = 0;
uint32_t lastMqttConnectAttemptMs = 0;
uint32_t lastRemoteStatePublishMs = 0;
uint32_t lastNtpSyncAttemptMs = 0;
uint32_t lastNtpApplyMs = 0;
bool ntpConfigured = false;
bool ntpTimeValid = false;
bool scanActive = false;
bool scanPaused = false;
ScanStage scanStage = SCAN_STAGE_IDLE;
int8_t scanDirection = 1;
float scanCenterPanDeg = PAN_START_DEG;
float scanCenterTiltDeg = TILT_START_DEG;
float scanRangePanDeg = 0.0f;
float scanRangeTiltDeg = 0.0f;
float scanStepDeg = 0.0f;
uint16_t scanGridCols = 0;
uint16_t scanGridRows = 0;
uint16_t scanPointIndex = 0;
uint16_t scanPointsTotal = 0;
uint32_t scanDwellMs = 0;
float scanMoveSpeedDegPerSec = SCAN_MOVE_SPEED_DEFAULT_DEG_PER_SEC;
uint32_t scanPointReachedMs = 0;
bool scanLockValid = false;
float scanLockPanDeg = PAN_START_DEG;
float scanLockTiltDeg = TILT_START_DEG;
bool approxTargetValid = false;
float approxTargetBearingDeg = 180.0f;
float approxTargetElevationDeg = 0.0f;
float approxTargetPanDeg = PAN_START_DEG;
float approxTargetTiltDeg = TILT_START_DEG;

#if defined(DEVICE_ROLE_CONTROLLER)
uint8_t remotePeerMac[6] = {0, 0, 0, 0, 0, 0};
uint8_t pendingAutoPairMac[6] = {0, 0, 0, 0, 0, 0};
bool lastCaptureButton = false;
bool lastAutoButton = false;
bool lastDiagButton = false;
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE] = {};
size_t serialCommandLength = 0;
float remoteReportedPanAngleDeg = PAN_START_DEG;
float remoteReportedTiltAngleDeg = TILT_START_DEG;
float remoteReportedPanTargetDeg = PAN_START_DEG;
float remoteReportedTiltTargetDeg = TILT_START_DEG;
ControlMode remoteReportedControlMode = CONTROL_MODE_MANUAL;
bool remoteReportedTargetDirectionValid = false;
bool remoteReportedSunTimeValid = false;
float remoteReportedCapturedPanAngleDeg = PAN_START_DEG;
float remoteReportedCapturedTiltAngleDeg = TILT_START_DEG;
float remoteReportedTimeScale = HELIOSTAT_TIME_SCALE;
int remoteReportedPanPulseUs = PAN_SERVO_MIN_PULSE_US;
int remoteReportedTiltPulseUs = TILT_SERVO_MIN_PULSE_US;
time_t remoteReportedUnixTimeUtc = 0;
float remoteReportedAutoReferencePanDeg = PAN_START_DEG;
float remoteReportedAutoReferenceTiltDeg = TILT_START_DEG;
time_t remoteReportedAutoTrackStartUnixTimeUtc = 0;
bool pendingAutoPair = false;
bool pendingControllerConnectLog = false;
bool pendingControllerDisconnectLog = false;
bool pendingControllerSyncPacket = false;
bool pendingEspNowTxFailureLog = false;
#endif

float applyDeadzone(float value) {
  const float normalized = constrain(value, -1.0f, 1.0f);
  const float magnitude = fabsf(normalized);
  if (magnitude < STICK_DEADZONE) {
    return 0.0f;
  }

  const float scaled = (magnitude - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  return copysignf(scaled, normalized);
}

float applySpeedCurve(float speed) {
  const float clamped = constrain(speed, -1.0f, 1.0f);
  const float magnitude = fabsf(clamped);
  return copysignf(powf(magnitude, SPEED_CURVE_EXPONENT), clamped);
}

float degToRad(float degrees) {
  return degrees * (PI / 180.0f);
}

float radToDeg(float radians) {
  return radians * (180.0f / PI);
}

String formatUnixTimeUtc(time_t unixTimeUtc) {
  if (unixTimeUtc <= 0) {
    return String("missing");
  }

  struct tm utcTm{};
  if (gmtime_r(&unixTimeUtc, &utcTm) == nullptr) {
    return String("invalid");
  }

  char buffer[24];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%SZ", &utcTm);
  return String(buffer);
}

Vec3 makeVec3(float x, float y, float z) {
  return {x, y, z};
}

Vec3 addVec3(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 scaleVec3(const Vec3& v, float scale) {
  return {v.x * scale, v.y * scale, v.z * scale};
}

float dotVec3(const Vec3& a, const Vec3& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float lengthVec3(const Vec3& v) {
  return sqrtf(dotVec3(v, v));
}

Vec3 normalizeVec3(const Vec3& v) {
  const float length = lengthVec3(v);
  if (length <= 1e-6f) {
    return {0.0f, 0.0f, 0.0f};
  }
  return scaleVec3(v, 1.0f / length);
}

Vec3 reflectVector(const Vec3& incoming, const Vec3& normal) {
  return addVec3(incoming, scaleVec3(normal, -2.0f * dotVec3(incoming, normal)));
}

Vec3 mirrorNormalFromPanTilt(float panDeg, float tiltDeg) {
  const float panAzimuthDeg = (PAN_MODEL_SIGN * (panDeg - 90.0f)) + 180.0f;
  const float panRad = degToRad(panAzimuthDeg);
  const float tiltRad = degToRad(TILT_MODEL_SIGN * (tiltDeg - 90.0f));
  const float cosTilt = cosf(tiltRad);
  return normalizeVec3({
      cosTilt * cosf(panRad),
      cosTilt * sinf(panRad),
      sinf(tiltRad),
  });
}

void panTiltFromMirrorNormal(const Vec3& normal, float& panDeg, float& tiltDeg) {
  const Vec3 normalized = normalizeVec3(normal);
  float panAzimuthDeg = radToDeg(atan2f(normalized.y, normalized.x));
  if (panAzimuthDeg < 0.0f) {
    panAzimuthDeg += 360.0f;
  }
  const float panModelDeg = 90.0f + (PAN_MODEL_SIGN * (panAzimuthDeg - 180.0f));
  panDeg = constrain(panModelDeg, PAN_MIN_DEG, PAN_MAX_DEG);
  const float tiltModelDeg = 90.0f + (TILT_MODEL_SIGN * radToDeg(asinf(normalized.z)));
  tiltDeg = constrain(tiltModelDeg, TILT_MIN_DEG, TILT_MAX_DEG);
}

bool currentUnixTimeUtc(time_t& unixTimeUtc) {
  if (heliostatStartUnixTimeUtc <= 0) {
    return false;
  }
  const uint32_t elapsedMs = millis() - heliostatTimeBaseMillis;
  unixTimeUtc =
      heliostatStartUnixTimeUtc + static_cast<time_t>((static_cast<double>(elapsedMs) *
                                                       static_cast<double>(heliostatTimeScale)) /
                                                      1000.0);
  return true;
}

bool systemClockTimeValid(time_t& unixTimeUtc) {
  unixTimeUtc = time(nullptr);
  return unixTimeUtc >= MIN_VALID_UNIX_TIME_UTC;
}

Vec3 sunVectorFromUnixTime(time_t unixTimeUtc) {
  const double jd = 2440587.5 + (static_cast<double>(unixTimeUtc) / 86400.0);
  const double t = (jd - 2451545.0) / 36525.0;
  const double l0 = fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360.0);
  const double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double c = sin(degToRad(m)) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                   sin(degToRad(2.0 * m)) * (0.019993 - 0.000101 * t) +
                   sin(degToRad(3.0 * m)) * 0.000289;
  const double trueLongitude = l0 + c;
  const double omega = 125.04 - (1934.136 * t);
  const double lambda = trueLongitude - 0.00569 - 0.00478 * sin(degToRad(omega));
  const double epsilon0 =
      23.0 + (26.0 + ((21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0)) / 60.0;
  const double epsilon = epsilon0 + 0.00256 * cos(degToRad(omega));
  const double declination = radToDeg(asinf(sinf(degToRad(epsilon)) * sin(degToRad(lambda))));

  const double y = tan(degToRad(epsilon / 2.0)) * tan(degToRad(epsilon / 2.0));
  const double equationOfTime =
      4.0 * radToDeg(y * sin(2.0 * degToRad(l0)) - 2.0 * e * sin(degToRad(m)) +
                     4.0 * e * y * sin(degToRad(m)) * cos(2.0 * degToRad(l0)) -
                     0.5 * y * y * sin(4.0 * degToRad(l0)) -
                     1.25 * e * e * sin(2.0 * degToRad(m)));

  const double fractionalDayUtc = fmod(static_cast<double>(unixTimeUtc), 86400.0) / 86400.0;
  const double trueSolarMinutes =
      fmod((fractionalDayUtc * 1440.0) + equationOfTime + (4.0 * heliostatLongitudeDeg) + 1440.0,
           1440.0);
  double hourAngleDeg = (trueSolarMinutes / 4.0) - 180.0;
  if (hourAngleDeg < -180.0) {
    hourAngleDeg += 360.0;
  }

  const double latitudeRad = degToRad(heliostatLatitudeDeg);
  const double declinationRad = degToRad(declination);
  const double hourAngleRad = degToRad(hourAngleDeg);

  const double elevationRad =
      asin(sin(latitudeRad) * sin(declinationRad) +
           cos(latitudeRad) * cos(declinationRad) * cos(hourAngleRad));
  const double azimuthRad =
      atan2(sin(hourAngleRad),
            cos(hourAngleRad) * sin(latitudeRad) - tan(declinationRad) * cos(latitudeRad));
  const float azimuthDeg = fmod(radToDeg(azimuthRad) + 180.0f + 360.0f, 360.0f);
  const float elevationDeg = radToDeg(elevationRad);

  const float azimuthFromXDeg = azimuthDeg;
  const float elevationRadF = degToRad(elevationDeg);
  const float azimuthRadF = degToRad(azimuthFromXDeg);
  return normalizeVec3({
      cosf(elevationRadF) * cosf(azimuthRadF),
      cosf(elevationRadF) * sinf(azimuthRadF),
      sinf(elevationRadF),
  });
}

Vec3 vectorFromBearingElevation(float bearingDeg, float elevationDeg) {
  const float bearingRad = degToRad(bearingDeg);
  const float elevationRad = degToRad(elevationDeg);
  const float cosElevation = cosf(elevationRad);
  return normalizeVec3({
      cosElevation * cosf(bearingRad),
      cosElevation * sinf(bearingRad),
      sinf(elevationRad),
  });
}

int angleToPulseUs(float angleDeg, float minDeg, float maxDeg, int minPulseUs, int maxPulseUs) {
  const float clampedAngle = constrain(angleDeg, minDeg, maxDeg);
  const float normalized = (clampedAngle - minDeg) / (maxDeg - minDeg);
  const float pulseSpan = static_cast<float>(maxPulseUs - minPulseUs);
  return minPulseUs + static_cast<int>(lroundf(normalized * pulseSpan));
}

float tiltModelDegToServoDeg(float tiltModelDeg) {
  return constrain(tiltModelDeg + TILT_SERVO_OFFSET_DEG, TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG);
}

float tiltServoDegToModelDeg(float tiltServoDeg) {
  return tiltServoDeg - TILT_SERVO_OFFSET_DEG;
}

int quantizePulseUs(int pulseUs, int minPulseUs, int maxPulseUs) {
  if (SERVO_COMMAND_STEP_US <= 1) {
    return constrain(pulseUs, minPulseUs, maxPulseUs);
  }

  const int offset = pulseUs - minPulseUs;
  const int quantizedOffset =
      static_cast<int>(lroundf(static_cast<float>(offset) / SERVO_COMMAND_STEP_US)) *
      SERVO_COMMAND_STEP_US;
  return constrain(minPulseUs + quantizedOffset, minPulseUs, maxPulseUs);
}

float stepToward(float current, float target, float maxStep) {
  if (target > current) {
    return min(current + maxStep, target);
  }
  if (target < current) {
    return max(current - maxStep, target);
  }
  return current;
}

void waitForSerialIfEnabled() {
  if (!WAIT_FOR_SERIAL) {
    return;
  }

  const uint32_t startMs = millis();
  while (!Serial && (millis() - startMs) < WAIT_FOR_SERIAL_TIMEOUT_MS) {
    delay(10);
  }
}

String formatMac(const uint8_t* mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

String mqttTopic(const char* suffix) {
  String topic = REMOTE_MQTT_BASE_TOPIC_VALUE;
  topic += "/";
  topic += suffix;
  return topic;
}

void beginStatusLed() {
  if (!STATUS_LED_ENABLED) {
    return;
  }

  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.show();
}

void showLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  currentRed = red;
  currentGreen = green;
  currentBlue = blue;

  if (!STATUS_LED_ENABLED) {
    return;
  }

  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

void setStatusLedBlue() {
  showLedColor(0, 0, 255);
}

void setStatusLedGreen() {
  showLedColor(0, 255, 0);
}

void updateDisplayedColor(uint32_t nowMs) {
  if (!blinkActive) {
    showLedColor(baseRed, baseGreen, baseBlue);
    return;
  }

  const bool blinkOn = ((nowMs / BLINK_PERIOD_MS) % 2U) == 0U;
  if (blinkOn) {
    showLedColor(baseRed, baseGreen, baseBlue);
  } else {
    showLedColor(0, 0, 0);
  }
}

void computeBaseColorFromPanTilt() {
  const float panNorm = (panAngleDeg - PAN_MIN_DEG) / (PAN_MAX_DEG - PAN_MIN_DEG);
  const float tiltNorm = (tiltAngleDeg - TILT_MIN_DEG) / (TILT_MAX_DEG - TILT_MIN_DEG);

  const float clampedPan = constrain(panNorm, 0.0f, 1.0f);
  const float clampedTilt = constrain(tiltNorm, 0.0f, 1.0f);

  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  if (clampedPan <= 0.5f) {
    const float blend = clampedPan / 0.5f;
    red = static_cast<uint8_t>((1.0f - blend) * 255.0f);
    green = static_cast<uint8_t>(blend * 255.0f);
  } else {
    const float blend = (clampedPan - 0.5f) / 0.5f;
    green = static_cast<uint8_t>((1.0f - blend) * 255.0f);
    blue = static_cast<uint8_t>(blend * 255.0f);
  }

  if (clampedTilt <= 0.5f) {
    const float dim = clampedTilt / 0.5f;
    red = static_cast<uint8_t>(red * dim);
    green = static_cast<uint8_t>(green * dim);
    blue = static_cast<uint8_t>(blue * dim);
  } else {
    const float whiten = (clampedTilt - 0.5f) / 0.5f;
    red = static_cast<uint8_t>(red + ((255.0f - red) * whiten));
    green = static_cast<uint8_t>(green + ((255.0f - green) * whiten));
    blue = static_cast<uint8_t>(blue + ((255.0f - blue) * whiten));
  }

  baseRed = red;
  baseGreen = green;
  baseBlue = blue;
}

void applyLedFromPanTilt(uint32_t nowMs) {
  computeBaseColorFromPanTilt();
  updateDisplayedColor(nowMs);
}

void setupEspNowWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

void ensureEspNow() {
  if (esp_now_init() == ESP_OK) {
    return;
  }

  Serial.println("ESP-NOW init failed, restarting.");
  delay(500);
  ESP.restart();
}

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  const esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
    return true;
  }

  Serial.printf("ESP-NOW add peer failed for %s: %d\n",
                formatMac(mac).c_str(), static_cast<int>(result));
  return false;
}

void ensureBroadcastPeer() {
  broadcastPeerReady = addPeer(ESPNOW_BROADCAST_MAC);
}

void setHeliostatUnixTimeUtc(time_t unixTimeUtc) {
  heliostatStartUnixTimeUtc = unixTimeUtc;
  heliostatTimeBaseMillis = millis();
}

#if defined(DEVICE_ROLE_CONTROLLER)
void setControllerHeliostatUnixTimeUtc(time_t unixTimeUtc) {
  setHeliostatUnixTimeUtc(unixTimeUtc);
  timeUpdatePending = true;
}

void setControllerHeliostatTimeScale(float scale) {
  heliostatTimeScale = max(scale, 0.01f);
  timeScaleUpdatePending = true;
}
#endif

#if defined(DEVICE_ROLE_REMOTE)
int lastPanPulseUs = 0;
int lastTiltPulseUs = 0;

float panMinLimitDeg();
float panMaxLimitDeg();
float tiltMinLimitDeg();
float tiltMaxLimitDeg();

const char* remoteActuatorBackendName() {
  switch (REMOTE_ACTUATOR_BACKEND) {
    case REMOTE_ACTUATOR_BACKEND_PWM:
      return "pwm";
    case REMOTE_ACTUATOR_BACKEND_ST3020:
      return "st3020";
    default:
      return "unknown";
  }
}

float panMinLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_PAN_MIN_DEG : PAN_MIN_DEG;
}

float panMaxLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_PAN_MAX_DEG : PAN_MAX_DEG;
}

float tiltMinLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_TILT_MIN_DEG : TILT_MIN_DEG;
}

float tiltMaxLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_TILT_MAX_DEG : TILT_MAX_DEG;
}

float tiltExternalFromInternalDeg(float internalDeg) {
  if (REMOTE_ACTUATOR_BACKEND != REMOTE_ACTUATOR_BACKEND_ST3020) {
    return internalDeg;
  }
  return 180.0f - internalDeg;
}

float tiltInternalFromExternalDeg(float externalDeg) {
  if (REMOTE_ACTUATOR_BACKEND != REMOTE_ACTUATOR_BACKEND_ST3020) {
    return externalDeg;
  }
  const float clampedExternal =
      constrain(externalDeg, ST3020_TILT_EXTERNAL_MIN_DEG, ST3020_TILT_EXTERNAL_MAX_DEG);
  return constrain(180.0f - clampedExternal, tiltMinLimitDeg(), tiltMaxLimitDeg());
}

float tiltExternalMinLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_TILT_EXTERNAL_MIN_DEG : tiltMinLimitDeg();
}

float tiltExternalMaxLimitDeg() {
  return (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) ? ST3020_TILT_EXTERNAL_MAX_DEG : tiltMaxLimitDeg();
}

int panAngleDegToLegacyPulseUs(float panDeg) {
  return quantizePulseUs(
      angleToPulseUs(panDeg, PAN_MIN_DEG, PAN_MAX_DEG,
                     PAN_SERVO_MIN_PULSE_US, PAN_SERVO_MAX_PULSE_US),
      PAN_SERVO_MIN_PULSE_US, PAN_SERVO_MAX_PULSE_US);
}

int tiltAngleDegToLegacyPulseUs(float tiltDeg) {
  return quantizePulseUs(
      angleToPulseUs(tiltModelDegToServoDeg(tiltDeg), TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG,
                     TILT_SERVO_MIN_PULSE_US, TILT_SERVO_MAX_PULSE_US),
      TILT_SERVO_MIN_PULSE_US, TILT_SERVO_MAX_PULSE_US);
}

int st3020PanPositionFromAngleDeg(float angleDeg) {
  const float clamped = constrain(angleDeg, ST3020_PAN_MIN_DEG, ST3020_PAN_MAX_DEG);
  const float stepsPerDeg =
      static_cast<float>(ST3020_POS_AT_180_DEG - ST3020_POS_AT_90_DEG) / 90.0f;
  const float signedOffsetDeg = (clamped - 90.0f) * ST3020_PAN_SIGN;
  return constrain(
      ST3020_POS_AT_90_DEG + static_cast<int>(lroundf(signedOffsetDeg * stepsPerDeg)),
      ST3020_PAN_POS_MIN, ST3020_PAN_POS_MAX);
}

int st3020TiltPositionFromAngleDeg(float angleDeg) {
  const float clamped = constrain(angleDeg, ST3020_TILT_MIN_DEG, ST3020_TILT_MAX_DEG);
  const float stepsPerDeg =
      static_cast<float>(ST3020_POS_AT_180_DEG - ST3020_POS_AT_90_DEG) / 90.0f;
  const float signedOffsetDeg = (clamped - 90.0f) * ST3020_TILT_SIGN;
  return constrain(
      ST3020_POS_AT_90_DEG + static_cast<int>(lroundf(signedOffsetDeg * stepsPerDeg)),
      ST3020_TILT_POS_MIN, ST3020_TILT_POS_MAX);
}

void beginRemoteActuatorsSt3020() {
  Serial1.begin(ST3020_UART_BAUD, SERIAL_8N1, ST3020_UART_RX_PIN, ST3020_UART_TX_PIN);
  st3020Bus.pSerial = &Serial1;
  st3020Bus.IOTimeOut = 100;

  Serial.printf("ST3020 bus: RX=%d TX=%d baud=%lu | pan_id=%u tilt_id=%u\n",
                ST3020_UART_RX_PIN, ST3020_UART_TX_PIN,
                static_cast<unsigned long>(ST3020_UART_BAUD),
                static_cast<unsigned>(ST3020_PAN_ID),
                static_cast<unsigned>(ST3020_TILT_ID));
  Serial.printf("ST3020 map: 0deg=%d 90deg=%d 180deg=%d\n",
                ST3020_POS_AT_0_DEG, ST3020_POS_AT_90_DEG, ST3020_POS_AT_180_DEG);

  const int panPing = st3020Bus.Ping(ST3020_PAN_ID);
  const int tiltPing = st3020Bus.Ping(ST3020_TILT_ID);
  Serial.printf("ST3020 ping: pan=%s tilt=%s\n",
                panPing >= 0 ? "ok" : "missing",
                tiltPing >= 0 ? "ok" : "missing");

  const int panTorque = st3020Bus.EnableTorque(ST3020_PAN_ID, 0);
  const int tiltTorque = st3020Bus.EnableTorque(ST3020_TILT_ID, 0);
  Serial.printf("ST3020 torque enable: pan=%d tilt=%d\n", panTorque, tiltTorque);

  const int panFeedback = st3020Bus.FeedBack(ST3020_PAN_ID);
  const int panPosition = panFeedback >= 0 ? st3020Bus.ReadPos(-1) : -1;
  const int tiltFeedback = st3020Bus.FeedBack(ST3020_TILT_ID);
  const int tiltPosition = tiltFeedback >= 0 ? st3020Bus.ReadPos(-1) : -1;
  Serial.printf("ST3020 feedback: pan=%d tilt=%d\n", panPosition, tiltPosition);
}

void writeRemoteActuatorsSt3020() {
  const int panPosition = st3020PanPositionFromAngleDeg(panAngleDeg);
  const int tiltPosition = st3020TiltPositionFromAngleDeg(tiltAngleDeg);
  lastPanPulseUs = panPosition;
  lastTiltPulseUs = tiltPosition;
  const int panResult =
      st3020Bus.WritePosEx(ST3020_PAN_ID, static_cast<s16>(panPosition), ST3020_DEFAULT_SPEED, ST3020_DEFAULT_ACC);
  const int tiltResult =
      st3020Bus.WritePosEx(ST3020_TILT_ID, static_cast<s16>(tiltPosition), ST3020_DEFAULT_SPEED, ST3020_DEFAULT_ACC);
  static uint32_t lastSt3020LogMs = 0;
  const uint32_t nowMs = millis();
  if ((nowMs - lastSt3020LogMs) >= 1000) {
    lastSt3020LogMs = nowMs;
    Serial.printf("ST3020 write: pan=%d r=%d | tilt=%d r=%d\n",
                  panPosition, panResult, tiltPosition, tiltResult);
  }
}

void writeRemoteActuatorsPwm() {
  lastPanPulseUs = panAngleDegToLegacyPulseUs(panAngleDeg);
  lastTiltPulseUs = tiltAngleDegToLegacyPulseUs(tiltAngleDeg);
  panServo.writeMicroseconds(lastPanPulseUs);
  tiltServo.writeMicroseconds(lastTiltPulseUs);
}

void beginRemoteActuatorsPwm() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(SERVO_FREQUENCY_HZ);
  tiltServo.setPeriodHertz(SERVO_FREQUENCY_HZ);
  panServo.attach(PAN_SERVO_PIN, PAN_SERVO_MIN_PULSE_US, PAN_SERVO_MAX_PULSE_US);
  tiltServo.attach(TILT_SERVO_PIN, TILT_SERVO_MIN_PULSE_US, TILT_SERVO_MAX_PULSE_US);
  writeRemoteActuatorsPwm();
  delay(250);
}

void writeRemoteActuators() {
  switch (REMOTE_ACTUATOR_BACKEND) {
    case REMOTE_ACTUATOR_BACKEND_PWM:
      writeRemoteActuatorsPwm();
      return;
    case REMOTE_ACTUATOR_BACKEND_ST3020:
      writeRemoteActuatorsSt3020();
      return;
    default:
      return;
  }
}

void beginRemoteActuators() {
  switch (REMOTE_ACTUATOR_BACKEND) {
    case REMOTE_ACTUATOR_BACKEND_PWM:
      beginRemoteActuatorsPwm();
      return;
    case REMOTE_ACTUATOR_BACKEND_ST3020:
      beginRemoteActuatorsSt3020();
      writeRemoteActuatorsSt3020();
      return;
    default:
      return;
  }
}

void markRemoteCommandReceived() {
  packetReceived = true;
  lastRxMs = millis();
}

String buildRemoteMqttClientId() {
  return String("heliostat-remote-") + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
}

void printRemoteTrackingDiagnostic() {
  time_t currentUnixTime = 0;
  if (!currentUnixTimeUtc(currentUnixTime)) {
    currentUnixTime = 0;
  }
  const double driftSeconds =
      (currentUnixTime > 0 && autoTrackStartUnixTimeUtc > 0)
          ? difftime(currentUnixTime, autoTrackStartUnixTimeUtc)
          : -1.0;
  Serial.printf(
      "TRACK_DIAG | mode=%s | now_utc=%s | auto_start_utc=%s | drift_s=%.0f | auto_ref_pan=%.3f | actual_pan=%.3f | delta_pan=%+.3f | auto_ref_tilt=%.3f | actual_tilt=%.3f | delta_tilt=%+.3f | target=%s | sun_time=%s\n",
      controlModeName(controlMode),
      formatUnixTimeUtc(currentUnixTime).c_str(),
      formatUnixTimeUtc(autoTrackStartUnixTimeUtc).c_str(),
      driftSeconds,
      lastAutoReferencePanDeg,
      panAngleDeg,
      panAngleDeg - lastAutoReferencePanDeg,
      lastAutoReferenceTiltDeg,
      tiltAngleDeg,
      tiltAngleDeg - lastAutoReferenceTiltDeg,
      targetDirectionValid ? "ok" : "missing",
      sunTimeValid ? "ok" : "missing");
}

void publishRemoteAvailability(const char* state) {
  if (!remoteMqttClient.connected()) {
    return;
  }
  const String topic = mqttTopic("availability");
  remoteMqttClient.publish(topic.c_str(), state, true);
}

void publishRemoteDiag(const char* event) {
  if (!remoteMqttClient.connected()) {
    return;
  }

  JsonDocument doc;
  doc["event"] = event;
  doc["mode"] = controlModeName(controlMode);
  doc["scan_stage"] = scanStageName(scanStage);
  doc["pan_deg"] = panAngleDeg;
  doc["tilt_deg"] = tiltExternalFromInternalDeg(tiltAngleDeg);
  time_t unixTimeUtc = 0;
  if (currentUnixTimeUtc(unixTimeUtc)) {
    doc["remote_utc"] = static_cast<int64_t>(unixTimeUtc);
  }

  String payload;
  serializeJson(doc, payload);
  const String topic = mqttTopic("diag");
  if (!remoteMqttClient.publish(topic.c_str(), payload.c_str(), false)) {
    Serial.printf("MQTT diag publish failed: topic=%s bytes=%u\n",
                  topic.c_str(), static_cast<unsigned>(payload.length()));
  }
}

void publishRemoteState(bool force = false) {
  const uint32_t nowMs = millis();
  if (!force && (nowMs - lastRemoteStatePublishMs) < REMOTE_STATE_PUBLISH_MS) {
    return;
  }
  if (!remoteMqttClient.connected()) {
    return;
  }

  lastRemoteStatePublishMs = nowMs;

  JsonDocument doc;
  doc["mode"] = controlModeName(controlMode);
  doc["pan_deg"] = panAngleDeg;
  doc["tilt_deg"] = tiltExternalFromInternalDeg(tiltAngleDeg);
  doc["pan_target_deg"] = panTargetDeg;
  doc["tilt_target_deg"] = tiltExternalFromInternalDeg(tiltTargetDeg);
  doc["pan_min_deg"] = panMinLimitDeg();
  doc["pan_max_deg"] = panMaxLimitDeg();
  doc["tilt_min_deg"] = tiltExternalMinLimitDeg();
  doc["tilt_max_deg"] = tiltExternalMaxLimitDeg();
  doc["pan_us"] = lastPanPulseUs;
  doc["tilt_us"] = lastTiltPulseUs;
  doc["site_latitude_deg"] = heliostatLatitudeDeg;
  doc["site_longitude_deg"] = heliostatLongitudeDeg;
  doc["scan_active"] = scanActive;
  doc["scan_paused"] = scanPaused;
  doc["scan_stage"] = scanStageName(scanStage);
  doc["scan_direction"] = (scanDirection >= 0) ? "forward" : "backward";
  doc["scan_center_pan_deg"] = scanCenterPanDeg;
  doc["scan_center_tilt_deg"] = tiltExternalFromInternalDeg(scanCenterTiltDeg);
  doc["scan_range_pan_deg"] = scanRangePanDeg;
  doc["scan_range_tilt_deg"] = scanRangeTiltDeg;
  doc["scan_step_deg"] = scanStepDeg;
  doc["scan_dwell_ms"] = scanDwellMs;
  doc["scan_move_speed_deg_per_sec"] = scanMoveSpeedDegPerSec;
  doc["scan_point_index"] = scanPointIndex;
  doc["scan_points_total"] = scanPointsTotal;
  doc["scan_lock_valid"] = scanLockValid;
  doc["scan_lock_pan_deg"] = scanLockPanDeg;
  doc["scan_lock_tilt_deg"] = tiltExternalFromInternalDeg(scanLockTiltDeg);
  doc["approx_target_valid"] = approxTargetValid;
  doc["approx_target_bearing_deg"] = approxTargetBearingDeg;
  doc["approx_target_elevation_deg"] = approxTargetElevationDeg;
  doc["approx_target_pan_deg"] = approxTargetPanDeg;
  doc["approx_target_tilt_deg"] = tiltExternalFromInternalDeg(approxTargetTiltDeg);
  doc["sun_time_ok"] = sunTimeValid;
  doc["target_ok"] = targetDirectionValid;
  doc["auto_enabled"] = autoTrackEnabled;
  doc["wifi_ok"] = wifiLinkOk;
  doc["mqtt_ok"] = mqttLinkOk;
  doc["ntp_ok"] = ntpTimeValid;
  doc["time_source"] = ntpTimeValid ? "ntp" : (heliostatStartUnixTimeUtc > 0 ? "mqtt" : "missing");
  time_t unixTimeUtc = 0;
  if (currentUnixTimeUtc(unixTimeUtc)) {
    doc["remote_utc"] = static_cast<int64_t>(unixTimeUtc);
  }

  String payload;
  serializeJson(doc, payload);
  const String topic = mqttTopic("state");
  if (!remoteMqttClient.publish(topic.c_str(), payload.c_str(), true)) {
    Serial.printf("MQTT state publish failed: topic=%s bytes=%u\n",
                  topic.c_str(), static_cast<unsigned>(payload.length()));
  }
}

void stopScan() {
  scanActive = false;
  scanStage = SCAN_STAGE_IDLE;
  scanPaused = false;
  scanDirection = 1;
  scanPointReachedMs = 0;
}

void pauseScan() {
  scanActive = false;
  scanPaused = true;
  scanPointReachedMs = 0;
}

uint16_t scanIndexFromGrid(uint16_t row, uint16_t col) {
  if (scanGridCols == 0 || scanGridRows == 0) {
    return 0;
  }
  if ((row % 2u) == 1u) {
    col = static_cast<uint16_t>((scanGridCols - 1u) - col);
  }
  return static_cast<uint16_t>((row * scanGridCols) + col);
}

uint16_t nearestScanIndexFromCurrentPosition() {
  if (scanGridCols == 0 || scanGridRows == 0 || scanStepDeg <= 0.0f) {
    return 0;
  }

  const float colFloat = (panAngleDeg - (scanCenterPanDeg - scanRangePanDeg)) / scanStepDeg;
  const float rowFloat = ((scanCenterTiltDeg + scanRangeTiltDeg) - tiltAngleDeg) / scanStepDeg;
  const uint16_t col = static_cast<uint16_t>(constrain(lroundf(colFloat), 0L, static_cast<long>(scanGridCols - 1u)));
  const uint16_t row = static_cast<uint16_t>(constrain(lroundf(rowFloat), 0L, static_cast<long>(scanGridRows - 1u)));
  return scanIndexFromGrid(row, col);
}

void setScanTargetForIndex(uint16_t index) {
  if (scanGridCols == 0 || scanGridRows == 0 || index >= scanPointsTotal) {
    return;
  }

  const uint16_t row = index / scanGridCols;
  uint16_t col = index % scanGridCols;
  if ((row % 2u) == 1u) {
    col = static_cast<uint16_t>((scanGridCols - 1u) - col);
  }

  const float pan = scanCenterPanDeg - scanRangePanDeg + (static_cast<float>(col) * scanStepDeg);
  const float tilt = scanCenterTiltDeg + scanRangeTiltDeg - (static_cast<float>(row) * scanStepDeg);
  panTargetDeg = constrain(pan, panMinLimitDeg(), panMaxLimitDeg());
  tiltTargetDeg = constrain(tilt, tiltMinLimitDeg(), tiltMaxLimitDeg());
}

void startScan(ScanStage stage,
               float centerPanDeg,
               float centerTiltDeg,
               uint32_t dwellMsOverride,
               float moveSpeedOverrideDegPerSec = -1.0f,
               float rangePanOverrideDeg = -1.0f,
               float rangeTiltOverrideDeg = -1.0f,
               int8_t direction = 1) {
  autoTrackEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
  remotePanInput = 0.0f;
  remoteTiltInput = 0.0f;
  precisionManualMode = false;

  scanStage = stage;
  scanActive = true;
  scanPaused = false;
  scanDirection = (direction < 0) ? -1 : 1;
  scanCenterPanDeg = constrain(centerPanDeg, panMinLimitDeg(), panMaxLimitDeg());
  scanCenterTiltDeg = constrain(centerTiltDeg, tiltMinLimitDeg(), tiltMaxLimitDeg());
  if (stage == SCAN_STAGE_COARSE) {
    scanRangePanDeg = SCAN_COARSE_RANGE_PAN_DEG;
    scanRangeTiltDeg = SCAN_COARSE_RANGE_TILT_DEG;
    scanStepDeg = SCAN_COARSE_STEP_DEG;
    scanDwellMs = SCAN_COARSE_DWELL_MS;
  } else if (stage == SCAN_STAGE_FINE) {
    scanRangePanDeg = SCAN_FINE_RANGE_PAN_DEG;
    scanRangeTiltDeg = SCAN_FINE_RANGE_TILT_DEG;
    scanStepDeg = SCAN_FINE_STEP_DEG;
    scanDwellMs = SCAN_FINE_DWELL_MS;
  } else {
    scanRangePanDeg = SCAN_MICRO_RANGE_PAN_DEG;
    scanRangeTiltDeg = SCAN_MICRO_RANGE_TILT_DEG;
    scanStepDeg = SCAN_MICRO_STEP_DEG;
    scanDwellMs = SCAN_MICRO_DWELL_MS;
  }

  if (dwellMsOverride > 0) {
    scanDwellMs = dwellMsOverride;
  }
  scanMoveSpeedDegPerSec = SCAN_MOVE_SPEED_DEFAULT_DEG_PER_SEC;
  if (moveSpeedOverrideDegPerSec > 0.0f) {
    scanMoveSpeedDegPerSec = moveSpeedOverrideDegPerSec;
  }
  if (rangePanOverrideDeg > 0.0f) {
    scanRangePanDeg = rangePanOverrideDeg;
  }
  if (rangeTiltOverrideDeg > 0.0f) {
    scanRangeTiltDeg = rangeTiltOverrideDeg;
  }

  scanGridCols = static_cast<uint16_t>(lroundf((scanRangePanDeg * 2.0f) / scanStepDeg)) + 1u;
  scanGridRows = static_cast<uint16_t>(lroundf((scanRangeTiltDeg * 2.0f) / scanStepDeg)) + 1u;
  scanPointsTotal = static_cast<uint16_t>(scanGridCols * scanGridRows);
  scanPointIndex = 0;
  scanPointReachedMs = 0;
  setScanTargetForIndex(scanPointIndex);
}

void handleRemoteScanCommand(const JsonDocument& doc) {
  const char* stage = doc["stage"] | "";
  if (strcmp(stage, "off") == 0) {
    panTargetDeg = panAngleDeg;
    tiltTargetDeg = tiltAngleDeg;
    pauseScan();
    return;
  }

  if (strcmp(stage, "resume") == 0) {
    if (scanStage != SCAN_STAGE_IDLE && scanPointsTotal > 0) {
      const char* directionString = doc["direction"] | "forward";
      scanDirection = (strcmp(directionString, "backward") == 0) ? -1 : 1;
      scanPointIndex = nearestScanIndexFromCurrentPosition();
      scanActive = true;
      scanPaused = false;
      scanPointReachedMs = 0;
      setScanTargetForIndex(scanPointIndex);
    }
    return;
  }

  const float defaultCenterPan = scanLockValid ? scanLockPanDeg : panTargetDeg;
  const float defaultCenterTilt = scanLockValid ? scanLockTiltDeg : tiltTargetDeg;
  const float centerPan = doc["center_pan_deg"] | defaultCenterPan;
  const float centerTiltExternal = doc["center_tilt_deg"] | tiltExternalFromInternalDeg(defaultCenterTilt);
  const uint32_t dwellMs = doc["dwell_ms"] | 0;
  const float moveSpeedDegPerSec = doc["move_speed_deg_per_sec"] | -1.0f;
  const char* directionString = doc["direction"] | "forward";
  const int8_t direction = (strcmp(directionString, "backward") == 0) ? -1 : 1;
  const float rangePanDeg = doc["range_pan_deg"] | -1.0f;
  const float rangeTiltDeg = doc["range_tilt_deg"] | -1.0f;

  if (strcmp(stage, "coarse") == 0) {
    startScan(SCAN_STAGE_COARSE, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), dwellMs, moveSpeedDegPerSec, rangePanDeg, rangeTiltDeg, direction);
  } else if (strcmp(stage, "fine") == 0) {
    startScan(SCAN_STAGE_FINE, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), dwellMs, moveSpeedDegPerSec, rangePanDeg, rangeTiltDeg, direction);
  } else if (strcmp(stage, "micro") == 0) {
    startScan(SCAN_STAGE_MICRO, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), dwellMs, moveSpeedDegPerSec, rangePanDeg, rangeTiltDeg, direction);
  }
}

void handleRemoteModeCommand(const JsonDocument& doc) {
  const char* mode = doc["mode"] | "";
  if (strcmp(mode, "manual") == 0) {
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
  } else if (strcmp(mode, "captured") == 0) {
    autoTrackEnabled = false;
    if (targetDirectionValid) {
      controlMode = CONTROL_MODE_TARGET_CAPTURED;
    }
  } else if (strcmp(mode, "auto") == 0) {
    if (targetDirectionValid) {
      autoTrackEnabled = true;
    }
  }
}

void handleRemoteManualCommand(const JsonDocument& doc) {
  stopScan();
  remotePanInput = constrain(doc["pan_rate"] | 0.0f, -1.0f, 1.0f);
  remoteTiltInput = constrain(doc["tilt_rate"] | 0.0f, -1.0f, 1.0f);
  precisionManualMode = doc["precision"] | false;
  autoTrackEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
}

void handleRemoteJogCommand(const JsonDocument& doc) {
  const char* axis = doc["axis"] | "";
  const float deltaDeg = doc["delta_deg"] | 0.0f;
  if (fabsf(deltaDeg) <= 1e-4f) {
    return;
  }

  stopScan();
  autoTrackEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
  remotePanInput = 0.0f;
  remoteTiltInput = 0.0f;
  precisionManualMode = false;

  if (strcmp(axis, "pan") == 0) {
    panTargetDeg = constrain(panTargetDeg + deltaDeg, panMinLimitDeg(), panMaxLimitDeg());
  } else if (strcmp(axis, "tilt") == 0) {
    tiltTargetDeg = constrain(tiltTargetDeg + deltaDeg, tiltMinLimitDeg(), tiltMaxLimitDeg());
  }
}

void handleRemoteMoveToCommand(const JsonDocument& doc) {
  stopScan();
  autoTrackEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
  remotePanInput = 0.0f;
  remoteTiltInput = 0.0f;
  precisionManualMode = false;
  panTargetDeg = constrain(doc["pan_deg"] | panTargetDeg, panMinLimitDeg(), panMaxLimitDeg());
  tiltTargetDeg = tiltInternalFromExternalDeg(doc["tilt_deg"] | tiltExternalFromInternalDeg(tiltTargetDeg));
}

void handleRemoteApproxTargetCommand(const JsonDocument& doc) {
  time_t unixTimeUtc = 0;
  if (!currentUnixTimeUtc(unixTimeUtc)) {
    approxTargetValid = false;
    publishRemoteDiag("approx_target_missing_time");
    return;
  }

  const float bearingDeg = fmodf((doc["bearing_deg"] | 180.0f) + 360.0f, 360.0f);
  const float elevationDeg = constrain(doc["elevation_deg"] | 0.0f, -10.0f, 90.0f);
  const Vec3 sunDirection = sunVectorFromUnixTime(unixTimeUtc);
  const Vec3 targetDirectionApprox = vectorFromBearingElevation(bearingDeg, elevationDeg);
  Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirectionApprox));
  if (lengthVec3(desiredNormal) <= 0.0f) {
    approxTargetValid = false;
    publishRemoteDiag("approx_target_invalid");
    return;
  }

  const Vec3 currentNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);
  if (dotVec3(currentNormal, desiredNormal) < 0.0f) {
    desiredNormal = scaleVec3(desiredNormal, -1.0f);
  }

  stopScan();
  autoTrackEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
  remotePanInput = 0.0f;
  remoteTiltInput = 0.0f;
  precisionManualMode = false;

  panTiltFromMirrorNormal(desiredNormal, approxTargetPanDeg, approxTargetTiltDeg);
  approxTargetBearingDeg = bearingDeg;
  approxTargetElevationDeg = elevationDeg;
  approxTargetValid = true;
  panTargetDeg = approxTargetPanDeg;
  tiltTargetDeg = approxTargetTiltDeg;
  publishRemoteDiag("approx_target_applied");
}

void handleRemoteActionCommand(const JsonDocument& doc) {
  const char* action = doc["action"] | "";
  if (strcmp(action, "capture_target") == 0) {
    captureTargetRequested = true;
  } else if (strcmp(action, "recenter") == 0) {
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    panTargetDeg = PAN_START_DEG;
    tiltTargetDeg = TILT_START_DEG;
  } else if (strcmp(action, "beam_seen") == 0) {
    scanLockValid = true;
    scanLockPanDeg = panAngleDeg;
    scanLockTiltDeg = tiltAngleDeg;
    panTargetDeg = panAngleDeg;
    tiltTargetDeg = tiltAngleDeg;
    stopScan();
    publishRemoteDiag("beam_seen");
  } else if (strcmp(action, "print_diag") == 0) {
    printRemoteTrackingDiagnostic();
    publishRemoteDiag("print_diag");
  }
}

void handleRemoteTimeCommand(const JsonDocument& doc) {
  const int64_t unixTimeUtc = doc["unix_utc"] | 0;
  const float timeScale = doc["time_scale"] | 1.0f;
  if (unixTimeUtc > 0) {
    setHeliostatUnixTimeUtc(static_cast<time_t>(unixTimeUtc));
    ntpTimeValid = false;
  }
  heliostatTimeScale = max(timeScale, 0.01f);
}

void handleRemoteLocationCommand(const JsonDocument& doc) {
  heliostatLatitudeDeg = constrain(doc["latitude_deg"] | heliostatLatitudeDeg, -90.0f, 90.0f);
  heliostatLongitudeDeg = fmodf((doc["longitude_deg"] | heliostatLongitudeDeg) + 540.0f, 360.0f) - 180.0f;
  publishRemoteDiag("location_updated");
}

void onRemoteMqttMessage(char* topic, uint8_t* payloadBytes, unsigned int length) {
  String payload;
  payload.reserve(length);
  for (unsigned int i = 0; i < length; ++i) {
    payload += static_cast<char>(payloadBytes[i]);
  }

  JsonDocument doc;
  const auto error = deserializeJson(doc, payload);
  if (error) {
    Serial.printf("MQTT invalid JSON on %s: %s\n", topic, error.c_str());
    return;
  }

  const String topicString(topic);
  if (topicString == mqttTopic("cmd/mode")) {
    handleRemoteModeCommand(doc);
  } else if (topicString == mqttTopic("cmd/jog")) {
    handleRemoteJogCommand(doc);
  } else if (topicString == mqttTopic("cmd/move_to")) {
    handleRemoteMoveToCommand(doc);
  } else if (topicString == mqttTopic("cmd/approx_target")) {
    handleRemoteApproxTargetCommand(doc);
  } else if (topicString == mqttTopic("cmd/scan")) {
    handleRemoteScanCommand(doc);
  } else if (topicString == mqttTopic("cmd/manual")) {
    handleRemoteManualCommand(doc);
  } else if (topicString == mqttTopic("cmd/action")) {
    handleRemoteActionCommand(doc);
  } else if (topicString == mqttTopic("cmd/time")) {
    handleRemoteTimeCommand(doc);
  } else if (topicString == mqttTopic("cmd/location")) {
    handleRemoteLocationCommand(doc);
  }

  markRemoteCommandReceived();
  publishRemoteState(true);
}

void ensureRemoteWifiConnected(uint32_t nowMs) {
  wifiLinkOk = (WiFi.status() == WL_CONNECTED);
  if (wifiLinkOk) {
    return;
  }
  if ((nowMs - lastWifiConnectAttemptMs) < REMOTE_WIFI_RETRY_MS) {
    return;
  }

  lastWifiConnectAttemptMs = nowMs;
  Serial.printf("WiFi connect: ssid=%s\n", REMOTE_WIFI_SSID_VALUE);
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(REMOTE_WIFI_SSID_VALUE, REMOTE_WIFI_PASSWORD_VALUE);
}

void ensureRemoteNtpTime(uint32_t nowMs) {
  if (!wifiLinkOk) {
    ntpTimeValid = false;
    return;
  }

  if (!ntpConfigured) {
    configTime(0, 0, REMOTE_NTP_SERVER_1, REMOTE_NTP_SERVER_2, REMOTE_NTP_SERVER_3);
    ntpConfigured = true;
    lastNtpSyncAttemptMs = nowMs;
  }

  time_t ntpUnixTime = 0;
  if (systemClockTimeValid(ntpUnixTime)) {
    ntpTimeValid = true;
    if (fabsf(heliostatTimeScale - 1.0f) < 0.001f &&
        ((nowMs - lastNtpApplyMs) >= REMOTE_NTP_RESYNC_MS || heliostatStartUnixTimeUtc <= 0)) {
      setHeliostatUnixTimeUtc(ntpUnixTime);
      lastNtpApplyMs = nowMs;
    }
    return;
  }

  ntpTimeValid = false;
  if ((nowMs - lastNtpSyncAttemptMs) >= REMOTE_NTP_RETRY_MS) {
    configTime(0, 0, REMOTE_NTP_SERVER_1, REMOTE_NTP_SERVER_2, REMOTE_NTP_SERVER_3);
    lastNtpSyncAttemptMs = nowMs;
  }
}

void ensureRemoteMqttConnected(uint32_t nowMs) {
  wifiLinkOk = (WiFi.status() == WL_CONNECTED);
  mqttLinkOk = remoteMqttClient.connected();
  if (!wifiLinkOk) {
    if (mqttLinkOk) {
      remoteMqttClient.disconnect();
      mqttLinkOk = false;
    }
    return;
  }

  if (mqttLinkOk) {
    return;
  }

  if ((nowMs - lastMqttConnectAttemptMs) < REMOTE_MQTT_RETRY_MS) {
    return;
  }

  lastMqttConnectAttemptMs = nowMs;
  const String willTopic = mqttTopic("availability");
  const String clientId = buildRemoteMqttClientId();
  Serial.printf("MQTT connect: %s:%u client=%s\n",
                REMOTE_MQTT_HOST_VALUE, static_cast<unsigned>(REMOTE_MQTT_PORT_VALUE), clientId.c_str());
  if (!remoteMqttClient.connect(clientId.c_str(), willTopic.c_str(), 1, true, "offline")) {
    Serial.printf("MQTT connect failed, state=%d\n", remoteMqttClient.state());
    mqttLinkOk = false;
    return;
  }

  mqttLinkOk = true;
  remoteMqttClient.subscribe(mqttTopic("cmd/mode").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/jog").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/move_to").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/approx_target").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/scan").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/manual").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/action").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/time").c_str());
  remoteMqttClient.subscribe(mqttTopic("cmd/location").c_str());
  publishRemoteAvailability("online");
  publishRemoteState(true);
  publishRemoteDiag("mqtt_connected");
}

void updateHeliostatTracking(uint32_t nowMs) {
  (void)nowMs;

  time_t unixTimeUtc = 0;
  sunTimeValid = currentUnixTimeUtc(unixTimeUtc);
  if (!sunTimeValid) {
    autoTrackEnabled = false;
    if (controlMode == CONTROL_MODE_AUTO_TRACK) {
      controlMode = targetDirectionValid ? CONTROL_MODE_TARGET_CAPTURED : CONTROL_MODE_MANUAL;
    }
    captureTargetRequested = false;
    return;
  }

  const Vec3 sunDirection = sunVectorFromUnixTime(unixTimeUtc);

  if (captureTargetRequested) {
    const Vec3 mirrorNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);
    targetDirection = normalizeVec3(reflectVector(scaleVec3(sunDirection, -1.0f), mirrorNormal));
    targetDirectionValid = (lengthVec3(targetDirection) > 0.0f);
    capturedPanAngleDeg = panAngleDeg;
    capturedTiltAngleDeg = tiltAngleDeg;
    lastAutoReferencePanDeg = capturedPanAngleDeg;
    lastAutoReferenceTiltDeg = capturedTiltAngleDeg;
    autoTrackStartUnixTimeUtc = 0;
    controlMode = targetDirectionValid ? CONTROL_MODE_TARGET_CAPTURED : CONTROL_MODE_MANUAL;
    captureTargetRequested = false;
    autoTrackEnabled = false;
    if (targetDirectionValid) {
      Serial.println("Heliostat target captured.");
    } else {
      Serial.println("Heliostat target capture failed.");
    }
  }

  if (!targetDirectionValid) {
    if (controlMode != CONTROL_MODE_MANUAL) {
      controlMode = CONTROL_MODE_MANUAL;
    }
    autoTrackEnabled = false;
    return;
  }

  if (autoTrackEnabled) {
    Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirection));
    if (lengthVec3(desiredNormal) > 0.0f) {
      const Vec3 currentNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);
      if (dotVec3(currentNormal, desiredNormal) < 0.0f) {
        desiredNormal = scaleVec3(desiredNormal, -1.0f);
      }
      panTiltFromMirrorNormal(desiredNormal, panTargetDeg, tiltTargetDeg);
      if (controlMode != CONTROL_MODE_AUTO_TRACK) {
        autoTrackStartUnixTimeUtc = unixTimeUtc;
      }
      lastAutoReferencePanDeg = panTargetDeg;
      lastAutoReferenceTiltDeg = tiltTargetDeg;
      controlMode = CONTROL_MODE_AUTO_TRACK;
    }
  } else if (controlMode == CONTROL_MODE_AUTO_TRACK) {
    controlMode = CONTROL_MODE_TARGET_CAPTURED;
  }
}

void updateTargetsFromRemoteInput(uint32_t nowMs) {
  static uint32_t lastMotionUpdateMs = 0;
  if (lastMotionUpdateMs == 0) {
    lastMotionUpdateMs = nowMs;
  }

  const uint32_t elapsedMs = nowMs - lastMotionUpdateMs;
  if (elapsedMs == 0) {
    return;
  }

  lastMotionUpdateMs = nowMs;

  if (!packetReceived || (nowMs - lastRxMs) > CONTROL_LINK_TIMEOUT_MS) {
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    if (!targetDirectionValid && !autoTrackEnabled) {
      controlMode = CONTROL_MODE_MANUAL;
    }
  }

  updateHeliostatTracking(nowMs);
  blinkActive = (controlMode == CONTROL_MODE_AUTO_TRACK);

  const float dt = static_cast<float>(elapsedMs) / 1000.0f;

  if (controlMode == CONTROL_MODE_AUTO_TRACK) {
    return;
  }

  if (recenterRequested) {
    panTargetDeg = PAN_START_DEG;
    tiltTargetDeg = TILT_START_DEG;
    recenterRequested = false;
  }

  const float manualSpeedDegPerSec =
      precisionManualMode ? MANUAL_PRECISION_SPEED_DEG_PER_SEC : MAX_TARGET_SPEED_DEG_PER_SEC;
  panTargetDeg = constrain(
      panTargetDeg + (applySpeedCurve(remotePanInput) * manualSpeedDegPerSec * dt),
      panMinLimitDeg(), panMaxLimitDeg());
  tiltTargetDeg = constrain(
      tiltTargetDeg + (applySpeedCurve(remoteTiltInput) * manualSpeedDegPerSec * dt),
      tiltMinLimitDeg(), tiltMaxLimitDeg());
}

void updateScanState(uint32_t nowMs) {
  if (!scanActive || controlMode == CONTROL_MODE_AUTO_TRACK || scanPointsTotal == 0) {
    return;
  }

  const bool onTarget =
      fabsf(panAngleDeg - panTargetDeg) <= SCAN_SETTLE_TOLERANCE_DEG &&
      fabsf(tiltAngleDeg - tiltTargetDeg) <= SCAN_SETTLE_TOLERANCE_DEG;

  if (!onTarget) {
    scanPointReachedMs = 0;
    return;
  }

  if (scanPointReachedMs == 0) {
    scanPointReachedMs = nowMs;
    return;
  }

  if ((nowMs - scanPointReachedMs) < scanDwellMs) {
    return;
  }

  const int32_t nextIndex = static_cast<int32_t>(scanPointIndex) + static_cast<int32_t>(scanDirection);
  if (nextIndex < 0 || nextIndex >= static_cast<int32_t>(scanPointsTotal)) {
    stopScan();
    publishRemoteDiag("scan_completed");
    return;
  }

  scanPointIndex = static_cast<uint16_t>(nextIndex);
  scanPointReachedMs = 0;
  setScanTargetForIndex(scanPointIndex);
}

void moveServosTowardTargets(uint32_t nowMs) {
  static uint32_t lastServoUpdateMs = 0;
  if (lastServoUpdateMs == 0) {
    lastServoUpdateMs = nowMs;
  }

  const uint32_t elapsedMs = nowMs - lastServoUpdateMs;
  if (elapsedMs == 0) {
    return;
  }

  lastServoUpdateMs = nowMs;

  const float slewSpeedDegPerSec = scanActive ? scanMoveSpeedDegPerSec : SERVO_MAX_SLEW_DEG_PER_SEC;
  const float maxStep = slewSpeedDegPerSec * (static_cast<float>(elapsedMs) / 1000.0f);

  panAngleDeg = stepToward(panAngleDeg, panTargetDeg, maxStep);
  tiltAngleDeg = stepToward(tiltAngleDeg, tiltTargetDeg, maxStep);
  writeRemoteActuators();
}

void beginRemoteMqtt() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  remoteMqttClient.setServer(REMOTE_MQTT_HOST_VALUE, REMOTE_MQTT_PORT_VALUE);
  remoteMqttClient.setCallback(onRemoteMqttMessage);
  remoteMqttClient.setBufferSize(REMOTE_MQTT_BUFFER_SIZE);
}
#endif

#if defined(DEVICE_ROLE_CONTROLLER)
bool isKnownRemotePeer() {
  static const uint8_t emptyMac[6] = {0, 0, 0, 0, 0, 0};
  return memcmp(remotePeerMac, emptyMac, sizeof(remotePeerMac)) != 0;
}

void rememberRemotePeer(const uint8_t* mac) {
  memcpy(remotePeerMac, mac, sizeof(remotePeerMac));
  espNowPeerReady = addPeer(remotePeerMac);
  if (espNowPeerReady) {
    Serial.printf("Remote auto-paired: %s\n", formatMac(remotePeerMac).c_str());
  }
}

void onEspNowSent(const uint8_t* macAddr, esp_now_send_status_t status) {
  sendPending = false;
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  if (!isKnownRemotePeer() || memcmp(macAddr, remotePeerMac, sizeof(remotePeerMac)) != 0) {
    return;
  }
  if (!lastSendOk) {
    pendingEspNowTxFailureLog = true;
  }
}

void sendLedPacket() {
  if (!espNowPeerReady || sendPending) {
    return;
  }

  EspNowPacket packet{};
  packet.magic = LED_PACKET_MAGIC;
  packet.type = PACKET_TYPE_CONTROL;
  packet.reserved[0] =
      (recenterRequested ? PACKET_FLAG_RECENTER : 0) |
      (captureTargetRequested ? PACKET_FLAG_CAPTURE_TARGET : 0) |
      (autoTrackEnabled ? PACKET_FLAG_AUTO_TRACK : 0) |
      (timeUpdatePending ? PACKET_FLAG_TIME_UPDATE : 0) |
      (timeScaleUpdatePending ? PACKET_FLAG_SCALE_UPDATE : 0) |
      (precisionManualMode ? PACKET_FLAG_PRECISION : 0);
  packet.seq = packetSequence++;
  packet.panInput = remotePanInput;
  packet.tiltInput = remoteTiltInput;
  packet.panAngleDeg = remoteReportedPanAngleDeg;
  packet.tiltAngleDeg = remoteReportedTiltAngleDeg;
  packet.panTargetDeg = remoteReportedPanTargetDeg;
  packet.tiltTargetDeg = remoteReportedTiltTargetDeg;
  packet.capturedPanAngleDeg = remoteReportedCapturedPanAngleDeg;
  packet.capturedTiltAngleDeg = remoteReportedCapturedTiltAngleDeg;
  packet.timeScale = heliostatTimeScale;
  packet.unixTimeUtc =
      timeUpdatePending ? static_cast<int64_t>(heliostatStartUnixTimeUtc) : static_cast<int64_t>(0);

  sendPending = true;
  const esp_err_t result =
      esp_now_send(remotePeerMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    sendPending = false;
    lastSendOk = false;
    Serial.printf("ESP-NOW send failed immediately: %d\n", static_cast<int>(result));
  } else {
    timeUpdatePending = false;
    timeScaleUpdatePending = false;
  }
}

void onEspNowReceived(const uint8_t* macAddr, const uint8_t* data, int len) {
  if (len != static_cast<int>(sizeof(EspNowPacket))) {
    Serial.printf("ESP-NOW RX wrong size from %s: %d\n", formatMac(macAddr).c_str(), len);
    return;
  }

  EspNowPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != LED_PACKET_MAGIC) {
    Serial.printf("ESP-NOW RX wrong magic from %s\n", formatMac(macAddr).c_str());
    return;
  }

  if (packet.type == PACKET_TYPE_ANNOUNCE) {
    remoteReportedControlMode = static_cast<ControlMode>(packet.reserved[0]);
    remoteReportedTargetDirectionValid = packet.reserved[1] != 0;
    remoteReportedSunTimeValid = packet.reserved[2] != 0;
    remoteReportedPanAngleDeg = packet.panAngleDeg;
    remoteReportedTiltAngleDeg = packet.tiltAngleDeg;
    remoteReportedPanTargetDeg = packet.panTargetDeg;
    remoteReportedTiltTargetDeg = packet.tiltTargetDeg;
    remoteReportedCapturedPanAngleDeg = packet.capturedPanAngleDeg;
    remoteReportedCapturedTiltAngleDeg = packet.capturedTiltAngleDeg;
    remoteReportedTimeScale = packet.timeScale;
    remoteReportedAutoReferencePanDeg = packet.autoReferencePanDeg;
    remoteReportedAutoReferenceTiltDeg = packet.autoReferenceTiltDeg;
    remoteReportedPanPulseUs = packet.panPulseUs;
    remoteReportedTiltPulseUs = packet.tiltPulseUs;
    remoteReportedUnixTimeUtc = static_cast<time_t>(packet.unixTimeUtc);
    remoteReportedAutoTrackStartUnixTimeUtc = static_cast<time_t>(packet.autoTrackStartUnixTimeUtc);
    if (!controller.isConnected()) {
      autoTrackEnabled = (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);
    }
    blinkActive = (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);
    if (!espNowPeerReady) {
      memcpy(pendingAutoPairMac, macAddr, sizeof(pendingAutoPairMac));
      pendingAutoPair = true;
      pendingControllerSyncPacket = true;
    }
    if ((packet.reserved[0] & PACKET_FLAG_TIME_UPDATE) != 0 && packet.unixTimeUtc > 0) {
      setHeliostatUnixTimeUtc(static_cast<time_t>(packet.unixTimeUtc));
    }
    return;
  }

  if (packet.type != PACKET_TYPE_CONTROL) {
    return;
  }

  if (!espNowPeerReady) {
    return;
  }

  if (memcmp(macAddr, remotePeerMac, sizeof(remotePeerMac)) != 0) {
    return;
  }

  packetReceived = true;
  lastRxMs = millis();
}

void handleSerialCommandLine(const char* line) {
  if (strncmp(line, "T=", 2) == 0 || strncmp(line, "t=", 2) == 0) {
    const long long parsed = atoll(line + 2);
    if (parsed > 0) {
      setControllerHeliostatUnixTimeUtc(static_cast<time_t>(parsed));
      Serial.printf("UTC time set: %lld\n", parsed);
      sendLedPacket();
      return;
    }
  }

  if (strncmp(line, "S=", 2) == 0 || strncmp(line, "s=", 2) == 0) {
    const float parsed = atof(line + 2);
    if (parsed > 0.0f) {
      setControllerHeliostatTimeScale(parsed);
      Serial.printf("Time scale set: %.3f\n", heliostatTimeScale);
      sendLedPacket();
      return;
    }
  }

  if (strcmp(line, "TIME?") == 0 || strcmp(line, "time?") == 0) {
    time_t unixTimeUtc = 0;
    if (currentUnixTimeUtc(unixTimeUtc)) {
      Serial.printf("UTC time current: %lld\n", static_cast<long long>(unixTimeUtc));
    } else {
      Serial.println("UTC time not set.");
    }
    return;
  }

  if (strcmp(line, "SCALE?") == 0 || strcmp(line, "scale?") == 0) {
    Serial.printf("Time scale current: %.3f\n", heliostatTimeScale);
    return;
  }

  if (strcmp(line, "HELP") == 0 || strcmp(line, "help") == 0) {
    Serial.println("Serial commands:");
    Serial.println("  T=<unix_utc_seconds>  set UTC time");
    Serial.println("  S=<scale>             set heliostat time scale");
    Serial.println("  TIME?                 show current UTC time");
    Serial.println("  SCALE?                show current time scale");
    return;
  }

  if (line[0] != '\0') {
    Serial.printf("Unknown serial command: %s\n", line);
  }
}

void handleControllerSerial() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';
      handleSerialCommandLine(serialCommandBuffer);
      serialCommandLength = 0;
      serialCommandBuffer[0] = '\0';
      continue;
    }

    if (serialCommandLength + 1 < SERIAL_COMMAND_BUFFER_SIZE) {
      serialCommandBuffer[serialCommandLength++] = ch;
    }
  }
}

void printTrackingDiagnostic() {
  const double driftSeconds =
      (remoteReportedUnixTimeUtc > 0 && remoteReportedAutoTrackStartUnixTimeUtc > 0)
          ? difftime(remoteReportedUnixTimeUtc, remoteReportedAutoTrackStartUnixTimeUtc)
          : -1.0;
  const String nowText = formatUnixTimeUtc(remoteReportedUnixTimeUtc);
  const String autoStartText = formatUnixTimeUtc(remoteReportedAutoTrackStartUnixTimeUtc);
  Serial.printf(
      "TRACK_DIAG | mode=%s | now_utc=%s | auto_start_utc=%s | drift_s=%.0f | auto_ref_pan=%.3f | actual_pan=%.3f | delta_pan=%+.3f | auto_ref_tilt=%.3f | actual_tilt=%.3f | delta_tilt=%+.3f | target=%s | sun_time=%s\n",
      controlModeName(remoteReportedControlMode),
      nowText.c_str(),
      autoStartText.c_str(),
      driftSeconds,
      remoteReportedAutoReferencePanDeg,
      remoteReportedPanAngleDeg,
      remoteReportedPanAngleDeg - remoteReportedAutoReferencePanDeg,
      remoteReportedAutoReferenceTiltDeg,
      remoteReportedTiltAngleDeg,
      remoteReportedTiltAngleDeg - remoteReportedAutoReferenceTiltDeg,
      remoteReportedTargetDirectionValid ? "ok" : "missing",
      remoteReportedSunTimeValid ? "ok" : "missing");
}

void serviceControllerDeferredActions(uint32_t nowMs) {
  if (pendingAutoPair) {
    pendingAutoPair = false;
    rememberRemotePeer(pendingAutoPairMac);
  }

  if (pendingControllerSyncPacket) {
    pendingControllerSyncPacket = false;
    sendLedPacket();
  }

  if (pendingEspNowTxFailureLog) {
    pendingEspNowTxFailureLog = false;
    if (isKnownRemotePeer()) {
      Serial.printf("ESP-NOW TX failed to %s\n", formatMac(remotePeerMac).c_str());
    } else {
      Serial.println("ESP-NOW TX failed.");
    }
  }

  if (pendingControllerConnectLog) {
    pendingControllerConnectLog = false;
    Serial.println("Xbox connected.");
  }

  if (pendingControllerDisconnectLog) {
    pendingControllerDisconnectLog = false;
    Serial.println("Xbox disconnected.");
  }

  applyLedFromPanTilt(nowMs);
}
#else
void sendAnnouncePacket(uint32_t nowMs) {
  if (!broadcastPeerReady || (nowMs - lastAnnounceMs) < REMOTE_ANNOUNCE_MS) {
    return;
  }

  lastAnnounceMs = nowMs;
  time_t currentUnixTime = 0;
  const bool haveCurrentUnixTime = currentUnixTimeUtc(currentUnixTime);

  EspNowPacket packet{};
  packet.magic = LED_PACKET_MAGIC;
  packet.type = PACKET_TYPE_ANNOUNCE;
  packet.reserved[0] = static_cast<uint8_t>(controlMode);
  packet.reserved[1] = targetDirectionValid ? 1 : 0;
  packet.reserved[2] = sunTimeValid ? 1 : 0;
  packet.seq = packetSequence++;
  packet.panAngleDeg = panAngleDeg;
  packet.tiltAngleDeg = tiltAngleDeg;
  packet.panTargetDeg = panTargetDeg;
  packet.tiltTargetDeg = tiltTargetDeg;
  packet.capturedPanAngleDeg = capturedPanAngleDeg;
  packet.capturedTiltAngleDeg = capturedTiltAngleDeg;
  packet.timeScale = heliostatTimeScale;
  packet.autoReferencePanDeg = lastAutoReferencePanDeg;
  packet.autoReferenceTiltDeg = lastAutoReferenceTiltDeg;
  packet.unixTimeUtc = haveCurrentUnixTime ? static_cast<int64_t>(currentUnixTime) : static_cast<int64_t>(0);
  packet.autoTrackStartUnixTimeUtc = static_cast<int64_t>(autoTrackStartUnixTimeUtc);
  packet.panPulseUs = lastPanPulseUs;
  packet.tiltPulseUs = lastTiltPulseUs;

  const esp_err_t result =
      esp_now_send(ESPNOW_BROADCAST_MAC, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW announce failed immediately: %d\n", static_cast<int>(result));
  }
}

void onEspNowReceived(const uint8_t* macAddr, const uint8_t* data, int len) {
  if (len != static_cast<int>(sizeof(EspNowPacket))) {
    Serial.printf("ESP-NOW RX wrong size from %s: %d\n", formatMac(macAddr).c_str(), len);
    return;
  }

  EspNowPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != LED_PACKET_MAGIC || packet.type != PACKET_TYPE_CONTROL) {
    return;
  }

  recenterRequested = (packet.reserved[0] & PACKET_FLAG_RECENTER) != 0;
  captureTargetRequested = (packet.reserved[0] & PACKET_FLAG_CAPTURE_TARGET) != 0;
  autoTrackEnabled = (packet.reserved[0] & PACKET_FLAG_AUTO_TRACK) != 0;
  precisionManualMode = (packet.reserved[0] & PACKET_FLAG_PRECISION) != 0;
  remotePanInput = constrain(packet.panInput, -1.0f, 1.0f);
  remoteTiltInput = constrain(packet.tiltInput, -1.0f, 1.0f);
  if ((packet.reserved[0] & PACKET_FLAG_TIME_UPDATE) != 0 && packet.unixTimeUtc > 0) {
    setHeliostatUnixTimeUtc(static_cast<time_t>(packet.unixTimeUtc));
  }
  if ((packet.reserved[0] & PACKET_FLAG_SCALE_UPDATE) != 0) {
    heliostatTimeScale = max(packet.timeScale, 0.01f);
  }
  packetReceived = true;
  lastRxMs = millis();
}
#endif

void updateLedFromXbox(uint32_t nowMs) {
#if defined(DEVICE_ROLE_CONTROLLER)
  if ((nowMs - lastControlUpdateMs) < CONTROL_UPDATE_MS) {
    return;
  }

  lastControlUpdateMs = nowMs;

  if (!controller.isConnected()) {
    return;
  }

  BLEControlsEvent state;
  controller.readControls(state);

  const bool capturePressed = state.buttonX && !lastCaptureButton;
  const bool autoPressed = state.buttonY && !lastAutoButton;
  const bool diagPressed = state.buttonB && !lastDiagButton;
  lastCaptureButton = state.buttonX;
  lastAutoButton = state.buttonY;
  lastDiagButton = state.buttonB;

  remotePanInput = applyDeadzone(-state.leftStickX);
  remoteTiltInput = applyDeadzone(-state.leftStickY);
  recenterRequested = state.buttonA;
  captureTargetRequested = capturePressed;
  precisionManualMode = state.leftBumper;
  if (autoPressed) {
    autoTrackEnabled = !autoTrackEnabled;
  }
  if (diagPressed) {
    printTrackingDiagnostic();
  }
  blinkActive = autoTrackEnabled || (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);

  panAngleDeg = constrain(
      PAN_START_DEG + (remotePanInput * ((panMaxLimitDeg() - panMinLimitDeg()) * 0.5f)),
      panMinLimitDeg(), panMaxLimitDeg());
  tiltAngleDeg = constrain(
      TILT_START_DEG + (remoteTiltInput * ((tiltMaxLimitDeg() - tiltMinLimitDeg()) * 0.5f)),
      tiltMinLimitDeg(), tiltMaxLimitDeg());
  applyLedFromPanTilt(nowMs);
  sendLedPacket();
#else
  (void)nowMs;
#endif
}

void printStatus(uint32_t nowMs) {
  const bool logsEnabled = ENABLE_RUNTIME_STATUS_LOGS;
  if (!logsEnabled) {
    return;
  }

  if ((nowMs - lastStatusPrintMs) < STATUS_PRINT_MS) {
    return;
  }
  lastStatusPrintMs = nowMs;

#if defined(DEVICE_ROLE_CONTROLLER)
  const char* xboxState = controller.isConnected() ? "ok" : "missing";
  const String remoteTimeText = formatUnixTimeUtc(remoteReportedUnixTimeUtc);
  if (SHOW_SERVO_PULSE_US_IN_LOGS) {
    Serial.printf(
        "ROLE=controller | xbox=%s | peer=%s | tx=%s | auto=%s | remote_mode=%s | pan=%7.3f us=%4d range=%d..%d | tilt=%7.3f us=%4d range=%d..%d | d_pan=%+7.3f d_tilt=%+7.3f | target=%s | sun_time=%s | remote_utc=%s | time_x=%.1f\n",
        xboxState,
        espNowPeerReady ? "ok" : "missing",
        lastSendOk ? "ok" : "pending",
        autoTrackEnabled ? "on" : "off",
        controlModeName(remoteReportedControlMode),
        remoteReportedPanAngleDeg,
        remoteReportedPanPulseUs,
        PAN_SERVO_MIN_PULSE_US, PAN_SERVO_MAX_PULSE_US,
        remoteReportedTiltAngleDeg,
        remoteReportedTiltPulseUs,
        TILT_SERVO_MIN_PULSE_US, TILT_SERVO_MAX_PULSE_US,
        remoteReportedPanTargetDeg - remoteReportedCapturedPanAngleDeg,
        remoteReportedTiltTargetDeg - remoteReportedCapturedTiltAngleDeg,
        remoteReportedTargetDirectionValid ? "ok" : "missing",
        remoteReportedSunTimeValid ? "ok" : "missing",
        remoteTimeText.c_str(),
        remoteReportedTimeScale);
  } else {
    Serial.printf(
        "ROLE=controller | xbox=%s | peer=%s | tx=%s | auto=%s | remote_mode=%s | pan=%7.3f | tilt=%7.3f | d_pan=%+7.3f d_tilt=%+7.3f | target=%s | sun_time=%s | remote_utc=%s | time_x=%.1f\n",
        xboxState,
        espNowPeerReady ? "ok" : "missing",
        lastSendOk ? "ok" : "pending",
        autoTrackEnabled ? "on" : "off",
        controlModeName(remoteReportedControlMode),
        remoteReportedPanAngleDeg,
        remoteReportedTiltAngleDeg,
        remoteReportedPanTargetDeg - remoteReportedCapturedPanAngleDeg,
        remoteReportedTiltTargetDeg - remoteReportedCapturedTiltAngleDeg,
        remoteReportedTargetDirectionValid ? "ok" : "missing",
        remoteReportedSunTimeValid ? "ok" : "missing",
        remoteTimeText.c_str(),
        remoteReportedTimeScale);
  }
#else
  const uint32_t ageMs = packetReceived ? (nowMs - lastRxMs) : 0;
  Serial.printf(
      "ROLE=remote | mode=%s | wifi=%s | mqtt=%s | ntp=%s | rx=%s | age_ms=%lu | pan=%7.3f | tilt=%7.3f | target=%7.3f/%7.3f | pan_us=%d | tilt_us=%d | sun_time=%s | target=%s\n",
      controlModeName(controlMode),
      wifiLinkOk ? "ok" : "down",
      mqttLinkOk ? "ok" : "down",
      ntpTimeValid ? "ok" : "down",
      packetReceived ? "ok" : "waiting",
      static_cast<unsigned long>(ageMs),
      panAngleDeg, tiltAngleDeg, panTargetDeg, tiltTargetDeg,
      lastPanPulseUs, lastTiltPulseUs,
      sunTimeValid ? "ok" : "missing",
      targetDirectionValid ? "ok" : "missing");
#endif
}

#if defined(DEVICE_ROLE_CONTROLLER)
void onControllerConnect(NimBLEAddress address) {
  (void)address;
  xboxConnected = true;
  autoTrackEnabled = (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);
  blinkActive = (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);
  pendingControllerSyncPacket = true;
  pendingControllerConnectLog = true;
}

void onControllerDisconnect(NimBLEAddress address) {
  (void)address;
  xboxConnected = false;
  blinkActive = (remoteReportedControlMode == CONTROL_MODE_AUTO_TRACK);
  pendingControllerSyncPacket = true;
  pendingControllerDisconnectLog = true;
}
#endif

}  // namespace

void setup() {
  Serial.begin(115200);
  waitForSerialIfEnabled();
  delay(300);

  beginStatusLed();
  setStatusLedBlue();
  delay(100);

  bootMs = millis();
  lastControlUpdateMs = millis();
  lastStatusPrintMs = millis();

#if defined(DEVICE_ROLE_CONTROLLER)
  setupEspNowWifi();
  ensureEspNow();
  ensureBroadcastPeer();
  esp_now_register_recv_cb(onEspNowReceived);
  esp_now_register_send_cb(onEspNowSent);
  if (CLEAR_XBOX_BONDS_ON_BOOT) {
    BLEControllerRegistry::deleteBonds();
    Serial.println("BLE bonds cleared on boot.");
  }
  controller.onConnect(onControllerConnect);
  controller.onDisconnect(onControllerDisconnect);
  controller.begin();
  applyLedFromPanTilt(millis());
#else
  beginRemoteActuators();
  beginRemoteMqtt();
#endif

  Serial.println();
  Serial.println("ESP32 heliostat link test");
  Serial.printf("ROLE=%s\n", ROLE_NAME);
  Serial.printf("WAIT_FOR_SERIAL=%s timeout=%lu ms\n",
                WAIT_FOR_SERIAL ? "true" : "false",
                static_cast<unsigned long>(WAIT_FOR_SERIAL_TIMEOUT_MS));
  Serial.printf("STATUS_LED_ENABLED=%s pin=%d\n",
                STATUS_LED_ENABLED ? "true" : "false", STATUS_LED_PIN);
  Serial.printf("WiFi STA MAC=%s | channel=%u\n",
                WiFi.macAddress().c_str(), static_cast<unsigned>(WiFi.channel()));

#if defined(DEVICE_ROLE_CONTROLLER)
  Serial.println("Remote peer MAC=auto");
  Serial.printf("CLEAR_XBOX_BONDS_ON_BOOT=%s\n", CLEAR_XBOX_BONDS_ON_BOOT ? "true" : "false");
  Serial.println("Xbox node: left stick = target angle movement, button A = recenter, LB = slow manual mode.");
  Serial.println("Pan model reference: pan=90 deg means mirror normal points south.");
  Serial.println("Button X captures the reflected target. Button Y toggles heliostat auto-track. Button B prints TRACK_DIAG.");
  Serial.println("Serial: T=<unix_utc_seconds>, S=<scale>, TIME?, SCALE?.");
  Serial.println("Flash env: controller on the ESP32 with the Xbox controller.");
#else
  Serial.println("Remote node ready.");
  Serial.printf("WiFi target SSID=%s | MQTT=%s:%u | topic=%s\n",
                REMOTE_WIFI_SSID_VALUE, REMOTE_MQTT_HOST_VALUE,
                static_cast<unsigned>(REMOTE_MQTT_PORT_VALUE), REMOTE_MQTT_BASE_TOPIC_VALUE);
  Serial.printf("Actuator backend: %s\n", remoteActuatorBackendName());
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_PWM) {
    Serial.printf("Servos: pan GPIO=%d | tilt GPIO=%d | %d Hz\n",
                  PAN_SERVO_PIN, TILT_SERVO_PIN, SERVO_FREQUENCY_HZ);
  }
  Serial.println("Pan model reference: pan=90 deg means mirror normal points south.");
  Serial.printf("Tilt model/servo: model %.1f deg -> servo %.1f deg | offset=%+.1f deg\n",
                TILT_START_DEG, TILT_SERVO_AT_MODEL_HORIZON_DEG, TILT_SERVO_OFFSET_DEG);
  Serial.printf("Servo command step: %d us\n", SERVO_COMMAND_STEP_US);
  Serial.printf("Slew rate: %.2f deg/s\n", SERVO_MAX_SLEW_DEG_PER_SEC);
  Serial.printf("Heliostat lat=%.4f lon=%.4f start_unix=%lld\n",
                HELIOSTAT_LATITUDE_DEG, HELIOSTAT_LONGITUDE_DEG,
                static_cast<long long>(heliostatStartUnixTimeUtc));
  Serial.printf("Runtime status logs: %s\n", ENABLE_RUNTIME_STATUS_LOGS ? "on" : "off");
  Serial.println("Use a dedicated 5-6V supply for MG996R servos and share GND with the ESP32.");
#endif
}

void loop() {
  const uint32_t nowMs = millis();
#if defined(DEVICE_ROLE_CONTROLLER)
  handleControllerSerial();
  serviceControllerDeferredActions(nowMs);
#endif
  updateLedFromXbox(nowMs);
#if defined(DEVICE_ROLE_REMOTE)
  ensureRemoteWifiConnected(nowMs);
  ensureRemoteNtpTime(nowMs);
  ensureRemoteMqttConnected(nowMs);
  if (remoteMqttClient.connected()) {
    remoteMqttClient.loop();
  }
  updateTargetsFromRemoteInput(nowMs);
  updateScanState(nowMs);
  moveServosTowardTargets(nowMs);
  applyLedFromPanTilt(nowMs);
  publishRemoteState();
#endif
  printStatus(nowMs);
}
