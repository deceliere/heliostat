#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <SCServo.h>
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 512
#endif
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>

#if __has_include("remote_secrets.h")
#include "remote_secrets.h"
#endif

// le miroir doit etre oriente SUD et a l'horizontal pour que les angles soient corrects, sinon il faudra faire des ajustements dans les calculs d'angles


namespace {

constexpr bool STATUS_LED_ENABLED = true;
constexpr int STATUS_LED_PIN = 23;
constexpr int STATUS_LED_COUNT = 1;
constexpr uint8_t LED_BRIGHTNESS = 32;

constexpr float PAN_MIN_DEG = 90.0f;
constexpr float PAN_MAX_DEG = 270.0f;
constexpr float PAN_START_DEG = 180.0f;
constexpr float PAN_MODEL_SIGN = 1.0f;

constexpr float TILT_MIN_DEG = 0.0f;
constexpr float TILT_MAX_DEG = 180.0f;
constexpr float TILT_START_DEG = 180.0f;
constexpr float TILT_MODEL_SIGN = -1.0f;
constexpr float TILT_SERVO_OFFSET_DEG = 0.0f;
constexpr float TILT_SERVO_MIN_DEG = 00.0f;
constexpr float TILT_SERVO_MAX_DEG = 180.0f;
constexpr float TILT_SERVO_AT_MODEL_HORIZON_DEG = TILT_START_DEG + TILT_SERVO_OFFSET_DEG;

constexpr float MAX_TARGET_SPEED_DEG_PER_SEC = 90.0f;
constexpr float MANUAL_PRECISION_SPEED_DEG_PER_SEC = 8.0f;
constexpr float SERVO_MAX_SLEW_DEG_PER_SEC = 500.0f;
constexpr uint32_t STATUS_PRINT_MS = 2000;
constexpr uint32_t CONTROL_LINK_TIMEOUT_MS = 250;
constexpr bool ENABLE_RUNTIME_STATUS_LOGS = false;
constexpr bool WAIT_FOR_SERIAL = false;
constexpr uint32_t WAIT_FOR_SERIAL_TIMEOUT_MS = 15000;
constexpr uint32_t BLINK_PERIOD_MS = 300;
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
constexpr bool ST3020_HOLD_TORQUE_ENABLED = false;
constexpr uint32_t ST3020_FEEDBACK_POLL_MS = 50;
constexpr int ST3020_SETTLE_TOLERANCE_POS = 2;
constexpr uint8_t ST3020_MAX_FINAL_CORRECTIONS = 2;
constexpr int ST3020_FEEDBACK_TRIM_STEP_POS = 3;
constexpr int ST3020_FEEDBACK_OVERSHOOT_POS = 4;
constexpr uint8_t ST3020_MAX_FEEDBACK_OVERSHOOTS = 4;
constexpr uint32_t ST3020_MOVE_SETTLE_MS = 120;
constexpr uint32_t ST3020_MOVE_TIMEOUT_MARGIN_MS = 250;
constexpr uint32_t ST3020_MOVE_MIN_COMMAND_MS = 80;
constexpr bool ST3020_AUTO_APPROACH_ENABLED = false;
constexpr float ST3020_AUTO_APPROACH_PAN_OFFSET_DEG = -5.0f;
constexpr float ST3020_AUTO_APPROACH_TILT_OFFSET_DEG = 5.0f;
constexpr bool ST3020_FEEDBACK_CORRECTION_ENABLED = false;
constexpr float ST3020_PAN_SIGN = 1.0f;
constexpr float ST3020_TILT_SIGN = -1.0f;
constexpr int ST3020_PAN_POS_AT_90_DEG = 1024;
constexpr int ST3020_PAN_POS_AT_180_DEG = 2048;
constexpr int ST3020_PAN_POS_AT_270_DEG = 3072;
constexpr float ST3020_TILT_CAL_EXT_0_DEG = 0.0f;
constexpr float ST3020_TILT_CAL_EXT_45_DEG = 45.0f;
constexpr float ST3020_TILT_CAL_EXT_90_DEG = 90.0f;
constexpr int ST3020_TILT_POS_AT_EXT_0_DEG = 1024;
constexpr int ST3020_TILT_POS_AT_EXT_45_DEG = 1528;
constexpr int ST3020_TILT_POS_AT_EXT_90_DEG = 2038;
constexpr float ST3020_PAN_MIN_DEG = PAN_MIN_DEG;
constexpr float ST3020_PAN_MAX_DEG = PAN_MAX_DEG;
constexpr float ST3020_TILT_MIN_DEG = 90.0f;
constexpr float ST3020_TILT_MAX_DEG = 210.0f;
constexpr float ST3020_TILT_EXTERNAL_MIN_DEG = -13.0f;
constexpr float ST3020_TILT_EXTERNAL_MAX_DEG = 90.0f;
constexpr float SPEED_CURVE_EXPONENT = 1.8f;
constexpr float HELIOSTAT_LATITUDE_DEG = 46.200316f;
constexpr float HELIOSTAT_LONGITUDE_DEG = 6.139345f;
constexpr time_t HELIOSTAT_START_UNIX_TIME_UTC = 0;
constexpr const char* TRACKING_MODEL_STAGE = "beta";
constexpr const char* TRACKING_MODEL_VERSION = "south180-beta-1";
constexpr uint32_t AUTO_TARGET_UPDATE_INTERVAL_MS = 5000;
constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 64;
constexpr uint32_t REMOTE_STATE_PUBLISH_MS = 150;
constexpr uint32_t REMOTE_WIFI_RETRY_MS = 10000;
constexpr uint32_t REMOTE_MQTT_RETRY_MS = 5000;
constexpr uint32_t REMOTE_NTP_RETRY_MS = 15000;
constexpr uint32_t REMOTE_NTP_RESYNC_MS = 60000;
constexpr time_t MIN_VALID_UNIX_TIME_UTC = 1704067200;
constexpr uint16_t REMOTE_MQTT_BUFFER_SIZE = 4096;
constexpr float SCAN_SETTLE_TOLERANCE_DEG = 0.35f;
constexpr float SCAN_COARSE_RANGE_PAN_DEG = 6.0f;
constexpr float SCAN_COARSE_RANGE_TILT_DEG = 6.0f;
constexpr float SCAN_COARSE_STEP_DEG = 1.0f;
constexpr float SCAN_FINE_RANGE_PAN_DEG = 1.5f;
constexpr float SCAN_FINE_RANGE_TILT_DEG = 1.5f;
constexpr float SCAN_FINE_STEP_DEG = 0.25f;
constexpr float SCAN_MICRO_RANGE_PAN_DEG = 0.5f;
constexpr float SCAN_MICRO_RANGE_TILT_DEG = 0.5f;
constexpr float SCAN_MICRO_STEP_DEG = 0.1f;
constexpr uint16_t SCAN_MOVE_SPEED_MIN = 100;
constexpr uint16_t SCAN_MOVE_SPEED_MAX = 3000;
constexpr uint16_t SCAN_MOVE_SPEED_DEFAULT = 1000;
constexpr uint16_t SCAN_DWELL_MIN_MS = 0;
constexpr uint16_t SCAN_DWELL_MAX_MS = 2000;
constexpr uint16_t SCAN_DWELL_DEFAULT_MS = 200;
constexpr uint32_t SCAN_ST3020_STEP_MIN_INTERVAL_MS = 25;
constexpr uint32_t SCAN_ST3020_STEP_MAX_INTERVAL_MS = 500;
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
constexpr const char* ROLE_NAME = "remote";

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

enum St3020MotionStage : uint8_t {
  ST3020_MOTION_IDLE = 0,
  ST3020_MOTION_APPROACH = 1,
  ST3020_MOTION_FINAL = 2,
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

struct St3020Calibration {
  int panPosAt90Deg;
  int panPosAt180Deg;
  int panPosAt270Deg;
  int tiltPosAtExt0Deg;
  int tiltPosAtExt45Deg;
  int tiltPosAtExt90Deg;
};

constexpr St3020Calibration ST3020_DEFAULT_CALIBRATION = {
    ST3020_PAN_POS_AT_90_DEG,
    ST3020_PAN_POS_AT_180_DEG,
    ST3020_PAN_POS_AT_270_DEG,
    ST3020_TILT_POS_AT_EXT_0_DEG,
    ST3020_TILT_POS_AT_EXT_45_DEG,
    ST3020_TILT_POS_AT_EXT_90_DEG,
};

Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);
Servo panServo;
Servo tiltServo;
SMS_STS st3020Bus;
St3020Calibration st3020Calibration = ST3020_DEFAULT_CALIBRATION;
bool st3020TorqueEnabled = false;
bool st3020PanFeedbackValid = false;
bool st3020TiltFeedbackValid = false;
uint32_t lastSt3020FeedbackMs = 0;
uint32_t lastSt3020MotionCommandMs = 0;
int st3020PanFeedbackPosition = -1;
int st3020TiltFeedbackPosition = -1;
int st3020PanVoltageTenths = -1;
int st3020TiltVoltageTenths = -1;
int st3020LastCommandedPanPosition = -1;
int st3020LastCommandedTiltPosition = -1;
St3020MotionStage st3020MotionStage = ST3020_MOTION_IDLE;
int st3020MotionCurrentPanPosition = -1;
int st3020MotionCurrentTiltPosition = -1;
int st3020MotionFinalPanPosition = -1;
int st3020MotionFinalTiltPosition = -1;
uint16_t st3020MotionSpeed = ST3020_DEFAULT_SPEED;
uint32_t st3020MotionDeadlineMs = 0;
uint32_t st3020MotionSettledSinceMs = 0;
uint8_t st3020MotionCorrectionAttempts = 0;
uint8_t st3020MotionFeedbackOvershootCount = 0;
WiFiClient remoteMqttNetClient;
PubSubClient remoteMqttClient(remoteMqttNetClient);

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

uint32_t lastStatusPrintMs = 0;
uint32_t lastRxMs = 0;
bool packetReceived = false;
bool blinkActive = false;
bool recenterRequested = false;
bool captureTargetRequested = false;
bool autoTrackEnabled = false;
bool precisionManualMode = false;
ControlMode controlMode = CONTROL_MODE_MANUAL;
bool targetDirectionValid = false;
bool sunTimeValid = false;
Vec3 targetDirection = {0.0f, 0.0f, 0.0f};
time_t heliostatStartUnixTimeUtc = HELIOSTAT_START_UNIX_TIME_UTC;
uint32_t heliostatTimeBaseMillis = 0;
uint32_t lastAutoTargetUpdateMs = 0;
float capturedPanAngleDeg = PAN_START_DEG;
float capturedTiltAngleDeg = TILT_START_DEG;
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
uint16_t scanMoveSpeed = SCAN_MOVE_SPEED_DEFAULT;
uint16_t scanDwellMs = SCAN_DWELL_DEFAULT_MS;
uint32_t scanPointReadyMs = 0;
bool scanLockValid = false;
float scanLockPanDeg = PAN_START_DEG;
float scanLockTiltDeg = TILT_START_DEG;
bool approxTargetValid = false;
float approxTargetBearingDeg = 180.0f;
float approxTargetElevationDeg = 0.0f;
float approxTargetPanDeg = PAN_START_DEG;
float approxTargetTiltDeg = TILT_START_DEG;
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE] = {};
size_t serialCommandLength = 0;

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

float panMinLimitDeg();
float panMaxLimitDeg();
float tiltMinLimitDeg();
float tiltMaxLimitDeg();

Vec3 reflectVector(const Vec3& incoming, const Vec3& normal) {
  return addVec3(incoming, scaleVec3(normal, -2.0f * dotVec3(incoming, normal)));
}

Vec3 mirrorNormalFromPanTilt(float panDeg, float tiltDeg) {
  const float panAzimuthDeg = (PAN_MODEL_SIGN * (panDeg - PAN_START_DEG)) + 180.0f;
  const float panRad = degToRad(panAzimuthDeg);
  const float tiltRad = degToRad(TILT_MODEL_SIGN * (tiltDeg - TILT_START_DEG));
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
  const float panModelDeg = PAN_START_DEG + (PAN_MODEL_SIGN * (panAzimuthDeg - 180.0f));
  panDeg = constrain(panModelDeg, PAN_MIN_DEG, PAN_MAX_DEG);
  const float tiltModelDeg = TILT_START_DEG + (TILT_MODEL_SIGN * radToDeg(asinf(normalized.z)));
  tiltDeg = constrain(tiltModelDeg, tiltMinLimitDeg(), tiltMaxLimitDeg());
}

bool currentUnixTimeUtc(time_t& unixTimeUtc) {
  if (heliostatStartUnixTimeUtc <= 0) {
    return false;
  }
  const uint32_t elapsedMs = millis() - heliostatTimeBaseMillis;
  unixTimeUtc = heliostatStartUnixTimeUtc + static_cast<time_t>(elapsedMs / 1000UL);
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

void bearingElevationFromVector(const Vec3& v, float& bearingDeg, float& elevationDeg) {
  const Vec3 normalized = normalizeVec3(v);
  bearingDeg = radToDeg(atan2f(normalized.y, normalized.x));
  if (bearingDeg < 0.0f) {
    bearingDeg += 360.0f;
  }
  elevationDeg = radToDeg(asinf(normalized.z));
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

void setHeliostatUnixTimeUtc(time_t unixTimeUtc) {
  heliostatStartUnixTimeUtc = unixTimeUtc;
  heliostatTimeBaseMillis = millis();
}
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

int st3020PanPositionMin() {
  return min(st3020Calibration.panPosAt90Deg,
             min(st3020Calibration.panPosAt180Deg, st3020Calibration.panPosAt270Deg));
}

int st3020PanPositionMax() {
  return max(st3020Calibration.panPosAt90Deg,
             max(st3020Calibration.panPosAt180Deg, st3020Calibration.panPosAt270Deg));
}

int st3020TiltPositionMin() {
  return min(st3020Calibration.tiltPosAtExt0Deg,
             min(st3020Calibration.tiltPosAtExt45Deg, st3020Calibration.tiltPosAtExt90Deg));
}

int st3020TiltPositionMax() {
  return max(st3020Calibration.tiltPosAtExt0Deg,
             max(st3020Calibration.tiltPosAtExt45Deg, st3020Calibration.tiltPosAtExt90Deg));
}

float st3020PanAngleDegFromPosition(int position);
float st3020TiltAngleDegFromPosition(int position);
float quantizeSt3020PanTargetDeg(float angleDeg);
float quantizeSt3020TiltTargetDeg(float angleDeg);

bool st3020CalibrationIsValid(const St3020Calibration& calibration) {
  const bool panOrdered =
      calibration.panPosAt90Deg < calibration.panPosAt180Deg &&
      calibration.panPosAt180Deg < calibration.panPosAt270Deg;
  const bool tiltOrdered =
      calibration.tiltPosAtExt0Deg < calibration.tiltPosAtExt45Deg &&
      calibration.tiltPosAtExt45Deg < calibration.tiltPosAtExt90Deg;
  const bool valuesInRange =
      calibration.panPosAt90Deg >= 0 && calibration.panPosAt90Deg <= 4095 &&
      calibration.panPosAt180Deg >= 0 && calibration.panPosAt180Deg <= 4095 &&
      calibration.panPosAt270Deg >= 0 && calibration.panPosAt270Deg <= 4095 &&
      calibration.tiltPosAtExt0Deg >= 0 && calibration.tiltPosAtExt0Deg <= 4095 &&
      calibration.tiltPosAtExt45Deg >= 0 && calibration.tiltPosAtExt45Deg <= 4095 &&
      calibration.tiltPosAtExt90Deg >= 0 && calibration.tiltPosAtExt90Deg <= 4095;
  return panOrdered && tiltOrdered && valuesInRange;
}

void refreshSt3020AnglesAndTargetsFromCalibration() {
  if (st3020PanFeedbackValid) {
    panAngleDeg = st3020PanAngleDegFromPosition(st3020PanFeedbackPosition);
  }
  if (st3020TiltFeedbackValid) {
    tiltAngleDeg = st3020TiltAngleDegFromPosition(st3020TiltFeedbackPosition);
  }
  panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
  tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
  capturedPanAngleDeg = quantizeSt3020PanTargetDeg(capturedPanAngleDeg);
  capturedTiltAngleDeg = quantizeSt3020TiltTargetDeg(capturedTiltAngleDeg);
  lastAutoReferencePanDeg = quantizeSt3020PanTargetDeg(lastAutoReferencePanDeg);
  lastAutoReferenceTiltDeg = quantizeSt3020TiltTargetDeg(lastAutoReferenceTiltDeg);
  scanCenterPanDeg = quantizeSt3020PanTargetDeg(scanCenterPanDeg);
  scanCenterTiltDeg = quantizeSt3020TiltTargetDeg(scanCenterTiltDeg);
  scanLockPanDeg = quantizeSt3020PanTargetDeg(scanLockPanDeg);
  scanLockTiltDeg = quantizeSt3020TiltTargetDeg(scanLockTiltDeg);
  approxTargetPanDeg = quantizeSt3020PanTargetDeg(approxTargetPanDeg);
  approxTargetTiltDeg = quantizeSt3020TiltTargetDeg(approxTargetTiltDeg);
  st3020LastCommandedPanPosition = -1;
  st3020LastCommandedTiltPosition = -1;
}

bool setActiveSt3020Calibration(const St3020Calibration& calibration) {
  if (!st3020CalibrationIsValid(calibration)) {
    return false;
  }
  st3020Calibration = calibration;
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    refreshSt3020AnglesAndTargetsFromCalibration();
  }
  return true;
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
  float position = 0.0f;
  if (clamped <= PAN_START_DEG) {
    const float segmentRatio =
        (clamped - ST3020_PAN_MIN_DEG) /
        (PAN_START_DEG - ST3020_PAN_MIN_DEG);
    position = static_cast<float>(st3020Calibration.panPosAt90Deg) +
               (segmentRatio * static_cast<float>(st3020Calibration.panPosAt180Deg -
                                                  st3020Calibration.panPosAt90Deg));
  } else {
    const float segmentRatio =
        (clamped - PAN_START_DEG) /
        (ST3020_PAN_MAX_DEG - PAN_START_DEG);
    position = static_cast<float>(st3020Calibration.panPosAt180Deg) +
               (segmentRatio * static_cast<float>(st3020Calibration.panPosAt270Deg -
                                                  st3020Calibration.panPosAt180Deg));
  }
  return constrain(static_cast<int>(lroundf(position)), st3020PanPositionMin(), st3020PanPositionMax());
}

int st3020TiltPositionFromAngleDeg(float angleDeg) {
  const float internalClamped = constrain(angleDeg, ST3020_TILT_MIN_DEG, ST3020_TILT_MAX_DEG);
  const float externalDeg = constrain(180.0f - internalClamped,
                                      ST3020_TILT_EXTERNAL_MIN_DEG,
                                      ST3020_TILT_EXTERNAL_MAX_DEG);

  float position = 0.0f;
  if (externalDeg <= ST3020_TILT_CAL_EXT_45_DEG) {
    const float segmentRatio =
        (externalDeg - ST3020_TILT_CAL_EXT_0_DEG) /
        (ST3020_TILT_CAL_EXT_45_DEG - ST3020_TILT_CAL_EXT_0_DEG);
    position = static_cast<float>(st3020Calibration.tiltPosAtExt0Deg) +
               (segmentRatio * static_cast<float>(st3020Calibration.tiltPosAtExt45Deg -
                                                  st3020Calibration.tiltPosAtExt0Deg));
  } else {
    const float segmentRatio =
        (externalDeg - ST3020_TILT_CAL_EXT_45_DEG) /
        (ST3020_TILT_CAL_EXT_90_DEG - ST3020_TILT_CAL_EXT_45_DEG);
    position = static_cast<float>(st3020Calibration.tiltPosAtExt45Deg) +
               (segmentRatio * static_cast<float>(st3020Calibration.tiltPosAtExt90Deg -
                                                  st3020Calibration.tiltPosAtExt45Deg));
  }

  return constrain(static_cast<int>(lroundf(position)), st3020TiltPositionMin(), st3020TiltPositionMax());
}

float st3020PanAngleDegFromPosition(int position) {
  const float clamped = static_cast<float>(constrain(position, st3020PanPositionMin(), st3020PanPositionMax()));

  float angleDeg = PAN_START_DEG;
  if (clamped <= static_cast<float>(st3020Calibration.panPosAt180Deg)) {
    const float segmentRatio =
        (clamped - static_cast<float>(st3020Calibration.panPosAt90Deg)) /
        static_cast<float>(st3020Calibration.panPosAt180Deg - st3020Calibration.panPosAt90Deg);
    angleDeg = ST3020_PAN_MIN_DEG +
               (segmentRatio * (PAN_START_DEG - ST3020_PAN_MIN_DEG));
  } else {
    const float segmentRatio =
        (clamped - static_cast<float>(st3020Calibration.panPosAt180Deg)) /
        static_cast<float>(st3020Calibration.panPosAt270Deg - st3020Calibration.panPosAt180Deg);
    angleDeg = PAN_START_DEG +
               (segmentRatio * (ST3020_PAN_MAX_DEG - PAN_START_DEG));
  }
  return constrain(angleDeg, ST3020_PAN_MIN_DEG, ST3020_PAN_MAX_DEG);
}

float st3020TiltAngleDegFromPosition(int position) {
  const float clamped = static_cast<float>(constrain(position, st3020TiltPositionMin(), st3020TiltPositionMax()));

  float externalDeg = 0.0f;
  if (clamped <= static_cast<float>(st3020Calibration.tiltPosAtExt45Deg)) {
    const float segmentRatio =
        (clamped - static_cast<float>(st3020Calibration.tiltPosAtExt0Deg)) /
        static_cast<float>(st3020Calibration.tiltPosAtExt45Deg - st3020Calibration.tiltPosAtExt0Deg);
    externalDeg = ST3020_TILT_CAL_EXT_0_DEG +
                  (segmentRatio * (ST3020_TILT_CAL_EXT_45_DEG - ST3020_TILT_CAL_EXT_0_DEG));
  } else {
    const float segmentRatio =
        (clamped - static_cast<float>(st3020Calibration.tiltPosAtExt45Deg)) /
        static_cast<float>(st3020Calibration.tiltPosAtExt90Deg - st3020Calibration.tiltPosAtExt45Deg);
    externalDeg = ST3020_TILT_CAL_EXT_45_DEG +
                  (segmentRatio * (ST3020_TILT_CAL_EXT_90_DEG - ST3020_TILT_CAL_EXT_45_DEG));
  }

  externalDeg = constrain(externalDeg, ST3020_TILT_EXTERNAL_MIN_DEG, ST3020_TILT_EXTERNAL_MAX_DEG);
  return constrain(180.0f - externalDeg, ST3020_TILT_MIN_DEG, ST3020_TILT_MAX_DEG);
}

float quantizeSt3020PanTargetDeg(float angleDeg) {
  return st3020PanAngleDegFromPosition(st3020PanPositionFromAngleDeg(angleDeg));
}

float quantizeSt3020TiltTargetDeg(float angleDeg) {
  return st3020TiltAngleDegFromPosition(st3020TiltPositionFromAngleDeg(angleDeg));
}

int st3020PositionDeltaForStep(float angleDeg,
                               float stepDeg,
                               float minDeg,
                               float maxDeg,
                               int (*positionFromAngle)(float)) {
  if (stepDeg <= 0.0f) {
    return 0;
  }

  const float forwardAngle = constrain(angleDeg + stepDeg, minDeg, maxDeg);
  const float backwardAngle = constrain(angleDeg - stepDeg, minDeg, maxDeg);
  const int currentPos = positionFromAngle(angleDeg);
  const int forwardDelta = abs(positionFromAngle(forwardAngle) - currentPos);
  const int backwardDelta = abs(currentPos - positionFromAngle(backwardAngle));
  return max(forwardDelta, backwardDelta);
}

uint32_t st3020ScanIntervalForTarget(int panTargetPos, int tiltTargetPos) {
  int panReferencePos = st3020LastCommandedPanPosition;
  int tiltReferencePos = st3020LastCommandedTiltPosition;

  if (panReferencePos < 0) {
    panReferencePos = st3020PanFeedbackValid ? st3020PanFeedbackPosition : st3020PanPositionFromAngleDeg(panAngleDeg);
  }
  if (tiltReferencePos < 0) {
    tiltReferencePos =
        st3020TiltFeedbackValid ? st3020TiltFeedbackPosition : st3020TiltPositionFromAngleDeg(tiltAngleDeg);
  }

  const int deltaPos = max(abs(panTargetPos - panReferencePos), abs(tiltTargetPos - tiltReferencePos));
  const uint32_t intervalMs =
      static_cast<uint32_t>((1000UL * static_cast<uint32_t>(max(1, deltaPos))) / max<uint16_t>(1, scanMoveSpeed));
  return constrain(intervalMs + SCAN_ST3020_STEP_MIN_INTERVAL_MS + static_cast<uint32_t>(scanDwellMs),
                   SCAN_ST3020_STEP_MIN_INTERVAL_MS,
                   SCAN_ST3020_STEP_MAX_INTERVAL_MS + static_cast<uint32_t>(SCAN_DWELL_MAX_MS));
}

bool scanPointReadyToAdvance(uint32_t nowMs) {
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    return nowMs >= scanPointReadyMs;
  }

  return fabsf(panAngleDeg - panTargetDeg) <= SCAN_SETTLE_TOLERANCE_DEG &&
         fabsf(tiltAngleDeg - tiltTargetDeg) <= SCAN_SETTLE_TOLERANCE_DEG;
}

void refreshSt3020TorqueState() {
  const int panReadback = st3020Bus.readByte(ST3020_PAN_ID, SMS_STS_TORQUE_ENABLE);
  const int tiltReadback = st3020Bus.readByte(ST3020_TILT_ID, SMS_STS_TORQUE_ENABLE);
  st3020TorqueEnabled = (panReadback == 1) || (tiltReadback == 1);
  // Serial.printf("ST3020 torque state: pan_rb=%d | tilt_rb=%d\n", panReadback, tiltReadback);
}

void setSt3020TorqueEnabled(bool enabled) {
  st3020Bus.EnableTorque(ST3020_PAN_ID, enabled ? 1 : 0);
  st3020Bus.EnableTorque(ST3020_TILT_ID, enabled ? 1 : 0);
  refreshSt3020TorqueState();
}

bool st3020FeedbackNearTarget(int panTargetPosition,
                              int tiltTargetPosition,
                              int tolerancePosition = ST3020_SETTLE_TOLERANCE_POS) {
  return st3020PanFeedbackValid &&
         st3020TiltFeedbackValid &&
         abs(st3020PanFeedbackPosition - panTargetPosition) <= tolerancePosition &&
         abs(st3020TiltFeedbackPosition - tiltTargetPosition) <= tolerancePosition;
}

uint32_t st3020EstimatedMoveDurationMs(int fromPanPosition,
                                       int toPanPosition,
                                       int fromTiltPosition,
                                       int toTiltPosition,
                                       uint16_t speed) {
  const int deltaPosition =
      max(abs(toPanPosition - fromPanPosition), abs(toTiltPosition - fromTiltPosition));
  const uint32_t motionMs =
      static_cast<uint32_t>((1000UL * static_cast<uint32_t>(max(1, deltaPosition))) /
                            max<uint16_t>(1, speed));
  return max<uint32_t>(ST3020_MOVE_MIN_COMMAND_MS, motionMs + ST3020_MOVE_TIMEOUT_MARGIN_MS);
}

int st3020StepPositionToward(int currentPosition, int targetPosition, int maxStepPosition) {
  if (currentPosition < targetPosition) {
    return min(currentPosition + maxStepPosition, targetPosition);
  }
  if (currentPosition > targetPosition) {
    return max(currentPosition - maxStepPosition, targetPosition);
  }
  return currentPosition;
}

int st3020OvershootPositionBeyondTarget(int referencePosition,
                                        int targetPosition,
                                        int overshootPosition,
                                        int minPosition,
                                        int maxPosition) {
  if (overshootPosition <= 0 || referencePosition == targetPosition) {
    return targetPosition;
  }
  if (referencePosition < targetPosition) {
    return min(targetPosition + overshootPosition, maxPosition);
  }
  return max(targetPosition - overshootPosition, minPosition);
}

void clearSt3020MotionState(bool releaseTorque = true) {
  st3020MotionStage = ST3020_MOTION_IDLE;
  st3020MotionCurrentPanPosition = -1;
  st3020MotionCurrentTiltPosition = -1;
  st3020MotionDeadlineMs = 0;
  st3020MotionSettledSinceMs = 0;
  st3020MotionCorrectionAttempts = 0;
  st3020MotionFeedbackOvershootCount = 0;
  if (releaseTorque && !ST3020_HOLD_TORQUE_ENABLED) {
    setSt3020TorqueEnabled(false);
  }
}

void updateSt3020Feedback(bool force = false) {
  const uint32_t nowMs = millis();
  if (!force && (nowMs - lastSt3020FeedbackMs) < ST3020_FEEDBACK_POLL_MS) {
    return;
  }
  lastSt3020FeedbackMs = nowMs;

  const int panFeedback = st3020Bus.FeedBack(ST3020_PAN_ID);
  st3020PanFeedbackValid = (panFeedback >= 0);
  if (st3020PanFeedbackValid) {
    st3020PanFeedbackPosition = st3020Bus.ReadPos(-1);
    st3020PanVoltageTenths = st3020Bus.ReadVoltage(-1);
    lastPanPulseUs = st3020PanFeedbackPosition;
    panAngleDeg = st3020PanAngleDegFromPosition(st3020PanFeedbackPosition);
  } else {
    st3020PanVoltageTenths = -1;
  }

  const int tiltFeedback = st3020Bus.FeedBack(ST3020_TILT_ID);
  st3020TiltFeedbackValid = (tiltFeedback >= 0);
  if (st3020TiltFeedbackValid) {
    st3020TiltFeedbackPosition = st3020Bus.ReadPos(-1);
    st3020TiltVoltageTenths = st3020Bus.ReadVoltage(-1);
    lastTiltPulseUs = st3020TiltFeedbackPosition;
    tiltAngleDeg = st3020TiltAngleDegFromPosition(st3020TiltFeedbackPosition);
  } else {
    st3020TiltVoltageTenths = -1;
  }

  refreshSt3020TorqueState();
}

void sendSt3020PositionCommand(int panPosition,
                               int tiltPosition,
                               uint16_t panSpeed,
                               uint16_t tiltSpeed,
                               bool force = false) {
  lastPanPulseUs = panPosition;
  lastTiltPulseUs = tiltPosition;
  if (!force &&
      panPosition == st3020LastCommandedPanPosition &&
      tiltPosition == st3020LastCommandedTiltPosition) {
    return;
  }

  lastSt3020MotionCommandMs = millis();
  const int panResult =
      st3020Bus.WritePosEx(ST3020_PAN_ID, static_cast<s16>(panPosition), panSpeed, ST3020_DEFAULT_ACC);
  const int tiltResult =
      st3020Bus.WritePosEx(ST3020_TILT_ID, static_cast<s16>(tiltPosition), tiltSpeed, ST3020_DEFAULT_ACC);
  st3020LastCommandedPanPosition = panPosition;
  st3020LastCommandedTiltPosition = tiltPosition;

  static uint32_t lastSt3020LogMs = 0;
  const uint32_t nowMs = millis();
  if ((nowMs - lastSt3020LogMs) >= 1000 || force) {
    lastSt3020LogMs = nowMs;
    Serial.printf("ST3020 write: pan=%d speed=%u r=%d | tilt=%d speed=%u r=%d\n",
                  panPosition, static_cast<unsigned>(panSpeed), panResult,
                  tiltPosition, static_cast<unsigned>(tiltSpeed), tiltResult);
  }
}

void beginSt3020MotionStage(St3020MotionStage stage,
                            int panPosition,
                            int tiltPosition,
                            uint16_t speed,
                            uint32_t nowMs,
                            bool force = false) {
  int referencePanPosition = st3020LastCommandedPanPosition;
  int referenceTiltPosition = st3020LastCommandedTiltPosition;
  if (st3020PanFeedbackValid) {
    referencePanPosition = st3020PanFeedbackPosition;
  } else if (referencePanPosition < 0) {
    referencePanPosition = st3020PanPositionFromAngleDeg(panAngleDeg);
  }
  if (st3020TiltFeedbackValid) {
    referenceTiltPosition = st3020TiltFeedbackPosition;
  } else if (referenceTiltPosition < 0) {
    referenceTiltPosition = st3020TiltPositionFromAngleDeg(tiltAngleDeg);
  }

  st3020MotionStage = stage;
  st3020MotionCurrentPanPosition = panPosition;
  st3020MotionCurrentTiltPosition = tiltPosition;
  st3020MotionSpeed = speed;
  st3020MotionSettledSinceMs = 0;
  st3020MotionDeadlineMs =
      nowMs + st3020EstimatedMoveDurationMs(referencePanPosition, panPosition,
                                            referenceTiltPosition, tiltPosition, speed);
  sendSt3020PositionCommand(panPosition, tiltPosition, speed, speed, force);
}

void scheduleSt3020AutoMotion(uint32_t nowMs) {
  st3020MotionFinalPanPosition = st3020PanPositionFromAngleDeg(panTargetDeg);
  st3020MotionFinalTiltPosition = st3020TiltPositionFromAngleDeg(tiltTargetDeg);
  st3020MotionCorrectionAttempts = 0;
  st3020MotionFeedbackOvershootCount = 0;

  if (ST3020_AUTO_APPROACH_ENABLED) {
    const float approachPanDeg = constrain(panTargetDeg + ST3020_AUTO_APPROACH_PAN_OFFSET_DEG,
                                           panMinLimitDeg(), panMaxLimitDeg());
    const float targetTiltExternalDeg = tiltExternalFromInternalDeg(tiltTargetDeg);
    const float approachTiltExternalDeg =
        constrain(targetTiltExternalDeg + ST3020_AUTO_APPROACH_TILT_OFFSET_DEG,
                  tiltExternalMinLimitDeg(), tiltExternalMaxLimitDeg());
    const int approachPanPosition = st3020PanPositionFromAngleDeg(approachPanDeg);
    const int approachTiltPosition =
        st3020TiltPositionFromAngleDeg(tiltInternalFromExternalDeg(approachTiltExternalDeg));
    if (approachPanPosition != st3020MotionFinalPanPosition ||
        approachTiltPosition != st3020MotionFinalTiltPosition) {
      beginSt3020MotionStage(ST3020_MOTION_APPROACH,
                             approachPanPosition,
                             approachTiltPosition,
                             ST3020_DEFAULT_SPEED,
                             nowMs);
      return;
    }
  }

  beginSt3020MotionStage(ST3020_MOTION_FINAL,
                         st3020MotionFinalPanPosition,
                         st3020MotionFinalTiltPosition,
                         ST3020_DEFAULT_SPEED,
                         nowMs);
}

void beginSt3020FeedbackTrimStep(uint32_t nowMs) {
  int referencePanPosition = st3020PanFeedbackValid ? st3020PanFeedbackPosition : st3020MotionCurrentPanPosition;
  int referenceTiltPosition = st3020TiltFeedbackValid ? st3020TiltFeedbackPosition : st3020MotionCurrentTiltPosition;
  if (referencePanPosition < 0) {
    referencePanPosition = st3020MotionFinalPanPosition;
  }
  if (referenceTiltPosition < 0) {
    referenceTiltPosition = st3020MotionFinalTiltPosition;
  }

  int trimPanPosition =
      st3020StepPositionToward(referencePanPosition, st3020MotionFinalPanPosition, ST3020_FEEDBACK_TRIM_STEP_POS);
  int trimTiltPosition =
      st3020StepPositionToward(referenceTiltPosition, st3020MotionFinalTiltPosition, ST3020_FEEDBACK_TRIM_STEP_POS);
  const bool canOvershootPan =
      st3020MotionFeedbackOvershootCount < ST3020_MAX_FEEDBACK_OVERSHOOTS &&
      referencePanPosition != st3020MotionFinalPanPosition &&
      trimPanPosition == st3020MotionFinalPanPosition;
  const bool canOvershootTilt =
      st3020MotionFeedbackOvershootCount < ST3020_MAX_FEEDBACK_OVERSHOOTS &&
      referenceTiltPosition != st3020MotionFinalTiltPosition &&
      trimTiltPosition == st3020MotionFinalTiltPosition;
  if (canOvershootPan) {
    trimPanPosition = st3020OvershootPositionBeyondTarget(referencePanPosition,
                                                          st3020MotionFinalPanPosition,
                                                          ST3020_FEEDBACK_OVERSHOOT_POS,
                                                          st3020PanPositionMin(),
                                                          st3020PanPositionMax());
  }
  if (canOvershootTilt) {
    trimTiltPosition = st3020OvershootPositionBeyondTarget(referenceTiltPosition,
                                                           st3020MotionFinalTiltPosition,
                                                           ST3020_FEEDBACK_OVERSHOOT_POS,
                                                           st3020TiltPositionMin(),
                                                           st3020TiltPositionMax());
  }
  if (canOvershootPan || canOvershootTilt) {
    ++st3020MotionFeedbackOvershootCount;
  }
  beginSt3020MotionStage(ST3020_MOTION_FINAL,
                         trimPanPosition,
                         trimTiltPosition,
                         ST3020_DEFAULT_SPEED,
                         nowMs,
                         true);
}

void serviceSt3020AutoMotion(uint32_t nowMs) {
  const int desiredFinalPanPosition = st3020PanPositionFromAngleDeg(panTargetDeg);
  const int desiredFinalTiltPosition = st3020TiltPositionFromAngleDeg(tiltTargetDeg);
  const bool finalTargetChanged =
      desiredFinalPanPosition != st3020MotionFinalPanPosition ||
      desiredFinalTiltPosition != st3020MotionFinalTiltPosition;
  const bool needsFeedbackCorrection =
      st3020PanFeedbackValid &&
      st3020TiltFeedbackValid &&
      !st3020FeedbackNearTarget(desiredFinalPanPosition, desiredFinalTiltPosition);
  const bool feedbackNearCurrentTarget =
      st3020FeedbackNearTarget(st3020MotionCurrentPanPosition, st3020MotionCurrentTiltPosition);
  const bool feedbackNearFinalTarget =
      st3020FeedbackNearTarget(st3020MotionFinalPanPosition, st3020MotionFinalTiltPosition);

  if (st3020MotionStage == ST3020_MOTION_IDLE) {
    if (finalTargetChanged) {
      scheduleSt3020AutoMotion(nowMs);
    } else if (ST3020_FEEDBACK_CORRECTION_ENABLED && needsFeedbackCorrection) {
      st3020MotionFinalPanPosition = desiredFinalPanPosition;
      st3020MotionFinalTiltPosition = desiredFinalTiltPosition;
      beginSt3020FeedbackTrimStep(nowMs);
    } else {
      if (st3020TorqueEnabled && !ST3020_HOLD_TORQUE_ENABLED) {
        setSt3020TorqueEnabled(false);
      }
      return;
    }
  } else if (finalTargetChanged) {
    scheduleSt3020AutoMotion(nowMs);
  }

  if (st3020MotionStage == ST3020_MOTION_IDLE) {
    return;
  }

  if (feedbackNearCurrentTarget) {
    if (st3020MotionSettledSinceMs == 0) {
      st3020MotionSettledSinceMs = nowMs;
    } else if ((nowMs - st3020MotionSettledSinceMs) >= ST3020_MOVE_SETTLE_MS) {
      if (st3020MotionStage == ST3020_MOTION_APPROACH) {
        beginSt3020MotionStage(ST3020_MOTION_FINAL,
                               st3020MotionFinalPanPosition,
                               st3020MotionFinalTiltPosition,
                               ST3020_DEFAULT_SPEED,
                               nowMs);
        return;
      }
      if (feedbackNearFinalTarget) {
        clearSt3020MotionState(true);
        return;
      }
      if (ST3020_FEEDBACK_CORRECTION_ENABLED &&
          st3020MotionCorrectionAttempts < ST3020_MAX_FINAL_CORRECTIONS) {
        ++st3020MotionCorrectionAttempts;
        beginSt3020FeedbackTrimStep(nowMs);
        return;
      }
      clearSt3020MotionState(true);
      return;
    }
  } else {
    st3020MotionSettledSinceMs = 0;
  }

  if (nowMs < st3020MotionDeadlineMs) {
    return;
  }

  if (st3020MotionStage == ST3020_MOTION_APPROACH) {
    beginSt3020MotionStage(ST3020_MOTION_FINAL,
                           st3020MotionFinalPanPosition,
                           st3020MotionFinalTiltPosition,
                           ST3020_DEFAULT_SPEED,
                           nowMs,
                           true);
    return;
  }

  if (ST3020_FEEDBACK_CORRECTION_ENABLED &&
      st3020PanFeedbackValid &&
      st3020TiltFeedbackValid &&
      st3020MotionCorrectionAttempts < ST3020_MAX_FINAL_CORRECTIONS) {
    ++st3020MotionCorrectionAttempts;
    beginSt3020FeedbackTrimStep(nowMs);
    return;
  }

  clearSt3020MotionState(true);
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
  Serial.printf("ST3020 pan map: 90deg=%d 180deg=%d 270deg=%d\n",
                st3020Calibration.panPosAt90Deg,
                st3020Calibration.panPosAt180Deg,
                st3020Calibration.panPosAt270Deg);

  const int panPing = st3020Bus.Ping(ST3020_PAN_ID);
  const int tiltPing = st3020Bus.Ping(ST3020_TILT_ID);
  Serial.printf("ST3020 ping: pan=%s tilt=%s\n",
                panPing >= 0 ? "ok" : "missing",
                tiltPing >= 0 ? "ok" : "missing");
  updateSt3020Feedback(true);
  Serial.printf("ST3020 feedback: pan=%d @ %.1fV | tilt=%d @ %.1fV\n",
                st3020PanFeedbackPosition, static_cast<float>(st3020PanVoltageTenths) / 10.0f,
                st3020TiltFeedbackPosition, static_cast<float>(st3020TiltVoltageTenths) / 10.0f);
  clearSt3020MotionState(true);
}

void writeRemoteActuatorsSt3020() {
  const int panPosition = st3020PanPositionFromAngleDeg(panTargetDeg);
  const int tiltPosition = st3020TiltPositionFromAngleDeg(tiltTargetDeg);
  uint16_t panSpeed = ST3020_DEFAULT_SPEED;
  uint16_t tiltSpeed = ST3020_DEFAULT_SPEED;
  if (scanActive) {
    panSpeed = scanMoveSpeed;
    tiltSpeed = scanMoveSpeed;
  }
  sendSt3020PositionCommand(panPosition, tiltPosition, panSpeed, tiltSpeed);
  updateSt3020Feedback();
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
  doc["tracking_model_stage"] = TRACKING_MODEL_STAGE;
  doc["tracking_model_version"] = TRACKING_MODEL_VERSION;
  doc["pan_deg"] = panAngleDeg;
  doc["tilt_deg"] = tiltExternalFromInternalDeg(tiltAngleDeg);
  doc["pan_target_deg"] = panTargetDeg;
  doc["tilt_target_deg"] = tiltExternalFromInternalDeg(tiltTargetDeg);
  doc["pan_min_deg"] = panMinLimitDeg();
  doc["pan_max_deg"] = panMaxLimitDeg();
  doc["tilt_min_deg"] = tiltExternalMinLimitDeg();
  doc["tilt_max_deg"] = tiltExternalMaxLimitDeg();
  doc["pan_pos"] = lastPanPulseUs;
  doc["tilt_pos"] = lastTiltPulseUs;
  if (st3020LastCommandedPanPosition >= 0) {
    doc["pan_command_pos"] = st3020LastCommandedPanPosition;
  } else {
    doc["pan_command_pos"] = nullptr;
  }
  if (st3020LastCommandedTiltPosition >= 0) {
    doc["tilt_command_pos"] = st3020LastCommandedTiltPosition;
  } else {
    doc["tilt_command_pos"] = nullptr;
  }
  if (st3020PanFeedbackValid) {
    doc["pan_feedback_pos"] = st3020PanFeedbackPosition;
  } else {
    doc["pan_feedback_pos"] = nullptr;
  }
  if (st3020TiltFeedbackValid) {
    doc["tilt_feedback_pos"] = st3020TiltFeedbackPosition;
  } else {
    doc["tilt_feedback_pos"] = nullptr;
  }
  doc["servo_backend"] = remoteActuatorBackendName();
  doc["torque_hold_enabled"] = ST3020_HOLD_TORQUE_ENABLED;
  doc["torque_enabled"] = st3020TorqueEnabled;
  doc["st3020_motion_stage"] =
      st3020MotionStage == ST3020_MOTION_APPROACH ? "approach" :
      st3020MotionStage == ST3020_MOTION_FINAL ? "final" : "idle";
  doc["auto_update_interval_ms"] = AUTO_TARGET_UPDATE_INTERVAL_MS;
  doc["st3020_auto_approach_enabled"] = ST3020_AUTO_APPROACH_ENABLED;
  doc["st3020_auto_approach_pan_offset_deg"] = ST3020_AUTO_APPROACH_PAN_OFFSET_DEG;
  doc["st3020_auto_approach_tilt_offset_deg"] = ST3020_AUTO_APPROACH_TILT_OFFSET_DEG;
  doc["st3020_feedback_correction_enabled"] = ST3020_FEEDBACK_CORRECTION_ENABLED;
  doc["st3020_feedback_trim_step_pos"] = ST3020_FEEDBACK_TRIM_STEP_POS;
  doc["st3020_feedback_overshoot_pos"] = ST3020_FEEDBACK_OVERSHOOT_POS;
  doc["st3020_max_feedback_overshoots"] = ST3020_MAX_FEEDBACK_OVERSHOOTS;
  doc["st3020_feedback_overshoot_count"] = st3020MotionFeedbackOvershootCount;
  doc["st3020_settle_tolerance_pos"] = ST3020_SETTLE_TOLERANCE_POS;
  doc["st3020_pan_pos_at_90_deg"] = st3020Calibration.panPosAt90Deg;
  doc["st3020_pan_pos_at_180_deg"] = st3020Calibration.panPosAt180Deg;
  doc["st3020_pan_pos_at_270_deg"] = st3020Calibration.panPosAt270Deg;
  doc["st3020_tilt_pos_at_ext_0_deg"] = st3020Calibration.tiltPosAtExt0Deg;
  doc["st3020_tilt_pos_at_ext_45_deg"] = st3020Calibration.tiltPosAtExt45Deg;
  doc["st3020_tilt_pos_at_ext_90_deg"] = st3020Calibration.tiltPosAtExt90Deg;
  doc["st3020_default_pan_pos_at_90_deg"] = ST3020_DEFAULT_CALIBRATION.panPosAt90Deg;
  doc["st3020_default_pan_pos_at_180_deg"] = ST3020_DEFAULT_CALIBRATION.panPosAt180Deg;
  doc["st3020_default_pan_pos_at_270_deg"] = ST3020_DEFAULT_CALIBRATION.panPosAt270Deg;
  doc["st3020_default_tilt_pos_at_ext_0_deg"] = ST3020_DEFAULT_CALIBRATION.tiltPosAtExt0Deg;
  doc["st3020_default_tilt_pos_at_ext_45_deg"] = ST3020_DEFAULT_CALIBRATION.tiltPosAtExt45Deg;
  doc["st3020_default_tilt_pos_at_ext_90_deg"] = ST3020_DEFAULT_CALIBRATION.tiltPosAtExt90Deg;
  doc["pan_feedback_ok"] = st3020PanFeedbackValid;
  doc["tilt_feedback_ok"] = st3020TiltFeedbackValid;
  if (st3020PanVoltageTenths >= 0) {
    doc["pan_voltage_v"] = static_cast<float>(st3020PanVoltageTenths) / 10.0f;
  }
  if (st3020TiltVoltageTenths >= 0) {
    doc["tilt_voltage_v"] = static_cast<float>(st3020TiltVoltageTenths) / 10.0f;
  }
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
  doc["scan_move_speed"] = scanMoveSpeed;
  doc["scan_dwell_ms"] = scanDwellMs;
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
    const Vec3 sunDirection = sunVectorFromUnixTime(unixTimeUtc);
    float sunBearingDeg = 0.0f;
    float sunElevationDeg = 0.0f;
    float currentNormalBearingDeg = 0.0f;
    float currentNormalElevationDeg = 0.0f;
    bearingElevationFromVector(sunDirection, sunBearingDeg, sunElevationDeg);
    const Vec3 currentNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);
    const Vec3 currentBeamDirection = normalizeVec3(reflectVector(scaleVec3(sunDirection, -1.0f), currentNormal));
    float currentBeamBearingDeg = 0.0f;
    float currentBeamElevationDeg = 0.0f;
    bearingElevationFromVector(currentNormal, currentNormalBearingDeg, currentNormalElevationDeg);
    bearingElevationFromVector(currentBeamDirection, currentBeamBearingDeg, currentBeamElevationDeg);
    doc["sun_bearing_deg"] = sunBearingDeg;
    doc["sun_elevation_deg"] = sunElevationDeg;
    doc["normal_bearing_deg"] = currentNormalBearingDeg;
    doc["normal_elevation_deg"] = currentNormalElevationDeg;
    doc["beam_bearing_deg"] = currentBeamBearingDeg;
    doc["beam_elevation_deg"] = currentBeamElevationDeg;
    if (targetDirectionValid) {
      float targetBearingDeg = 0.0f;
      float targetElevationDeg = 0.0f;
      bearingElevationFromVector(targetDirection, targetBearingDeg, targetElevationDeg);
      doc["target_bearing_deg"] = targetBearingDeg;
      doc["target_elevation_deg"] = targetElevationDeg;

      Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirection));
      if (lengthVec3(desiredNormal) > 0.0f && dotVec3(currentNormal, desiredNormal) < 0.0f) {
        desiredNormal = scaleVec3(desiredNormal, -1.0f);
      }
      float desiredNormalBearingDeg = 0.0f;
      float desiredNormalElevationDeg = 0.0f;
      float predictedPanDeg = 0.0f;
      float predictedTiltDeg = 0.0f;
      bearingElevationFromVector(desiredNormal, desiredNormalBearingDeg, desiredNormalElevationDeg);
      panTiltFromMirrorNormal(desiredNormal, predictedPanDeg, predictedTiltDeg);
      doc["desired_normal_bearing_deg"] = desiredNormalBearingDeg;
      doc["desired_normal_elevation_deg"] = desiredNormalElevationDeg;
      doc["predicted_pan_deg"] = predictedPanDeg;
      doc["predicted_tilt_deg"] = tiltExternalFromInternalDeg(predictedTiltDeg);
      doc["pan_tracking_error_deg"] = predictedPanDeg - panAngleDeg;
      doc["tilt_tracking_error_deg"] =
          tiltExternalFromInternalDeg(predictedTiltDeg) - tiltExternalFromInternalDeg(tiltAngleDeg);
    }
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
  scanPointReadyMs = 0;
}

void pauseScan() {
  scanActive = false;
  scanPaused = true;
  scanPointReadyMs = 0;
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
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
    scanPointReadyMs = millis() + st3020ScanIntervalForTarget(
                                      st3020PanPositionFromAngleDeg(panTargetDeg),
                                      st3020TiltPositionFromAngleDeg(tiltTargetDeg));
  }
}

void startScan(ScanStage stage,
               float centerPanDeg,
               float centerTiltDeg,
               int moveSpeedOverride = -1,
               int dwellOverrideMs = -1,
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
  } else if (stage == SCAN_STAGE_FINE) {
    scanRangePanDeg = SCAN_FINE_RANGE_PAN_DEG;
    scanRangeTiltDeg = SCAN_FINE_RANGE_TILT_DEG;
    scanStepDeg = SCAN_FINE_STEP_DEG;
  } else {
    scanRangePanDeg = SCAN_MICRO_RANGE_PAN_DEG;
    scanRangeTiltDeg = SCAN_MICRO_RANGE_TILT_DEG;
    scanStepDeg = SCAN_MICRO_STEP_DEG;
  }
  scanMoveSpeed = SCAN_MOVE_SPEED_DEFAULT;
  if (moveSpeedOverride > 0) {
    scanMoveSpeed = static_cast<uint16_t>(constrain(moveSpeedOverride,
                                                    static_cast<int>(SCAN_MOVE_SPEED_MIN),
                                                    static_cast<int>(SCAN_MOVE_SPEED_MAX)));
  }
  scanDwellMs = SCAN_DWELL_DEFAULT_MS;
  if (dwellOverrideMs >= 0) {
    scanDwellMs = static_cast<uint16_t>(constrain(dwellOverrideMs,
                                                  static_cast<int>(SCAN_DWELL_MIN_MS),
                                                  static_cast<int>(SCAN_DWELL_MAX_MS)));
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
      setScanTargetForIndex(scanPointIndex);
    }
    return;
  }

  const float defaultCenterPan = scanLockValid ? scanLockPanDeg : panTargetDeg;
  const float defaultCenterTilt = scanLockValid ? scanLockTiltDeg : tiltTargetDeg;
  const float centerPan = doc["center_pan_deg"] | defaultCenterPan;
  const float centerTiltExternal = doc["center_tilt_deg"] | tiltExternalFromInternalDeg(defaultCenterTilt);
  int moveSpeed = doc["move_speed"] | -1;
  const int dwellMs = doc["dwell_ms"] | -1;
  if (moveSpeed <= 0) {
    moveSpeed = static_cast<int>(doc["move_speed_deg_per_sec"] | -1.0f);
  }
  const char* directionString = doc["direction"] | "forward";
  const int8_t direction = (strcmp(directionString, "backward") == 0) ? -1 : 1;
  const float rangePanDeg = doc["range_pan_deg"] | -1.0f;
  const float rangeTiltDeg = doc["range_tilt_deg"] | -1.0f;

  if (strcmp(stage, "coarse") == 0) {
    startScan(SCAN_STAGE_COARSE, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), moveSpeed, dwellMs, rangePanDeg, rangeTiltDeg, direction);
  } else if (strcmp(stage, "fine") == 0) {
    startScan(SCAN_STAGE_FINE, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), moveSpeed, dwellMs, rangePanDeg, rangeTiltDeg, direction);
  } else if (strcmp(stage, "micro") == 0) {
    startScan(SCAN_STAGE_MICRO, centerPan, tiltInternalFromExternalDeg(centerTiltExternal), moveSpeed, dwellMs, rangePanDeg, rangeTiltDeg, direction);
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
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    }
  } else if (strcmp(axis, "tilt") == 0) {
    tiltTargetDeg = constrain(tiltTargetDeg + deltaDeg, tiltMinLimitDeg(), tiltMaxLimitDeg());
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
    }
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
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
  }
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
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    approxTargetPanDeg = quantizeSt3020PanTargetDeg(approxTargetPanDeg);
    approxTargetTiltDeg = quantizeSt3020TiltTargetDeg(approxTargetTiltDeg);
  }
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
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
      tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
    }
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
  if (unixTimeUtc > 0) {
    setHeliostatUnixTimeUtc(static_cast<time_t>(unixTimeUtc));
    ntpTimeValid = false;
  }
}

void handleRemoteLocationCommand(const JsonDocument& doc) {
  const float requestedLatitudeDeg = doc["latitude_deg"] | heliostatLatitudeDeg;
  const float requestedLongitudeDeg = doc["longitude_deg"] | heliostatLongitudeDeg;
  heliostatLatitudeDeg = constrain(requestedLatitudeDeg, -90.0f, 90.0f);
  if (requestedLongitudeDeg >= -180.0f && requestedLongitudeDeg <= 180.0f) {
    heliostatLongitudeDeg = requestedLongitudeDeg;
  } else {
    const double normalizedLongitudeDeg =
        fmod(static_cast<double>(requestedLongitudeDeg) + 540.0, 360.0) - 180.0;
    heliostatLongitudeDeg = static_cast<float>(normalizedLongitudeDeg);
  }
  publishRemoteDiag("location_updated");
}

void handleRemoteCalibrationCommand(const JsonDocument& doc) {
  const char* action = doc["action"] | "apply";
  if (strcmp(action, "reset_default") == 0) {
    if (setActiveSt3020Calibration(ST3020_DEFAULT_CALIBRATION)) {
      publishRemoteDiag("calibration_default_applied");
    }
    return;
  }

  St3020Calibration requested = st3020Calibration;
  bool touched = false;

  if (!doc["pan_pos_at_90_deg"].isNull()) {
    requested.panPosAt90Deg = doc["pan_pos_at_90_deg"].as<int>();
    touched = true;
  }
  if (!doc["pan_pos_at_180_deg"].isNull()) {
    requested.panPosAt180Deg = doc["pan_pos_at_180_deg"].as<int>();
    touched = true;
  }
  if (!doc["pan_pos_at_270_deg"].isNull()) {
    requested.panPosAt270Deg = doc["pan_pos_at_270_deg"].as<int>();
    touched = true;
  }
  if (!doc["tilt_pos_at_ext_0_deg"].isNull()) {
    requested.tiltPosAtExt0Deg = doc["tilt_pos_at_ext_0_deg"].as<int>();
    touched = true;
  }
  if (!doc["tilt_pos_at_ext_45_deg"].isNull()) {
    requested.tiltPosAtExt45Deg = doc["tilt_pos_at_ext_45_deg"].as<int>();
    touched = true;
  }
  if (!doc["tilt_pos_at_ext_90_deg"].isNull()) {
    requested.tiltPosAtExt90Deg = doc["tilt_pos_at_ext_90_deg"].as<int>();
    touched = true;
  }

  if (!touched) {
    return;
  }

  if (!setActiveSt3020Calibration(requested)) {
    Serial.println("ST3020 calibration rejected: invalid ordering or out-of-range values");
    publishRemoteDiag("calibration_invalid");
    return;
  }

  publishRemoteDiag("calibration_updated");
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
  } else if (topicString == mqttTopic("cmd/calibration")) {
    handleRemoteCalibrationCommand(doc);
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
    if ((nowMs - lastNtpApplyMs) >= REMOTE_NTP_RESYNC_MS || heliostatStartUnixTimeUtc <= 0) {
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
  remoteMqttClient.subscribe(mqttTopic("cmd/calibration").c_str());
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
    const bool enteringAuto = (controlMode != CONTROL_MODE_AUTO_TRACK);
    const bool updateTargetNow =
        enteringAuto || lastAutoTargetUpdateMs == 0 ||
        (nowMs - lastAutoTargetUpdateMs) >= AUTO_TARGET_UPDATE_INTERVAL_MS;
    if (updateTargetNow) {
      Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirection));
      if (lengthVec3(desiredNormal) > 0.0f) {
        const Vec3 currentNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);
        if (dotVec3(currentNormal, desiredNormal) < 0.0f) {
          desiredNormal = scaleVec3(desiredNormal, -1.0f);
        }
        panTiltFromMirrorNormal(desiredNormal, panTargetDeg, tiltTargetDeg);
        if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
          panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
          tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
        }
        lastAutoReferencePanDeg = panTargetDeg;
        lastAutoReferenceTiltDeg = tiltTargetDeg;
        lastAutoTargetUpdateMs = nowMs;
      }
    }
    if (enteringAuto) {
      autoTrackStartUnixTimeUtc = unixTimeUtc;
    }
    controlMode = CONTROL_MODE_AUTO_TRACK;
  } else if (controlMode == CONTROL_MODE_AUTO_TRACK) {
    controlMode = CONTROL_MODE_TARGET_CAPTURED;
    lastAutoTargetUpdateMs = 0;
    clearSt3020MotionState(true);
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
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
  }
}

void updateScanState(uint32_t nowMs) {
  (void)nowMs;
  if (!scanActive || controlMode == CONTROL_MODE_AUTO_TRACK || scanPointsTotal == 0) {
    return;
  }

  const bool onTarget = scanPointReadyToAdvance(nowMs);

  if (!onTarget) {
    return;
  }

  const int32_t nextIndex = static_cast<int32_t>(scanPointIndex) + static_cast<int32_t>(scanDirection);
  if (nextIndex < 0 || nextIndex >= static_cast<int32_t>(scanPointsTotal)) {
    stopScan();
    publishRemoteDiag("scan_completed");
    return;
  }

  scanPointIndex = static_cast<uint16_t>(nextIndex);
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

  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
    updateSt3020Feedback();
    if (controlMode == CONTROL_MODE_AUTO_TRACK && !scanActive) {
      serviceSt3020AutoMotion(nowMs);
      return;
    }
    if (st3020MotionStage != ST3020_MOTION_IDLE) {
      clearSt3020MotionState(true);
    }
    writeRemoteActuators();
    return;
  }

  const float slewSpeedDegPerSec = SERVO_MAX_SLEW_DEG_PER_SEC;
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

void handleSerialCommandLine(const char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  String normalized(line);
  normalized.trim();
  line = normalized.c_str();

  if (strcmp(line, "POS?") == 0 || strcmp(line, "pos?") == 0 ||
      strcmp(line, "POS") == 0 || strcmp(line, "pos") == 0 ||
      strcmp(line, "STATE?") == 0 || strcmp(line, "state?") == 0) {
    updateSt3020Feedback(true);
    Serial.printf(
        "REMOTE_POS | pan_deg=%.3f pan_target_deg=%.3f pan_target_pos=%d pan_feedback_pos=%d | tilt_deg=%.3f tilt_target_deg=%.3f tilt_target_pos=%d tilt_feedback_pos=%d\n",
        panAngleDeg,
        panTargetDeg,
        st3020PanPositionFromAngleDeg(panTargetDeg),
        st3020PanFeedbackPosition,
        tiltExternalFromInternalDeg(tiltAngleDeg),
        tiltExternalFromInternalDeg(tiltTargetDeg),
        st3020TiltPositionFromAngleDeg(tiltTargetDeg),
        st3020TiltFeedbackPosition);
    return;
  }

  if (strcmp(line, "CAL?") == 0 || strcmp(line, "cal?") == 0) {
    Serial.printf(
        "REMOTE_CAL | pan_model_sign=%+.0f pan_servo_sign=%+.0f pan_start=%.1f | tilt_model_sign=%+.0f tilt_servo_sign=%+.0f tilt_start_internal=%.1f tilt_start_external=%.1f\n",
        PAN_MODEL_SIGN,
        ST3020_PAN_SIGN,
        PAN_START_DEG,
        TILT_MODEL_SIGN,
        ST3020_TILT_SIGN,
        TILT_START_DEG,
        tiltExternalFromInternalDeg(TILT_START_DEG));
    return;
  }

  if (strcmp(line, "SUN?") == 0 || strcmp(line, "sun?") == 0) {
    time_t unixTimeUtc = 0;
    if (!currentUnixTimeUtc(unixTimeUtc)) {
      Serial.println("SUN: time unavailable");
      return;
    }
    const Vec3 sunDirection = sunVectorFromUnixTime(unixTimeUtc);
    float sunBearingDeg = 0.0f;
    float sunElevationDeg = 0.0f;
    bearingElevationFromVector(sunDirection, sunBearingDeg, sunElevationDeg);
    Serial.printf(
        "SUN | utc=%lld | bearing=%.3f elevation=%.3f | vec=(%.4f, %.4f, %.4f)\n",
        static_cast<long long>(unixTimeUtc),
        sunBearingDeg,
        sunElevationDeg,
        sunDirection.x,
        sunDirection.y,
        sunDirection.z);
    return;
  }

  if (strcmp(line, "AUTO?") == 0 || strcmp(line, "auto?") == 0) {
    time_t unixTimeUtc = 0;
    if (!currentUnixTimeUtc(unixTimeUtc)) {
      Serial.println("AUTO: time unavailable");
      return;
    }
    const Vec3 sunDirection = sunVectorFromUnixTime(unixTimeUtc);
    const Vec3 currentNormal = mirrorNormalFromPanTilt(panAngleDeg, tiltAngleDeg);

    float sunBearingDeg = 0.0f;
    float sunElevationDeg = 0.0f;
    float currentNormalBearingDeg = 0.0f;
    float currentNormalElevationDeg = 0.0f;
    bearingElevationFromVector(sunDirection, sunBearingDeg, sunElevationDeg);
    bearingElevationFromVector(currentNormal, currentNormalBearingDeg, currentNormalElevationDeg);

    Serial.printf(
        "AUTO | mode=%s | pan=%.3f tilt=%.3f | normal_bearing=%.3f normal_elevation=%.3f | sun_bearing=%.3f sun_elevation=%.3f | target_valid=%s\n",
        controlModeName(controlMode),
        panAngleDeg,
        tiltExternalFromInternalDeg(tiltAngleDeg),
        currentNormalBearingDeg,
        currentNormalElevationDeg,
        sunBearingDeg,
        sunElevationDeg,
        targetDirectionValid ? "yes" : "no");

    if (targetDirectionValid) {
      float targetBearingDeg = 0.0f;
      float targetElevationDeg = 0.0f;
      bearingElevationFromVector(targetDirection, targetBearingDeg, targetElevationDeg);
      Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirection));
      if (lengthVec3(desiredNormal) > 0.0f && dotVec3(currentNormal, desiredNormal) < 0.0f) {
        desiredNormal = scaleVec3(desiredNormal, -1.0f);
      }
      float desiredNormalBearingDeg = 0.0f;
      float desiredNormalElevationDeg = 0.0f;
      float predictedPanDeg = 0.0f;
      float predictedTiltDeg = 0.0f;
      bearingElevationFromVector(desiredNormal, desiredNormalBearingDeg, desiredNormalElevationDeg);
      panTiltFromMirrorNormal(desiredNormal, predictedPanDeg, predictedTiltDeg);
      Serial.printf(
          "AUTO | target_bearing=%.3f target_elevation=%.3f | desired_normal_bearing=%.3f desired_normal_elevation=%.3f | predicted_pan=%.3f predicted_tilt=%.3f\n",
          targetBearingDeg,
          targetElevationDeg,
          desiredNormalBearingDeg,
          desiredNormalElevationDeg,
          predictedPanDeg,
          tiltExternalFromInternalDeg(predictedTiltDeg));
    }
    return;
  }

  if (strncmp(line, "PAN=", 4) == 0 || strncmp(line, "pan=", 4) == 0) {
    const float parsed = atof(line + 4);
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    panTargetDeg = constrain(parsed, panMinLimitDeg(), panMaxLimitDeg());
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    }
    Serial.printf("PAN target set: %.3f\n", panTargetDeg);
    return;
  }

  if (strncmp(line, "TILT=", 5) == 0 || strncmp(line, "tilt=", 5) == 0) {
    const float parsed = atof(line + 5);
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    tiltTargetDeg = constrain(
        tiltInternalFromExternalDeg(parsed),
        tiltMinLimitDeg(),
        tiltMaxLimitDeg());
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
    }
    Serial.printf("TILT target set: %.3f\n", tiltExternalFromInternalDeg(tiltTargetDeg));
    return;
  }

  if (strncmp(line, "JP=", 3) == 0 || strncmp(line, "jp=", 3) == 0) {
    const float parsed = atof(line + 3);
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    panTargetDeg = constrain(panTargetDeg + parsed, panMinLimitDeg(), panMaxLimitDeg());
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      panTargetDeg = quantizeSt3020PanTargetDeg(panTargetDeg);
    }
    Serial.printf("PAN jog target: %.3f\n", panTargetDeg);
    return;
  }

  if (strncmp(line, "JT=", 3) == 0 || strncmp(line, "jt=", 3) == 0) {
    const float parsed = atof(line + 3);
    stopScan();
    autoTrackEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    remotePanInput = 0.0f;
    remoteTiltInput = 0.0f;
    precisionManualMode = false;
    tiltTargetDeg = constrain(tiltTargetDeg + parsed, tiltMinLimitDeg(), tiltMaxLimitDeg());
    if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_ST3020) {
      tiltTargetDeg = quantizeSt3020TiltTargetDeg(tiltTargetDeg);
    }
    Serial.printf("TILT jog target: %.3f\n", tiltExternalFromInternalDeg(tiltTargetDeg));
    return;
  }

  if (strcmp(line, "HELP") == 0 || strcmp(line, "help") == 0) {
    Serial.println("Serial commands:");
    Serial.println("  POS? / POS            show target/feedback degrees and positions");
    Serial.println("  CAL?                  show model/servo sign configuration");
    Serial.println("  SUN?                  show computed sun bearing/elevation");
    Serial.println("  AUTO?                 show auto-track geometry diagnostics");
    Serial.println("  PAN=<deg>             set pan target directly");
    Serial.println("  TILT=<deg>            set public tilt target directly");
    Serial.println("  JP=<delta_deg>        jog pan target by delta");
    Serial.println("  JT=<delta_deg>        jog tilt target by delta (internal sign)");
    return;
  }

  if (line[0] != '\0') {
    Serial.printf("Unknown serial command: %s\n", line);
  }
}

void handleSerialInput() {
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

void printStatus(uint32_t nowMs) {
  const bool logsEnabled = ENABLE_RUNTIME_STATUS_LOGS;
  if (!logsEnabled) {
    return;
  }

  if ((nowMs - lastStatusPrintMs) < STATUS_PRINT_MS) {
    return;
  }
  lastStatusPrintMs = nowMs;

  const uint32_t ageMs = packetReceived ? (nowMs - lastRxMs) : 0;
  Serial.printf(
      "ROLE=remote | mode=%s | wifi=%s | mqtt=%s | ntp=%s | rx=%s | age_ms=%lu | pan=%7.3f | tilt=%7.3f | target=%7.3f/%7.3f | pan_pos=%d | tilt_pos=%d | pan_v=%.1f | tilt_v=%.1f | sun_time=%s | target=%s\n",
      controlModeName(controlMode),
      wifiLinkOk ? "ok" : "down",
      mqttLinkOk ? "ok" : "down",
      ntpTimeValid ? "ok" : "down",
      packetReceived ? "ok" : "waiting",
      static_cast<unsigned long>(ageMs),
      panAngleDeg, tiltExternalFromInternalDeg(tiltAngleDeg),
      panTargetDeg, tiltExternalFromInternalDeg(tiltTargetDeg),
      lastPanPulseUs, lastTiltPulseUs,
      static_cast<float>(st3020PanVoltageTenths) / 10.0f,
      static_cast<float>(st3020TiltVoltageTenths) / 10.0f,
      sunTimeValid ? "ok" : "missing",
      targetDirectionValid ? "ok" : "missing");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  waitForSerialIfEnabled();
  delay(300);

  beginStatusLed();
  setStatusLedBlue();
  delay(100);

  lastStatusPrintMs = millis();

  beginRemoteActuators();
  beginRemoteMqtt();

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

  Serial.println("Remote node ready.");
  Serial.printf("WiFi target SSID=%s | MQTT=%s:%u | topic=%s\n",
                REMOTE_WIFI_SSID_VALUE, REMOTE_MQTT_HOST_VALUE,
                static_cast<unsigned>(REMOTE_MQTT_PORT_VALUE), REMOTE_MQTT_BASE_TOPIC_VALUE);
  Serial.printf("Actuator backend: %s\n", remoteActuatorBackendName());
  if (REMOTE_ACTUATOR_BACKEND == REMOTE_ACTUATOR_BACKEND_PWM) {
    Serial.printf("Servos: pan GPIO=%d | tilt GPIO=%d | %d Hz\n",
                  PAN_SERVO_PIN, TILT_SERVO_PIN, SERVO_FREQUENCY_HZ);
  }
  Serial.println("Pan model reference: pan=180 deg means mirror normal points south.");
  Serial.printf("Tilt model/servo: model %.1f deg -> servo %.1f deg | offset=%+.1f deg\n",
                TILT_START_DEG, TILT_SERVO_AT_MODEL_HORIZON_DEG, TILT_SERVO_OFFSET_DEG);
  Serial.printf("Servo command step: %d us\n", SERVO_COMMAND_STEP_US);
  Serial.printf("Slew rate: %.2f deg/s\n", SERVO_MAX_SLEW_DEG_PER_SEC);
  Serial.printf("Heliostat lat=%.4f lon=%.4f start_unix=%lld\n",
                HELIOSTAT_LATITUDE_DEG, HELIOSTAT_LONGITUDE_DEG,
                static_cast<long long>(heliostatStartUnixTimeUtc));
  Serial.printf("Runtime status logs: %s\n", ENABLE_RUNTIME_STATUS_LOGS ? "on" : "off");
  Serial.println("Use a dedicated 5-6V supply for MG996R servos and share GND with the ESP32.");
}

void loop() {
  const uint32_t nowMs = millis();
  handleSerialInput();
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
  printStatus(nowMs);
}
