#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEController.h>
#include <BLEControllerRegistry.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>

namespace {

#if defined(DEVICE_ROLE_CONTROLLER) && defined(DEVICE_ROLE_REMOTE)
#error "Choose only one role: DEVICE_ROLE_CONTROLLER or DEVICE_ROLE_REMOTE"
#endif

#if !defined(DEVICE_ROLE_CONTROLLER) && !defined(DEVICE_ROLE_REMOTE)
#error "Define DEVICE_ROLE_CONTROLLER or DEVICE_ROLE_REMOTE in platformio.ini"
#endif

constexpr bool STATUS_LED_ENABLED = true;
constexpr int STATUS_LED_PIN = 38;
constexpr int STATUS_LED_COUNT = 1;
constexpr uint8_t LED_BRIGHTNESS = 32;

constexpr float PAN_MIN_DEG = 15.0f;
constexpr float PAN_MAX_DEG = 165.0f;
constexpr float PAN_START_DEG = 90.0f;

constexpr float TILT_MIN_DEG = 20.0f;
constexpr float TILT_MAX_DEG = 160.0f;
constexpr float TILT_START_DEG = 90.0f;

constexpr float STICK_DEADZONE = 0.12f;
constexpr float MAX_TARGET_SPEED_DEG_PER_SEC = 45.0f;
constexpr float SERVO_MAX_SLEW_DEG_PER_SEC = 60.0f;
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
constexpr int SERVO_MIN_PULSE_US = 500 ;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr int SERVO_COMMAND_STEP_US = 1;
constexpr int SERVO_FREQUENCY_HZ = 300;
constexpr float SPEED_CURVE_EXPONENT = 2.0f;
constexpr float HELIOSTAT_LATITUDE_DEG = 46.20027148248908f;
constexpr float HELIOSTAT_LONGITUDE_DEG = 6.139431924071319f;
constexpr time_t HELIOSTAT_START_UNIX_TIME_UTC = 0;
constexpr float HELIOSTAT_TIME_SCALE = 1.0f;
constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 64;

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

enum ControlMode : uint8_t {
  CONTROL_MODE_MANUAL = 0,
  CONTROL_MODE_TARGET_CAPTURED = 1,
  CONTROL_MODE_AUTO_TRACK = 2,
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
  int64_t unixTimeUtc;
};

constexpr uint32_t LED_PACKET_MAGIC = 0x48454C31;  // "HEL1"
BLEController controller;
Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);
#if defined(DEVICE_ROLE_REMOTE)
Servo panServo;
Servo tiltServo;
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
bool sendPending = false;
bool timeUpdatePending = false;
ControlMode controlMode = CONTROL_MODE_MANUAL;
bool targetDirectionValid = false;
bool sunTimeValid = false;
Vec3 targetDirection = {0.0f, 0.0f, 0.0f};
time_t heliostatStartUnixTimeUtc = HELIOSTAT_START_UNIX_TIME_UTC;
uint32_t heliostatTimeBaseMillis = 0;
float capturedPanAngleDeg = PAN_START_DEG;
float capturedTiltAngleDeg = TILT_START_DEG;

#if defined(DEVICE_ROLE_CONTROLLER)
uint8_t remotePeerMac[6] = {0, 0, 0, 0, 0, 0};
bool lastCaptureButton = false;
bool lastAutoButton = false;
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
  const float panRad = degToRad(panDeg - 90.0f);
  const float tiltRad = degToRad(tiltDeg - 90.0f);
  const float cosTilt = cosf(tiltRad);
  return normalizeVec3({
      cosTilt * cosf(panRad),
      cosTilt * sinf(panRad),
      sinf(tiltRad),
  });
}

void panTiltFromMirrorNormal(const Vec3& normal, float& panDeg, float& tiltDeg) {
  const Vec3 normalized = normalizeVec3(normal);
  panDeg = constrain(radToDeg(atan2f(normalized.y, normalized.x)) + 90.0f, PAN_MIN_DEG, PAN_MAX_DEG);
  tiltDeg = constrain(radToDeg(asinf(normalized.z)) + 90.0f, TILT_MIN_DEG, TILT_MAX_DEG);
}

bool currentUnixTimeUtc(time_t& unixTimeUtc) {
  if (heliostatStartUnixTimeUtc <= 0) {
    return false;
  }
  const uint32_t elapsedMs = millis() - heliostatTimeBaseMillis;
  unixTimeUtc =
      heliostatStartUnixTimeUtc + static_cast<time_t>((static_cast<double>(elapsedMs) *
                                                       static_cast<double>(HELIOSTAT_TIME_SCALE)) /
                                                      1000.0);
  return true;
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
      fmod((fractionalDayUtc * 1440.0) + equationOfTime + (4.0 * HELIOSTAT_LONGITUDE_DEG) + 1440.0,
           1440.0);
  double hourAngleDeg = (trueSolarMinutes / 4.0) - 180.0;
  if (hourAngleDeg < -180.0) {
    hourAngleDeg += 360.0;
  }

  const double latitudeRad = degToRad(HELIOSTAT_LATITUDE_DEG);
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

int angleToPulseUs(float angleDeg, float minDeg, float maxDeg) {
  const float clampedAngle = constrain(angleDeg, minDeg, maxDeg);
  const float normalized = (clampedAngle - minDeg) / (maxDeg - minDeg);
  const float pulseSpan = static_cast<float>(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  return SERVO_MIN_PULSE_US + static_cast<int>(lroundf(normalized * pulseSpan));
}

int quantizePulseUs(int pulseUs) {
  if (SERVO_COMMAND_STEP_US <= 1) {
    return constrain(pulseUs, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  }

  const int offset = pulseUs - SERVO_MIN_PULSE_US;
  const int quantizedOffset =
      static_cast<int>(lroundf(static_cast<float>(offset) / SERVO_COMMAND_STEP_US)) *
      SERVO_COMMAND_STEP_US;
  return constrain(SERVO_MIN_PULSE_US + quantizedOffset, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
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
#endif

#if defined(DEVICE_ROLE_REMOTE)
int lastPanPulseUs = 0;
int lastTiltPulseUs = 0;

void writeServos() {
  lastPanPulseUs = quantizePulseUs(angleToPulseUs(panAngleDeg, PAN_MIN_DEG, PAN_MAX_DEG));
  lastTiltPulseUs = quantizePulseUs(angleToPulseUs(tiltAngleDeg, TILT_MIN_DEG, TILT_MAX_DEG));
  panServo.writeMicroseconds(lastPanPulseUs);
  tiltServo.writeMicroseconds(lastTiltPulseUs);
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
    const Vec3 desiredNormal = normalizeVec3(addVec3(sunDirection, targetDirection));
    if (lengthVec3(desiredNormal) > 0.0f) {
      panTiltFromMirrorNormal(desiredNormal, panTargetDeg, tiltTargetDeg);
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
    blinkActive = false;
    recenterRequested = false;
    captureTargetRequested = false;
    autoTrackEnabled = false;
    if (!targetDirectionValid) {
      controlMode = CONTROL_MODE_MANUAL;
    }
  }

  updateHeliostatTracking(nowMs);

  const float dt = static_cast<float>(elapsedMs) / 1000.0f;

  if (controlMode == CONTROL_MODE_AUTO_TRACK) {
    return;
  }

  if (recenterRequested) {
    panTargetDeg = PAN_START_DEG;
    tiltTargetDeg = TILT_START_DEG;
    recenterRequested = false;
  }

  panTargetDeg = constrain(
      panTargetDeg + (applySpeedCurve(remotePanInput) * MAX_TARGET_SPEED_DEG_PER_SEC * dt),
      PAN_MIN_DEG, PAN_MAX_DEG);
  tiltTargetDeg = constrain(
      tiltTargetDeg + (applySpeedCurve(remoteTiltInput) * MAX_TARGET_SPEED_DEG_PER_SEC * dt),
      TILT_MIN_DEG, TILT_MAX_DEG);
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

  const float maxStep = SERVO_MAX_SLEW_DEG_PER_SEC * (static_cast<float>(elapsedMs) / 1000.0f);

  panAngleDeg = stepToward(panAngleDeg, panTargetDeg, maxStep);
  tiltAngleDeg = stepToward(tiltAngleDeg, tiltTargetDeg, maxStep);
  writeServos();
}

void beginServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(SERVO_FREQUENCY_HZ);
  tiltServo.setPeriodHertz(SERVO_FREQUENCY_HZ);
  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  writeServos();
  delay(250);
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
    Serial.printf("ESP-NOW TX failed to %s\n", formatMac(macAddr).c_str());
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
      (timeUpdatePending ? PACKET_FLAG_TIME_UPDATE : 0);
  packet.seq = packetSequence++;
  packet.panInput = remotePanInput;
  packet.tiltInput = remoteTiltInput;
  packet.panAngleDeg = remoteReportedPanAngleDeg;
  packet.tiltAngleDeg = remoteReportedTiltAngleDeg;
  packet.panTargetDeg = remoteReportedPanTargetDeg;
  packet.tiltTargetDeg = remoteReportedTiltTargetDeg;
  packet.capturedPanAngleDeg = remoteReportedCapturedPanAngleDeg;
  packet.capturedTiltAngleDeg = remoteReportedCapturedTiltAngleDeg;
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
    if (!espNowPeerReady) {
      rememberRemotePeer(macAddr);
      sendLedPacket();
    }
    remoteReportedControlMode = static_cast<ControlMode>(packet.reserved[0]);
    remoteReportedTargetDirectionValid = packet.reserved[1] != 0;
    remoteReportedSunTimeValid = packet.reserved[2] != 0;
    remoteReportedPanAngleDeg = packet.panAngleDeg;
    remoteReportedTiltAngleDeg = packet.tiltAngleDeg;
    remoteReportedPanTargetDeg = packet.panTargetDeg;
    remoteReportedTiltTargetDeg = packet.tiltTargetDeg;
    remoteReportedCapturedPanAngleDeg = packet.capturedPanAngleDeg;
    remoteReportedCapturedTiltAngleDeg = packet.capturedTiltAngleDeg;
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

  if (strcmp(line, "TIME?") == 0 || strcmp(line, "time?") == 0) {
    time_t unixTimeUtc = 0;
    if (currentUnixTimeUtc(unixTimeUtc)) {
      Serial.printf("UTC time current: %lld\n", static_cast<long long>(unixTimeUtc));
    } else {
      Serial.println("UTC time not set.");
    }
    return;
  }

  if (strcmp(line, "HELP") == 0 || strcmp(line, "help") == 0) {
    Serial.println("Serial commands:");
    Serial.println("  T=<unix_utc_seconds>  set UTC time");
    Serial.println("  TIME?                 show current UTC time");
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
#else
void sendAnnouncePacket(uint32_t nowMs) {
  if (!broadcastPeerReady || (nowMs - lastAnnounceMs) < REMOTE_ANNOUNCE_MS) {
    return;
  }

  lastAnnounceMs = nowMs;

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
  packet.unixTimeUtc = static_cast<int64_t>(heliostatStartUnixTimeUtc);

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
  blinkActive = false;
  remotePanInput = constrain(packet.panInput, -1.0f, 1.0f);
  remoteTiltInput = constrain(packet.tiltInput, -1.0f, 1.0f);
  if ((packet.reserved[0] & PACKET_FLAG_TIME_UPDATE) != 0 && packet.unixTimeUtc > 0) {
    setHeliostatUnixTimeUtc(static_cast<time_t>(packet.unixTimeUtc));
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
  lastCaptureButton = state.buttonX;
  lastAutoButton = state.buttonY;

  remotePanInput = applyDeadzone(-state.leftStickX);
  remoteTiltInput = applyDeadzone(-state.leftStickY);
  recenterRequested = state.buttonA;
  captureTargetRequested = capturePressed;
  if (autoPressed) {
    autoTrackEnabled = !autoTrackEnabled;
  }
  blinkActive = false;

  panAngleDeg = constrain(
      PAN_START_DEG + (remotePanInput * ((PAN_MAX_DEG - PAN_MIN_DEG) * 0.5f)),
      PAN_MIN_DEG, PAN_MAX_DEG);
  tiltAngleDeg = constrain(
      TILT_START_DEG + (remoteTiltInput * ((TILT_MAX_DEG - TILT_MIN_DEG) * 0.5f)),
      TILT_MIN_DEG, TILT_MAX_DEG);
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
  if (controller.isConnected()) {
    BLEControlsEvent state;
    controller.readControls(state);
    Serial.printf(
        "ROLE=controller | xbox=ok | peer=%s | tx=%s | auto=%s | remote_mode=%s | pan=%7.3f/%7.3f d=%+7.3f cap=%7.3f | tilt=%7.3f/%7.3f d=%+7.3f cap=%7.3f | target=%s | sun_time=%s | time_x=%.1f\n",
        espNowPeerReady ? "ok" : "missing",
        lastSendOk ? "ok" : "pending",
        autoTrackEnabled ? "on" : "off",
        controlModeName(remoteReportedControlMode),
        remoteReportedPanAngleDeg, remoteReportedPanTargetDeg,
        remoteReportedPanTargetDeg - remoteReportedCapturedPanAngleDeg,
        remoteReportedCapturedPanAngleDeg,
        remoteReportedTiltAngleDeg, remoteReportedTiltTargetDeg,
        remoteReportedTiltTargetDeg - remoteReportedCapturedTiltAngleDeg,
        remoteReportedCapturedTiltAngleDeg,
        remoteReportedTargetDirectionValid ? "ok" : "missing",
        remoteReportedSunTimeValid ? "ok" : "missing",
        HELIOSTAT_TIME_SCALE);
  } else {
    const uint32_t uptimeSec = (millis() - bootMs) / 1000;
    Serial.printf(
        "ROLE=controller | wait_xbox | uptime=%lus | peer=%s | remote=%s\n",
        static_cast<unsigned long>(uptimeSec),
        espNowPeerReady ? "ok" : "missing",
        isKnownRemotePeer() ? formatMac(remotePeerMac).c_str() : "auto");
  }
#else
  const uint32_t ageMs = packetReceived ? (nowMs - lastRxMs) : 0;
  Serial.printf(
      "ROLE=remote | mode=%u | rx=%s | age_ms=%lu | step_us=%d | pan_us=%d | tilt_us=%d | sun_time=%s | target=%s\n",
      static_cast<unsigned>(controlMode),
      packetReceived ? "ok" : "waiting",
      static_cast<unsigned long>(ageMs),
      SERVO_COMMAND_STEP_US,
      lastPanPulseUs, lastTiltPulseUs,
      sunTimeValid ? "ok" : "missing",
      targetDirectionValid ? "ok" : "missing");
#endif
}

#if defined(DEVICE_ROLE_CONTROLLER)
void onControllerConnect(NimBLEAddress address) {
  xboxConnected = true;
  blinkActive = false;
  applyLedFromPanTilt(millis());
  sendLedPacket();
  Serial.printf("Xbox connected: %s\n", address.toString().c_str());
}

void onControllerDisconnect(NimBLEAddress address) {
  xboxConnected = false;
  setStatusLedBlue();
  sendLedPacket();
  Serial.printf("Xbox disconnected: %s\n", address.toString().c_str());
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

  setupEspNowWifi();
  ensureEspNow();
  ensureBroadcastPeer();
  esp_now_register_recv_cb(onEspNowReceived);

#if defined(DEVICE_ROLE_CONTROLLER)
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
  beginServos();
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
  Serial.println("Xbox node: left stick = target angle movement, button A = recenter.");
  Serial.println("Button X captures the reflected target. Button Y toggles heliostat auto-track.");
  Serial.println("Serial: T=<unix_utc_seconds> sets UTC time, TIME? shows current UTC time.");
  Serial.println("Flash env: controller on the ESP32 with the Xbox controller.");
#else
  Serial.println("Remote node ready.");
  Serial.println("Auto-pairing enabled: power the controller ESP32 and it will lock onto this remote.");
  Serial.printf("Servos: pan GPIO=%d | tilt GPIO=%d | %d Hz\n",
                PAN_SERVO_PIN, TILT_SERVO_PIN, SERVO_FREQUENCY_HZ);
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
#endif
  updateLedFromXbox(nowMs);
#if defined(DEVICE_ROLE_REMOTE)
  updateTargetsFromRemoteInput(nowMs);
  moveServosTowardTargets(nowMs);
  applyLedFromPanTilt(nowMs);
  sendAnnouncePacket(nowMs);
#endif
  printStatus(nowMs);
}
