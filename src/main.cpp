#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEController.h>
#include <BLEControllerRegistry.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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
constexpr float PRECISION_TARGET_SPEED_DEG_PER_SEC = 10.0f;
constexpr float SERVO_MAX_SLEW_DEG_PER_SEC = 60.0f;
constexpr float SERVO_PRECISION_SLEW_DEG_PER_SEC = 0.6f;
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

#if defined(DEVICE_ROLE_CONTROLLER)
constexpr const char* ROLE_NAME = "controller";
#else
constexpr const char* ROLE_NAME = "remote";
#endif

enum PacketType : uint8_t {
  PACKET_TYPE_CONTROL = 1,
  PACKET_TYPE_ANNOUNCE = 2,
};

constexpr uint8_t PACKET_FLAG_PRECISION = 0x01;
constexpr uint8_t PACKET_FLAG_RECENTER = 0x02;

struct EspNowPacket {
  uint32_t magic;
  uint8_t type;
  uint8_t reserved[3];
  uint32_t seq;
  float panInput;
  float tiltInput;
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
bool precisionMode = false;
bool recenterRequested = false;
bool sendPending = false;

#if defined(DEVICE_ROLE_CONTROLLER)
uint8_t remotePeerMac[6] = {0, 0, 0, 0, 0, 0};
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

#if defined(DEVICE_ROLE_REMOTE)
int lastPanPulseUs = 0;
int lastTiltPulseUs = 0;

void writeServos() {
  lastPanPulseUs = quantizePulseUs(angleToPulseUs(panAngleDeg, PAN_MIN_DEG, PAN_MAX_DEG));
  lastTiltPulseUs = quantizePulseUs(angleToPulseUs(tiltAngleDeg, TILT_MIN_DEG, TILT_MAX_DEG));
  panServo.writeMicroseconds(lastPanPulseUs);
  tiltServo.writeMicroseconds(lastTiltPulseUs);
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
    precisionMode = false;
    blinkActive = false;
    recenterRequested = false;
  }

  const float targetSpeedDegPerSec =
      precisionMode ? PRECISION_TARGET_SPEED_DEG_PER_SEC : MAX_TARGET_SPEED_DEG_PER_SEC;
  const float dt = static_cast<float>(elapsedMs) / 1000.0f;

  if (recenterRequested) {
    panTargetDeg = PAN_START_DEG;
    tiltTargetDeg = TILT_START_DEG;
    recenterRequested = false;
  }

  panTargetDeg = constrain(
      panTargetDeg + (applySpeedCurve(remotePanInput) * targetSpeedDegPerSec * dt),
      PAN_MIN_DEG, PAN_MAX_DEG);
  tiltTargetDeg = constrain(
      tiltTargetDeg + (applySpeedCurve(remoteTiltInput) * targetSpeedDegPerSec * dt),
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

  const float slewDegPerSec =
      precisionMode ? SERVO_PRECISION_SLEW_DEG_PER_SEC : SERVO_MAX_SLEW_DEG_PER_SEC;
  const float maxStep = slewDegPerSec * (static_cast<float>(elapsedMs) / 1000.0f);

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
      (precisionMode ? PACKET_FLAG_PRECISION : 0) | (recenterRequested ? PACKET_FLAG_RECENTER : 0);
  packet.seq = packetSequence++;
  packet.panInput = remotePanInput;
  packet.tiltInput = remoteTiltInput;

  sendPending = true;
  const esp_err_t result =
      esp_now_send(remotePeerMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    sendPending = false;
    lastSendOk = false;
    Serial.printf("ESP-NOW send failed immediately: %d\n", static_cast<int>(result));
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
#else
void sendAnnouncePacket(uint32_t nowMs) {
  if (!broadcastPeerReady || (nowMs - lastAnnounceMs) < REMOTE_ANNOUNCE_MS) {
    return;
  }

  lastAnnounceMs = nowMs;

  EspNowPacket packet{};
  packet.magic = LED_PACKET_MAGIC;
  packet.type = PACKET_TYPE_ANNOUNCE;
  packet.seq = packetSequence++;

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

  precisionMode = (packet.reserved[0] & PACKET_FLAG_PRECISION) != 0;
  recenterRequested = (packet.reserved[0] & PACKET_FLAG_RECENTER) != 0;
  blinkActive = precisionMode;
  remotePanInput = constrain(packet.panInput, -1.0f, 1.0f);
  remoteTiltInput = constrain(packet.tiltInput, -1.0f, 1.0f);
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

  remotePanInput = applyDeadzone(-state.leftStickX);
  remoteTiltInput = applyDeadzone(-state.leftStickY);
  precisionMode = state.buttonB || state.leftBumper;
  recenterRequested = state.buttonA;
  blinkActive = precisionMode;

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
        "ROLE=controller | xbox=ok | peer=%s | tx=%s | precision=%s | pan_in=%+.2f | tilt_in=%+.2f | lx=%+.2f ly=%+.2f\n",
        espNowPeerReady ? "ok" : "missing",
        lastSendOk ? "ok" : "pending",
        precisionMode ? "on" : "off",
        remotePanInput, remoteTiltInput,
        state.leftStickX, state.leftStickY);
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
      "ROLE=remote | rx=%s | age_ms=%lu | step_us=%d | pan_us=%d | tilt_us=%d\n",
      packetReceived ? "ok" : "waiting",
      static_cast<unsigned long>(ageMs),
      SERVO_COMMAND_STEP_US,
      lastPanPulseUs, lastTiltPulseUs);
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
  Serial.println("Precision mode: hold B or LB for the slowest movement tests.");
  Serial.println("Flash env: controller on the ESP32 with the Xbox controller.");
#else
  Serial.println("Remote node ready.");
  Serial.println("Auto-pairing enabled: power the controller ESP32 and it will lock onto this remote.");
  Serial.printf("Servos: pan GPIO=%d | tilt GPIO=%d | %d Hz\n",
                PAN_SERVO_PIN, TILT_SERVO_PIN, SERVO_FREQUENCY_HZ);
  Serial.printf("Servo command step: %d us\n", SERVO_COMMAND_STEP_US);
  Serial.printf("Slew rates: normal=%.2f deg/s | precision=%.2f deg/s\n",
                SERVO_MAX_SLEW_DEG_PER_SEC, SERVO_PRECISION_SLEW_DEG_PER_SEC);
  Serial.printf("Runtime status logs: %s\n", ENABLE_RUNTIME_STATUS_LOGS ? "on" : "off");
  Serial.println("Use a dedicated 5-6V supply for MG996R servos and share GND with the ESP32.");
#endif
}

void loop() {
  const uint32_t nowMs = millis();
  updateLedFromXbox(nowMs);
#if defined(DEVICE_ROLE_REMOTE)
  updateTargetsFromRemoteInput(nowMs);
  moveServosTowardTargets(nowMs);
  applyLedFromPanTilt(nowMs);
  sendAnnouncePacket(nowMs);
#endif
  printStatus(nowMs);
}
