#include <Arduino.h>
#include <ESP32Servo.h>

namespace {

constexpr int SERVO_PIN = 5;
constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 180;
constexpr int SERVO_MIN_PULSE_US = 2300;
constexpr int SERVO_MAX_PULSE_US = 2400;
constexpr int SERVO_FREQUENCY_HZ = 350;
constexpr float DEFAULT_SPEED_DEG_PER_SEC = 30.0f;
constexpr uint32_t LOOP_DELAY_MS = 200;

Servo panServo;
float currentAngleDeg = 0.0f;
int currentUs = 0;
float speedDegPerSec = DEFAULT_SPEED_DEG_PER_SEC;
int direction = 1;

void printHelp() {
  Serial.println();
  Serial.println("Commande:");
  Serial.println("  envoyer un nombre en deg/s, ex: 15 ou 60");
  Serial.print("Vitesse actuelle: ");
  Serial.print(speedDegPerSec, 1);
  Serial.println(" deg/s");
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.isEmpty()) {
    return;
  }

  const float newSpeed = line.toFloat();
  if (newSpeed <= 0.0f) {
    Serial.print("Valeur invalide: ");
    Serial.println(line);
    printHelp();
    return;
  }

  speedDegPerSec = newSpeed;
  Serial.print("Nouvelle vitesse: ");
  Serial.print(speedDegPerSec, 1);
  Serial.println(" deg/s");
}

void updateSweep() {
  // const float stepDeg = speedDegPerSec * (static_cast<float>(LOOP_DELAY_MS) / 1000.0f);
  const float stepDeg = 0.1;
  currentAngleDeg += stepDeg * static_cast<float>(direction);

  if (currentAngleDeg >= SERVO_MAX_DEG) {
    currentAngleDeg = static_cast<float>(SERVO_MAX_DEG);
    direction = -1;
  } else if (currentAngleDeg <= SERVO_MIN_DEG) {
    currentAngleDeg = static_cast<float>(SERVO_MIN_DEG);
    direction = 1;
  }

  Serial.print("Angle analog: ");
  Serial.println(currentAngleDeg);
  // panServo.write(static_cast<int>(lroundf(currentAngleDeg)));
  float pulseUs = map(currentAngleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  panServo.writeMicroseconds(pulseUs);  // for better timing accuracy
  Serial.print("Angle Us: ");
  Serial.println(pulseUs);
}

void updateSweepUs() {
  // const float stepDeg = speedDegPerSec * (static_cast<float>(LOOP_DELAY_MS) / 1000.0f);
  const int stepUs = 1;
  currentUs += stepUs * direction;

  if (currentUs >= SERVO_MAX_PULSE_US) {
    currentUs = SERVO_MAX_PULSE_US;
    direction = -1;
  } else if (currentUs <= SERVO_MIN_PULSE_US) {
    currentUs = SERVO_MIN_PULSE_US;
    direction = 1;
  }

  // Serial.print("Angle analog: ");
  // Serial.println(currentAngleDeg);
  // panServo.write(static_cast<int>(lroundf(currentAngleDeg)));
  // float pulseUs = map(currentAngleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  panServo.writeMicroseconds(currentUs);  // for better timing accuracy
  Serial.print("Angle Us: ");
  Serial.println(currentUs);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  ESP32PWM::allocateTimer(0);
  panServo.setPeriodHertz(SERVO_FREQUENCY_HZ);
  panServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  panServo.write(SERVO_MIN_DEG);

  Serial.println("Test servo pan simple");
  Serial.print("GPIO servo: ");
  Serial.println(SERVO_PIN);
  printHelp();
}

void loop() {
  handleSerial();
  // updateSweep();
  updateSweepUs();
  delay(LOOP_DELAY_MS);
}
