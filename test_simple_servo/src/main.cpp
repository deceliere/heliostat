#include <Arduino.h>
#include <ESP32Servo.h>

namespace {

constexpr int SERVO_PIN = 5;
constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 160;
constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr int SERVO_FREQUENCY_HZ = 300;
constexpr float DEFAULT_SPEED_DEG_PER_SEC = 30.0f;
constexpr uint32_t LOOP_DELAY_MS = 20;

Servo panServo;
float currentAngleDeg = 0.0f;
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
  const float stepDeg = speedDegPerSec * (static_cast<float>(LOOP_DELAY_MS) / 1000.0f);
  currentAngleDeg += stepDeg * static_cast<float>(direction);

  if (currentAngleDeg >= SERVO_MAX_DEG) {
    currentAngleDeg = static_cast<float>(SERVO_MAX_DEG);
    direction = -1;
  } else if (currentAngleDeg <= SERVO_MIN_DEG) {
    currentAngleDeg = static_cast<float>(SERVO_MIN_DEG);
    direction = 1;
  }

  panServo.write(static_cast<int>(lroundf(currentAngleDeg)));
  Serial.print("Angle: ");
  Serial.println(currentAngleDeg, 1);
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
  updateSweep();
  delay(LOOP_DELAY_MS);
}
