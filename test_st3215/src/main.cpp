#include <Arduino.h>

namespace {

constexpr int SERVO_BUS_TX_PIN = 17;
constexpr int SERVO_BUS_RX_PIN = 18;
constexpr int SERVO_BUS_TXEN_PIN = 16;
constexpr uint32_t SERVO_BUS_BAUD = 1000000;
constexpr uint32_t STATUS_PRINT_MS = 2000;

HardwareSerial servoBus(1);
uint32_t lastStatusPrintMs = 0;

void printWiring() {
  Serial.println("ST3215 test scaffold");
  Serial.println("This servo is a TTL serial bus servo, not a PWM servo.");
  Serial.println("Recommended ESP32-side pins:");
  Serial.printf("  TX   -> GPIO%d\n", SERVO_BUS_TX_PIN);
  Serial.printf("  RX   -> GPIO%d\n", SERVO_BUS_RX_PIN);
  Serial.printf("  TXEN -> GPIO%d\n", SERVO_BUS_TXEN_PIN);
  Serial.println("Servo connector:");
  Serial.println("  V -> external servo supply positive");
  Serial.println("  G -> common ground");
  Serial.println("  A -> DATA line via half-duplex bus interface");
  Serial.println();
  Serial.println("Important: do not connect A directly to a naked ESP32 GPIO for a proper two-way bus.");
  Serial.println("Use a half-duplex interface circuit between ESP32 TX/RX and the single-wire A bus.");
}

void setBusTransmitEnabled(bool enabled) {
  digitalWrite(SERVO_BUS_TXEN_PIN, enabled ? HIGH : LOW);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(SERVO_BUS_TXEN_PIN, OUTPUT);
  setBusTransmitEnabled(false);

  servoBus.begin(SERVO_BUS_BAUD, SERIAL_8N1, SERVO_BUS_RX_PIN, SERVO_BUS_TX_PIN);

  printWiring();
}

void loop() {
  const uint32_t nowMs = millis();
  if ((nowMs - lastStatusPrintMs) >= STATUS_PRINT_MS) {
    lastStatusPrintMs = nowMs;
    Serial.printf("UART1 ready | baud=%lu | TX=%d | RX=%d | TXEN=%d\n",
                  static_cast<unsigned long>(SERVO_BUS_BAUD),
                  SERVO_BUS_TX_PIN,
                  SERVO_BUS_RX_PIN,
                  SERVO_BUS_TXEN_PIN);
  }

  if (Serial.available()) {
    const String line = Serial.readStringUntil('\n');
    if (line.equalsIgnoreCase("tx on")) {
      setBusTransmitEnabled(true);
      Serial.println("TXEN -> HIGH");
    } else if (line.equalsIgnoreCase("tx off")) {
      setBusTransmitEnabled(false);
      Serial.println("TXEN -> LOW");
    } else if (line.equalsIgnoreCase("help")) {
      printWiring();
      Serial.println("Commands: tx on | tx off | help");
    }
  }
}
