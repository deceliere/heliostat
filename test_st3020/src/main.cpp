#include <Arduino.h>
#include <SCServo.h>

namespace {

constexpr int ST3020_UART_RX_PIN = 18;
constexpr int ST3020_UART_TX_PIN = 19;
constexpr uint32_t ST3020_UART_BAUD = 1000000;
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t STATUS_PRINT_MS = 4000;
constexpr uint8_t DEFAULT_SERVO_ID = 1;
constexpr uint16_t DEFAULT_MOVE_SPEED = 1500;
constexpr uint8_t DEFAULT_MOVE_ACC = 100;
constexpr int32_t SERVO_DIGITAL_MIN = 0;
constexpr int32_t SERVO_DIGITAL_MAX = 4095;
constexpr uint8_t SCAN_ID_MIN = 1;
constexpr uint8_t SCAN_ID_MAX = 20;

SMS_STS st;
uint32_t lastStatusPrintMs = 0;
uint8_t selectedServoId = DEFAULT_SERVO_ID;

void beginServoBus() {
  Serial1.begin(ST3020_UART_BAUD, SERIAL_8N1, ST3020_UART_RX_PIN, ST3020_UART_TX_PIN);
  st.pSerial = &Serial1;
  st.IOTimeOut = 100;
}

void printHelp() {
  Serial.println();
  Serial.println("Commandes:");
  Serial.println("  help");
  Serial.println("  id?");
  Serial.println("  id=<n>");
  Serial.println("  scan");
  Serial.println("  ping");
  Serial.println("  read");
  Serial.println("  setid=<old>,<new>");
  Serial.println("  torque=on");
  Serial.println("  torque=off");
  Serial.println("  move=<pos>");
  Serial.println("  move=<pos>,<speed>");
  Serial.println("  move=<pos>,<speed>,<acc>");
  Serial.println("  stop");
}

void printStatusHeader() {
  Serial.println();
  Serial.println("ST3020 test bench");
  Serial.print("UART RX pin: ");
  Serial.println(ST3020_UART_RX_PIN);
  Serial.print("UART TX pin: ");
  Serial.println(ST3020_UART_TX_PIN);
  Serial.print("UART baud: ");
  Serial.println(ST3020_UART_BAUD);
  Serial.print("Selected servo ID: ");
  Serial.println(selectedServoId);
}

bool readFeedback(uint8_t id) {
  if (st.FeedBack(id) == -1) {
    Serial.print("FeedBack error on ID ");
    Serial.println(id);
    return false;
  }
  return true;
}

void printFeedback(uint8_t id) {
  if (!readFeedback(id)) {
    return;
  }

  const int pos = st.ReadPos(-1);
  const int speed = st.ReadSpeed(-1);
  const int load = st.ReadLoad(-1);
  const int voltage = st.ReadVoltage(-1);
  const int current = st.ReadCurrent(-1);
  const int temperature = st.ReadTemper(-1);
  const int mode = st.ReadMode(id);

  Serial.print("ID ");
  Serial.print(id);
  Serial.print(" | pos=");
  Serial.print(pos);
  Serial.print(" | speed=");
  Serial.print(speed);
  Serial.print(" | load=");
  Serial.print(load);
  Serial.print(" | voltage=");
  Serial.print(voltage / 10.0f, 1);
  Serial.print("V | current=");
  Serial.print(current);
  Serial.print(" | temp=");
  Serial.print(temperature);
  Serial.print("C | mode=");
  Serial.println(mode);
}

void pingServo(uint8_t id) {
  const int pingResult = st.Ping(id);
  if (pingResult < 0) {
    Serial.print("No response on ID ");
    Serial.println(id);
    return;
  }

  const int model = st.readWord(id, SMS_STS_MODEL_L);

  Serial.print("Ping OK on ID ");
  Serial.print(id);
  if (model >= 0) {
    Serial.print(" | model=");
    Serial.println(model);
  } else {
    Serial.println(" | model=?");
  }
}

void setTorque(uint8_t id, bool enabled) {
  st.EnableTorque(id, enabled ? 1 : 0);
  Serial.print("Torque ");
  Serial.print(enabled ? "ON" : "OFF");
  Serial.print(" for ID ");
  Serial.println(id);
}

void stopServo(uint8_t id) {
  st.EnableTorque(id, 0);
  delay(10);
  st.EnableTorque(id, 1);
  Serial.print("Stop pulse sent to ID ");
  Serial.println(id);
}

void moveServo(uint8_t id, int32_t position, uint16_t speed, uint8_t acc) {
  position = constrain(position, SERVO_DIGITAL_MIN, SERVO_DIGITAL_MAX);
  st.WritePosEx(id, static_cast<s16>(position), speed, acc);
  Serial.print("Move ID ");
  Serial.print(id);
  Serial.print(" -> pos=");
  Serial.print(position);
  Serial.print(", speed=");
  Serial.print(speed);
  Serial.print(", acc=");
  Serial.println(acc);
}

void scanServoIds() {
  Serial.print("Scan IDs ");
  Serial.print(SCAN_ID_MIN);
  Serial.print("..");
  Serial.println(SCAN_ID_MAX);

  bool foundAny = false;
  for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX; ++id) {
    const int pingResult = st.Ping(id);
    if (pingResult < 0) {
      continue;
    }

    foundAny = true;
    const int model = st.readWord(id, SMS_STS_MODEL_L);
    Serial.print("Found ID ");
    Serial.print(id);
    if (model >= 0) {
      Serial.print(" | model=");
      Serial.println(model);
    } else {
      Serial.println(" | model=?");
    }
    delay(5);
  }

  if (!foundAny) {
    Serial.println("No servo found in scan range.");
  }
}

void setServoId(uint8_t oldId, uint8_t newId) {
  if (newId == 0 || newId > 253) {
    Serial.println("New ID invalide.");
    return;
  }

  const int pingResult = st.Ping(oldId);
  if (pingResult < 0) {
    Serial.print("No response on old ID ");
    Serial.println(oldId);
    return;
  }

  st.unLockEprom(oldId);
  delay(10);
  st.writeByte(oldId, SMS_STS_ID, newId);
  delay(10);
  st.LockEprom(newId);
  delay(20);

  Serial.print("ID change requested: ");
  Serial.print(oldId);
  Serial.print(" -> ");
  Serial.println(newId);

  if (st.Ping(newId) >= 0) {
    selectedServoId = newId;
    Serial.print("New ID confirmed: ");
    Serial.println(newId);
  } else {
    Serial.println("Warning: new ID not confirmed yet.");
  }
}

void handleMoveCommand(const String& args) {
  int firstComma = args.indexOf(',');
  int secondComma = args.indexOf(',', firstComma + 1);

  const String posString = firstComma >= 0 ? args.substring(0, firstComma) : args;
  const String speedString =
      firstComma >= 0 ? (secondComma >= 0 ? args.substring(firstComma + 1, secondComma)
                                          : args.substring(firstComma + 1))
                      : String("");
  const String accString = secondComma >= 0 ? args.substring(secondComma + 1) : String("");

  const int32_t position = posString.toInt();
  const uint16_t speed =
      speedString.isEmpty() ? DEFAULT_MOVE_SPEED : static_cast<uint16_t>(speedString.toInt());
  const uint8_t acc =
      accString.isEmpty() ? DEFAULT_MOVE_ACC : static_cast<uint8_t>(accString.toInt());

  moveServo(selectedServoId, position, speed, acc);
}

void handleSetIdCommand(const String& args) {
  const int comma = args.indexOf(',');
  if (comma < 0) {
    Serial.println("Format attendu: setid=<old>,<new>");
    return;
  }

  const int oldId = args.substring(0, comma).toInt();
  const int newId = args.substring(comma + 1).toInt();
  if (oldId <= 0 || oldId > 253 || newId <= 0 || newId > 253) {
    Serial.println("ID invalide.");
    return;
  }

  setServoId(static_cast<uint8_t>(oldId), static_cast<uint8_t>(newId));
}

void handleCommand(const String& line) {
  if (line == "help") {
    printHelp();
    return;
  }

  if (line == "id?") {
    Serial.print("Selected servo ID: ");
    Serial.println(selectedServoId);
    return;
  }

  if (line.startsWith("id=")) {
    const int value = line.substring(3).toInt();
    if (value < 0 || value > 253) {
      Serial.println("ID invalide.");
      return;
    }
    selectedServoId = static_cast<uint8_t>(value);
    Serial.print("Selected servo ID: ");
    Serial.println(selectedServoId);
    return;
  }

  if (line == "scan") {
    scanServoIds();
    return;
  }

  if (line == "ping") {
    pingServo(selectedServoId);
    return;
  }

  if (line == "read") {
    printFeedback(selectedServoId);
    return;
  }

  if (line == "torque=on") {
    setTorque(selectedServoId, true);
    return;
  }

  if (line == "torque=off") {
    setTorque(selectedServoId, false);
    return;
  }

  if (line == "stop") {
    stopServo(selectedServoId);
    return;
  }

  if (line.startsWith("move=")) {
    handleMoveCommand(line.substring(5));
    return;
  }

  if (line.startsWith("setid=")) {
    handleSetIdCommand(line.substring(6));
    return;
  }

  Serial.print("Commande inconnue: ");
  Serial.println(line);
  printHelp();
}

void handleUsbSerial() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  handleCommand(line);
}

}  // namespace

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(500);

  beginServoBus();

  printStatusHeader();
  printHelp();
}

void loop() {
  handleUsbSerial();

  const uint32_t nowMs = millis();
  if ((nowMs - lastStatusPrintMs) >= STATUS_PRINT_MS) {
    lastStatusPrintMs = nowMs;
    printStatusHeader();
  }
}
