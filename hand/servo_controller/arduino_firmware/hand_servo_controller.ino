#include <Arduino.h>
#include <Servo.h>
#include <string.h>

namespace {

constexpr size_t SERVO_COUNT = 5;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t MOTION_UPDATE_MS = 20;
constexpr float DEFAULT_SPEED_DEG_S = 60.0f;
constexpr size_t LINE_BUFFER_SIZE = 128;
constexpr float DEFAULT_TEST_SPEED_DEG_S = 45.0f;
constexpr float DEFAULT_PULSE_CONTRACT_DEG = 10.0f;
constexpr float DEFAULT_PULSE_SPLAY_DEG = 145.0f;
constexpr uint32_t DEFAULT_PULSE_HOLD_MS = 5000;
constexpr float DEFAULT_PULSE_SPEED_DEG_S = 90.0f;

struct ServoChannelConfig {
  const char* name;
  uint8_t pin;
  int min_us;
  int max_us;
  float home_deg;
  float open_deg;
  float closed_deg;
  float min_deg;
  float max_deg;
  float trim_deg;
  bool invert;
};

struct ServoChannelState {
  float current_deg;
  float target_deg;
  float speed_deg_s;
  bool attached;
};

enum PulsePhase {
  PULSE_MOVING_TO_CONTRACT,
  PULSE_HOLDING_CONTRACT,
  PULSE_MOVING_TO_SPLAY,
  PULSE_HOLDING_SPLAY,
};

struct PulseModeState {
  bool enabled;
  float contract_deg;
  float splay_deg;
  uint32_t hold_ms;
  float speed_deg_s;
  PulsePhase phase;
  uint32_t phase_started_ms;
};

ServoChannelConfig kChannels[SERVO_COUNT] = {
  { "thumb", 3,  500, 2500, 40.0f, 20.0f, 120.0f,  0.0f, 140.0f, 0.0f, false },
  { "index", 5,  500, 2500, 30.0f, 10.0f, 135.0f,  0.0f, 150.0f, 0.0f, false },
  { "middle", 6, 500, 2500, 30.0f, 10.0f, 135.0f,  0.0f, 150.0f, 0.0f, false },
  { "ring", 9,   500, 2500, 35.0f, 15.0f, 130.0f,  0.0f, 150.0f, 0.0f, false },
  { "pinky", 10, 500, 2500, 35.0f, 15.0f, 125.0f,  0.0f, 150.0f, 0.0f, false },
};

Servo kDrivers[SERVO_COUNT];
ServoChannelState kState[SERVO_COUNT];
PulseModeState kPulseMode;
char kLineBuffer[LINE_BUFFER_SIZE];
size_t kLineLength = 0;
uint32_t kLastMotionUpdateMs = 0;

float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float lerpf(float start, float stop, float t) {
  return start + ((stop - start) * t);
}

float absoluteValue(float value) {
  return value >= 0.0f ? value : -value;
}

bool isWhitespace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

void trimInPlace(char* text) {
  if (text == nullptr) return;
  size_t length = strlen(text);
  size_t start = 0;
  while (start < length && isWhitespace(text[start])) {
    start++;
  }
  size_t end = length;
  while (end > start && isWhitespace(text[end - 1])) {
    end--;
  }
  if (start > 0) {
    memmove(text, text + start, end - start);
  }
  text[end - start] = '\0';
}

int servoIndexFromName(const char* token) {
  if (token == nullptr) return -1;
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    if (strcmp(token, kChannels[index].name) == 0) return static_cast<int>(index);
  }
  if (token[0] == 's' && token[2] == '\0') {
    int numeric = token[1] - '0';
    if (numeric >= 0 && numeric < static_cast<int>(SERVO_COUNT)) return numeric;
  }
  return -1;
}

void writeServoPhysical(size_t index, float logical_deg) {
  float clamped = clampf(logical_deg, kChannels[index].min_deg, kChannels[index].max_deg);
  float physical = clamped + kChannels[index].trim_deg;
  if (kChannels[index].invert) {
    physical = 180.0f - physical;
  }
  physical = clampf(physical, 0.0f, 180.0f);
  kDrivers[index].write(static_cast<int>(roundf(physical)));
}

void attachAll() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    if (!kState[index].attached) {
      kDrivers[index].attach(
        kChannels[index].pin,
        kChannels[index].min_us,
        kChannels[index].max_us
      );
      kState[index].attached = true;
    }
    writeServoPhysical(index, kState[index].current_deg);
  }
}

void detachAll() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    if (kState[index].attached) {
      kDrivers[index].detach();
      kState[index].attached = false;
    }
  }
}

void setTarget(size_t index, float logical_deg, float speed_deg_s = DEFAULT_SPEED_DEG_S) {
  kState[index].target_deg = clampf(
    logical_deg,
    kChannels[index].min_deg,
    kChannels[index].max_deg
  );
  kState[index].speed_deg_s = clampf(speed_deg_s, 1.0f, 360.0f);
}

void setAllTargets(
  float thumb_deg,
  float index_deg,
  float middle_deg,
  float ring_deg,
  float pinky_deg,
  float speed_deg_s = DEFAULT_SPEED_DEG_S
) {
  setTarget(0, thumb_deg, speed_deg_s);
  setTarget(1, index_deg, speed_deg_s);
  setTarget(2, middle_deg, speed_deg_s);
  setTarget(3, ring_deg, speed_deg_s);
  setTarget(4, pinky_deg, speed_deg_s);
}

void applyPose(const char* pose_name) {
  if (strcmp(pose_name, "home") == 0) {
    for (size_t index = 0; index < SERVO_COUNT; index++) {
      setTarget(index, kChannels[index].home_deg);
    }
    Serial.println(F("ok pose home"));
    return;
  }
  if (strcmp(pose_name, "open") == 0) {
    for (size_t index = 0; index < SERVO_COUNT; index++) {
      setTarget(index, kChannels[index].open_deg);
    }
    Serial.println(F("ok pose open"));
    return;
  }
  if (strcmp(pose_name, "close") == 0) {
    for (size_t index = 0; index < SERVO_COUNT; index++) {
      setTarget(index, kChannels[index].closed_deg);
    }
    Serial.println(F("ok pose close"));
    return;
  }
  if (strcmp(pose_name, "pregrasp") == 0) {
    for (size_t index = 0; index < SERVO_COUNT; index++) {
      setTarget(
        index,
        lerpf(kChannels[index].open_deg, kChannels[index].closed_deg, 0.45f)
      );
    }
    Serial.println(F("ok pose pregrasp"));
    return;
  }
  Serial.print(F("err unknown pose "));
  Serial.println(pose_name);
}

void setGrasp(float amount) {
  float t = clampf(amount, 0.0f, 1.0f);
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    setTarget(
      index,
      lerpf(kChannels[index].open_deg, kChannels[index].closed_deg, t)
    );
  }
  Serial.print(F("ok grasp "));
  Serial.println(t, 3);
}

bool anyMoving() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    if (absoluteValue(kState[index].target_deg - kState[index].current_deg) > 0.5f) {
      return true;
    }
  }
  return false;
}

void stopMotion() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    kState[index].target_deg = kState[index].current_deg;
  }
  Serial.println(F("ok stop"));
}

void freezeMotion() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    kState[index].target_deg = kState[index].current_deg;
  }
}

void disablePulseMode() {
  kPulseMode.enabled = false;
}

void startPulseMove(PulsePhase phase) {
  float target_deg = phase == PULSE_MOVING_TO_CONTRACT
    ? kPulseMode.contract_deg
    : kPulseMode.splay_deg;
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    setTarget(index, target_deg, kPulseMode.speed_deg_s);
  }
  kPulseMode.phase = phase;
  kPulseMode.phase_started_ms = millis();
}

void enablePulseMode() {
  kPulseMode.enabled = true;
  startPulseMove(PULSE_MOVING_TO_CONTRACT);
}

void printHelp() {
  Serial.println(F("ok commands ping status map attach detach home stop pose grasp set setall trim test pulse help"));
}

void printStatus() {
  Serial.print(F("status moving="));
  Serial.print(anyMoving() ? 1 : 0);
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    Serial.print(F(" "));
    Serial.print(kChannels[index].name);
    Serial.print(F("="));
    Serial.print(kState[index].current_deg, 1);
    Serial.print(F("/"));
    Serial.print(kState[index].target_deg, 1);
    Serial.print(F("@"));
    Serial.print(kState[index].speed_deg_s, 1);
    Serial.print(kState[index].attached ? ":1" : ":0");
  }
  Serial.println();
}

void printMap() {
  Serial.println(F("map name pin min_us max_us min_deg max_deg home open close invert trim"));
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    Serial.print(F("map "));
    Serial.print(kChannels[index].name);
    Serial.print(F(" "));
    Serial.print(kChannels[index].pin);
    Serial.print(F(" "));
    Serial.print(kChannels[index].min_us);
    Serial.print(F(" "));
    Serial.print(kChannels[index].max_us);
    Serial.print(F(" "));
    Serial.print(kChannels[index].min_deg, 1);
    Serial.print(F(" "));
    Serial.print(kChannels[index].max_deg, 1);
    Serial.print(F(" "));
    Serial.print(kChannels[index].home_deg, 1);
    Serial.print(F(" "));
    Serial.print(kChannels[index].open_deg, 1);
    Serial.print(F(" "));
    Serial.print(kChannels[index].closed_deg, 1);
    Serial.print(F(" "));
    Serial.print(kChannels[index].invert ? 1 : 0);
    Serial.print(F(" "));
    Serial.println(kChannels[index].trim_deg, 1);
  }
}

void printPulseStatus() {
  Serial.print(F("pulse enabled="));
  Serial.print(kPulseMode.enabled ? 1 : 0);
  Serial.print(F(" contract="));
  Serial.print(kPulseMode.contract_deg, 1);
  Serial.print(F(" splay="));
  Serial.print(kPulseMode.splay_deg, 1);
  Serial.print(F(" hold_ms="));
  Serial.print(kPulseMode.hold_ms);
  Serial.print(F(" speed="));
  Serial.print(kPulseMode.speed_deg_s, 1);
  Serial.print(F(" phase="));
  switch (kPulseMode.phase) {
    case PULSE_MOVING_TO_CONTRACT:
      Serial.println(F("moving_contract"));
      return;
    case PULSE_HOLDING_CONTRACT:
      Serial.println(F("holding_contract"));
      return;
    case PULSE_MOVING_TO_SPLAY:
      Serial.println(F("moving_splay"));
      return;
    case PULSE_HOLDING_SPLAY:
      Serial.println(F("holding_splay"));
      return;
  }
}

void updateMotion() {
  uint32_t now = millis();
  if (now - kLastMotionUpdateMs < MOTION_UPDATE_MS) {
    return;
  }

  float dt_s = (kLastMotionUpdateMs == 0)
    ? (static_cast<float>(MOTION_UPDATE_MS) / 1000.0f)
    : (static_cast<float>(now - kLastMotionUpdateMs) / 1000.0f);
  kLastMotionUpdateMs = now;

  for (size_t index = 0; index < SERVO_COUNT; index++) {
    if (!kState[index].attached) continue;

    float error = kState[index].target_deg - kState[index].current_deg;
    float max_step = kState[index].speed_deg_s * dt_s;
    if (absoluteValue(error) <= max_step) {
      kState[index].current_deg = kState[index].target_deg;
    } else {
      kState[index].current_deg += (error > 0.0f ? max_step : -max_step);
    }
    writeServoPhysical(index, kState[index].current_deg);
  }
}

void updatePulseMode() {
  if (!kPulseMode.enabled) {
    return;
  }

  uint32_t now = millis();
  if (kPulseMode.phase == PULSE_MOVING_TO_CONTRACT && !anyMoving()) {
    kPulseMode.phase = PULSE_HOLDING_CONTRACT;
    kPulseMode.phase_started_ms = now;
    return;
  }
  if (kPulseMode.phase == PULSE_MOVING_TO_SPLAY && !anyMoving()) {
    kPulseMode.phase = PULSE_HOLDING_SPLAY;
    kPulseMode.phase_started_ms = now;
    return;
  }
  if (
    kPulseMode.phase == PULSE_HOLDING_CONTRACT &&
    now - kPulseMode.phase_started_ms >= kPulseMode.hold_ms
  ) {
    startPulseMove(PULSE_MOVING_TO_SPLAY);
    return;
  }
  if (
    kPulseMode.phase == PULSE_HOLDING_SPLAY &&
    now - kPulseMode.phase_started_ms >= kPulseMode.hold_ms
  ) {
    startPulseMove(PULSE_MOVING_TO_CONTRACT);
  }
}

void waitForMotion(uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  while (anyMoving() && (millis() - start_ms) < timeout_ms) {
    updateMotion();
    delay(MOTION_UPDATE_MS);
  }
}

void initializeState() {
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    kState[index].current_deg = kChannels[index].home_deg;
    kState[index].target_deg = kChannels[index].home_deg;
    kState[index].speed_deg_s = DEFAULT_SPEED_DEG_S;
    kState[index].attached = false;
  }
  kPulseMode.enabled = false;
  kPulseMode.contract_deg = DEFAULT_PULSE_CONTRACT_DEG;
  kPulseMode.splay_deg = DEFAULT_PULSE_SPLAY_DEG;
  kPulseMode.hold_ms = DEFAULT_PULSE_HOLD_MS;
  kPulseMode.speed_deg_s = DEFAULT_PULSE_SPEED_DEG_S;
  kPulseMode.phase = PULSE_MOVING_TO_CONTRACT;
  kPulseMode.phase_started_ms = 0;
}

void handleSetCommand(char* save_ptr) {
  disablePulseMode();
  char* servo_name = strtok_r(nullptr, " ", &save_ptr);
  char* angle_text = strtok_r(nullptr, " ", &save_ptr);
  char* speed_text = strtok_r(nullptr, " ", &save_ptr);
  if (servo_name == nullptr || angle_text == nullptr) {
    Serial.println(F("err usage set <servo> <deg> [speed_deg_s]"));
    return;
  }

  int index = servoIndexFromName(servo_name);
  if (index < 0) {
    Serial.println(F("err unknown servo"));
    return;
  }

  float target_deg = static_cast<float>(atof(angle_text));
  float speed_deg_s = speed_text == nullptr ? DEFAULT_SPEED_DEG_S : static_cast<float>(atof(speed_text));
  setTarget(static_cast<size_t>(index), target_deg, speed_deg_s);
  Serial.print(F("ok set "));
  Serial.print(kChannels[index].name);
  Serial.print(F(" "));
  Serial.print(kState[index].target_deg, 1);
  Serial.print(F(" "));
  Serial.println(kState[index].speed_deg_s, 1);
}

void handleSetAllCommand(char* save_ptr) {
  disablePulseMode();
  float values[SERVO_COUNT];
  for (size_t index = 0; index < SERVO_COUNT; index++) {
    char* token = strtok_r(nullptr, " ", &save_ptr);
    if (token == nullptr) {
      Serial.println(F("err usage setall <d0> <d1> <d2> <d3> <d4> [speed_deg_s]"));
      return;
    }
    values[index] = static_cast<float>(atof(token));
  }

  char* speed_text = strtok_r(nullptr, " ", &save_ptr);
  float speed_deg_s = speed_text == nullptr ? DEFAULT_SPEED_DEG_S : static_cast<float>(atof(speed_text));
  setAllTargets(values[0], values[1], values[2], values[3], values[4], speed_deg_s);
  Serial.println(F("ok setall"));
}

void handleTrimCommand(char* save_ptr) {
  char* servo_name = strtok_r(nullptr, " ", &save_ptr);
  char* trim_text = strtok_r(nullptr, " ", &save_ptr);
  if (servo_name == nullptr || trim_text == nullptr) {
    Serial.println(F("err usage trim <servo> <deg>"));
    return;
  }

  int index = servoIndexFromName(servo_name);
  if (index < 0) {
    Serial.println(F("err unknown servo"));
    return;
  }

  kChannels[index].trim_deg = clampf(static_cast<float>(atof(trim_text)), -30.0f, 30.0f);
  writeServoPhysical(static_cast<size_t>(index), kState[index].current_deg);
  Serial.print(F("ok trim "));
  Serial.print(kChannels[index].name);
  Serial.print(F(" "));
  Serial.println(kChannels[index].trim_deg, 1);
}

void handleTestCommand(char* save_ptr) {
  disablePulseMode();
  char* servo_name = strtok_r(nullptr, " ", &save_ptr);
  if (servo_name == nullptr) {
    Serial.println(F("err usage test <servo|all>"));
    return;
  }

  if (strcmp(servo_name, "all") == 0) {
    applyPose("open");
    waitForMotion(3000);
    applyPose("pregrasp");
    waitForMotion(3000);
    applyPose("close");
    waitForMotion(3000);
    applyPose("home");
    waitForMotion(3000);
    Serial.println(F("ok test all"));
    return;
  }

  int index = servoIndexFromName(servo_name);
  if (index < 0) {
    Serial.println(F("err unknown servo"));
    return;
  }

  setTarget(static_cast<size_t>(index), kChannels[index].open_deg, DEFAULT_TEST_SPEED_DEG_S);
  waitForMotion(3000);
  setTarget(static_cast<size_t>(index), kChannels[index].closed_deg, DEFAULT_TEST_SPEED_DEG_S);
  waitForMotion(3000);
  setTarget(static_cast<size_t>(index), kChannels[index].home_deg, DEFAULT_TEST_SPEED_DEG_S);
  waitForMotion(3000);
  Serial.print(F("ok test "));
  Serial.println(kChannels[index].name);
}

void handlePulseCommand(char* save_ptr) {
  char* subcommand = strtok_r(nullptr, " ", &save_ptr);
  if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
    printPulseStatus();
    return;
  }

  if (strcmp(subcommand, "on") == 0) {
    enablePulseMode();
    Serial.println(F("ok pulse on"));
    return;
  }

  if (strcmp(subcommand, "off") == 0) {
    disablePulseMode();
    freezeMotion();
    Serial.println(F("ok pulse off"));
    return;
  }

  if (strcmp(subcommand, "set") == 0) {
    char* contract_text = strtok_r(nullptr, " ", &save_ptr);
    char* splay_text = strtok_r(nullptr, " ", &save_ptr);
    char* hold_text = strtok_r(nullptr, " ", &save_ptr);
    char* speed_text = strtok_r(nullptr, " ", &save_ptr);
    if (contract_text == nullptr || splay_text == nullptr) {
      Serial.println(F("err usage pulse set <contract_deg> <splay_deg> [hold_ms] [speed_deg_s]"));
      return;
    }
    kPulseMode.contract_deg = static_cast<float>(atof(contract_text));
    kPulseMode.splay_deg = static_cast<float>(atof(splay_text));
    if (hold_text != nullptr) {
      kPulseMode.hold_ms = static_cast<uint32_t>(atol(hold_text));
    }
    if (speed_text != nullptr) {
      kPulseMode.speed_deg_s = clampf(static_cast<float>(atof(speed_text)), 1.0f, 360.0f);
    }
    if (kPulseMode.enabled) {
      startPulseMove(PULSE_MOVING_TO_CONTRACT);
    }
    printPulseStatus();
    return;
  }

  Serial.println(F("err usage pulse <on|off|status|set>"));
}

void handleLine(char* line) {
  trimInPlace(line);
  if (line[0] == '\0') return;

  char* save_ptr = nullptr;
  char* command = strtok_r(line, " ", &save_ptr);
  if (command == nullptr) return;

  if (strcmp(command, "ping") == 0) {
    Serial.println(F("pong"));
    return;
  }
  if (strcmp(command, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "map") == 0) {
    printMap();
    return;
  }
  if (strcmp(command, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(command, "attach") == 0) {
    attachAll();
    Serial.println(F("ok attach"));
    return;
  }
  if (strcmp(command, "detach") == 0) {
    detachAll();
    Serial.println(F("ok detach"));
    return;
  }
  if (strcmp(command, "home") == 0) {
    disablePulseMode();
    applyPose("home");
    return;
  }
  if (strcmp(command, "stop") == 0) {
    disablePulseMode();
    stopMotion();
    return;
  }
  if (strcmp(command, "pose") == 0) {
    disablePulseMode();
    char* pose_name = strtok_r(nullptr, " ", &save_ptr);
    if (pose_name == nullptr) {
      Serial.println(F("err usage pose <open|pregrasp|close|home>"));
      return;
    }
    applyPose(pose_name);
    return;
  }
  if (strcmp(command, "grasp") == 0) {
    disablePulseMode();
    char* amount_text = strtok_r(nullptr, " ", &save_ptr);
    if (amount_text == nullptr) {
      Serial.println(F("err usage grasp <0.0..1.0>"));
      return;
    }
    setGrasp(static_cast<float>(atof(amount_text)));
    return;
  }
  if (strcmp(command, "set") == 0) {
    handleSetCommand(save_ptr);
    return;
  }
  if (strcmp(command, "setall") == 0) {
    handleSetAllCommand(save_ptr);
    return;
  }
  if (strcmp(command, "trim") == 0) {
    handleTrimCommand(save_ptr);
    return;
  }
  if (strcmp(command, "test") == 0) {
    handleTestCommand(save_ptr);
    return;
  }
  if (strcmp(command, "pulse") == 0) {
    handlePulseCommand(save_ptr);
    return;
  }

  Serial.print(F("err unknown command "));
  Serial.println(command);
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      kLineBuffer[kLineLength] = '\0';
      handleLine(kLineBuffer);
      kLineLength = 0;
      continue;
    }
    if (kLineLength + 1 < LINE_BUFFER_SIZE) {
      kLineBuffer[kLineLength++] = incoming;
    } else {
      kLineLength = 0;
      Serial.println(F("err line too long"));
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  initializeState();
  attachAll();
  enablePulseMode();
  Serial.println(F("ready hand-servo-controller"));
  Serial.println(F("wiring signal pins: thumb=3 index=5 middle=6 ring=9 pinky=10"));
  Serial.println(F("use external 5V/6V servo power and common ground"));
  Serial.println(F("default pulse: contract=10.0 deg, splay=145.0 deg, hold=5000 ms"));
  printHelp();
}

void loop() {
  readSerialLines();
  updateMotion();
  updatePulseMode();
}
