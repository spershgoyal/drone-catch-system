// ============================================================================
// arm_v4.ino — main file
// Target: Teensy 4.1
// Date: 2026-04-30
//
// v4 changes vs v3p_4:
//   * Switched target Teensy 3.5 -> 4.1
//   * Magnet pin moved 16 -> 33 (frees Serial4 for tip anchor UART)
//   * Added: 4 RYUW122 UWB modules (3 base + 1 tip) on Serial1/2/4/6
//   * Added: trilateration + light tip-anchor fusion for drone position
//   * Added: full catch FSM (CATCH_MODE) — autonomous catch sequence
//   * Added: tilt mode v2 — quaternion + lookahead, optimized for boats
//   * Added: software stall detection (no IS pins yet)
//   * Added: slip detection — flags uncommanded joint motion gracefully
//   * Added: sim drone mode — UI feeds fake drone position via $D X Y Z
//   * Added: anchor position cal commands + EEPROM persistence
//   * Telemetry $T extended with catch/UWB fields (back-compat)
//   * IK cost function adds trajectory penalty for forbidden zone
//
// File layout:
//   config.h           pins, constants, anchor positions
//   state.h            shared global state
//   sensors.h          pot + FK + slip + tip anchor FK
//   motors.h           BTS7960 abstraction + stall detect
//   safety.h           floor guard + magnet (with magLocked)
//   kinematics.h       IK with cost function
//   trajectory.h       s-curve + synced moves + position update
//   wrist.h            servo + auto-level
//   poses.h            preset poses
//   dance.h            dance sequences
//   tilt.h             tilt mode v2
//   uwb.h              RYUW122 driver
//   trilat.h           trilateration + tip fusion
//   catch_fsm.h        catch FSM
//   eeprom_persist.h   EEPROM save/load
//   telemetry.h        $T format
//   serial_cmd.h       command parsing
// ============================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <PWMServo.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#include "config.h"
#include "state.h"
#include "sensors.h"
#include "motors.h"
#include "safety.h"
#include "kinematics.h"
#include "trajectory.h"
#include "wrist.h"
#include "poses.h"
#include "dance.h"
#include "tilt.h"
#include "uwb.h"
#include "trilat.h"
#include "catch_fsm.h"
#include "eeprom_persist.h"
#include "telemetry.h"
#include "serial_cmd.h"

// ===========================================================================
// GLOBAL DEFINITIONS (extern in state.h)
// ===========================================================================

// sensors / pose
int   curRaw[3]   = { 0, 0, 0 };
float curAngle[3] = { 0, 0, 0 };
int   POT_AT_0[3];
float COUNTS_PER_DEG[3];
float JOINT_OFFSET[3]    = { 0, -17.0f, 0 };
float ROLL_LEVEL_OFFSET  = -10.0f;
float PITCH_LEVEL_OFFSET = 0;
float MAG_PITCH_BIAS     = MAG_PITCH_BIAS_DEFAULT;
bool  magAttached        = true;

// anchors (RAM-mutable copies of defaults)
float ANCHOR_POS[ANCHOR_COUNT][3];
float TIP_ANCHOR_LOCAL[3];

// mode / speed
Mode  mode = MANUAL;
int   speed = 40;
unsigned long pulseMs = 500;
float stepIn = 1.0f;
bool  danceLoop = false;

// joint move state
bool  jMoving[3]    = { false, false, false };
float jTarget[3]    = { 0, 0, 0 };
float jStart[3]     = { 0, 0, 0 };
float jTotalDist[3] = { 0, 0, 0 };
int   jPeakPWM[3]   = { 40, 40, 40 };
unsigned long jMoveStartMs[3] = { 0, 0, 0 };
bool  jReversed[3]  = { false, false, false };
bool  jWrongChecked[3] = { false, false, false };
bool  pulseActive[3] = { false, false, false };
unsigned long pulseStopMs[3] = { 0, 0, 0 };

// magnet
bool magOn = false;
bool magLocked = false;

// slip + stall
bool          slipDetected[3]   = { false, false, false };
uint32_t      slipDetectedMs[3] = { 0, 0, 0 };
uint32_t      lastMoveMs[3]     = { 0, 0, 0 };
bool          stalled[3]        = { false, false, false };

// IMU
BNO080 imu;
bool  imuReady = false;
float qNow_w = 1, qNow_x = 0, qNow_y = 0, qNow_z = 0;
float qRef_w = 1, qRef_x = 0, qRef_y = 0, qRef_z = 0;
float omegaX = 0, omegaY = 0, omegaZ = 0;
uint32_t lastImuMs = 0;
int TILT_SIGN_ROLL = +1, TILT_SIGN_PITCH = +1, TILT_SIGN_YAW = +1;

// tilt v2
float wtX = 0, wtY = 0, wtZ = 20;
bool  hasWorldTarget = false;
float jSmooth[3] = { 0, 0, 0 };
uint32_t lastTiltUpdateMs = 0;
bool  tiltCompCatch = true;

// catch FSM
CatchState catchState = CATCH_IDLE;
uint32_t   catchStateMs = 0;
bool       catchArmed = false;
uint32_t   catchArmedMs = 0;
uint32_t   integrityFailMs = 0;
uint32_t   droneLastSeenMs = 0;
float      droneLastVarSq = 999;

// drone position
float dronePosX = 0, dronePosY = 0, dronePosZ = 0;
bool  droneValid = false;
float droneResidual = 0;

// sim drone
bool  simDroneOn = false;
float simDroneX = 0, simDroneY = 0, simDroneZ = 24;

// telemetry
bool  telemetryEnabled = true;
unsigned long lastTelemetryMs = 0;
unsigned long lastDebugPrint = 0;

// input
String inputBuf = "";

// dance
int  activeDance = 0;
int  danceStep = 0;
unsigned long danceStepT = 0;
bool danceRunning = false;

// floor
bool floorTripped = false;

// wrist servos
PWMServo sRoll, sPitch;
int rollPos = 90, pitchPos = 90;

// uwb anchors
UwbAnchor uwbAnchors[UWB_NUM_ANCHORS];

// ===========================================================================
// FORWARDED FUNCTIONS (state.h declared)
// ===========================================================================
void stopMotor(int j) { stopMotor_impl(j); }
void stopAll() { stopAll_impl(); }
void runMotor(int j, bool d, int p) { runMotor_impl(j, d, p); }
void magnetOn_silent()  { magnetOn_silent_impl(); }
void magnetOff_silent() { magnetOff_silent_impl(); }
void magnetOn()  { magnetOn_impl(); }
void magnetOff() { magnetOff_impl(); }
void emitTelemetry() { emitTelemetry_impl(); }

// ===========================================================================
// HELPERS
// ===========================================================================
inline void buildJointMotorActive(bool out[3]) {
  for (int j = 0; j < 3; j++) {
    out[j] = jMoving[j] || pulseActive[j];
  }
}

inline void printStatus() {
  Serial.print("mode="); Serial.print((int)mode);
  Serial.print(" base="); Serial.print(curAngle[0],1);
  Serial.print(" shld="); Serial.print(curAngle[1],1);
  Serial.print(" elbw="); Serial.print(curAngle[2],1);
  Serial.println();
  float x, y, z; currentTipXYZ(x, y, z);
  Serial.print("tip: ("); Serial.print(x,1); Serial.print(",");
  Serial.print(y,1); Serial.print(","); Serial.print(z,1); Serial.println(")");
  Serial.print("IMU: "); Serial.println(imuReady ? "ok" : "off");
  Serial.print("CATCH: "); Serial.println(catchStateName(catchState));
}

inline void printHelp() {
  Serial.println("=== ARM v4 ===");
  Serial.print("FW: "); Serial.print(ARM_FW_VERSION);
  Serial.print(" date "); Serial.println(ARM_FW_DATE);
  Serial.println("modes: M manual D dance H height K xyz B debug T tilt C catch");
  Serial.println("       G start dance  X abort  S telem toggle  i info  ? help");
  Serial.println("poses: 6 home 7 ready 8 catch 9 ext 0 retract");
  Serial.println("manual: 1/q 2/w 3/e 4/f 5/t");
  Serial.println("xyz: wasd qe + 'c X Y Z'");
  Serial.println("catch: 'arm' 'disarm' 'catch reset' 'tcompcatch on/off'");
  Serial.println("sim: 'simdrone on' '$D X Y Z'");
  Serial.println("uwb: 'uwb status' 'uwb reinit' 'cal anchor N x y z' 'cal tip p l z'");
  Serial.println("test: 'test motors|pots|uwb|imu|mag|all'");
  Serial.println("cal: 'cal' 'cal b/s/e <deg>' 'cal wr/wp <deg>' 'cal save|load|reset'");
  Serial.println("mag: 'mag on/off' 'mag lock on/off' 'magattach on/off'");
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(250);

  // Init defaults to RAM
  for (int i = 0; i < 3; i++) {
    POT_AT_0[i]      = POT_AT_0_DEFAULT[i];
    COUNTS_PER_DEG[i] = COUNTS_PER_DEG_DEFAULT[i];
  }
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    for (int k = 0; k < 3; k++) ANCHOR_POS[i][k] = ANCHOR_POS_DEFAULT[i][k];
  }
  for (int k = 0; k < 3; k++) TIP_ANCHOR_LOCAL[k] = TIP_ANCHOR_LOCAL_DEFAULT[k];

  // Pins
  analogReadResolution(12);
  pinMode(BASE_POT_PIN, INPUT);
  pinMode(SHLD_POT_PIN, INPUT);
  pinMode(ELBW_POT_PIN, INPUT);
  pinMode(MAG_PIN, OUTPUT);
  digitalWrite(MAG_PIN, LOW);
  for (int j = 0; j < 3; j++) {
    pinMode(rPwmPin(j), OUTPUT);
    pinMode(lPwmPin(j), OUTPUT);
    analogWriteFrequency(rPwmPin(j), 1000);
    analogWriteFrequency(lPwmPin(j), 1000);
    stopMotor_impl(j);
  }

  // Servos
  sRoll.attach(SERVO_ROLL_PIN);
  sPitch.attach(SERVO_PITCH_PIN);
  wristLevel();

  // EEPROM
  if (loadOffsetsFromEEPROM()) Serial.println("[eeprom] loaded");
  else Serial.println("[eeprom] no valid data, using defaults");
  printOffsets();

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  if (imu.begin()) {
    imu.enableRotationVector(50);
    imuReady = true;
    Serial.println("[imu] online");
  } else {
    Serial.println("[imu] not found");
  }

  // UWB
  uwb_initBegin();

  // Initial sensor read
  bool dummy[3] = { false, false, false };
  for (int i = 0; i < 3; i++) updateSensors(dummy);  // settle filter
  validatePoses();
  printHelp();
  printStatus();
}

// ===========================================================================
// LOOP
// ===========================================================================
void loop() {
  uint32_t now = millis();

  // --- sensors + UWB (always run) ---
  bool active[3]; buildJointMotorActive(active);
  updateSensors(active);
  updateImu(imuReady);
  uwb_update();

  // --- drone position update ---
  if (simDroneOn) {
    dronePosX = simDroneX; dronePosY = simDroneY; dronePosZ = simDroneZ;
    droneValid = true;
  } else {
    float x, y, z, res, tipDisc;
    if (trilatSolveDrone(x, y, z, res, tipDisc)) {
      dronePosX = x; dronePosY = y; dronePosZ = z;
      droneValid = true;
      droneResidual = res;
      updateTipIntegrity(tipDisc);
    } else {
      droneValid = false;
    }
  }

  // --- safety ---
  hardLimitCheck();
  bool allowFloor = (mode == CATCH_MODE && (catchState == CATCH_COMMITTING || catchState == CATCH_CATCHING));
  floorGuard(allowFloor);
  updateStallDetect(active);

  // --- mode-specific updates ---
  updateWristAutoLevel();
  if (mode == TILT || (mode == CATCH_MODE && tiltCompCatch && hasWorldTarget)) {
    updateTiltMode_v2(tiltCompCatch);
  }
  if (mode != TILT) {
    bool tightMargin = (mode == CATCH_MODE && (catchState == CATCH_COMMITTING || catchState == CATCH_CATCHING));
    bool disableWrongWay = (mode == CATCH_MODE);
    positionUpdate(tightMargin, disableWrongWay);
    if (mode == MANUAL || mode == DEBUG_MODE) updateManualPulses();
  }
  if (mode == DANCE_MODE) updateDance();
  if (mode == CATCH_MODE) updateCatchFsm();

  // --- serial input ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) { handleLine(inputBuf); inputBuf = ""; }
      continue;
    }
    if (inputBuf.length() > 0) { inputBuf += c; continue; }

    // single-char shortcuts
    if (c == 'M' || c == 'D' || c == 'H' || c == 'K' || c == 'B' || c == 'T' || c == 'C') {
      doModeSwitch(c); continue;
    }
    if (c == 'X') { stopAll_impl(); Serial.println("[abort]"); continue; }
    if (c == 'G') { if (mode == DANCE_MODE && !danceRunning) startDance(); continue; }
    if (c == 'F') { inputBuf = "F"; continue; }
    if (c == '+') { speed = min(255, speed + 10); Serial.print("speed="); Serial.println(speed); continue; }
    if (c == '-') { speed = max(10, speed - 10); Serial.print("speed="); Serial.println(speed); continue; }
    if (c == ']') { pulseMs += 100; Serial.print("pulseMs="); Serial.println(pulseMs); continue; }
    if (c == '[') { if (pulseMs >= 200) pulseMs -= 100; Serial.print("pulseMs="); Serial.println(pulseMs); continue; }
    if (c == '.') { stepIn += 0.25f; Serial.print("step="); Serial.println(stepIn,2); continue; }
    if (c == ',') { if (stepIn > 0.25f) stepIn -= 0.25f; Serial.print("step="); Serial.println(stepIn,2); continue; }
    if (c == 'L') { danceLoop = !danceLoop; Serial.print("loop="); Serial.println(danceLoop?"on":"off"); continue; }
    if (c == 'S') { telemetryEnabled = !telemetryEnabled; continue; }
    if (c == '?') { printHelp(); continue; }
    if (c == 'i') { printStatus(); continue; }

    // poses (auto-switch)
    if (c == '6' || c == '7' || c == '8' || c == '9' || c == '0') {
      if (tryPoseKey(c)) continue;
    }
    if (mode == TILT && c == 'h') { captureHoldPosition(); continue; }
    if (c == 'c') { inputBuf = "c"; continue; }
    if (mode == HEIGHT && (isdigit(c) || c == '-' || c == '.')) { inputBuf = String(c); continue; }
    // XYZ teleop wins over buffer-line entry for these chars
    if (mode == XYZ_MODE && (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'q' || c == 'e')) {
      doXYZKey(c); continue;
    }
    // line-starters
    if (c == 'd' && mode != XYZ_MODE) { inputBuf = "d"; continue; }
    if (c == 't' && mode != MANUAL && mode != DEBUG_MODE) { inputBuf = "t"; continue; }
    if (c == 'W') { inputBuf = "W"; continue; }
    if (c == 'm' || c == 'j' || c == 'a' || c == '$') { inputBuf = String(c); continue; }
    if (c == 'u') { inputBuf = "u"; continue; }

    if (mode == MANUAL || mode == DEBUG_MODE) doManualKey(c);
    else if (mode == XYZ_MODE) doXYZKey(c);
  }

  if (mode == DEBUG_MODE && now - lastDebugPrint >= DEBUG_PRINT_MS) {
    lastDebugPrint = now;
    Serial.print("p38="); Serial.print(curRaw[0]);
    Serial.print(" p21="); Serial.print(curRaw[1]);
    Serial.print(" p20="); Serial.println(curRaw[2]);
  }

  uint32_t telemInterval = (mode == CATCH_MODE) ? TELEMETRY_INTERVAL_MS_CATCH : TELEMETRY_INTERVAL_MS_NORMAL;
  if (telemetryEnabled && now - lastTelemetryMs >= telemInterval) {
    lastTelemetryMs = now;
    emitTelemetry_impl();
  }
}
