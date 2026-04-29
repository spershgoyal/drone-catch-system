// ============================================================================
// arm_main.ino   (v3)
//
// CHANGES vs v2:
//   - JOINT_OFFSET[3] array (degrees, added to every pot reading) so reported
//     angles match physical reality. Default: shoulder = +17° (compensates
//     for the pot mounting being 17° forward of zero), base/elbow = 0°.
//   - Persisted to EEPROM. Loaded on boot.
//   - Runtime adjustable via "cal" commands:
//        cal              print current offsets
//        cal b <deg>      set base offset (-90 to +90)
//        cal s <deg>      set shoulder offset
//        cal e <deg>      set elbow offset
//        cal save         save current offsets to EEPROM
//        cal reset        zero all offsets and save
//        cal load         reload offsets from EEPROM (discard unsaved changes)
//   - Telemetry stream extended with offsets at end so the UI always knows
//     what the firmware is using:
//       $T,bA,sA,eA,tipX,tipY,tipZ,roll,pitch,mode,qw,qx,qy,qz,bOff,sOff,eOff
//
// HARDWARE (Teensy 3.5):  unchanged from v2
//   Base:     RPWM=2   LPWM=29  pot=pin 38
//   Shoulder: RPWM=3   LPWM=5   pot=pin 21
//   Elbow:    RPWM=14  LPWM=35  pot=pin 20
//   Wrist roll  servo1 pin 22
//   Wrist pitch servo2 pin 23
//   BNO08x IMU on Wire (SDA=18, SCL=19)
//
// COMMANDS: same as v2 plus "cal ..." family above.
// ============================================================================

#include <PWMServo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// ---------------------------------------------------------------------------
// PINS  (Teensy 3.5 — all motor PWM pins must be hardware-PWM-capable:
//   2,3,4,5,6,7,8,9,10,14,20,21,22,23,29,30,35,36,37,38)
// ---------------------------------------------------------------------------
#define BASE_RPWM      2
#define BASE_LPWM      29
#define SHLD_RPWM      3
#define SHLD_LPWM      5
#define ELBW_RPWM      14
#define ELBW_LPWM      35
#define SERVO_ROLL_PIN 22
#define SERVO_PITCH_PIN 23

#define BASE_POT_PIN   38
#define SHLD_POT_PIN   21
#define ELBW_POT_PIN   20

const int RPWM[3]    = { BASE_RPWM, SHLD_RPWM, ELBW_RPWM };
const int LPWM[3]    = { BASE_LPWM, SHLD_LPWM, ELBW_LPWM };
const int POT_PIN[3] = { BASE_POT_PIN, SHLD_POT_PIN, ELBW_POT_PIN };

// ---------------------------------------------------------------------------
// CALIBRATION
// ---------------------------------------------------------------------------
const int   POT_AT_0[3]        = { 2061, 1900, 1745 };
const float COUNTS_PER_DEG[3]  = { 8.556f, 14.861f, -15.750f };

// Joint offsets (degrees). Added to the raw-converted angle so curAngle
// reflects physical reality. Mutable at runtime via "cal" commands.
// Defaults: shoulder pot reads ~17° too high relative to physical zero,
// so we subtract 17° to bring reported angle down to match reality.
// Sign convention: positive shoulder = leaning back, negative = leaning forward.
float JOINT_OFFSET[3] = { 0.0f, -17.0f, 0.0f };

// Wrist mounting offsets (degrees). Added to the firmware's nominal "level"
// values so the magnet face is genuinely horizontal at the user's mount.
// Default ROLL offset = -10. We tried +10 first (was 10° clockwise) but that
// overshot to 20° the OPPOSITE way, meaning the original was 10° CCW not CW,
// so the correction needs to subtract.
// EEPROM-saved alongside JOINT_OFFSETs.
float ROLL_LEVEL_OFFSET  = -10.0f;
float PITCH_LEVEL_OFFSET = 0.0f;

// Magic word changes if the layout ever changes, so old saves get rejected.
// JOF3 = bumped because user had a stale +10 saved that needs to be tossed.
const uint16_t EEPROM_ADDR_OFFSETS = 0;
const uint32_t EEPROM_MAGIC        = 0x4A4F4633;  // "JOF3"

// ---------------------------------------------------------------------------
// JOINT LIMITS  <<< EDIT THESE TO CHANGE PER-AXIS RANGE >>>
//
// Each pair is the soft limit in degrees. Hard limits applied in
// hardLimitCheck() (motor PWM cut), and IK rejects any solution outside
// the bounds. Slow-down zone is the last LIMIT_SLOW_ZONE deg before each
// limit where motion is throttled to LIMIT_SLOW_PWM.
//
// Order: { base, shoulder, elbow }.
// ---------------------------------------------------------------------------
const float JOINT_MIN[3] = { -180.0f, -80.0f, -110.0f };
const float JOINT_MAX[3] = {  180.0f,  80.0f,  110.0f };

// Forbidden zone: shoulder is mechanically weird right around vertical
// (worm gear backlash + balance point). IK will avoid solutions in this band.
#define SHLD_FORBID_LO  -10.0f
#define SHLD_FORBID_HI   10.0f

// Slow-zone: how many degrees before a hard limit to start throttling speed.
#define LIMIT_SLOW_ZONE   18.0f
#define LIMIT_SLOW_PWM    28

#define Z_FLOOR            0.0f
#define Z_FLOOR_MARGIN     0.5f

// ---------------------------------------------------------------------------
// LINKS
// ---------------------------------------------------------------------------
#define L1  13.75f
#define L2  17.00f
#define REACH_MAX (L1 + L2)
#define SHOULDER_OFFSET_Z 3.0f

// ---------------------------------------------------------------------------
// SERVOS
// ---------------------------------------------------------------------------
PWMServo sRoll, sPitch;
int rollPos  = 90;
int pitchPos = 90;
const int ROLL_LEVEL  = 90;
const int PITCH_LEVEL = 120;

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------
BNO080 imu;
bool imuReady = false;

float qRefW = 1, qRefX = 0, qRefY = 0, qRefZ = 0;
float wtX = 0, wtY = 0, wtZ = 20;
bool  hasWorldTarget = false;

// ---------------------------------------------------------------------------
// GLOBAL STATE
// ---------------------------------------------------------------------------
enum Mode { MANUAL, DEBUG_MODE, HEIGHT, XYZ_MODE, DANCE_MODE, TILT };
Mode mode = MANUAL;

enum IKResult { IK_OK = 0, IK_TOO_FAR, IK_TOO_CLOSE, IK_NO_VALID, IK_BELOW_FLOOR };

int   speed     = 40;
unsigned long pulseMs = 500;
float stepIn    = 1.0f;
bool  danceLoop = false;

float curAngle[3] = { 0, 0, 0 };
int   curRaw[3]   = { 0, 0, 0 };

bool  jMoving[3]    = { false, false, false };
float jTarget[3]    = { 0, 0, 0 };
float jStart[3]     = { 0, 0, 0 };
float jTotalDist[3] = { 0, 0, 0 };
int   jPeakPWM[3]   = { 40, 40, 40 };

unsigned long jMoveStartMs[3] = { 0, 0, 0 };
bool  jReversed[3]  = { false, false, false };
bool  jWrongChecked[3] = { false, false, false };
const unsigned long WRONG_WAY_CHECK_MS = 250;
const float         WRONG_WAY_THRESH   = 8.0f;

bool          pulseActive[3] = { false, false, false };
unsigned long pulseStopMs[3] = { 0, 0, 0 };

String inputBuf = "";

unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_PRINT_MS = 400;

bool          telemetryEnabled = true;
unsigned long lastTelemetryMs  = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

int           danceStep    = 0;
unsigned long danceStepT   = 0;
bool          danceRunning = false;

const float SCURVE_ACCEL_FRAC  = 0.25f;
const float SCURVE_DECEL_FRAC  = 0.25f;
const float POS_MARGIN_PCT     = 3.0f;
const float POS_MARGIN_MIN_DEG = 0.6f;

void stopMotor(int j);
void stopAll();
void runMotor(int j, bool dirPositive, int spd);
void printStatus();
void printHelp();
void printOffsets();

struct Pose;
struct DanceStep;

// ---------------------------------------------------------------------------
// EEPROM JOINT OFFSET PERSISTENCE
// ---------------------------------------------------------------------------
void saveOffsetsToEEPROM() {
  int addr = EEPROM_ADDR_OFFSETS;
  EEPROM.put(addr, EEPROM_MAGIC);  addr += sizeof(EEPROM_MAGIC);
  for (int j = 0; j < 3; j++) {
    EEPROM.put(addr, JOINT_OFFSET[j]);
    addr += sizeof(float);
  }
  EEPROM.put(addr, ROLL_LEVEL_OFFSET);  addr += sizeof(float);
  EEPROM.put(addr, PITCH_LEVEL_OFFSET); addr += sizeof(float);
  Serial.println("[cal] saved to EEPROM");
}

bool loadOffsetsFromEEPROM() {
  int addr = EEPROM_ADDR_OFFSETS;
  uint32_t magic = 0;
  EEPROM.get(addr, magic);  addr += sizeof(magic);
  if (magic != EEPROM_MAGIC) return false;
  for (int j = 0; j < 3; j++) {
    float v;
    EEPROM.get(addr, v);
    addr += sizeof(float);
    if (isnan(v) || v < -90.0f || v > 90.0f) return false;
    JOINT_OFFSET[j] = v;
  }
  // Wrist offsets (added in JOF2)
  float wr, wp;
  EEPROM.get(addr, wr);  addr += sizeof(float);
  EEPROM.get(addr, wp);  addr += sizeof(float);
  if (!isnan(wr) && wr >= -90.0f && wr <= 90.0f) ROLL_LEVEL_OFFSET = wr;
  if (!isnan(wp) && wp >= -90.0f && wp <= 90.0f) PITCH_LEVEL_OFFSET = wp;
  return true;
}

void resetOffsets() {
  for (int j = 0; j < 3; j++) JOINT_OFFSET[j] = 0.0f;
  ROLL_LEVEL_OFFSET  = 0.0f;
  PITCH_LEVEL_OFFSET = 0.0f;
  saveOffsetsToEEPROM();
  Serial.println("[cal] reset to zero and saved");
}

void printOffsets() {
  // Single-line tagged format the UI parses
  Serial.print("[cal] B=");
  Serial.print(JOINT_OFFSET[0], 2);
  Serial.print(" S=");
  Serial.print(JOINT_OFFSET[1], 2);
  Serial.print(" E=");
  Serial.print(JOINT_OFFSET[2], 2);
  Serial.print(" WR=");
  Serial.print(ROLL_LEVEL_OFFSET, 2);
  Serial.print(" WP=");
  Serial.println(PITCH_LEVEL_OFFSET, 2);
}

// ---------------------------------------------------------------------------
// POT READING / FK
// ---------------------------------------------------------------------------
int readPotAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(pin);
  return (int)(sum / 16);
}

float rawToAngle(int j, int raw) {
  return (float)(raw - POT_AT_0[j]) / COUNTS_PER_DEG[j];
}

void updateSensors() {
  for (int j = 0; j < 3; j++) {
    curRaw[j] = readPotAvg(POT_PIN[j]);
    // Apply per-joint offset so the rest of the firmware (IK, motion, FK,
    // limits, telemetry) all work in physical-reality angle space.
    curAngle[j] = rawToAngle(j, curRaw[j]) + JOINT_OFFSET[j];
  }
}

void anglesToXYZ(float bDeg, float sDeg, float eDeg,
                 float &x, float &y, float &z) {
  float b = bDeg * DEG_TO_RAD;
  float s = sDeg * DEG_TO_RAD;
  float e = eDeg * DEG_TO_RAD;
  float r = -L1 * sin(s) + L2 * sin(e - s);
  z       =  L1 * cos(s) + L2 * cos(e - s) + SHOULDER_OFFSET_Z;
  x = r * cos(b);
  y = r * sin(b);
}

void currentTipXYZ(float &x, float &y, float &z) {
  anglesToXYZ(curAngle[0], curAngle[1], curAngle[2], x, y, z);
}

// ---------------------------------------------------------------------------
// MOTOR DIRECTION
// ---------------------------------------------------------------------------
bool RPWM_IS_PLUS[3] = {
  (COUNTS_PER_DEG[0] > 0),
  (COUNTS_PER_DEG[1] > 0),
  (COUNTS_PER_DEG[2] > 0)
};

void stopMotor(int j) {
  analogWrite(RPWM[j], 0);
  analogWrite(LPWM[j], 0);
  pulseActive[j] = false;
}

void stopAll() {
  for (int j = 0; j < 3; j++) stopMotor(j);
  for (int j = 0; j < 3; j++) {
    jMoving[j] = false;
    jReversed[j] = false;
    jWrongChecked[j] = false;
  }
  danceRunning = false;
  hasWorldTarget = false;
}

void runMotor(int j, bool dirPositive, int spd) {
  bool useRPWM = (dirPositive == RPWM_IS_PLUS[j]);
  if (useRPWM) {
    analogWrite(LPWM[j], 0);
    analogWrite(RPWM[j], spd);
  } else {
    analogWrite(RPWM[j], 0);
    analogWrite(LPWM[j], spd);
  }
}

void hardLimitCheck() {
  for (int j = 0; j < 3; j++) {
    if (curAngle[j] >= JOINT_MAX[j]) {
      if (RPWM_IS_PLUS[j]) analogWrite(RPWM[j], 0);
      else                 analogWrite(LPWM[j], 0);
    }
    if (curAngle[j] <= JOINT_MIN[j]) {
      if (RPWM_IS_PLUS[j]) analogWrite(LPWM[j], 0);
      else                 analogWrite(RPWM[j], 0);
    }
  }
}

// ---------------------------------------------------------------------------
// S-CURVE PROFILE
// ---------------------------------------------------------------------------
int sCurvePWM(float totalDist, float traveled, int peakPWM) {
  if (totalDist <= 0.1f) return LIMIT_SLOW_PWM;
  float frac = traveled / totalDist;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  float scale = 1.0f;
  if (frac < SCURVE_ACCEL_FRAC) {
    float x = frac / SCURVE_ACCEL_FRAC;
    scale = x * x * (3.0f - 2.0f * x);
  } else if (frac > (1.0f - SCURVE_DECEL_FRAC)) {
    float x = (1.0f - frac) / SCURVE_DECEL_FRAC;
    scale = x * x * (3.0f - 2.0f * x);
  }
  int pwm = LIMIT_SLOW_PWM + (int)((peakPWM - LIMIT_SLOW_PWM) * scale);
  if (pwm < LIMIT_SLOW_PWM) pwm = LIMIT_SLOW_PWM;
  if (pwm > peakPWM)        pwm = peakPWM;
  return pwm;
}

int applySlowZone(int j, bool goingPositive, int pwm) {
  float a = curAngle[j];
  if (goingPositive && a >= (JOINT_MAX[j] - LIMIT_SLOW_ZONE)) {
    if (pwm > LIMIT_SLOW_PWM) pwm = LIMIT_SLOW_PWM;
  }
  if (!goingPositive && a <= (JOINT_MIN[j] + LIMIT_SLOW_ZONE)) {
    if (pwm > LIMIT_SLOW_PWM) pwm = LIMIT_SLOW_PWM;
  }
  return pwm;
}

// ---------------------------------------------------------------------------
// JOINT MOVES
// ---------------------------------------------------------------------------
void startJointMoveAt(int j, float targetDeg, int peak) {
  if (targetDeg > JOINT_MAX[j]) targetDeg = JOINT_MAX[j];
  if (targetDeg < JOINT_MIN[j]) targetDeg = JOINT_MIN[j];

  jTarget[j]    = targetDeg;
  jStart[j]     = curAngle[j];
  jTotalDist[j] = fabs(targetDeg - jStart[j]);
  jPeakPWM[j]   = peak;
  jMoving[j]    = (jTotalDist[j] > POS_MARGIN_MIN_DEG);
  jMoveStartMs[j]  = millis();
  jReversed[j]     = false;
  jWrongChecked[j] = false;
  if (!jMoving[j]) stopMotor(j);
}

void startSyncedMove(float bT, float sT, float eT,
                     bool mB, bool mS, bool mE) {
  float dB = mB ? fabs(bT - curAngle[0]) : 0;
  float dS = mS ? fabs(sT - curAngle[1]) : 0;
  float dE = mE ? fabs(eT - curAngle[2]) : 0;
  float dMax = max(max(dB, dS), dE);
  if (dMax < POS_MARGIN_MIN_DEG) {
    for (int j = 0; j < 3; j++) jMoving[j] = false;
    return;
  }

  int range = speed - LIMIT_SLOW_PWM;
  if (range < 0) range = 0;

  if (mB) {
    int pk = LIMIT_SLOW_PWM + (int)(range * (dB / dMax));
    if (pk < LIMIT_SLOW_PWM) pk = LIMIT_SLOW_PWM;
    startJointMoveAt(0, bT, pk);
  }
  if (mS) {
    int pk = LIMIT_SLOW_PWM + (int)(range * (dS / dMax));
    if (pk < LIMIT_SLOW_PWM) pk = LIMIT_SLOW_PWM;
    startJointMoveAt(1, sT, pk);
  }
  if (mE) {
    int pk = LIMIT_SLOW_PWM + (int)(range * (dE / dMax));
    if (pk < LIMIT_SLOW_PWM) pk = LIMIT_SLOW_PWM;
    startJointMoveAt(2, eT, pk);
  }
}

void positionUpdate() {
  for (int j = 0; j < 3; j++) {
    if (!jMoving[j]) continue;

    float err = jTarget[j] - curAngle[j];
    float marginDeg = jTotalDist[j] * (POS_MARGIN_PCT / 100.0f);
    if (marginDeg < POS_MARGIN_MIN_DEG) marginDeg = POS_MARGIN_MIN_DEG;

    if (fabs(err) <= marginDeg) {
      stopMotor(j);
      jMoving[j] = false;
      continue;
    }

    bool goPos = (err > 0);
    float traveled = fabs(curAngle[j] - jStart[j]);

    if (!jWrongChecked[j] && (millis() - jMoveStartMs[j]) > WRONG_WAY_CHECK_MS) {
      float progress = (jTarget[j] - jStart[j]);
      float actual   = (curAngle[j] - jStart[j]);
      bool wrongWay = false;
      if (progress > 0 && actual < -WRONG_WAY_THRESH) wrongWay = true;
      if (progress < 0 && actual >  WRONG_WAY_THRESH) wrongWay = true;
      if (wrongWay) {
        jReversed[j] = true;
        const char *jn[] = { "base", "shoulder", "elbow" };
        Serial.print("[wrong-way] ");
        Serial.print(jn[j]);
        Serial.print(" moved ");
        Serial.print(actual, 1);
        Serial.print("° instead of toward target, reversing for this move");
        Serial.println();
      }
      jWrongChecked[j] = true;
    }

    int pwm;
    if (traveled > jTotalDist[j]) {
      pwm = LIMIT_SLOW_PWM;
    } else {
      pwm = sCurvePWM(jTotalDist[j], traveled, jPeakPWM[j]);
    }
    pwm = applySlowZone(j, goPos, pwm);
    bool driveDir = jReversed[j] ? !goPos : goPos;
    runMotor(j, driveDir, pwm);
  }
}

bool anyJointMoving() {
  return jMoving[0] || jMoving[1] || jMoving[2];
}

// ---------------------------------------------------------------------------
// IK
// ---------------------------------------------------------------------------
static bool solve2Link(float r, float z, bool elbowPositive,
                       float &sOut, float &eOut) {
  float d2 = r * r + z * z;
  float cosE = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosE > 1.0f || cosE < -1.0f) return false;
  float e = acos(cosE);
  if (!elbowPositive) e = -e;
  float A = L2 * sin(e);
  float B = -L1 - L2 * cos(e);
  sOut = atan2(B * r + A * z, A * r - B * z) * RAD_TO_DEG;
  eOut = e * RAD_TO_DEG;
  return true;
}

// IK cost. Two factors that matter:
//   1) Hard limit violations (huge penalty)
//   2) Forbidden zone (medium penalty)
//   3) JOINT continuity (small) - mild preference for current angles
//   4) ELBOW SIGN continuity (BIG) - if config flips current elbow direction,
//      add huge penalty. This is THE fix for "X moves cause big Z swings":
//      previously the IK happily flipped between elbow-up/elbow-down configs
//      when costs were near-equal, which makes the tip take a long arc through
//      space. Now we strongly prefer the config that keeps the elbow on the
//      same side it's already on, so tip motion stays smooth.
static float ikCost(float s, float e) {
  float c = 0;

  // Hard limit penalties
  if (s > JOINT_MAX[1]) c += 1000.0f * (s - JOINT_MAX[1]);
  if (s < JOINT_MIN[1]) c += 1000.0f * (JOINT_MIN[1] - s);
  if (e > JOINT_MAX[2]) c += 1000.0f * (e - JOINT_MAX[2]);
  if (e < JOINT_MIN[2]) c += 1000.0f * (JOINT_MIN[2] - e);

  // Forbidden zone (shoulder near vertical = backlash)
  if (s > SHLD_FORBID_LO && s < SHLD_FORBID_HI) c += 50.0f;

  // Elbow-sign continuity. The elbow's "side" is the sign of the elbow angle.
  // If our candidate flips the sign relative to current pose, that's a big
  // reconfigure and the tip will swing wildly through the workspace getting
  // there. Heavy penalty discourages it unless there's no other option.
  bool curElbowPositive = (curAngle[2] >= 0);
  bool newElbowPositive = (e >= 0);
  if (curElbowPositive != newElbowPositive) c += 200.0f;

  // Continuity (mild) - prefer staying close to current angles when ties exist
  c += 0.05f * fabs(s - curAngle[1]);
  c += 0.05f * fabs(e - curAngle[2]);

  return c;
}

IKResult solveXYZ(float x, float y, float z,
                  float &baseDeg, float &shldDeg, float &elbwDeg) {
  if (z < Z_FLOOR) return IK_BELOW_FLOOR;
  float zShld = z - SHOULDER_OFFSET_Z;
  float r = sqrt(x * x + y * y);
  float d = sqrt(r * r + zShld * zShld);
  if (d > REACH_MAX + 0.01f) return IK_TOO_FAR;
  if (d < fabs(L1 - L2) - 0.01f) return IK_TOO_CLOSE;

  baseDeg = atan2(y, x) * RAD_TO_DEG;

  float sA, eA, sB, eB;
  bool okA = solve2Link(r, zShld, true,  sA, eA);
  bool okB = solve2Link(r, zShld, false, sB, eB);
  if (!okA && !okB) return IK_NO_VALID;

  float costA = okA ? ikCost(sA, eA) : 1e9f;
  float costB = okB ? ikCost(sB, eB) : 1e9f;
  if (costA <= costB) { shldDeg = sA; elbwDeg = eA; }
  else                { shldDeg = sB; elbwDeg = eB; }

  if (shldDeg > JOINT_MAX[1] + 0.5f || shldDeg < JOINT_MIN[1] - 0.5f)
    return IK_NO_VALID;
  if (elbwDeg > JOINT_MAX[2] + 0.5f || elbwDeg < JOINT_MIN[2] - 0.5f)
    return IK_NO_VALID;
  return IK_OK;
}

bool solveHeight(float targetZ, float &shldDeg, float &elbwDeg) {
  if (targetZ < Z_FLOOR || targetZ > REACH_MAX + SHOULDER_OFFSET_Z) return false;
  float zShld = targetZ - SHOULDER_OFFSET_Z;
  if (zShld < -REACH_MAX || zShld > REACH_MAX) return false;
  float cosE = (zShld * zShld - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosE > 1.0f || cosE < -1.0f) return false;

  float best = 1e9f;
  float bestS = 0, bestE = 0;
  bool  foundValid = false;
  for (int sign = 0; sign < 2; sign++) {
    float e = (sign ? acos(cosE) : -acos(cosE));
    float A = L2 * sin(e);
    float B = -L1 - L2 * cos(e);
    float sRad = atan2(-A, B);
    float zCheck = L1 * cos(sRad) + L2 * cos(e - sRad);
    if (fabs(zCheck - zShld) > 0.5f) {
      sRad = atan2(A, -B);
      zCheck = L1 * cos(sRad) + L2 * cos(e - sRad);
      if (fabs(zCheck - zShld) > 0.5f) continue;
    }
    float sDeg = sRad * RAD_TO_DEG;
    float eDeg = e * RAD_TO_DEG;
    // Reject hard-limit-violators directly. New cost weights make threshold-
    // based validity checks unreliable.
    if (sDeg > JOINT_MAX[1] + 0.5f || sDeg < JOINT_MIN[1] - 0.5f) continue;
    if (eDeg > JOINT_MAX[2] + 0.5f || eDeg < JOINT_MIN[2] - 0.5f) continue;
    float c = ikCost(sDeg, eDeg);
    if (c < best) { best = c; bestS = sDeg; bestE = eDeg; foundValid = true; }
  }
  if (!foundValid) return false;
  shldDeg = bestS;
  elbwDeg = bestE;
  return true;
}

// ---------------------------------------------------------------------------
// HIGH-LEVEL MOVES
// ---------------------------------------------------------------------------
bool moveToXYZ(float x, float y, float z) {
  float b, s, e;
  IKResult r = solveXYZ(x, y, z, b, s, e);
  if (r != IK_OK) {
    Serial.print("[XYZ] ");
    if (r == IK_TOO_FAR)          Serial.println("too far");
    else if (r == IK_TOO_CLOSE)   Serial.println("too close");
    else if (r == IK_BELOW_FLOOR) Serial.println("below floor");
    else                          Serial.println("no valid config");
    return false;
  }
  Serial.print("[XYZ] (");
  Serial.print(x,1); Serial.print(","); Serial.print(y,1);
  Serial.print(","); Serial.print(z,1); Serial.print(") -> B:");
  Serial.print(b,1); Serial.print(" S:"); Serial.print(s,1);
  Serial.print(" E:"); Serial.println(e,1);
  startSyncedMove(b, s, e, true, true, true);
  return true;
}

bool moveToAnglesSynced(float b, float s, float e) {
  float x, y, z;
  anglesToXYZ(b, s, e, x, y, z);
  if (z < Z_FLOOR) {
    Serial.print("[pose] tip Z=");
    Serial.print(z, 1);
    Serial.println(" below floor, abort");
    return false;
  }
  startSyncedMove(b, s, e, true, true, true);
  return true;
}

void teleopNudge(float dx, float dy, float dz) {
  float cx, cy, cz;
  currentTipXYZ(cx, cy, cz);
  float nx = cx + dx, ny = cy + dy, nz = cz + dz;
  if (!moveToXYZ(nx, ny, nz)) {
    Serial.print("   kept at ("); Serial.print(cx,1);
    Serial.print(","); Serial.print(cy,1);
    Serial.print(","); Serial.print(cz,1); Serial.println(")");
  }
}

void setRoll(int pos)  { rollPos  = constrain(pos, 0, 180); sRoll.write(rollPos); }
void setPitch(int pos) { pitchPos = constrain(pos, 0, 180); sPitch.write(pitchPos); }

// Set wrist to "level" using the calibrated mounting offsets so the magnet
// face is genuinely horizontal regardless of how the servos were assembled.
void wristLevel() {
  setRoll((int)roundf(ROLL_LEVEL  + ROLL_LEVEL_OFFSET));
  setPitch((int)roundf(PITCH_LEVEL + PITCH_LEVEL_OFFSET));
}

// ---------------------------------------------------------------------------
// WRIST AUTO-LEVEL
// ---------------------------------------------------------------------------
// Holds the magnet face pointing UP regardless of arm pose. The forearm
// makes an angle (shoulder - elbow) with vertical, so to keep the magnet
// horizontal, the wrist pitch servo has to counter-rotate by that amount.
//
//   target_pitch = PITCH_LEVEL + PITCH_LEVEL_OFFSET - (shoulder - elbow)
//
// Roll just stays at calibrated level (magnet is circular, doesn't matter).
//
// ACTIVE only in XYZ_MODE and HEIGHT modes. Disabled in MANUAL/DEBUG/DANCE/
// TILT so the user gets full wrist control or the existing routines keep
// theirs.
const unsigned long WRIST_LEVEL_MS = 80;   // 12.5 Hz, smooth without spamming
unsigned long lastWristLevelMs = 0;

void updateWristAutoLevel() {
  if (mode != XYZ_MODE && mode != HEIGHT) return;
  if (millis() - lastWristLevelMs < WRIST_LEVEL_MS) return;
  lastWristLevelMs = millis();

  float forearm_deg = curAngle[1] - curAngle[2];
  int target_pitch = (int)roundf(PITCH_LEVEL + PITCH_LEVEL_OFFSET - forearm_deg);
  int target_roll  = (int)roundf(ROLL_LEVEL  + ROLL_LEVEL_OFFSET);

  // Only write servo if it would change by 1° or more (avoids servo jitter)
  if (abs(target_pitch - pitchPos) >= 1) setPitch(target_pitch);
  if (abs(target_roll  - rollPos)  >= 1) setRoll(target_roll);
}

// ---------------------------------------------------------------------------
// PRESET POSES
// ---------------------------------------------------------------------------
struct Pose {
  char key;
  bool useXYZ;
  float a, b, c;
  int   roll, pitch;
  const char *name;
};

Pose POSES[] = {
  // key, useXYZ, a/x, b/y, c/z, roll, pitch, name
  { '6', false,   0.0f,  50.0f,  90.0f,  90, 90, "home"       },
  { '7', false,   0.0f, -40.0f,  30.0f,  90, 90, "ready"      },
  { '8', true,   10.0f,   0.0f,  25.0f,  90, 90, "catch ready"},
  { '9', true,   14.0f,   0.0f,  20.0f,  90, 90, "extended"   },
  // retract: pulled back from elbow=110 (which sat right at JOINT_MAX) to 105
  // so the move actually reaches its target instead of stopping in the slow
  // zone at ~107°. Edit alongside JOINT_MAX[2] if limits change.
  { '0', false,   0.0f,  60.0f, 105.0f,  90, 90, "retract"    },
};
const int NUM_POSES = sizeof(POSES) / sizeof(POSES[0]);

void runPose(const Pose &p) {
  Serial.print("[pose] "); Serial.println(p.name);
  setRoll(p.roll); setPitch(p.pitch);
  if (p.useXYZ) moveToXYZ(p.a, p.b, p.c);
  else          moveToAnglesSynced(p.a, p.b, p.c);
}

bool tryPoseKey(char c) {
  for (int i = 0; i < NUM_POSES; i++) {
    if (POSES[i].key == c) { runPose(POSES[i]); return true; }
  }
  return false;
}

// Sanity-check all poses against the configured joint limits. Called from
// setup(). Prints warnings for any pose target that lies outside the soft
// limits or inside the slow-down zone (where the move will get throttled).
//
// For useXYZ=true poses we run IK and check the resulting joint angles. For
// angle poses we check directly. This way, edits to JOINT_MIN/MAX or to any
// pose surface conflicts on boot rather than at runtime.
void validatePoses() {
  Serial.println("[poses] validating against limits...");
  for (int i = 0; i < NUM_POSES; i++) {
    const Pose &p = POSES[i];
    float b, s, e;
    bool ok = true;
    if (p.useXYZ) {
      IKResult r = solveXYZ(p.a, p.b, p.c, b, s, e);
      if (r != IK_OK) {
        Serial.print("  [WARN] pose '"); Serial.print(p.name);
        Serial.println("' IK FAILED at given XYZ");
        ok = false;
      }
    } else {
      b = p.a; s = p.b; e = p.c;
    }
    if (!ok) continue;
    const float vals[3] = { b, s, e };
    const char *jn[] = { "base", "shoulder", "elbow" };
    for (int j = 0; j < 3; j++) {
      if (vals[j] > JOINT_MAX[j] || vals[j] < JOINT_MIN[j]) {
        Serial.print("  [WARN] pose '"); Serial.print(p.name);
        Serial.print("' "); Serial.print(jn[j]); Serial.print("=");
        Serial.print(vals[j], 1); Serial.print(" outside [");
        Serial.print(JOINT_MIN[j], 0); Serial.print(",");
        Serial.print(JOINT_MAX[j], 0); Serial.println("]");
      } else if (vals[j] > JOINT_MAX[j] - LIMIT_SLOW_ZONE ||
                 vals[j] < JOINT_MIN[j] + LIMIT_SLOW_ZONE) {
        Serial.print("  [note] pose '"); Serial.print(p.name);
        Serial.print("' "); Serial.print(jn[j]); Serial.print("=");
        Serial.print(vals[j], 1); Serial.println(" inside slow-zone (will be throttled)");
      }
    }
  }
}

// ---------------------------------------------------------------------------
// DANCE
// ---------------------------------------------------------------------------
struct DanceStep {
  float x, y, z;
  int   roll, pitch;
  unsigned long holdMs;
  const char *name;
};

struct Dance {
  const char *name;
  const DanceStep *steps;
  int nSteps;
};

const DanceStep SHOWCASE_STEPS[] = {
  {  6.0f, 0.0f, 24.0f,  90, 120,   500,  "wake"        },
  {  8.0f, 0.0f, 26.0f,  90,  90,   300,  "point fwd"   },
  {  7.1f, 7.1f, 22.0f, 120, 110,   250,  "CW1"         },
  {  0.0f,10.0f, 26.0f, 150, 120,   250,  "CW2 high"    },
  { -7.1f, 7.1f, 22.0f, 170, 110,   250,  "CW3"         },
  {-10.0f, 0.0f, 26.0f, 170, 130,   250,  "CW4 high"    },
  { -7.1f,-7.1f, 22.0f, 140, 110,   250,  "CW5"         },
  {  0.0f,-10.0f,26.0f,  90, 120,   250,  "CW6 high"    },
  {  7.1f,-7.1f, 22.0f,  40, 110,   250,  "CW7"         },
  { 10.0f, 0.0f, 26.0f,  90, 130,   400,  "front"       },
  { 10.0f, 0.0f, 24.0f,   0, 120,   300,  "roll L"      },
  { 10.0f, 0.0f, 24.0f, 180, 120,   300,  "roll R"      },
  { 10.0f, 0.0f, 24.0f,  90, 165,   300,  "nod down"    },
  { 10.0f, 0.0f, 24.0f,  90,  75,   300,  "nod up"      },
  { 10.0f, 0.0f, 24.0f,  90, 120,   200,  "center"      },
  { 20.0f, 0.0f,  8.0f,  90, 175,   600,  "bow"         },
  {  5.0f, 0.0f, 28.0f,  90,  60,   500,  "rise up"     },
  {  0.0f,-10.0f,22.0f,  90, 120,   300,  "pivot"       },
  { -8.0f, 0.0f, 24.0f,  90, 120,   300,  "pivot 2"     },
  {  0.0f, 0.0f, 28.0f,  90, 120,   500,  "tall"        },
  {  6.0f, 0.0f, 22.0f,  90, 120,   400,  "home"        },
};

const DanceStep GREET_STEPS[] = {
  {  8.0f, 0.0f, 26.0f,  90, 120,   400,  "rise"        },
  { 10.0f, 0.0f, 22.0f,  90, 100,   200,  "lean down"   },
  { 10.0f, 0.0f, 26.0f,  90, 130,   200,  "lean up"     },
  { 10.0f, 0.0f, 22.0f,  90, 100,   200,  "lean down 2" },
  { 10.0f, 0.0f, 26.0f,  90, 130,   200,  "lean up 2"   },
  {  9.0f, 0.0f, 24.0f,  60, 120,   300,  "tilt R"      },
  {  9.0f, 0.0f, 24.0f, 120, 120,   300,  "tilt L"      },
  { 14.0f, 0.0f, 16.0f,  90, 165,   600,  "small bow"   },
  {  6.0f, 0.0f, 24.0f,  90, 120,   400,  "back home"   },
};

const DanceStep IDLE_STEPS[] = {
  {  4.0f, 0.0f, 25.0f,  90, 120,  1500,  "rest"        },
  {  4.0f, 0.0f, 25.5f,  92, 122,  1200,  "breathe in"  },
  {  4.0f, 0.0f, 24.5f,  88, 118,  1200,  "breathe out" },
  {  5.0f, 1.0f, 25.0f,  85, 120,   800,  "look L"      },
  {  4.0f, 0.0f, 25.0f,  90, 120,  1000,  "center"      },
  {  5.0f,-1.0f, 25.0f,  95, 120,   800,  "look R"      },
  {  4.0f, 0.0f, 25.5f,  90, 125,  1200,  "stretch up"  },
  {  4.0f, 0.0f, 25.0f,  90, 120,  1500,  "settle"      },
};

const DanceStep COMBAT_STEPS[] = {
  {  0.0f, 0.0f, 28.0f,  90, 120,   200,  "ready"       },
  { 12.0f, 0.0f, 18.0f,  90, 100,   100,  "strike fwd"  },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"     },
  {  0.0f,12.0f, 18.0f,  90, 100,   100,  "strike L"    },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"     },
  {-12.0f, 0.0f, 18.0f,  90, 100,   100,  "strike back" },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"     },
  {  0.0f,-12.0f,18.0f,  90, 100,   100,  "strike R"    },
  {  0.0f, 0.0f, 28.0f,  90, 120,   200,  "guard"       },
  {  8.0f, 8.0f, 22.0f,  45, 110,   150,  "scan FL"     },
  { -8.0f, 8.0f, 22.0f, 135, 110,   150,  "scan BL"     },
  { -8.0f,-8.0f, 22.0f, 135, 110,   150,  "scan BR"     },
  {  8.0f,-8.0f, 22.0f,  45, 110,   150,  "scan FR"     },
  {  0.0f, 0.0f, 30.0f,  90, 120,   400,  "all clear"   },
};

const DanceStep CHASE_STEPS[] = {
  {  0.0f, 0.0f, 26.0f,  90, 120,   300,  "scan"        },
  {  9.0f, 5.0f, 24.0f,  60, 110,   180,  "track 1"     },
  { 10.0f, 0.0f, 23.0f,  90, 105,   180,  "track 2"     },
  {  9.0f,-5.0f, 24.0f, 120, 110,   180,  "track 3"     },
  {  5.0f,-9.0f, 25.0f, 150, 115,   180,  "track 4"     },
  { -5.0f,-9.0f, 25.0f, 150, 115,   180,  "track 5"     },
  { -9.0f,-5.0f, 24.0f, 120, 110,   180,  "track 6"     },
  {-10.0f, 0.0f, 23.0f,  90, 105,   180,  "track 7"     },
  { -9.0f, 5.0f, 24.0f,  60, 110,   180,  "track 8"     },
  { -5.0f, 9.0f, 25.0f,  30, 115,   180,  "track 9"     },
  {  5.0f, 9.0f, 25.0f,  30, 115,   180,  "track 10"    },
  {  9.0f, 5.0f, 24.0f,  60, 110,   180,  "loop"        },
  { 11.0f, 0.0f, 18.0f,  90,  90,   200,  "GRAB"        },
  {  6.0f, 0.0f, 26.0f,  90, 120,   400,  "secure"      },
};

const Dance DANCES[] = {
  { "showcase", SHOWCASE_STEPS, sizeof(SHOWCASE_STEPS) / sizeof(SHOWCASE_STEPS[0]) },
  { "greet",    GREET_STEPS,    sizeof(GREET_STEPS)    / sizeof(GREET_STEPS[0])    },
  { "idle",     IDLE_STEPS,     sizeof(IDLE_STEPS)     / sizeof(IDLE_STEPS[0])     },
  { "combat",   COMBAT_STEPS,   sizeof(COMBAT_STEPS)   / sizeof(COMBAT_STEPS[0])   },
  { "chase",    CHASE_STEPS,    sizeof(CHASE_STEPS)    / sizeof(CHASE_STEPS[0])    },
};
const int N_DANCES = sizeof(DANCES) / sizeof(DANCES[0]);

int activeDance = 0;

void advanceDance(int idx) {
  const Dance &d = DANCES[activeDance];
  if (idx >= d.nSteps) return;
  const DanceStep &s = d.steps[idx];
  moveToXYZ(s.x, s.y, s.z);
  setRoll(s.roll);
  setPitch(s.pitch);
  danceStepT = millis();
  Serial.print("[dance "); Serial.print(d.name);
  Serial.print(" "); Serial.print(idx); Serial.print("] ");
  Serial.println(s.name);
}

void startDance() {
  danceRunning = true;
  danceStep = 0;
  Serial.print("[dance] starting ");
  Serial.println(DANCES[activeDance].name);
  advanceDance(0);
}

void selectDance(int idx) {
  if (idx < 0 || idx >= N_DANCES) return;
  activeDance = idx;
  Serial.print("[dance] selected: ");
  Serial.println(DANCES[idx].name);
}

void updateDance() {
  if (!danceRunning) return;
  const Dance &d = DANCES[activeDance];
  unsigned long elapsed = millis() - danceStepT;
  if (anyJointMoving()) return;
  if (elapsed < d.steps[danceStep].holdMs) return;

  danceStep++;
  if (danceStep >= d.nSteps) {
    if (danceLoop) {
      danceStep = 0;
      Serial.println("[dance] -- loop --");
    } else {
      danceRunning = false;
      wristLevel();
      Serial.println("[dance] done");
      return;
    }
  }
  advanceDance(danceStep);
}

// ---------------------------------------------------------------------------
// TILT-COMP MODE
// ---------------------------------------------------------------------------
const float TILT_DEADBAND   = 2.5f;
const float TILT_KP         = 2.5f;
const float TILT_KD         = 0.4f;
const float TILT_NEAR_DAMP  = 4.0f;
const float TILT_NEAR_DEG   = 5.0f;

int  TILT_SIGN_ROLL  = +1;
int  TILT_SIGN_PITCH = +1;
int  TILT_SIGN_YAW   = +1;

static void quatMul(float aw, float ax, float ay, float az,
                    float bw, float bx, float by, float bz,
                    float &rw, float &rx, float &ry, float &rz) {
  rw = aw*bw - ax*bx - ay*by - az*bz;
  rx = aw*bx + ax*bw + ay*bz - az*by;
  ry = aw*by - ax*bz + ay*bw + az*bx;
  rz = aw*bz + ax*by - ay*bx + az*bw;
}

static void quatToEuler(float qw, float qx, float qy, float qz,
                        float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp);
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  pitch = asin(sinp);
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp);
}

float refRoll = 0, refPitch = 0, refYaw = 0;
float lastTiltErr[3] = { 0, 0, 0 };
unsigned long lastTiltErrMs = 0;

void setTiltReference() {
  if (!imuReady) {
    Serial.println("[tilt] IMU offline, ref=identity");
    qRefW = 1; qRefX = 0; qRefY = 0; qRefZ = 0;
    refRoll = refPitch = refYaw = 0;
    return;
  }
  unsigned long t = millis();
  while (millis() - t < 200) {
    if (imu.dataAvailable()) {
      qRefW = imu.getQuatReal();
      qRefX = imu.getQuatI();
      qRefY = imu.getQuatJ();
      qRefZ = imu.getQuatK();
    }
  }
  quatToEuler(qRefW, qRefX, qRefY, qRefZ, refRoll, refPitch, refYaw);
  Serial.print("[tilt] ref euler r/p/y = ");
  Serial.print(refRoll * RAD_TO_DEG, 1); Serial.print("/");
  Serial.print(refPitch * RAD_TO_DEG, 1); Serial.print("/");
  Serial.println(refYaw * RAD_TO_DEG, 1);
}

void captureHoldPosition() {
  if (!imuReady) {
    Serial.println("[tilt] IMU offline, cannot capture");
    return;
  }
  setTiltReference();
  float x, y, z;
  currentTipXYZ(x, y, z);
  wtX = x; wtY = y; wtZ = z;
  hasWorldTarget = true;
  Serial.print("[tilt] holding tip at (");
  Serial.print(x,1); Serial.print(","); Serial.print(y,1);
  Serial.print(","); Serial.print(z,1); Serial.println(")");
}

unsigned long lastTiltUpdate = 0;
const unsigned long TILT_UPDATE_MS = 20;

void updateTiltMode() {
  if (mode != TILT || !hasWorldTarget || !imuReady) return;
  if (millis() - lastTiltUpdate < TILT_UPDATE_MS) return;
  unsigned long now = millis();
  float dt = (now - lastTiltUpdate) / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  lastTiltUpdate = now;
  if (!imu.dataAvailable()) return;

  float curRoll, curPitch, curYaw;
  quatToEuler(imu.getQuatReal(), imu.getQuatI(),
              imu.getQuatJ(), imu.getQuatK(),
              curRoll, curPitch, curYaw);

  auto wrap = [](float a) -> float {
    while (a >  PI) a -= 2*PI;
    while (a < -PI) a += 2*PI;
    return a;
  };
  float dRoll  = TILT_SIGN_ROLL  * wrap(curRoll  - refRoll);
  float dPitch = TILT_SIGN_PITCH * wrap(curPitch - refPitch);
  float dYaw   = TILT_SIGN_YAW   * wrap(curYaw   - refYaw);

  float hr = -dRoll  * 0.5f, hp = -dPitch * 0.5f, hy = -dYaw * 0.5f;
  float qY_w = cos(hy), qY_z = sin(hy);
  float qP_w = cos(hp), qP_y = sin(hp);
  float qR_w = cos(hr), qR_x = sin(hr);
  float qP_qR_w, qP_qR_x, qP_qR_y, qP_qR_z;
  quatMul(qP_w, 0, qP_y, 0,  qR_w, qR_x, 0, 0,
          qP_qR_w, qP_qR_x, qP_qR_y, qP_qR_z);
  float qDw, qDx, qDy, qDz;
  quatMul(qY_w, 0, 0, qY_z,  qP_qR_w, qP_qR_x, qP_qR_y, qP_qR_z,
          qDw, qDx, qDy, qDz);

  auto rotate = [](float qw, float qx, float qy, float qz,
                   float vx, float vy, float vz,
                   float &ox, float &oy, float &oz) {
    float t0 = 2.0f * (qy*vz - qz*vy);
    float t1 = 2.0f * (qz*vx - qx*vz);
    float t2 = 2.0f * (qx*vy - qy*vx);
    ox = vx + qw*t0 + (qy*t2 - qz*t1);
    oy = vy + qw*t1 + (qz*t0 - qx*t2);
    oz = vz + qw*t2 + (qx*t1 - qy*t0);
  };
  float lx, ly, lz;
  rotate(qDw, qDx, qDy, qDz, wtX, wtY, wtZ, lx, ly, lz);

  float bT, sT, eT;
  IKResult res = solveXYZ(lx, ly, lz, bT, sT, eT);
  if (res != IK_OK) return;

  float tgt[3] = { bT, sT, eT };
  for (int j = 0; j < 3; j++) {
    float err = tgt[j] - curAngle[j];
    if (fabs(err) < TILT_DEADBAND) {
      stopMotor(j);
      lastTiltErr[j] = err;
      continue;
    }
    float dErr = (err - lastTiltErr[j]) / dt;
    lastTiltErr[j] = err;

    float pwmF = TILT_KP * fabs(err);
    if (fabs(err) < TILT_NEAR_DEG) {
      pwmF -= TILT_NEAR_DAMP * fabs(dErr);
      if (pwmF < 0) pwmF = 0;
    }
    int pwm = (int)pwmF;
    if (pwm < LIMIT_SLOW_PWM && pwm > 0) pwm = LIMIT_SLOW_PWM;
    if (pwm > speed) pwm = speed;
    if (pwm == 0) { stopMotor(j); continue; }
    pwm = applySlowZone(j, err > 0, pwm);
    runMotor(j, err > 0, pwm);
  }
}

// ---------------------------------------------------------------------------
// MANUAL PULSE
// ---------------------------------------------------------------------------
void manualPulse(int j, bool dirPositive) {
  if (dirPositive && curAngle[j] >= JOINT_MAX[j]) {
    Serial.print("[limit] j"); Serial.print(j); Serial.println(" +max");
    return;
  }
  if (!dirPositive && curAngle[j] <= JOINT_MIN[j]) {
    Serial.print("[limit] j"); Serial.print(j); Serial.println(" -min");
    return;
  }
  int spd = applySlowZone(j, dirPositive, speed);
  runMotor(j, dirPositive, spd);
  pulseActive[j] = true;
  pulseStopMs[j] = millis() + pulseMs;
}

void updateManualPulses() {
  for (int j = 0; j < 3; j++) {
    if (pulseActive[j] && millis() >= pulseStopMs[j]) stopMotor(j);
  }
}

// ---------------------------------------------------------------------------
// RUNTIME FLOOR GUARD
// ---------------------------------------------------------------------------
bool floorTripped = false;
void floorGuard() {
  float x, y, z;
  currentTipXYZ(x, y, z);
  if (z < Z_FLOOR - Z_FLOOR_MARGIN) {
    if (!floorTripped) {
      Serial.print("[FLOOR] tip Z=");
      Serial.print(z, 1);
      Serial.println(" below, e-stop");
      stopAll();
      floorTripped = true;
    }
  } else {
    floorTripped = false;
  }
}

// ---------------------------------------------------------------------------
// CAL COMMANDS
// ---------------------------------------------------------------------------
// Format:
//   cal                      print current offsets
//   cal b <deg>              set base offset
//   cal s <deg>              set shoulder offset
//   cal e <deg>              set elbow offset
//   cal save                 save to EEPROM
//   cal load                 reload from EEPROM
//   cal reset                zero all and save
//
// Sets are NOT auto-saved. User must follow with "cal save" to persist.
void handleCalLine(String body) {
  body.trim();
  if (body.length() == 0) {
    printOffsets();
    return;
  }
  if (body == "save") {
    saveOffsetsToEEPROM();
    printOffsets();
    return;
  }
  if (body == "load") {
    if (anyJointMoving() || hasWorldTarget || danceRunning) {
      Serial.println("[cal] motion in progress, halting before reload");
      stopAll();
    }
    if (loadOffsetsFromEEPROM()) {
      Serial.println("[cal] reloaded from EEPROM");
    } else {
      Serial.println("[cal] no valid EEPROM data, kept current values");
    }
    printOffsets();
    return;
  }
  if (body == "reset") {
    if (anyJointMoving() || hasWorldTarget || danceRunning) {
      Serial.println("[cal] motion in progress, halting before reset");
      stopAll();
    }
    resetOffsets();
    printOffsets();
    return;
  }
  // Format: "<axis> <degrees>"  e.g. "s -17.0"
  // Also: "wr <deg>" / "wp <deg>" for wrist roll/pitch mounting offsets
  if (body.length() < 3) {
    Serial.println("[cal] usage: cal | cal b|s|e|wr|wp <deg> | cal save|load|reset");
    return;
  }

  // Wrist mounting offsets (handle BEFORE single-letter axis check since
  // "wr"/"wp" are 2 chars).
  if (body.startsWith("wr ") || body.startsWith("wp ")) {
    bool isRoll = (body.charAt(1) == 'r');
    String numS = body.substring(3); numS.trim();
    if (numS.length() == 0) { Serial.println("[cal] missing number"); return; }
    float v = numS.toFloat();
    if (v < -90.0f || v > 90.0f) { Serial.println("[cal] wrist offset must be -90..+90 deg"); return; }
    if (isRoll) ROLL_LEVEL_OFFSET  = v;
    else        PITCH_LEVEL_OFFSET = v;
    printOffsets();
    return;
  }

  char axis = body.charAt(0);
  if (axis != 'b' && axis != 's' && axis != 'e') {
    Serial.println("[cal] axis must be b, s, e, or wr|wp");
    return;
  }
  String numS = body.substring(1); numS.trim();
  if (numS.length() == 0) {
    Serial.println("[cal] missing number");
    return;
  }
  float v = numS.toFloat();
  if (v < -90.0f || v > 90.0f) {
    Serial.println("[cal] offset must be -90..+90 deg");
    return;
  }
  int j = (axis == 'b') ? 0 : (axis == 's') ? 1 : 2;
  if (anyJointMoving() || hasWorldTarget || danceRunning) {
    Serial.println("[cal] motion in progress, halting before applying offset");
    stopAll();
  }
  JOINT_OFFSET[j] = v;
  printOffsets();
}

// ---------------------------------------------------------------------------
// LINE COMMANDS
// ---------------------------------------------------------------------------
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "Fb" || line == "Fs" || line == "Fe") {
    int j = (line.charAt(1) == 'b') ? 0 : (line.charAt(1) == 's') ? 1 : 2;
    RPWM_IS_PLUS[j] = !RPWM_IS_PLUS[j];
    const char *jn[] = { "base", "shoulder", "elbow" };
    Serial.print("[flip] "); Serial.print(jn[j]);
    Serial.print(" RPWM_IS_PLUS="); Serial.println(RPWM_IS_PLUS[j] ? "t" : "f");
    return;
  }

  // CAL family — line starts with "cal" optionally followed by space + body
  if (line == "cal") { handleCalLine(""); return; }
  if (line.startsWith("cal ")) { handleCalLine(line.substring(4)); return; }

  if (line.startsWith("dn ")) {
    String rest = line.substring(3); rest.trim();
    if (rest.length() == 1 && isdigit(rest.charAt(0))) {
      selectDance(rest.toInt());
    } else {
      for (int i = 0; i < N_DANCES; i++) {
        if (rest == DANCES[i].name) { selectDance(i); return; }
      }
      Serial.print("[dn] not found. options: ");
      for (int i = 0; i < N_DANCES; i++) {
        Serial.print(DANCES[i].name); Serial.print(" ");
      }
      Serial.println();
    }
    return;
  }

  if (line.startsWith("tflip ")) {
    char a = line.charAt(6);
    if (a == 'r')      { TILT_SIGN_ROLL  = -TILT_SIGN_ROLL;  Serial.print("[tilt] roll sign ="); Serial.println(TILT_SIGN_ROLL); }
    else if (a == 'p') { TILT_SIGN_PITCH = -TILT_SIGN_PITCH; Serial.print("[tilt] pitch sign ="); Serial.println(TILT_SIGN_PITCH); }
    else if (a == 'y') { TILT_SIGN_YAW   = -TILT_SIGN_YAW;   Serial.print("[tilt] yaw sign ="); Serial.println(TILT_SIGN_YAW); }
    else Serial.println("[tflip] usage: tflip r|p|y");
    return;
  }

  // Wrist absolute set commands. Used by the web app's catch sequence, also
  // handy from the terminal. Range 0-180. PITCH_LEVEL=120 (face up) on this
  // build, ROLL_LEVEL=90.
  //   wr <deg> / WR <deg>    set wrist roll
  //   wp <deg> / WP <deg>    set wrist pitch
  // Note: only the UPPERCASE form starts a buffer from a keystroke, because
  // lowercase 'w' is a manual-mode jog key. Lines arriving as a single chunk
  // (like from the web app) will land in handleLine() either way.
  if (line.startsWith("wr ") || line.startsWith("wp ") ||
      line.startsWith("WR ") || line.startsWith("WP ")) {
    int deg = line.substring(3).toInt();
    if (deg < 0 || deg > 180) {
      Serial.println("[wrist] deg must be 0-180");
      return;
    }
    char which = line.charAt(1);
    if (which == 'r' || which == 'R') { setRoll(deg);  Serial.print("[wrist] roll = ");  Serial.println(deg); }
    else                              { setPitch(deg); Serial.print("[wrist] pitch = "); Serial.println(deg); }
    return;
  }

  if (line.startsWith("c ") || line.startsWith("C ") ||
      line.startsWith("xyz ") || line.startsWith("XYZ ")) {
    int sp1 = line.indexOf(' ');
    String rest = line.substring(sp1 + 1); rest.trim();
    int sp2 = rest.indexOf(' ');
    int sp3 = rest.indexOf(' ', sp2 + 1);
    if (sp2 < 0 || sp3 < 0) { Serial.println("[c] usage: c X Y Z"); return; }
    float x = rest.substring(0, sp2).toFloat();
    float y = rest.substring(sp2 + 1, sp3).toFloat();
    float z = rest.substring(sp3 + 1).toFloat();

    if (mode == TILT) {
      wtX = x; wtY = y; wtZ = z;
      hasWorldTarget = true;
      Serial.print("[tilt] world target=(");
      Serial.print(x,1); Serial.print(","); Serial.print(y,1);
      Serial.print(","); Serial.print(z,1); Serial.println(")");
    } else {
      moveToXYZ(x, y, z);
    }
    return;
  }

  if (mode == HEIGHT) {
    float z = line.toFloat();
    if (z <= Z_FLOOR || z > REACH_MAX + SHOULDER_OFFSET_Z) {
      Serial.print("[h] height must be (");
      Serial.print(Z_FLOOR, 1); Serial.print(", ");
      Serial.print(REACH_MAX + SHOULDER_OFFSET_Z, 1); Serial.println("]");
      return;
    }
    float s, e;
    if (solveHeight(z, s, e)) {
      Serial.print("[H] "); Serial.print(z, 1);
      Serial.print(" -> S:"); Serial.print(s, 1);
      Serial.print(" E:"); Serial.println(e, 1);
      startSyncedMove(0, s, e, false, true, true);
    } else {
      Serial.println("[h] unreachable");
    }
    return;
  }

  Serial.print("[?] "); Serial.println(line);
}

// ---------------------------------------------------------------------------
// MODE SWITCH
// ---------------------------------------------------------------------------
void doModeSwitch(char c) {
  switch (c) {
    case 'M': mode = MANUAL;     break;
    case 'D': mode = DANCE_MODE; break;
    case 'H': mode = HEIGHT;     break;
    case 'K': mode = XYZ_MODE;   break;
    case 'B': mode = DEBUG_MODE; break;
    case 'T': mode = TILT;       break;
    default: return;
  }
  stopAll();
  inputBuf = "";
  wristLevel();

  const char *names[] = { "MANUAL", "DEBUG", "HEIGHT", "XYZ", "DANCE", "TILT" };
  Serial.print(">> "); Serial.println(names[mode]);

  if (mode == HEIGHT) {
    Serial.print("   type inches in ("); Serial.print(Z_FLOOR,1);
    Serial.print(","); Serial.print(REACH_MAX + SHOULDER_OFFSET_Z, 1);
    Serial.println("] + Enter");
  } else if (mode == XYZ_MODE) {
    Serial.println("   wasd=X/Y, qe=Z, 'c X Y Z'+Enter");
    float x, y, z; currentTipXYZ(x, y, z);
    Serial.print("   tip at ("); Serial.print(x,1); Serial.print(",");
    Serial.print(y,1); Serial.print(","); Serial.print(z,1); Serial.println(")");
  } else if (mode == DANCE_MODE) {
    Serial.println("   G=go  L=loop");
  } else if (mode == TILT) {
    setTiltReference();
    Serial.println("   h = hold current tip here.  'c X Y Z'+Enter = hold custom target.");
    Serial.println("   X clears. M exits.");
  }
}

void doManualKey(char c) {
  switch (c) {
    case '1': manualPulse(0, true);  break;
    case 'q': manualPulse(0, false); break;
    case '2': manualPulse(1, true);  break;
    case 'w': manualPulse(1, false); break;
    case '3': manualPulse(2, true);  break;
    case 'e': manualPulse(2, false); break;
    case '4': setRoll(rollPos + 10);   break;
    case 'f': setRoll(rollPos - 10);   break;
    case '5': setPitch(pitchPos + 10); break;
    case 't': setPitch(pitchPos - 10); break;
  }
}

void doXYZKey(char c) {
  switch (c) {
    case 'w': teleopNudge( stepIn, 0, 0); break;
    case 's': teleopNudge(-stepIn, 0, 0); break;
    case 'a': teleopNudge(0,  stepIn, 0); break;
    case 'd': teleopNudge(0, -stepIn, 0); break;
    case 'q': teleopNudge(0, 0,  stepIn); break;
    case 'e': teleopNudge(0, 0, -stepIn); break;
  }
}

// ---------------------------------------------------------------------------
// STATUS / HELP
// ---------------------------------------------------------------------------
void printStatus() {
  float x, y, z; currentTipXYZ(x, y, z);
  const char *names[] = { "MANUAL", "DEBUG", "HEIGHT", "XYZ", "DANCE", "TILT" };
  Serial.println();
  Serial.print("mode="); Serial.print(names[mode]);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" pulseMs="); Serial.print(pulseMs);
  Serial.print(" stepIn="); Serial.print(stepIn, 2);
  Serial.print(" loop="); Serial.println(danceLoop ? "on" : "off");
  Serial.print("angles: B="); Serial.print(curAngle[0],1);
  Serial.print(" S="); Serial.print(curAngle[1],1);
  Serial.print(" E="); Serial.println(curAngle[2],1);
  Serial.print("offsets: B="); Serial.print(JOINT_OFFSET[0], 2);
  Serial.print(" S="); Serial.print(JOINT_OFFSET[1], 2);
  Serial.print(" E="); Serial.println(JOINT_OFFSET[2], 2);
  Serial.print("tip: ("); Serial.print(x,1); Serial.print(",");
  Serial.print(y,1); Serial.print(","); Serial.print(z,1); Serial.println(")");
  Serial.print("wrist r="); Serial.print(rollPos);
  Serial.print(" p="); Serial.println(pitchPos);
  Serial.print("IMU: "); Serial.println(imuReady ? "online" : "offline");
}

void printHelp() {
  Serial.println();
  Serial.println("=== ARM v3 ===");
  Serial.println("All XYZ in base frame (z=0 at base joint, shoulder pivot at z=3).");
  Serial.println("modes: M manual  D dance  H height  K XYZ  B debug  T tilt");
  Serial.println("       G go(dance)  X abort  S stream telemetry");
  Serial.println("poses: 6 home  7 ready  8 catch  9 extended  0 retract");
  Serial.println("manual/debug: 1/q 2/w 3/e  4/f 5/t");
  Serial.println("XYZ: wasd+qe teleop, 'c X Y Z'+Enter, . , step");
  Serial.println("tilt: h=hold here,  'c X Y Z'+Enter=hold custom,  X clears");
  Serial.println("        tflip r|p|y = flip tilt sign for that axis");
  Serial.println("dance: dn 0=showcase 1=greet 2=idle 3=combat 4=chase");
  Serial.println("settings: +/- speed  ]/[ pulseMs  L loop  i info  ? help");
  Serial.println("flip motor: Fb/Fs/Fe + Enter");
  Serial.println("wrist set: WR <deg> + Enter / WP <deg> + Enter (0-180, 90/120 = level)");
  Serial.println("CAL (joint offsets, deg, added to pot reading):");
  Serial.println("    cal               print current offsets");
  Serial.println("    cal b|s|e <deg>   set joint offset (-90..+90) -- not auto-saved");
  Serial.println("    cal wr|wp <deg>   set wrist roll/pitch mounting offset");
  Serial.println("    cal save          persist current to EEPROM");
  Serial.println("    cal load          reload from EEPROM");
  Serial.println("    cal reset         zero all and save");
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(250);

  analogReadResolution(12);
  pinMode(BASE_POT_PIN, INPUT);
  pinMode(SHLD_POT_PIN, INPUT);
  pinMode(ELBW_POT_PIN, INPUT);

  for (int j = 0; j < 3; j++) {
    pinMode(RPWM[j], OUTPUT);
    pinMode(LPWM[j], OUTPUT);
    analogWriteFrequency(RPWM[j], 1000);
    analogWriteFrequency(LPWM[j], 1000);
    stopMotor(j);
  }

  sRoll.attach(SERVO_ROLL_PIN);
  sPitch.attach(SERVO_PITCH_PIN);
  wristLevel();

  // Load saved joint offsets BEFORE first sensor read so curAngle is correct
  // immediately. If no valid EEPROM data, defaults are used.
  if (loadOffsetsFromEEPROM()) {
    Serial.println("[cal] loaded from EEPROM");
  } else {
    Serial.println("[cal] no valid EEPROM data, using defaults");
  }
  printOffsets();

  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  if (imu.begin()) {
    imu.enableRotationVector(50);
    imuReady = true;
    Serial.println("[imu] BNO08x online");
  } else {
    Serial.println("[imu] not found, tilt mode disabled");
  }

  updateSensors();
  validatePoses();
  printHelp();
  printStatus();
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
void loop() {
  updateSensors();
  hardLimitCheck();
  floorGuard();
  updateWristAutoLevel();

  if (mode == TILT) {
    updateTiltMode();
  } else {
    positionUpdate();
    if (mode == MANUAL || mode == DEBUG_MODE) {
      updateManualPulses();
    }
  }
  if (mode == DANCE_MODE) updateDance();

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) { handleLine(inputBuf); inputBuf = ""; }
      continue;
    }
    if (inputBuf.length() > 0) { inputBuf += c; continue; }

    if (c == 'M' || c == 'D' || c == 'H' || c == 'K' || c == 'B' || c == 'T') {
      doModeSwitch(c); continue;
    }
    if (c == 'X') { stopAll(); Serial.println("[abort]"); continue; }
    if (c == 'G') {
      if (mode != DANCE_MODE) Serial.println("[G] press D first");
      else if (!danceRunning) startDance();
      continue;
    }
    if (c == 'F') { inputBuf = "F"; continue; }

    if (c == '+') { speed = min(255, speed + 10); Serial.print("speed="); Serial.println(speed); continue; }
    if (c == '-') { speed = max(10,  speed - 10); Serial.print("speed="); Serial.println(speed); continue; }
    if (c == ']') { pulseMs += 100; Serial.print("pulseMs="); Serial.println(pulseMs); continue; }
    if (c == '[') { if (pulseMs >= 200) pulseMs -= 100; Serial.print("pulseMs="); Serial.println(pulseMs); continue; }
    if (c == '.') { stepIn += 0.25f; Serial.print("stepIn="); Serial.println(stepIn,2); continue; }
    if (c == ',') { if (stepIn > 0.25f) stepIn -= 0.25f; Serial.print("stepIn="); Serial.println(stepIn,2); continue; }
    if (c == 'L') { danceLoop = !danceLoop; Serial.print("loop="); Serial.println(danceLoop ? "on" : "off"); continue; }
    if (c == 'S') { telemetryEnabled = !telemetryEnabled; Serial.print("telemetry="); Serial.println(telemetryEnabled ? "on" : "off"); continue; }
    if (c == '?') { printHelp(); continue; }
    if (c == 'i') { printStatus(); continue; }

    if (mode != HEIGHT && (c == '6' || c == '7' || c == '8' || c == '9' || c == '0')) {
      if (tryPoseKey(c)) continue;
    }

    if (mode == TILT && c == 'h') { captureHoldPosition(); continue; }

    if (c == 'c' && (mode == XYZ_MODE || mode == TILT)) { inputBuf = String(c); continue; }
    if (mode == HEIGHT && (isdigit(c) || c == '-' || c == '.')) {
      inputBuf = String(c); continue;
    }

    // line starters: "cal ...", "dn ...", "tflip ...", "WR/WP ..."
    // Note: 'c' is reserved for mode-switch / xyz / tilt above. "cal" gets in
    // because the user types it inside an input buffer that starts with 'c'.
    // To not collide, we only enter buffer mode for lowercase non-mode commands.
    if (c == 'd' && mode != XYZ_MODE) { inputBuf = String(c); continue; }
    if (c == 't' && mode != MANUAL && mode != DEBUG_MODE) { inputBuf = String(c); continue; }
    if (c == 'W') { inputBuf = String(c); continue; }  // uppercase W = wrist set

    if (mode == MANUAL || mode == DEBUG_MODE) doManualKey(c);
    else if (mode == XYZ_MODE)                doXYZKey(c);
  }

  if (mode == DEBUG_MODE && millis() - lastDebugPrint >= DEBUG_PRINT_MS) {
    lastDebugPrint = millis();
    Serial.print("p38="); Serial.print(curRaw[0]);
    Serial.print("("); Serial.print(curAngle[0],1); Serial.print(")  ");
    Serial.print("p21="); Serial.print(curRaw[1]);
    Serial.print("("); Serial.print(curAngle[1],1); Serial.print(")  ");
    Serial.print("p20="); Serial.print(curRaw[2]);
    Serial.print("("); Serial.print(curAngle[2],1); Serial.println(")");
  }

  if (telemetryEnabled && millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    float tx, ty, tz; currentTipXYZ(tx, ty, tz);
    Serial.print("$T,");
    Serial.print(curAngle[0],2); Serial.print(",");
    Serial.print(curAngle[1],2); Serial.print(",");
    Serial.print(curAngle[2],2); Serial.print(",");
    Serial.print(tx,2); Serial.print(",");
    Serial.print(ty,2); Serial.print(",");
    Serial.print(tz,2); Serial.print(",");
    Serial.print(rollPos); Serial.print(",");
    Serial.print(pitchPos); Serial.print(",");
    Serial.print((int)mode); Serial.print(",");
    if (imuReady && imu.dataAvailable()) {
      Serial.print(imu.getQuatReal(),3); Serial.print(",");
      Serial.print(imu.getQuatI(),3); Serial.print(",");
      Serial.print(imu.getQuatJ(),3); Serial.print(",");
      Serial.print(imu.getQuatK(),3);
    } else {
      Serial.print("1,0,0,0");
    }
    // v3: appended joint offsets (degrees) so UI always knows firmware state
    Serial.print(",");
    Serial.print(JOINT_OFFSET[0], 2); Serial.print(",");
    Serial.print(JOINT_OFFSET[1], 2); Serial.print(",");
    Serial.print(JOINT_OFFSET[2], 2);
    Serial.println();
  }
}
