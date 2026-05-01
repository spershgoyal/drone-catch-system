// sensors.h — pot reads, FK, slip detection, tip anchor world position.
#ifndef ARM_SENSORS_H
#define ARM_SENSORS_H
#include "config.h"
#include "state.h"

inline int potPin(int j) {
  static const int pins[3] = { BASE_POT_PIN, SHLD_POT_PIN, ELBW_POT_PIN };
  return pins[j];
}

// 4-sample average + α=0.4 EMA. Faster than v3 16-sample boxcar with
// cleaner derivative for tilt PD.
inline int readPotFiltered(int pin, int prev) {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  int sample = (int)(sum >> 2);
  return (int)(0.4f * sample + 0.6f * prev);
}

inline float rawToAngle(int j, int raw) {
  return (float)(raw - POT_AT_0[j]) / COUNTS_PER_DEG[j];
}

inline bool magOffsetActive() {
  return magAttached && (mode == XYZ_MODE || mode == HEIGHT || mode == CATCH_MODE);
}

// Updates curRaw, curAngle. Detects slips (uncommanded joint motion).
// jointMotorActive[] tells us whether each joint is currently being driven.
inline void updateSensors(const bool *jointMotorActive) {
  static int prevRaw[3] = { 0, 0, 0 };
  static float angleAtT0[3] = { 0, 0, 0 };
  static uint32_t t0[3] = { 0, 0, 0 };
  uint32_t now = millis();
  for (int j = 0; j < 3; j++) {
    curRaw[j] = readPotFiltered(potPin(j), curRaw[j]);
    curAngle[j] = rawToAngle(j, curRaw[j]) + JOINT_OFFSET[j];
    if (now - t0[j] >= SLIP_DETECT_MS) {
      float dA = curAngle[j] - angleAtT0[j];
      if (!jointMotorActive[j] && fabs(dA) > SLIP_DETECT_DEG) {
        if (!slipDetected[j]) {
          slipDetected[j] = true;
          slipDetectedMs[j] = now;
          Serial.print("[slip] j"); Serial.print(j);
          Serial.print(" d="); Serial.print(dA, 1); Serial.println("deg");
        }
      }
      angleAtT0[j] = curAngle[j];
      t0[j] = now;
    }
  }
  for (int j = 0; j < 3; j++) {
    if (slipDetected[j] && now - slipDetectedMs[j] > 1000) slipDetected[j] = false;
  }
}

// FK: wrist joint position from joint angles (deg).
inline void anglesToXYZ(float bDeg, float sDeg, float eDeg,
                        float &x, float &y, float &z) {
  float b = bDeg * DEG_TO_RAD;
  float s = sDeg * DEG_TO_RAD;
  float e = eDeg * DEG_TO_RAD;
  float r = -L1 * sin(s) + L2 * sin(e - s);
  z       =  L1 * cos(s) + L2 * cos(e - s) + SHOULDER_OFFSET_Z;
  x = r * cos(b);
  y = r * sin(b);
}

// User-facing tip (magnet face if auto-leveled, else wrist).
inline void currentTipXYZ(float &x, float &y, float &z) {
  anglesToXYZ(curAngle[0], curAngle[1], curAngle[2], x, y, z);
  if (magOffsetActive()) z += MAG_TIP_OFFSET;
}

// Tip anchor world position. Boom is mounted on forearm before wrist joints,
// so wrist roll/pitch don't affect it. TIP_ANCHOR_LOCAL = [perp, along, perpZ]
// in forearm-local frame.
inline void tipAnchorXYZ(float &x, float &y, float &z) {
  float bRad = curAngle[0] * DEG_TO_RAD;
  float sRad = curAngle[1] * DEG_TO_RAD;
  float eRad = curAngle[2] * DEG_TO_RAD;

  // Forearm direction unit vector (in r-z plane, before base yaw).
  float fr = sin(eRad - sRad);
  float fz = cos(eRad - sRad);

  // Wrist position
  float wristR = -L1 * sin(sRad) + L2 * fr;
  float wristZ =  L1 * cos(sRad) + L2 * fz + SHOULDER_OFFSET_Z;

  // Boom-base point: along-forearm offset from wrist (negative = toward elbow)
  float along = TIP_ANCHOR_LOCAL[1];
  float boomR = wristR + along * fr;
  float boomZ = wristZ + along * fz + TIP_ANCHOR_LOCAL[2];

  // Boom sticks out perpendicular to forearm in the rotating-base X direction.
  // Before base yaw: arm radial extends along Y. Perp is X.
  float perp = TIP_ANCHOR_LOCAL[0];

  // Apply base yaw. With base=0 putting arm along +X (per existing FK convention
  // where x = r*cos(b), y = r*sin(b)):
  float cb = cos(bRad), sb = sin(bRad);
  // arm radial direction = (cb, sb, 0); perpendicular in plane = (-sb, cb, 0)
  x = boomR * cb + perp * (-sb);
  y = boomR * sb + perp * cb;
  z = boomZ;
}

#endif
