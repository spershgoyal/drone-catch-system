// motors.h — BTS7960 control + software stall detection.
#ifndef ARM_MOTORS_H
#define ARM_MOTORS_H
#include "config.h"
#include "state.h"

inline int rPwmPin(int j) {
  static const int p[3] = { BASE_RPWM, SHLD_RPWM, ELBW_RPWM };
  return p[j];
}
inline int lPwmPin(int j) {
  static const int p[3] = { BASE_LPWM, SHLD_LPWM, ELBW_LPWM };
  return p[j];
}

inline bool RPWM_IS_PLUS(int j) { return COUNTS_PER_DEG[j] > 0; }

inline void stopMotor_impl(int j) {
  analogWrite(rPwmPin(j), 0);
  analogWrite(lPwmPin(j), 0);
  pulseActive[j] = false;
}

inline void runMotor_impl(int j, bool dirPositive, int spd) {
  bool useR = (dirPositive == RPWM_IS_PLUS(j));
  if (useR) {
    analogWrite(lPwmPin(j), 0);
    analogWrite(rPwmPin(j), spd);
  } else {
    analogWrite(rPwmPin(j), 0);
    analogWrite(lPwmPin(j), spd);
  }
}

// Hard-limit PWM cut. Doesn't stop the move state, just zeros PWM if past limit.
inline void hardLimitCheck() {
  for (int j = 0; j < 3; j++) {
    if (curAngle[j] >= JOINT_MAX[j]) {
      if (RPWM_IS_PLUS(j)) analogWrite(rPwmPin(j), 0);
      else                 analogWrite(lPwmPin(j), 0);
    }
    if (curAngle[j] <= JOINT_MIN[j]) {
      if (RPWM_IS_PLUS(j)) analogWrite(lPwmPin(j), 0);
      else                 analogWrite(rPwmPin(j), 0);
    }
  }
}

// Stall detect: motor commanded but pot not moving for STALL_TIMEOUT_MS.
// jointMotorActive[] from caller (true if PWM > 0 / pulse active / move active).
inline void updateStallDetect(const bool *jointMotorActive) {
  static int lastRaw[3] = { 0, 0, 0 };
  uint32_t now = millis();
  for (int j = 0; j < 3; j++) {
    if (abs(curRaw[j] - lastRaw[j]) > STALL_THRESH_RAW) {
      lastMoveMs[j] = now;
      lastRaw[j] = curRaw[j];
      stalled[j] = false;
    } else if (jointMotorActive[j] && now - lastMoveMs[j] > STALL_TIMEOUT_MS) {
      if (!stalled[j]) {
        stalled[j] = true;
        Serial.print("[STALL] j"); Serial.println(j);
        stopMotor_impl(j);
        jMoving[j] = false;
        pulseActive[j] = false;
      }
    }
  }
}

// Slow-down zone enforcement.
inline int applySlowZone(int j, bool goingPositive, int pwm) {
  float a = curAngle[j];
  if (goingPositive && a >= (JOINT_MAX[j] - LIMIT_SLOW_ZONE)) {
    if (pwm > LIMIT_SLOW_PWM) pwm = LIMIT_SLOW_PWM;
  }
  if (!goingPositive && a <= (JOINT_MIN[j] + LIMIT_SLOW_ZONE)) {
    if (pwm > LIMIT_SLOW_PWM) pwm = LIMIT_SLOW_PWM;
  }
  return pwm;
}

#endif
