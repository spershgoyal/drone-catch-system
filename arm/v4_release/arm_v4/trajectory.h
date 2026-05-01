// trajectory.h — s-curve PWM profile, synced moves, position-update loop.
#ifndef ARM_TRAJECTORY_H
#define ARM_TRAJECTORY_H
#include "config.h"
#include "state.h"
#include "motors.h"

inline int sCurvePWM(float totalDist, float traveled, int peakPWM) {
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

inline void startJointMoveAt(int j, float targetDeg, int peak) {
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
  if (!jMoving[j]) stopMotor_impl(j);
}

inline void startSyncedMove(float bT, float sT, float eT,
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

// Position-update loop. tightMargin = use tight margins (used during catch
// CONFIRMING). disableWrongWay = don't reverse motor on slip-induced motion
// (catch micromoves; trust the position loop to re-target).
inline void positionUpdate(bool tightMargin, bool disableWrongWay) {
  for (int j = 0; j < 3; j++) {
    if (!jMoving[j]) continue;

    float err = jTarget[j] - curAngle[j];
    float marginDeg = jTotalDist[j] * (POS_MARGIN_PCT / 100.0f);
    float minMargin = tightMargin ? POS_MARGIN_TIGHT_DEG : POS_MARGIN_MIN_DEG;
    if (marginDeg < minMargin) marginDeg = minMargin;

    if (fabs(err) <= marginDeg) {
      stopMotor_impl(j);
      jMoving[j] = false;
      continue;
    }

    bool goPos = (err > 0);
    float traveled = fabs(curAngle[j] - jStart[j]);

    if (!disableWrongWay && !jWrongChecked[j] &&
        (millis() - jMoveStartMs[j]) > WRONG_WAY_CHECK_MS) {
      float progress = (jTarget[j] - jStart[j]);
      float actual   = (curAngle[j] - jStart[j]);
      bool wrongWay = false;
      if (progress > 0 && actual < -WRONG_WAY_THRESH) wrongWay = true;
      if (progress < 0 && actual >  WRONG_WAY_THRESH) wrongWay = true;
      if (wrongWay) {
        jReversed[j] = true;
        const char *jn[] = { "base", "shoulder", "elbow" };
        Serial.print("[wrong-way] "); Serial.print(jn[j]);
        Serial.print(" "); Serial.print(actual, 1); Serial.println(" reversing");
      }
      jWrongChecked[j] = true;
    }

    int pwm = (traveled > jTotalDist[j])
              ? LIMIT_SLOW_PWM
              : sCurvePWM(jTotalDist[j], traveled, jPeakPWM[j]);
    pwm = applySlowZone(j, goPos, pwm);
    bool driveDir = jReversed[j] ? !goPos : goPos;
    runMotor_impl(j, driveDir, pwm);
  }
}

inline bool anyJointMoving() { return jMoving[0] || jMoving[1] || jMoving[2]; }

inline void updateManualPulses() {
  for (int j = 0; j < 3; j++) {
    if (pulseActive[j] && millis() >= pulseStopMs[j]) stopMotor_impl(j);
  }
}

inline void manualPulse(int j, bool dirPositive) {
  if (dirPositive && curAngle[j] >= JOINT_MAX[j]) { Serial.print("[limit] j"); Serial.print(j); Serial.println(" max"); return; }
  if (!dirPositive && curAngle[j] <= JOINT_MIN[j]) { Serial.print("[limit] j"); Serial.print(j); Serial.println(" min"); return; }
  int spd = applySlowZone(j, dirPositive, speed);
  runMotor_impl(j, dirPositive, spd);
  pulseActive[j] = true;
  pulseStopMs[j] = millis() + pulseMs;
}

#endif
