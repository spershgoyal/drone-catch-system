// tilt.h — tilt v2: quaternion-based, lookahead-predicted, smoothed.
// Optimized for boats: low-frequency oscillation, anticipates motion.
#ifndef ARM_TILT_H
#define ARM_TILT_H
#include "config.h"
#include "state.h"
#include "kinematics.h"
#include "motors.h"
#include "trajectory.h"
#include "SparkFun_BNO080_Arduino_Library.h"

extern BNO080 imu;

// Quaternion ops
inline void qMul(float aw,float ax,float ay,float az,
                 float bw,float bx,float by,float bz,
                 float &rw,float &rx,float &ry,float &rz) {
  rw = aw*bw - ax*bx - ay*by - az*bz;
  rx = aw*bx + ax*bw + ay*bz - az*by;
  ry = aw*by - ax*bz + ay*bw + az*bx;
  rz = aw*bz + ax*by - ay*bx + az*bw;
}

inline void qInv(float w,float x,float y,float z, float &rw,float &rx,float &ry,float &rz) {
  // Assumes unit quaternion (true for IMU output)
  rw = w; rx = -x; ry = -y; rz = -z;
}

inline void qRotate(float qw,float qx,float qy,float qz,
                    float vx,float vy,float vz,
                    float &ox,float &oy,float &oz) {
  float t0 = 2.0f * (qy*vz - qz*vy);
  float t1 = 2.0f * (qz*vx - qx*vz);
  float t2 = 2.0f * (qx*vy - qy*vx);
  ox = vx + qw*t0 + (qy*t2 - qz*t1);
  oy = vy + qw*t1 + (qz*t0 - qx*t2);
  oz = vz + qw*t2 + (qx*t1 - qy*t0);
}

// Read IMU, update angular velocity estimate (deg/s).
inline void updateImu(bool imuReady_) {
  if (!imuReady_) return;
  if (!imu.dataAvailable()) return;
  static float prevW = 1, prevX = 0, prevY = 0, prevZ = 0;
  static uint32_t prevMs = 0;
  uint32_t now = millis();
  float w = imu.getQuatReal(), x = imu.getQuatI(), y = imu.getQuatJ(), z = imu.getQuatK();
  float dt = (now - prevMs) / 1000.0f;
  if (prevMs > 0 && dt > 0.005f && dt < 0.2f) {
    // Relative rotation: q_rel = q_now * inv(q_prev)
    float invW, invX, invY, invZ;
    qInv(prevW, prevX, prevY, prevZ, invW, invX, invY, invZ);
    float rw, rx, ry, rz;
    qMul(w, x, y, z, invW, invX, invY, invZ, rw, rx, ry, rz);
    // Small-angle: omega = 2*[rx,ry,rz]/dt (in radians/sec)
    float ox = 2.0f * rx / dt;
    float oy = 2.0f * ry / dt;
    float oz = 2.0f * rz / dt;
    // EMA smooth
    omegaX = TILT_OMEGA_ALPHA * ox * RAD_TO_DEG + (1 - TILT_OMEGA_ALPHA) * omegaX;
    omegaY = TILT_OMEGA_ALPHA * oy * RAD_TO_DEG + (1 - TILT_OMEGA_ALPHA) * omegaY;
    omegaZ = TILT_OMEGA_ALPHA * oz * RAD_TO_DEG + (1 - TILT_OMEGA_ALPHA) * omegaZ;
  }
  qNow_w = w; qNow_x = x; qNow_y = y; qNow_z = z;
  prevW = w; prevX = x; prevY = y; prevZ = z;
  prevMs = now;
  lastImuMs = now;
}

inline void setTiltReference() {
  if (!imuReady) {
    qRef_w = 1; qRef_x = 0; qRef_y = 0; qRef_z = 0;
    Serial.println("[tilt] IMU offline, ref=identity");
    return;
  }
  // wait briefly for fresh data
  uint32_t t = millis();
  while (millis() - t < 200) {
    if (imu.dataAvailable()) {
      qRef_w = imu.getQuatReal();
      qRef_x = imu.getQuatI();
      qRef_y = imu.getQuatJ();
      qRef_z = imu.getQuatK();
    }
  }
  Serial.println("[tilt] reference captured");
}

inline void captureHoldPosition() {
  if (!imuReady) { Serial.println("[tilt] IMU offline"); return; }
  setTiltReference();
  float x, y, z;
  currentTipXYZ(x, y, z);
  wtX = x; wtY = y; wtZ = z;
  hasWorldTarget = true;
  Serial.print("[tilt] holding ("); Serial.print(x,1); Serial.print(",");
  Serial.print(y,1); Serial.print(","); Serial.print(z,1); Serial.println(")");
}

// Tilt v2 update. Run at TILT_UPDATE_MS rate (100Hz).
//   1. Get current quat + angular velocity
//   2. Predict quat at lookahead time
//   3. Compute relative rotation from reference
//   4. Rotate world target into arm-local frame using inv(rel)
//   5. IK to that local point
//   6. EMA smooth the joint targets
//   7. PD-drive joints toward smoothed targets
inline void updateTiltMode_v2(bool useTiltCompForCatch) {
  // Active in TILT mode, or in CATCH_MODE if user has tiltCompCatch enabled
  bool active = (mode == TILT) || (mode == CATCH_MODE && useTiltCompForCatch);
  if (!active || !hasWorldTarget || !imuReady) return;
  uint32_t now = millis();
  if (now - lastTiltUpdateMs < TILT_UPDATE_MS) return;
  float dt = (now - lastTiltUpdateMs) / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  lastTiltUpdateMs = now;

  // Predict quat at lookahead. q_pred = q * exp(omega * dt_lookahead/2)
  // Small-angle approximation: dq = (1, omega.x*dt/2, omega.y*dt/2, omega.z*dt/2)
  float la = TILT_LOOKAHEAD_MS / 1000.0f;
  float dax = (omegaX * DEG_TO_RAD) * la * 0.5f;
  float day = (omegaY * DEG_TO_RAD) * la * 0.5f;
  float daz = (omegaZ * DEG_TO_RAD) * la * 0.5f;
  float dqW = 1.0f - 0.5f*(dax*dax + day*day + daz*daz);   // tiny correction
  float qpW, qpX, qpY, qpZ;
  qMul(qNow_w, qNow_x, qNow_y, qNow_z,  dqW, dax, day, daz,  qpW, qpX, qpY, qpZ);

  // Relative: q_rel = q_pred * inv(q_ref)
  float qiw, qix, qiy, qiz;
  qInv(qRef_w, qRef_x, qRef_y, qRef_z, qiw, qix, qiy, qiz);
  float qrW, qrX, qrY, qrZ;
  qMul(qpW, qpX, qpY, qpZ, qiw, qix, qiy, qiz, qrW, qrX, qrY, qrZ);

  // Rotate world target by inv(q_rel) to get position in arm-local frame
  float qrInvW, qrInvX, qrInvY, qrInvZ;
  qInv(qrW, qrX, qrY, qrZ, qrInvW, qrInvX, qrInvY, qrInvZ);
  float lx, ly, lz;
  qRotate(qrInvW, qrInvX, qrInvY, qrInvZ, wtX, wtY, wtZ, lx, ly, lz);

  // IK
  float bT, sT, eT;
  IKResult res = solveXYZ(lx, ly, lz, bT, sT, eT);
  if (res != IK_OK) return;

  // EMA-smooth joint targets
  jSmooth[0] = TILT_TARGET_ALPHA * bT + (1 - TILT_TARGET_ALPHA) * jSmooth[0];
  jSmooth[1] = TILT_TARGET_ALPHA * sT + (1 - TILT_TARGET_ALPHA) * jSmooth[1];
  jSmooth[2] = TILT_TARGET_ALPHA * eT + (1 - TILT_TARGET_ALPHA) * jSmooth[2];

  // PD-drive each joint
  static float lastErr[3] = { 0, 0, 0 };
  for (int j = 0; j < 3; j++) {
    float err = jSmooth[j] - curAngle[j];
    if (fabs(err) < TILT_DEADBAND) {
      stopMotor_impl(j);
      lastErr[j] = err;
      continue;
    }
    float dErr = (err - lastErr[j]) / dt;
    lastErr[j] = err;
    float pwmF = TILT_KP * fabs(err);
    if (fabs(err) < TILT_NEAR_DEG) {
      pwmF -= TILT_NEAR_DAMP * fabs(dErr);
      if (pwmF < 0) pwmF = 0;
    }
    int pwm = (int)pwmF;
    if (pwm < LIMIT_SLOW_PWM && pwm > 0) pwm = LIMIT_SLOW_PWM;
    if (pwm > speed) pwm = speed;
    if (pwm == 0) { stopMotor_impl(j); continue; }
    pwm = applySlowZone(j, err > 0, pwm);
    runMotor_impl(j, err > 0, pwm);
  }
}

#endif
