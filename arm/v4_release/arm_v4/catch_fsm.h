// catch_fsm.h — autonomous catch state machine.
// IDLE → SEARCHING → TRACKING → STAGING → COMMITTING → CATCHING → SECURED → STOWING → IDLE
#ifndef ARM_CATCH_FSM_H
#define ARM_CATCH_FSM_H
#include "config.h"
#include "state.h"
#include "trilat.h"
#include "kinematics.h"
#include "trajectory.h"
#include "safety.h"
#include "wrist.h"

inline const char* catchStateName(CatchState s) {
  switch (s) {
    case CATCH_IDLE:       return "IDLE";
    case CATCH_SEARCHING:  return "SEARCHING";
    case CATCH_TRACKING:   return "TRACKING";
    case CATCH_STAGING:    return "STAGING";
    case CATCH_COMMITTING: return "COMMITTING";
    case CATCH_CATCHING:   return "CATCHING";
    case CATCH_SECURED:    return "SECURED";
    case CATCH_STOWING:    return "STOWING";
    case CATCH_ABORT:      return "ABORT";
    case CATCH_ERROR:      return "ERROR";
  }
  return "?";
}

inline void catchSetState(CatchState ns) {
  if (catchState == ns) return;
  Serial.print("[catch] "); Serial.print(catchStateName(catchState));
  Serial.print(" -> "); Serial.println(catchStateName(ns));
  catchState = ns;
  catchStateMs = millis();
}

inline void catchArm() {
  catchArmed = true;
  catchArmedMs = millis();
  if (mode != CATCH_MODE) { mode = CATCH_MODE; Serial.println("[catch] mode → CATCH"); }
  catchSetState(CATCH_SEARCHING);
}

inline void catchDisarm() {
  catchArmed = false;
  if (catchState != CATCH_IDLE && catchState != CATCH_STOWING) {
    catchSetState(CATCH_ABORT);
  }
}

// Drone position stability tracking (variance over recent samples)
inline void catchUpdateLockTracking() {
  static float buf[3][16] = {{0}};
  static int idx = 0;
  static int count = 0;
  if (!droneValid) return;
  buf[0][idx] = dronePosX;
  buf[1][idx] = dronePosY;
  buf[2][idx] = dronePosZ;
  idx = (idx + 1) % 16;
  if (count < 16) count++;
  if (count < 8) { droneLastVarSq = 999; return; }
  float mean[3] = {0};
  for (int k = 0; k < 3; k++) {
    for (int i = 0; i < count; i++) mean[k] += buf[k][i];
    mean[k] /= count;
  }
  float varSq = 0;
  for (int i = 0; i < count; i++) {
    float dx = buf[0][i] - mean[0];
    float dy = buf[1][i] - mean[1];
    float dz = buf[2][i] - mean[2];
    varSq += dx*dx + dy*dy + dz*dz;
  }
  droneLastVarSq = varSq / count;
}

// Compute distance from current magnet face to drone
inline float magToDroneDist() {
  if (!droneValid) return 999.0f;
  float mx, my, mz;
  currentTipXYZ(mx, my, mz);
  float dx = dronePosX - mx, dy = dronePosY - my, dz = dronePosZ - mz;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

// Drive arm toward (tx, ty, tz) at given peak PWM, using IK + synced move.
// Returns true if IK succeeded.
inline bool catchDriveTowards(float tx, float ty, float tz, int peakPWM) {
  float bT, sT, eT;
  IKResult r = solveXYZ(tx, ty, tz, bT, sT, eT);
  if (r != IK_OK) return false;
  // Use synced move with overridden peak PWM
  int prevSpeed = speed;
  speed = peakPWM;
  startSyncedMove(bT, sT, eT, true, true, true);
  speed = prevSpeed;
  return true;
}

// Tip integrity: compare measured tip range to FK-predicted.
inline void updateTipIntegrity(float tipDiscrepancy) {
  uint32_t now = millis();
  if (tipDiscrepancy < CATCH_INTEGRITY_THRESH) {
    integrityFailMs = now;
  }
}

inline bool integrityFailing() {
  return (millis() - integrityFailMs) > CATCH_INTEGRITY_MS;
}

// Main catch FSM update. Called from main loop when mode==CATCH_MODE.
inline void updateCatchFsm() {
  uint32_t now = millis();
  uint32_t inState = now - catchStateMs;

  if (droneValid) droneLastSeenMs = now;
  catchUpdateLockTracking();

  switch (catchState) {
    case CATCH_IDLE: {
      // wait for arm
      if (catchArmed) catchSetState(CATCH_SEARCHING);
      break;
    }

    case CATCH_SEARCHING: {
      if (!catchArmed) { catchSetState(CATCH_IDLE); break; }
      if (droneValid && (now - droneLastSeenMs < 200)) {
        catchSetState(CATCH_TRACKING);
        break;
      }
      if (inState > CATCH_AUTO_DISARM_MS) {
        Serial.println("[catch] auto-disarm (no drone)");
        catchArmed = false;
        catchSetState(CATCH_IDLE);
      }
      break;
    }

    case CATCH_TRACKING: {
      if (!catchArmed) { catchSetState(CATCH_ABORT); break; }
      if (now - droneLastSeenMs > CATCH_LOST_TIMEOUT_MS) {
        Serial.println("[catch] drone lost");
        catchSetState(CATCH_SEARCHING); break;
      }
      // Position arm under drone, with TRACK_HOLDOFF below
      if (droneValid) {
        float tz = dronePosZ - CATCH_TRACK_HOLDOFF_Z;
        if (tz < Z_FLOOR) tz = Z_FLOOR + 1.0f;
        catchDriveTowards(dronePosX, dronePosY, tz, CATCH_PWM_TRACK);
      }
      // Transition to STAGING when drone position is stable
      if (droneLastVarSq < CATCH_LOCK_VARIANCE * CATCH_LOCK_VARIANCE) {
        if (inState > 800) catchSetState(CATCH_STAGING);
      }
      break;
    }

    case CATCH_STAGING: {
      if (!catchArmed) { catchSetState(CATCH_ABORT); break; }
      if (now - droneLastSeenMs > CATCH_LOST_TIMEOUT_MS) {
        catchSetState(CATCH_TRACKING); break;
      }
      if (inState > CATCH_STAGE_TIMEOUT_MS) {
        Serial.println("[catch] stage timeout");
        catchSetState(CATCH_ABORT); break;
      }
      // Drive to STAGE holdoff (4" below)
      if (droneValid) {
        float tz = dronePosZ - CATCH_STAGE_HOLDOFF_Z;
        if (tz < Z_FLOOR) tz = Z_FLOOR + 1.0f;
        catchDriveTowards(dronePosX, dronePosY, tz, CATCH_PWM_STAGE);
      }
      // If drone moved a lot, go back to TRACKING
      if (droneLastVarSq > 36.0f) {  // 6" std-dev
        catchSetState(CATCH_TRACKING); break;
      }
      // Transition to COMMITTING when staged + arm settled
      float dist = magToDroneDist();
      if (dist < CATCH_STAGE_HOLDOFF_Z + 1.0f && !anyJointMoving()) {
        catchSetState(CATCH_COMMITTING);
      }
      break;
    }

    case CATCH_COMMITTING: {
      if (!catchArmed) { catchSetState(CATCH_ABORT); break; }
      if (inState > CATCH_COMMIT_TIMEOUT_MS) {
        Serial.println("[catch] commit timeout");
        catchSetState(CATCH_ABORT); break;
      }
      if (integrityFailing()) {
        Serial.println("[catch] integrity FAIL — abort");
        catchSetState(CATCH_ABORT); break;
      }
      // Final approach: drive magnet UP to drone position
      if (droneValid) {
        catchDriveTowards(dronePosX, dronePosY, dronePosZ, CATCH_PWM_COMMIT);
      }
      // Fire when tip range very small
      float tipMeasured = uwbAnchors[3].lastRange;
      if (uwb_anchorAlive(3) && tipMeasured < CATCH_COMMIT_TIP_DIST + 4.0f) {
        // tip's predicted range (from FK) gives us a sanity check
        float tipX, tipY, tipZ;
        tipAnchorXYZ(tipX, tipY, tipZ);
        float dx = dronePosX - tipX, dy = dronePosY - tipY, dz = dronePosZ - tipZ;
        float tipPred = sqrt(dx*dx + dy*dy + dz*dz);
        // Use measured (more direct) for fire trigger
        if (tipMeasured < CATCH_COMMIT_TIP_DIST + 1.0f) {
          // Magnet on, lock it
          magLocked = true;
          magnetOn_silent_impl();
          Serial.println("[catch] MAGNET FIRE");
          catchSetState(CATCH_CATCHING);
        }
      }
      // Fallback: if we can't read tip, use FK distance
      else if (!uwb_anchorAlive(3)) {
        if (magToDroneDist() < CATCH_COMMIT_TIP_DIST + 0.5f) {
          magLocked = true;
          magnetOn_silent_impl();
          Serial.println("[catch] MAGNET FIRE (no tip uwb, FK fallback)");
          catchSetState(CATCH_CATCHING);
        }
      }
      break;
    }

    case CATCH_CATCHING: {
      // Hold pose for CATCH_HOLD_AFTER_MAG_MS
      if (droneValid) {
        catchDriveTowards(dronePosX, dronePosY, dronePosZ, CATCH_PWM_SECURED);
      }
      if (inState > CATCH_HOLD_AFTER_MAG_MS) catchSetState(CATCH_SECURED);
      break;
    }

    case CATCH_SECURED: {
      if (inState > CATCH_SETTLE_MS) catchSetState(CATCH_STOWING);
      break;
    }

    case CATCH_STOWING: {
      // Move to home pose (use first pose entry "home")
      static bool stowStarted = false;
      if (!stowStarted) {
        Serial.println("[catch] stowing");
        startSyncedMove(0, 50, 90, true, true, true);
        stowStarted = true;
      }
      if (!anyJointMoving() && inState > 500) {
        magLocked = false;
        magnetOff_silent_impl();
        Serial.println("[catch] magnet released");
        catchArmed = false;
        catchSetState(CATCH_IDLE);
        stowStarted = false;
      }
      break;
    }

    case CATCH_ABORT: {
      // Safe retract: stop motion, go to ready pose. Magnet stays in current state
      // (don't drop a caught drone on abort — user can call mag off explicitly).
      static bool abortStarted = false;
      if (!abortStarted) {
        Serial.println("[catch] aborting");
        for (int j = 0; j < 3; j++) { stopMotor_impl(j); jMoving[j] = false; }
        startSyncedMove(0, 50, 90, true, true, true);  // back to home
        abortStarted = true;
      }
      if (!anyJointMoving() && inState > 500) {
        catchSetState(CATCH_IDLE);
        catchArmed = false;
        abortStarted = false;
      }
      break;
    }

    case CATCH_ERROR: {
      // Wait for user reset
      break;
    }
  }
}

inline void catchReset() {
  catchArmed = false;
  catchState = CATCH_IDLE;
  magLocked = false;
  Serial.println("[catch] reset");
}

#endif
