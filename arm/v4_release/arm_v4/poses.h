// poses.h — preset poses + validator.
#ifndef ARM_POSES_H
#define ARM_POSES_H
#include "config.h"
#include "state.h"
#include "kinematics.h"
#include "trajectory.h"
#include "wrist.h"

struct Pose {
  char key;
  bool useXYZ;
  float a, b, c;
  int   roll, pitch;
  const char *name;
};

const Pose POSES[] = {
  { '6', false,  0.0f,  50.0f,  90.0f,  90,  90, "home"        },
  { '7', false,  0.0f, -40.0f,  30.0f,  90,  90, "ready"       },
  { '8', true,  10.0f,   0.0f,  25.0f,  90,  90, "catch ready" },
  { '9', true,  14.0f,   0.0f,  20.0f,  90,  90, "extended"    },
  { '0', false,  0.0f,  60.0f, 105.0f,  90,  90, "retract"     },
};
const int NUM_POSES = sizeof(POSES) / sizeof(POSES[0]);

inline bool moveToXYZ(float x, float y, float z) {
  float b, s, e;
  IKResult r = solveXYZ(x, y, z, b, s, e);
  if (r != IK_OK) {
    Serial.print("[XYZ] ");
    Serial.println(r == IK_TOO_FAR ? "too far" :
                   r == IK_TOO_CLOSE ? "too close" :
                   r == IK_BELOW_FLOOR ? "below floor" : "no valid");
    return false;
  }
  Serial.print("[XYZ] ("); Serial.print(x,1); Serial.print(",");
  Serial.print(y,1); Serial.print(","); Serial.print(z,1);
  Serial.print(") -> B:"); Serial.print(b,1);
  Serial.print(" S:"); Serial.print(s,1);
  Serial.print(" E:"); Serial.println(e,1);
  startSyncedMove(b, s, e, true, true, true);
  return true;
}

inline bool moveToAngles(float b, float s, float e) {
  float x, y, z;
  anglesToXYZ(b, s, e, x, y, z);
  if (z < Z_FLOOR) {
    Serial.print("[pose] tip Z="); Serial.print(z, 1); Serial.println(" below floor");
    return false;
  }
  startSyncedMove(b, s, e, true, true, true);
  return true;
}

inline void runPose(const Pose &p) {
  Serial.print("[pose] "); Serial.println(p.name);
  setRoll(p.roll); setPitch(p.pitch);
  if (p.useXYZ) moveToXYZ(p.a, p.b, p.c);
  else          moveToAngles(p.a, p.b, p.c);
}

// Auto-switch into XYZ if pose uses XYZ. v4 buttons-everywhere behavior:
// running a pose always works; mode adapts.
inline bool tryPoseKey(char c) {
  for (int i = 0; i < NUM_POSES; i++) {
    if (POSES[i].key == c) {
      if (POSES[i].useXYZ && mode != XYZ_MODE && mode != CATCH_MODE) {
        Serial.println("[pose] auto-switch → XYZ");
        mode = XYZ_MODE;
      }
      runPose(POSES[i]);
      return true;
    }
  }
  return false;
}

inline void validatePoses() {
  Serial.println("[poses] validating...");
  for (int i = 0; i < NUM_POSES; i++) {
    const Pose &p = POSES[i];
    float b, s, e;
    if (p.useXYZ) {
      IKResult r = solveXYZ(p.a, p.b, p.c, b, s, e);
      if (r != IK_OK) {
        Serial.print("  [WARN] '"); Serial.print(p.name);
        Serial.println("' IK fail");
        continue;
      }
    } else { b = p.a; s = p.b; e = p.c; }
    const float v[3] = { b, s, e };
    const char *jn[] = { "base", "shoulder", "elbow" };
    for (int j = 0; j < 3; j++) {
      if (v[j] > JOINT_MAX[j] || v[j] < JOINT_MIN[j]) {
        Serial.print("  [WARN] '"); Serial.print(p.name); Serial.print("' ");
        Serial.print(jn[j]); Serial.print("="); Serial.print(v[j], 1);
        Serial.println(" outside limits");
      }
    }
  }
}

#endif
