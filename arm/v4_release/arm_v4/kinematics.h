// kinematics.h — IK solvers (XYZ, height) with cost function that prefers
// continuity, avoids forbidden zone, and penalizes elbow flips.
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H
#include "config.h"
#include "state.h"
#include "sensors.h"

inline bool solve2Link(float r, float z, bool elbowPositive,
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

// Cost: hard-limit penalties + forbid zone + elbow-flip + continuity +
// trajectory cost (passing through forbid zone on the way to target).
inline float ikCost(float s, float e) {
  float c = 0;
  if (s > JOINT_MAX[1]) c += IK_COST_LIMIT_VIO * (s - JOINT_MAX[1]);
  if (s < JOINT_MIN[1]) c += IK_COST_LIMIT_VIO * (JOINT_MIN[1] - s);
  if (e > JOINT_MAX[2]) c += IK_COST_LIMIT_VIO * (e - JOINT_MAX[2]);
  if (e < JOINT_MIN[2]) c += IK_COST_LIMIT_VIO * (JOINT_MIN[2] - e);

  // Final pose lands in forbid zone = bad
  if (s > SHLD_FORBID_LO && s < SHLD_FORBID_HI) c += IK_COST_FORBID_ZONE;

  // Elbow flip vs current pose
  bool curPos = (curAngle[2] >= 0);
  bool newPos = (e >= 0);
  if (curPos != newPos) c += IK_COST_ELBOW_FLIP;

  // Continuity preference
  c += IK_COST_CONTINUITY * fabs(s - curAngle[1]);
  c += IK_COST_CONTINUITY * fabs(e - curAngle[2]);

  // Trajectory: if path from current shoulder to candidate crosses forbid
  // zone, add penalty proportional to how much of the path is in the zone.
  // (We don't avoid crossing if necessary — gravity slip is acceptable —
  //  but we prefer paths that don't dwell.)
  float sLo = min(curAngle[1], s);
  float sHi = max(curAngle[1], s);
  if (sLo < SHLD_FORBID_HI && sHi > SHLD_FORBID_LO) {
    float overlap = min(sHi, SHLD_FORBID_HI) - max(sLo, SHLD_FORBID_LO);
    if (overlap > 0) c += IK_COST_FORBID_TRAJ * (overlap / (sHi - sLo + 0.001f));
  }
  return c;
}

inline IKResult solveXYZ(float x, float y, float z,
                         float &baseDeg, float &shldDeg, float &elbwDeg) {
  if (z < Z_FLOOR) return IK_BELOW_FLOOR;

  float wristZ = z;
  if (magOffsetActive()) wristZ -= MAG_TIP_OFFSET;
  if (wristZ < Z_FLOOR) return IK_BELOW_FLOOR;

  float zShld = wristZ - SHOULDER_OFFSET_Z;
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

  if (shldDeg > JOINT_MAX[1] + 0.5f || shldDeg < JOINT_MIN[1] - 0.5f) return IK_NO_VALID;
  if (elbwDeg > JOINT_MAX[2] + 0.5f || elbwDeg < JOINT_MIN[2] - 0.5f) return IK_NO_VALID;
  return IK_OK;
}

inline bool solveHeight(float targetZ, float &shldDeg, float &elbwDeg) {
  float wristZ = targetZ;
  if (magOffsetActive()) wristZ -= MAG_TIP_OFFSET;
  if (wristZ < Z_FLOOR || wristZ > REACH_MAX + SHOULDER_OFFSET_Z) return false;
  float zShld = wristZ - SHOULDER_OFFSET_Z;
  float cosE = (zShld * zShld - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosE > 1.0f || cosE < -1.0f) return false;

  float best = 1e9f, bestS = 0, bestE = 0;
  bool found = false;
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
    if (sDeg > JOINT_MAX[1] + 0.5f || sDeg < JOINT_MIN[1] - 0.5f) continue;
    if (eDeg > JOINT_MAX[2] + 0.5f || eDeg < JOINT_MIN[2] - 0.5f) continue;
    float c = ikCost(sDeg, eDeg);
    if (c < best) { best = c; bestS = sDeg; bestE = eDeg; found = true; }
  }
  if (!found) return false;
  shldDeg = bestS;
  elbwDeg = bestE;
  return true;
}

#endif
