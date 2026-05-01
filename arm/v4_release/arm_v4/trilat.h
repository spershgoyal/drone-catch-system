// trilat.h — 3-anchor trilateration with mirror disambiguation, plus
// light tip-anchor fusion and integrity checking. Output: world position
// of drone tag, residual, and integrity discrepancy.
#ifndef ARM_TRILAT_H
#define ARM_TRILAT_H
#include "config.h"
#include "state.h"
#include "uwb.h"
#include "sensors.h"

inline float vlen(float x, float y, float z) { return sqrt(x*x + y*y + z*z); }

// Solve 3-anchor trilat. Returns 2 candidate solutions (mirror across
// the plane through the 3 anchors). Caller picks the "above baseplate" one.
// Anchors[3][3] = positions in baseplate frame. ranges[3] = measured.
// Returns true on success.
inline bool trilat3(const float A[3], const float B[3], const float C[3],
                    float rA, float rB, float rC,
                    float P1[3], float P2[3]) {
  float ex[3] = { B[0]-A[0], B[1]-A[1], B[2]-A[2] };
  float d = vlen(ex[0], ex[1], ex[2]);
  if (d < 1e-3f) return false;
  for (int k = 0; k < 3; k++) ex[k] /= d;

  float ac[3] = { C[0]-A[0], C[1]-A[1], C[2]-A[2] };
  float i = ex[0]*ac[0] + ex[1]*ac[1] + ex[2]*ac[2];

  float ey[3] = { ac[0]-i*ex[0], ac[1]-i*ex[1], ac[2]-i*ex[2] };
  float eyl = vlen(ey[0], ey[1], ey[2]);
  if (eyl < 1e-3f) return false;
  for (int k = 0; k < 3; k++) ey[k] /= eyl;

  float ez[3] = {
    ex[1]*ey[2] - ex[2]*ey[1],
    ex[2]*ey[0] - ex[0]*ey[2],
    ex[0]*ey[1] - ex[1]*ey[0]
  };
  float j = ey[0]*ac[0] + ey[1]*ac[1] + ey[2]*ac[2];
  float xs = (rA*rA - rB*rB + d*d) / (2.0f*d);
  float ys = (rA*rA - rC*rC + i*i + j*j) / (2.0f*j) - (i/j)*xs;
  float z2 = rA*rA - xs*xs - ys*ys;
  if (z2 < 0) z2 = 0;
  float zs = sqrt(z2);
  for (int k = 0; k < 3; k++) {
    P1[k] = A[k] + xs*ex[k] + ys*ey[k] + zs*ez[k];
    P2[k] = A[k] + xs*ex[k] + ys*ey[k] - zs*ez[k];
  }
  return true;
}

// Compute trilateration residual: sum of (predicted - measured)² for each anchor.
inline float trilatResidual(const float P[3], const float A[ANCHOR_COUNT][3],
                            const float ranges[ANCHOR_COUNT]) {
  float sum = 0;
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    float dx = P[0] - A[i][0], dy = P[1] - A[i][1], dz = P[2] - A[i][2];
    float predicted = vlen(dx, dy, dz);
    float r = ranges[i] - predicted;
    sum += r * r;
  }
  return sum;
}

// Main solve. Reads ranges from UWB anchors, returns drone position +
// residual + tip discrepancy. Returns true if valid solution found.
//
// tipDiscrepancy = |measured tip range  -  predicted tip range from FK|
inline bool trilatSolveDrone(float &outX, float &outY, float &outZ,
                             float &outResidual, float &outTipDiscrepancy) {
  // Need all 3 base anchors alive
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    if (!uwb_anchorAlive(i)) return false;
  }
  float ranges[ANCHOR_COUNT];
  for (int i = 0; i < ANCHOR_COUNT; i++) ranges[i] = uwbAnchors[i].lastRange;

  float P1[3], P2[3];
  if (!trilat3(ANCHOR_POS[0], ANCHOR_POS[1], ANCHOR_POS[2],
               ranges[0], ranges[1], ranges[2], P1, P2)) return false;

  // Pick "above baseplate" solution
  float P[3];
  if (P1[2] > P2[2]) { P[0]=P1[0]; P[1]=P1[1]; P[2]=P1[2]; }
  else               { P[0]=P2[0]; P[1]=P2[1]; P[2]=P2[2]; }

  // Sanity: drone should be in some kind of reasonable height range
  if (P[2] < -10 || P[2] > 80) return false;

  // EMA smoothing on output
  static bool first = true;
  static float emaX = 0, emaY = 0, emaZ = 0;
  const float ALPHA = 0.4f;
  if (first) { emaX = P[0]; emaY = P[1]; emaZ = P[2]; first = false; }
  else {
    emaX = ALPHA * P[0] + (1 - ALPHA) * emaX;
    emaY = ALPHA * P[1] + (1 - ALPHA) * emaY;
    emaZ = ALPHA * P[2] + (1 - ALPHA) * emaZ;
  }

  // Tip anchor fusion: light correction toward tip's implied position
  // Only apply if tip anchor alive AND tip range looks reasonable.
  float tipX, tipY, tipZ;
  tipAnchorXYZ(tipX, tipY, tipZ);
  float tipDx = emaX - tipX, tipDy = emaY - tipY, tipDz = emaZ - tipZ;
  float predictedTipRange = vlen(tipDx, tipDy, tipDz);

  if (uwb_anchorAlive(3)) {
    float measuredTipRange = uwbAnchors[3].lastRange;
    outTipDiscrepancy = fabs(measuredTipRange - predictedTipRange);
    // Light fusion: pull drone position toward (tip + (drone-tip-vec scaled to measured range))
    if (predictedTipRange > 0.1f && outTipDiscrepancy < 8.0f) {
      float scale = measuredTipRange / predictedTipRange;
      float impliedX = tipX + tipDx * scale;
      float impliedY = tipY + tipDy * scale;
      float impliedZ = tipZ + tipDz * scale;
      const float TIP_WEIGHT = 0.2f;
      emaX = (1 - TIP_WEIGHT) * emaX + TIP_WEIGHT * impliedX;
      emaY = (1 - TIP_WEIGHT) * emaY + TIP_WEIGHT * impliedY;
      emaZ = (1 - TIP_WEIGHT) * emaZ + TIP_WEIGHT * impliedZ;
    }
  } else {
    outTipDiscrepancy = 0;
  }

  outX = emaX; outY = emaY; outZ = emaZ;

  float Pemafilt[3] = { emaX, emaY, emaZ };
  outResidual = trilatResidual(Pemafilt, ANCHOR_POS, ranges);

  return true;
}

#endif
