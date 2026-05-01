// telemetry.h — extended $T format for v4.
#ifndef ARM_TELEMETRY_H
#define ARM_TELEMETRY_H
#include "config.h"
#include "state.h"
#include "sensors.h"
#include "uwb.h"

inline void emitTelemetry_impl() {
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
  Serial.print(qNow_w,3); Serial.print(",");
  Serial.print(qNow_x,3); Serial.print(",");
  Serial.print(qNow_y,3); Serial.print(",");
  Serial.print(qNow_z,3); Serial.print(",");
  Serial.print(JOINT_OFFSET[0], 2); Serial.print(",");
  Serial.print(JOINT_OFFSET[1], 2); Serial.print(",");
  Serial.print(JOINT_OFFSET[2], 2); Serial.print(",");
  Serial.print(magOn ? 1 : 0); Serial.print(",");
  Serial.print(magAttached ? 1 : 0); Serial.print(",");
  // v4 fields
  Serial.print((int)catchState); Serial.print(",");
  Serial.print(dronePosX, 2); Serial.print(",");
  Serial.print(dronePosY, 2); Serial.print(",");
  Serial.print(dronePosZ, 2); Serial.print(",");
  Serial.print(droneValid ? 1 : 0); Serial.print(",");
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    Serial.print(uwbAnchors[i].lastRange, 2); Serial.print(",");
  }
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    Serial.print(uwbAnchors[i].quality, 2); Serial.print(",");
  }
  // Stall flags as bitmap
  uint8_t stallBits = (stalled[0]?1:0) | (stalled[1]?2:0) | (stalled[2]?4:0);
  Serial.print(stallBits); Serial.print(",");
  Serial.print(magLocked ? 1 : 0); Serial.print(",");
  Serial.print(simDroneOn ? 1 : 0); Serial.print(",");
  Serial.print(catchArmed ? 1 : 0);
  Serial.println();
}

#endif
