// safety.h — floor guard, magnet control, stopAll with magLocked support.
#ifndef ARM_SAFETY_H
#define ARM_SAFETY_H
#include "config.h"
#include "state.h"
#include "sensors.h"
#include "motors.h"

// Magnet control. Use _silent variants from inside FSM / safety paths to
// avoid log spam during catch sequences.
inline void magnetOn_silent_impl()  { digitalWrite(MAG_PIN, HIGH); magOn = true; }
inline void magnetOff_silent_impl() { digitalWrite(MAG_PIN, LOW);  magOn = false; }
inline void magnetOn_impl()  { magnetOn_silent_impl();  Serial.println("[mag] ON"); }
inline void magnetOff_impl() { magnetOff_silent_impl(); Serial.println("[mag] OFF"); }

// Stop all motors. If magLocked is true (during CATCH states), magnet stays.
inline void stopAll_impl() {
  for (int j = 0; j < 3; j++) {
    stopMotor_impl(j);
    jMoving[j] = false;
    jReversed[j] = false;
    jWrongChecked[j] = false;
  }
  danceRunning = false;
  hasWorldTarget = false;
  if (!magLocked) magnetOff_silent_impl();
}

// Floor guard: wrist joint must stay above Z_FLOOR (catch FSM allows brief
// dips during COMMITTING).
inline void floorGuard(bool allowFloor) {
  if (allowFloor) { floorTripped = false; return; }
  float x, y, z;
  anglesToXYZ(curAngle[0], curAngle[1], curAngle[2], x, y, z);
  if (z < Z_FLOOR - Z_FLOOR_MARGIN) {
    if (!floorTripped) {
      Serial.print("[FLOOR] wrist Z="); Serial.print(z, 1); Serial.println(" e-stop");
      stopAll_impl();
      floorTripped = true;
    }
  } else floorTripped = false;
}

#endif
