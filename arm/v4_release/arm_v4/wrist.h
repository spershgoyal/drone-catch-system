// wrist.h — wrist roll/pitch servo control + auto-level.
#ifndef ARM_WRIST_H
#define ARM_WRIST_H
#include "config.h"
#include "state.h"
#include <PWMServo.h>

extern PWMServo sRoll, sPitch;
extern int rollPos, pitchPos;

inline float levelPitch() {
  return PITCH_LEVEL_DEFAULT + PITCH_LEVEL_OFFSET + (magAttached ? MAG_PITCH_BIAS : 0.0f);
}
inline float levelRoll() {
  return ROLL_LEVEL_DEFAULT + ROLL_LEVEL_OFFSET;
}

inline void setRoll(int pos)  { rollPos  = constrain(pos, 0, 180); sRoll.write(rollPos); }
inline void setPitch(int pos) { pitchPos = constrain(pos, 0, 180); sPitch.write(pitchPos); }

inline void wristLevel() {
  setRoll((int)roundf(levelRoll()));
  setPitch((int)roundf(levelPitch()));
}

// Run wrist auto-level so magnet face stays horizontal/up regardless of
// shoulder+elbow pose. Active in modes that need a known magnet face direction.
inline void updateWristAutoLevel() {
  static uint32_t lastMs = 0;
  bool active = (mode == XYZ_MODE || mode == HEIGHT || mode == CATCH_MODE);
  if (!active) return;
  uint32_t now = millis();
  if (now - lastMs < 80) return;
  lastMs = now;
  float forearm_deg = curAngle[1] - curAngle[2];
  int target_pitch = (int)roundf(levelPitch() - forearm_deg);
  int target_roll  = (int)roundf(levelRoll());
  if (abs(target_pitch - pitchPos) >= 1) setPitch(target_pitch);
  if (abs(target_roll  - rollPos)  >= 1) setRoll(target_roll);
}

#endif
