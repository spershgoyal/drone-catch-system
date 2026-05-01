// dance.h — preset dance sequences.
#ifndef ARM_DANCE_H
#define ARM_DANCE_H
#include "config.h"
#include "state.h"
#include "trajectory.h"
#include "poses.h"
#include "wrist.h"

struct DanceStep {
  float x, y, z;
  int   roll, pitch;
  unsigned long holdMs;
  const char *name;
};
struct Dance {
  const char *name;
  const DanceStep *steps;
  int nSteps;
};

const DanceStep SHOWCASE_STEPS[] = {
  {  6.0f, 0.0f, 24.0f,  90, 120,   500,  "wake"      },
  {  8.0f, 0.0f, 26.0f,  90,  90,   300,  "point fwd" },
  {  7.1f, 7.1f, 22.0f, 120, 110,   250,  "CW1"       },
  {  0.0f,10.0f, 26.0f, 150, 120,   250,  "CW2"       },
  { -7.1f, 7.1f, 22.0f, 170, 110,   250,  "CW3"       },
  {-10.0f, 0.0f, 26.0f, 170, 130,   250,  "CW4"       },
  { -7.1f,-7.1f, 22.0f, 140, 110,   250,  "CW5"       },
  {  0.0f,-10.0f,26.0f,  90, 120,   250,  "CW6"       },
  {  7.1f,-7.1f, 22.0f,  40, 110,   250,  "CW7"       },
  { 10.0f, 0.0f, 26.0f,  90, 130,   400,  "front"     },
  { 10.0f, 0.0f, 24.0f,   0, 120,   300,  "roll L"    },
  { 10.0f, 0.0f, 24.0f, 180, 120,   300,  "roll R"    },
  { 10.0f, 0.0f, 24.0f,  90, 165,   300,  "nod down"  },
  { 10.0f, 0.0f, 24.0f,  90,  75,   300,  "nod up"    },
  { 20.0f, 0.0f,  8.0f,  90, 175,   600,  "bow"       },
  {  5.0f, 0.0f, 28.0f,  90,  60,   500,  "rise"      },
  {  6.0f, 0.0f, 22.0f,  90, 120,   400,  "home"      },
};
const DanceStep GREET_STEPS[] = {
  {  8.0f, 0.0f, 26.0f,  90, 120,   400,  "rise"     },
  { 10.0f, 0.0f, 22.0f,  90, 100,   200,  "lean d"   },
  { 10.0f, 0.0f, 26.0f,  90, 130,   200,  "lean u"   },
  { 10.0f, 0.0f, 22.0f,  90, 100,   200,  "lean d2"  },
  { 10.0f, 0.0f, 26.0f,  90, 130,   200,  "lean u2"  },
  {  9.0f, 0.0f, 24.0f,  60, 120,   300,  "tilt R"   },
  {  9.0f, 0.0f, 24.0f, 120, 120,   300,  "tilt L"   },
  { 14.0f, 0.0f, 16.0f,  90, 165,   600,  "bow"      },
  {  6.0f, 0.0f, 24.0f,  90, 120,   400,  "home"     },
};
const DanceStep IDLE_STEPS[] = {
  {  4.0f, 0.0f, 25.0f,  90, 120,  1500,  "rest"     },
  {  4.0f, 0.0f, 25.5f,  92, 122,  1200,  "in"       },
  {  4.0f, 0.0f, 24.5f,  88, 118,  1200,  "out"      },
  {  5.0f, 1.0f, 25.0f,  85, 120,   800,  "look L"   },
  {  4.0f, 0.0f, 25.0f,  90, 120,  1000,  "center"   },
  {  5.0f,-1.0f, 25.0f,  95, 120,   800,  "look R"   },
  {  4.0f, 0.0f, 25.5f,  90, 125,  1200,  "stretch"  },
  {  4.0f, 0.0f, 25.0f,  90, 120,  1500,  "settle"   },
};
const DanceStep COMBAT_STEPS[] = {
  {  0.0f, 0.0f, 28.0f,  90, 120,   200,  "ready"    },
  { 12.0f, 0.0f, 18.0f,  90, 100,   100,  "strike F" },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"  },
  {  0.0f,12.0f, 18.0f,  90, 100,   100,  "strike L" },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"  },
  {-12.0f, 0.0f, 18.0f,  90, 100,   100,  "strike B" },
  {  0.0f, 0.0f, 28.0f,  90, 120,   100,  "retract"  },
  {  0.0f,-12.0f,18.0f,  90, 100,   100,  "strike R" },
  {  0.0f, 0.0f, 28.0f,  90, 120,   200,  "guard"    },
  {  0.0f, 0.0f, 30.0f,  90, 120,   400,  "clear"    },
};
const DanceStep CHASE_STEPS[] = {
  {  0.0f, 0.0f, 26.0f,  90, 120,   300,  "scan"     },
  {  9.0f, 5.0f, 24.0f,  60, 110,   180,  "track1"   },
  { 10.0f, 0.0f, 23.0f,  90, 105,   180,  "track2"   },
  {  9.0f,-5.0f, 24.0f, 120, 110,   180,  "track3"   },
  {  5.0f,-9.0f, 25.0f, 150, 115,   180,  "track4"   },
  { -5.0f,-9.0f, 25.0f, 150, 115,   180,  "track5"   },
  { -9.0f,-5.0f, 24.0f, 120, 110,   180,  "track6"   },
  { 11.0f, 0.0f, 18.0f,  90,  90,   200,  "GRAB"     },
  {  6.0f, 0.0f, 26.0f,  90, 120,   400,  "secure"   },
};

const Dance DANCES[] = {
  { "showcase", SHOWCASE_STEPS, sizeof(SHOWCASE_STEPS) / sizeof(SHOWCASE_STEPS[0]) },
  { "greet",    GREET_STEPS,    sizeof(GREET_STEPS)    / sizeof(GREET_STEPS[0])    },
  { "idle",     IDLE_STEPS,     sizeof(IDLE_STEPS)     / sizeof(IDLE_STEPS[0])     },
  { "combat",   COMBAT_STEPS,   sizeof(COMBAT_STEPS)   / sizeof(COMBAT_STEPS[0])   },
  { "chase",    CHASE_STEPS,    sizeof(CHASE_STEPS)    / sizeof(CHASE_STEPS[0])    },
};
const int N_DANCES = sizeof(DANCES) / sizeof(DANCES[0]);

inline void advanceDance(int idx) {
  const Dance &d = DANCES[activeDance];
  if (idx >= d.nSteps) return;
  const DanceStep &s = d.steps[idx];
  moveToXYZ(s.x, s.y, s.z);
  setRoll(s.roll); setPitch(s.pitch);
  danceStepT = millis();
  Serial.print("[dance "); Serial.print(d.name); Serial.print(" "); Serial.print(idx);
  Serial.print("] "); Serial.println(s.name);
}

inline void startDance() {
  // v4: dance auto-switches mode if needed
  if (mode != DANCE_MODE) {
    Serial.println("[dance] auto-switch → DANCE");
    mode = DANCE_MODE;
  }
  danceRunning = true;
  danceStep = 0;
  Serial.print("[dance] starting "); Serial.println(DANCES[activeDance].name);
  advanceDance(0);
}

inline void selectDance(int idx) {
  if (idx < 0 || idx >= N_DANCES) return;
  activeDance = idx;
  Serial.print("[dance] selected: "); Serial.println(DANCES[idx].name);
}

inline void updateDance() {
  if (!danceRunning) return;
  const Dance &d = DANCES[activeDance];
  unsigned long elapsed = millis() - danceStepT;
  if (anyJointMoving()) return;
  if (elapsed < d.steps[danceStep].holdMs) return;
  danceStep++;
  if (danceStep >= d.nSteps) {
    if (danceLoop) {
      danceStep = 0;
      Serial.println("[dance] -- loop --");
    } else {
      danceRunning = false;
      wristLevel();
      Serial.println("[dance] done");
      return;
    }
  }
  advanceDance(danceStep);
}

#endif
