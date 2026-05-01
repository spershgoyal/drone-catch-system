// state.h — shared global state. Definitions live in arm_v4.ino.
#ifndef ARM_STATE_H
#define ARM_STATE_H
#include "config.h"

// ---- sensor / pose state ----
extern int   curRaw[3];
extern float curAngle[3];
extern int   POT_AT_0[3];
extern float COUNTS_PER_DEG[3];

// ---- calibration (RAM, EEPROM-persisted) ----
extern float JOINT_OFFSET[3];
extern float ROLL_LEVEL_OFFSET;
extern float PITCH_LEVEL_OFFSET;
extern float MAG_PITCH_BIAS;
extern bool  magAttached;

// ---- anchor positions (RAM, EEPROM-persisted) ----
extern float ANCHOR_POS[ANCHOR_COUNT][3];
extern float TIP_ANCHOR_LOCAL[3];

// ---- mode + speed ----
extern Mode mode;
extern int  speed;
extern unsigned long pulseMs;
extern float stepIn;
extern bool  danceLoop;

// ---- joint move state ----
extern bool  jMoving[3];
extern float jTarget[3];
extern float jStart[3];
extern float jTotalDist[3];
extern int   jPeakPWM[3];
extern unsigned long jMoveStartMs[3];
extern bool  jReversed[3];
extern bool  jWrongChecked[3];
extern bool  pulseActive[3];
extern unsigned long pulseStopMs[3];

// ---- magnet ----
extern bool magOn;
extern bool magLocked;        // true during CATCH states; stopAll won't drop

// ---- slip + stall ----
extern bool          slipDetected[3];
extern uint32_t      slipDetectedMs[3];
extern uint32_t      lastMoveMs[3];
extern bool          stalled[3];

// ---- IMU ----
extern bool  imuReady;
extern float qNow_w, qNow_x, qNow_y, qNow_z;       // current
extern float qRef_w, qRef_x, qRef_y, qRef_z;       // reference (set by 'h')
extern float omegaX, omegaY, omegaZ;               // angular vel deg/s
extern uint32_t lastImuMs;

// ---- tilt v2 ----
extern float wtX, wtY, wtZ;       // world target
extern bool  hasWorldTarget;
extern float jSmooth[3];          // smoothed joint targets
extern uint32_t lastTiltUpdateMs;
extern bool  tiltCompCatch;       // toggle: tilt comp during CATCH_MODE

// ---- catch FSM ----
extern CatchState catchState;
extern uint32_t   catchStateMs;       // when entered current state
extern bool       catchArmed;
extern uint32_t   catchArmedMs;
extern uint32_t   integrityFailMs;    // last time tip discrepancy was OK
extern uint32_t   droneLastSeenMs;
extern float      droneLastVarSq;     // variance² of recent drone positions

// ---- drone position (latest fused) ----
extern float dronePosX, dronePosY, dronePosZ;
extern bool  droneValid;
extern float droneResidual;          // trilat least-squares residual

// ---- sim drone ----
extern bool  simDroneOn;
extern float simDroneX, simDroneY, simDroneZ;

// ---- telemetry ----
extern bool  telemetryEnabled;
extern unsigned long lastTelemetryMs;

// ---- input buffer ----
extern String inputBuf;

// ---- dance ----
extern int  activeDance;
extern int  danceStep;
extern unsigned long danceStepT;
extern bool danceRunning;

// ---- floor guard latch ----
extern bool floorTripped;

// ---- wrist servos (definitions in arm_v4.ino) ----
extern int rollPos, pitchPos;

// ---- functions used cross-file ----
void stopMotor(int j);
void stopAll();
void runMotor(int j, bool dirPositive, int spd);
void magnetOn_silent();
void magnetOff_silent();
void magnetOn();
void magnetOff();
void emitTelemetry();

#endif
