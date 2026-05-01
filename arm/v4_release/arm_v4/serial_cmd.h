// serial_cmd.h — all line-based command handling.
#ifndef ARM_SERIAL_CMD_H
#define ARM_SERIAL_CMD_H
#include "config.h"
#include "state.h"
#include "sensors.h"
#include "kinematics.h"
#include "trajectory.h"
#include "poses.h"
#include "dance.h"
#include "wrist.h"
#include "tilt.h"
#include "catch_fsm.h"
#include "uwb.h"
#include "eeprom_persist.h"
#include "safety.h"

inline void doModeSwitch(char c) {
  Mode old = mode;
  switch (c) {
    case 'M': mode = MANUAL; break;
    case 'D': mode = DANCE_MODE; break;
    case 'H': mode = HEIGHT; break;
    case 'K': mode = XYZ_MODE; break;
    case 'B': mode = DEBUG_MODE; break;
    case 'T': mode = TILT; break;
    case 'C': mode = CATCH_MODE; break;
  }
  if (old != mode) {
    stopAll_impl();
    Serial.print("[mode] -> ");
    Serial.println(mode == MANUAL ? "MANUAL" :
                   mode == DEBUG_MODE ? "DEBUG" :
                   mode == HEIGHT ? "HEIGHT" :
                   mode == XYZ_MODE ? "XYZ" :
                   mode == DANCE_MODE ? "DANCE" :
                   mode == TILT ? "TILT" : "CATCH");
  }
}

inline void teleopNudge(float dx, float dy, float dz) {
  float cx, cy, cz;
  currentTipXYZ(cx, cy, cz);
  if (!moveToXYZ(cx + dx, cy + dy, cz + dz)) {
    Serial.print("   kept at ("); Serial.print(cx,1); Serial.print(",");
    Serial.print(cy,1); Serial.print(","); Serial.print(cz,1); Serial.println(")");
  }
}

inline void doManualKey(char c) {
  switch (c) {
    case '1': manualPulse(0, true);  break;
    case 'q': manualPulse(0, false); break;
    case '2': manualPulse(1, true);  break;
    case 'w': manualPulse(1, false); break;
    case '3': manualPulse(2, true);  break;
    case 'e': manualPulse(2, false); break;
    case '4': setRoll(rollPos + 5);  break;
    case 'f': setRoll(rollPos - 5);  break;
    case '5': setPitch(pitchPos + 5); break;
    case 't': setPitch(pitchPos - 5); break;
  }
}

inline void doXYZKey(char c) {
  switch (c) {
    case 'w': teleopNudge(0, stepIn, 0); break;
    case 's': teleopNudge(0, -stepIn, 0); break;
    case 'a': teleopNudge(-stepIn, 0, 0); break;
    case 'd': teleopNudge(stepIn, 0, 0); break;
    case 'q': teleopNudge(0, 0, stepIn); break;
    case 'e': teleopNudge(0, 0, -stepIn); break;
  }
}

// Cal command: "cal", "cal save", "cal load", "cal reset", "cal b/s/e <deg>"
//              "cal wr/wp <deg>"
inline void handleCalLine(String body) {
  body.trim();
  if (body.length() == 0) { printOffsets(); return; }
  if (body == "save")  { saveOffsetsToEEPROM(); printOffsets(); return; }
  if (body == "load")  { stopAll_impl(); if (loadOffsetsFromEEPROM()) Serial.println("[cal] loaded"); printOffsets(); return; }
  if (body == "reset") { stopAll_impl(); resetOffsets(); printOffsets(); return; }
  if (body.startsWith("wr ") || body.startsWith("wp ")) {
    bool isRoll = (body.charAt(1) == 'r');
    String n = body.substring(3); n.trim();
    if (n.length() == 0) { Serial.println("[cal] missing num"); return; }
    float v = n.toFloat();
    if (v < -90 || v > 90) { Serial.println("[cal] out of range"); return; }
    if (isRoll) ROLL_LEVEL_OFFSET = v; else PITCH_LEVEL_OFFSET = v;
    printOffsets();
    return;
  }
  // Anchor cal: "cal anchor N x y z" or "cal tip x y z"
  if (body.startsWith("anchor ")) {
    String rest = body.substring(7); rest.trim();
    int n = rest.charAt(0) - '0';
    if (n < 1 || n > ANCHOR_COUNT) { Serial.println("[cal] anchor 1..3"); return; }
    String coords = rest.substring(2); coords.trim();
    int sp1 = coords.indexOf(' ');
    int sp2 = coords.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) { Serial.println("[cal] usage: cal anchor N x y z"); return; }
    float x = coords.substring(0, sp1).toFloat();
    float y = coords.substring(sp1+1, sp2).toFloat();
    float z = coords.substring(sp2+1).toFloat();
    ANCHOR_POS[n-1][0] = x;
    ANCHOR_POS[n-1][1] = y;
    ANCHOR_POS[n-1][2] = z;
    Serial.print("[cal] anchor "); Serial.print(n);
    Serial.print(" set to ("); Serial.print(x,1); Serial.print(",");
    Serial.print(y,1); Serial.print(","); Serial.print(z,1); Serial.println(")");
    return;
  }
  if (body.startsWith("tip ")) {
    String coords = body.substring(4); coords.trim();
    int sp1 = coords.indexOf(' ');
    int sp2 = coords.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) { Serial.println("[cal] usage: cal tip perp along z"); return; }
    TIP_ANCHOR_LOCAL[0] = coords.substring(0, sp1).toFloat();
    TIP_ANCHOR_LOCAL[1] = coords.substring(sp1+1, sp2).toFloat();
    TIP_ANCHOR_LOCAL[2] = coords.substring(sp2+1).toFloat();
    Serial.print("[cal] tip local set");
    return;
  }
  // Single axis: "b 5.0", "s -17", "e 0"
  char axis = body.charAt(0);
  if (axis != 'b' && axis != 's' && axis != 'e') {
    Serial.println("[cal] usage: cal | cal b|s|e|wr|wp <deg> | cal anchor N x y z | cal tip p l z | save|load|reset");
    return;
  }
  String n = body.substring(1); n.trim();
  if (n.length() == 0) { Serial.println("[cal] missing num"); return; }
  float v = n.toFloat();
  if (v < -90 || v > 90) { Serial.println("[cal] out of range"); return; }
  int j = (axis == 'b') ? 0 : (axis == 's') ? 1 : 2;
  stopAll_impl();
  JOINT_OFFSET[j] = v;
  printOffsets();
}

// Run motor pulse test (each joint ~50ms each direction)
inline void testMotors() {
  Serial.println("[test] motor pulse test");
  for (int j = 0; j < 3; j++) {
    Serial.print("  j"); Serial.print(j); Serial.println(" +"); 
    runMotor_impl(j, true, 35); delay(150); stopMotor_impl(j); delay(150);
    Serial.print("  j"); Serial.print(j); Serial.println(" -"); 
    runMotor_impl(j, false, 35); delay(150); stopMotor_impl(j); delay(300);
  }
  Serial.println("[test] motors done");
}
inline void testPots() {
  Serial.println("[test] pot reads (move each joint by hand, watch values)");
  for (int i = 0; i < 50; i++) {
    Serial.print("p38="); Serial.print(curRaw[0]);
    Serial.print(" p21="); Serial.print(curRaw[1]);
    Serial.print(" p20="); Serial.println(curRaw[2]);
    delay(100);
  }
}
inline void testUWB() {
  Serial.println("[test] UWB stability (5s)");
  uint32_t t = millis();
  while (millis() - t < 5000) { uwb_update(); delay(10); }
  uwb_status();
}
inline void testIMU() {
  Serial.println("[test] IMU q over 5s");
  uint32_t t = millis();
  while (millis() - t < 5000) {
    if (imuReady && imu.dataAvailable()) {
      Serial.print(imu.getQuatReal(),3); Serial.print(",");
      Serial.print(imu.getQuatI(),3); Serial.print(",");
      Serial.print(imu.getQuatJ(),3); Serial.print(",");
      Serial.println(imu.getQuatK(),3);
    }
    delay(100);
  }
}
inline void testMag() {
  Serial.println("[test] mag pulse 1s");
  magnetOn_impl(); delay(1000); magnetOff_impl();
}
inline void testAll() {
  testMotors();
  testPots();
  testUWB();
  testIMU();
  testMag();
  Serial.println("[test] all done");
}

inline void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  // ====== CAL ======
  if (s.startsWith("cal ") || s == "cal") {
    String body = (s == "cal") ? "" : s.substring(4);
    handleCalLine(body); return;
  }
  // ====== MAG ======
  if (s == "mag on")  { magnetOn_impl();  return; }
  if (s == "mag off") { magLocked = false; magnetOff_impl(); return; }
  if (s == "mag lock on")  { magLocked = true;  Serial.println("[mag] LOCKED"); return; }
  if (s == "mag lock off") { magLocked = false; Serial.println("[mag] unlocked"); return; }
  if (s == "magattach on")  { magAttached = true;  Serial.println("[mag] attach"); return; }
  if (s == "magattach off") { magAttached = false; Serial.println("[mag] detach"); return; }
  // ====== CATCH ======
  if (s == "arm")    { catchArm(); return; }
  if (s == "disarm") { catchDisarm(); return; }
  if (s == "catch reset") { catchReset(); return; }
  if (s == "tcompcatch on")  { tiltCompCatch = true;  Serial.println("[catch] tilt-comp ON"); return; }
  if (s == "tcompcatch off") { tiltCompCatch = false; Serial.println("[catch] tilt-comp OFF"); return; }
  // ====== SIM DRONE ======
  if (s == "simdrone on")  { simDroneOn = true;  Serial.println("[sim] ON"); return; }
  if (s == "simdrone off") { simDroneOn = false; Serial.println("[sim] OFF"); return; }
  if (s.startsWith("simdrone ") || s.startsWith("$D ")) {
    String coords = s.startsWith("$D ") ? s.substring(3) : s.substring(9);
    coords.trim();
    int sp1 = coords.indexOf(' ');
    int sp2 = coords.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) return;
    simDroneX = coords.substring(0, sp1).toFloat();
    simDroneY = coords.substring(sp1+1, sp2).toFloat();
    simDroneZ = coords.substring(sp2+1).toFloat();
    if (!s.startsWith("$D")) {
      Serial.print("[sim] drone @ ("); Serial.print(simDroneX,1); Serial.print(",");
      Serial.print(simDroneY,1); Serial.print(","); Serial.print(simDroneZ,1); Serial.println(")");
    }
    return;
  }
  // ====== UWB ======
  if (s == "uwb status") { uwb_status(); return; }
  if (s == "uwb reinit") {
    for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
      uwbAnchors[i].initStep = 0;
      uwbAnchors[i].initDone = false;
      uwbAnchors[i].ready = false;
    }
    Serial.println("[uwb] re-init triggered");
    return;
  }
  // ====== TESTS ======
  if (s == "test motors") { testMotors(); return; }
  if (s == "test pots")   { testPots(); return; }
  if (s == "test uwb")    { testUWB(); return; }
  if (s == "test imu")    { testIMU(); return; }
  if (s == "test mag")    { testMag(); return; }
  if (s == "test all")    { testAll(); return; }
  // ====== TILT ======
  if (s.startsWith("tflip ")) {
    char axis = s.charAt(6);
    extern int TILT_SIGN_ROLL, TILT_SIGN_PITCH, TILT_SIGN_YAW;
    if (axis == 'r') { TILT_SIGN_ROLL  = -TILT_SIGN_ROLL;  Serial.print("[tilt] roll sign=");  Serial.println(TILT_SIGN_ROLL); }
    if (axis == 'p') { TILT_SIGN_PITCH = -TILT_SIGN_PITCH; Serial.print("[tilt] pitch sign="); Serial.println(TILT_SIGN_PITCH); }
    if (axis == 'y') { TILT_SIGN_YAW   = -TILT_SIGN_YAW;   Serial.print("[tilt] yaw sign=");   Serial.println(TILT_SIGN_YAW); }
    return;
  }
  // ====== DANCE ======
  if (s.startsWith("dn ")) {
    int n = s.substring(3).toInt();
    selectDance(n);
    return;
  }
  // ====== WRIST ======
  if (s.startsWith("WR ")) { setRoll(s.substring(3).toInt());  return; }
  if (s.startsWith("WP ")) { setPitch(s.substring(3).toInt()); return; }
  // ====== XYZ DIRECT ======
  if (s.startsWith("c ") && (mode == XYZ_MODE || mode == TILT || mode == CATCH_MODE)) {
    String coords = s.substring(2); coords.trim();
    int sp1 = coords.indexOf(' ');
    int sp2 = coords.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) { Serial.println("[c] usage: c X Y Z"); return; }
    float x = coords.substring(0, sp1).toFloat();
    float y = coords.substring(sp1+1, sp2).toFloat();
    float z = coords.substring(sp2+1).toFloat();
    if (mode == TILT) {
      wtX = x; wtY = y; wtZ = z; hasWorldTarget = true;
      Serial.println("[tilt] target set");
    } else moveToXYZ(x, y, z);
    return;
  }
  // ====== HEIGHT MODE NUMBER ======
  if (mode == HEIGHT) {
    float z = s.toFloat();
    if (z > 0 || s.startsWith("0") || s.startsWith("-")) {
      float sD, eD;
      if (solveHeight(z, sD, eD)) startSyncedMove(0, sD, eD, false, true, true);
      else Serial.println("[H] unreachable");
    }
    return;
  }
  // ====== FLIP MOTOR DIR (Fb / Fs / Fe) ======
  if (s.startsWith("F") && s.length() == 2) {
    char a = s.charAt(1);
    int j = (a == 'b') ? 0 : (a == 's') ? 1 : (a == 'e') ? 2 : -1;
    if (j >= 0) {
      COUNTS_PER_DEG[j] = -COUNTS_PER_DEG[j];
      Serial.print("[flip] j"); Serial.print(j); Serial.print(" cpd=");
      Serial.println(COUNTS_PER_DEG[j], 3);
    }
    return;
  }
  Serial.print("[?] "); Serial.println(s);
}

#endif
