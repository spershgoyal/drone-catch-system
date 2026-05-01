// eeprom_persist.h — EEPROM persistence for cal data + anchor positions.
#ifndef ARM_EEPROM_H
#define ARM_EEPROM_H
#include "config.h"
#include "state.h"
#include <EEPROM.h>

// Layout (after magic at +0):
//   joint offsets (3 * float)
//   roll/pitch level offsets (2 * float)
//   anchor positions (3 anchors × 3 floats)
//   tip anchor local offsets (3 floats)
//   magAttached (1 byte)
//   simDroneOn (1 byte)
inline void saveOffsetsToEEPROM() {
  int addr = EEPROM_ADDR_BASE;
  EEPROM.put(addr, EEPROM_MAGIC); addr += sizeof(EEPROM_MAGIC);
  for (int j = 0; j < 3; j++) { EEPROM.put(addr, JOINT_OFFSET[j]); addr += sizeof(float); }
  EEPROM.put(addr, ROLL_LEVEL_OFFSET);  addr += sizeof(float);
  EEPROM.put(addr, PITCH_LEVEL_OFFSET); addr += sizeof(float);
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    for (int k = 0; k < 3; k++) { EEPROM.put(addr, ANCHOR_POS[i][k]); addr += sizeof(float); }
  }
  for (int k = 0; k < 3; k++) { EEPROM.put(addr, TIP_ANCHOR_LOCAL[k]); addr += sizeof(float); }
  uint8_t flags = (magAttached ? 1 : 0) | (simDroneOn ? 2 : 0);
  EEPROM.put(addr, flags); addr += sizeof(flags);
  Serial.println("[eeprom] saved");
}

inline bool loadOffsetsFromEEPROM() {
  int addr = EEPROM_ADDR_BASE;
  uint32_t magic = 0;
  EEPROM.get(addr, magic); addr += sizeof(magic);
  if (magic != EEPROM_MAGIC) return false;
  for (int j = 0; j < 3; j++) {
    float v;
    EEPROM.get(addr, v); addr += sizeof(float);
    if (isnan(v) || v < -90 || v > 90) return false;
    JOINT_OFFSET[j] = v;
  }
  float wr, wp;
  EEPROM.get(addr, wr); addr += sizeof(float);
  EEPROM.get(addr, wp); addr += sizeof(float);
  if (!isnan(wr) && wr >= -90 && wr <= 90) ROLL_LEVEL_OFFSET = wr;
  if (!isnan(wp) && wp >= -90 && wp <= 90) PITCH_LEVEL_OFFSET = wp;
  // Anchors
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    for (int k = 0; k < 3; k++) {
      float v;
      EEPROM.get(addr, v); addr += sizeof(float);
      if (!isnan(v) && fabs(v) < 100.0f) ANCHOR_POS[i][k] = v;
    }
  }
  for (int k = 0; k < 3; k++) {
    float v;
    EEPROM.get(addr, v); addr += sizeof(float);
    if (!isnan(v) && fabs(v) < 30.0f) TIP_ANCHOR_LOCAL[k] = v;
  }
  uint8_t flags;
  EEPROM.get(addr, flags); addr += sizeof(flags);
  magAttached = (flags & 1);
  simDroneOn  = (flags & 2);
  return true;
}

inline void resetOffsets() {
  for (int j = 0; j < 3; j++) JOINT_OFFSET[j] = 0;
  ROLL_LEVEL_OFFSET = 0;
  PITCH_LEVEL_OFFSET = 0;
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    for (int k = 0; k < 3; k++) ANCHOR_POS[i][k] = ANCHOR_POS_DEFAULT[i][k];
  }
  for (int k = 0; k < 3; k++) TIP_ANCHOR_LOCAL[k] = TIP_ANCHOR_LOCAL_DEFAULT[k];
  saveOffsetsToEEPROM();
  Serial.println("[eeprom] reset");
}

inline void printOffsets() {
  Serial.print("[cal] B="); Serial.print(JOINT_OFFSET[0], 2);
  Serial.print(" S="); Serial.print(JOINT_OFFSET[1], 2);
  Serial.print(" E="); Serial.print(JOINT_OFFSET[2], 2);
  Serial.print(" WR="); Serial.print(ROLL_LEVEL_OFFSET, 2);
  Serial.print(" WP="); Serial.println(PITCH_LEVEL_OFFSET, 2);
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    Serial.print("[anchor "); Serial.print(i+1); Serial.print("] ");
    Serial.print(ANCHOR_POS[i][0], 2); Serial.print(", ");
    Serial.print(ANCHOR_POS[i][1], 2); Serial.print(", ");
    Serial.println(ANCHOR_POS[i][2], 2);
  }
  Serial.print("[tip local] ");
  Serial.print(TIP_ANCHOR_LOCAL[0], 2); Serial.print(", ");
  Serial.print(TIP_ANCHOR_LOCAL[1], 2); Serial.print(", ");
  Serial.println(TIP_ANCHOR_LOCAL[2], 2);
}

#endif
