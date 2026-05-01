// uwb.h — RYUW122 driver for 4 anchors (3 base + 1 tip) on Serial1/2/4/6.
//
// PROTOCOL ASSUMPTIONS (verify against your RYUW122 datasheet — these are
// the most common AT command set for REYAX UWB modules; adjust if yours
// differs):
//   AT+ADDRESS=<n>           set anchor address (1..4)
//   AT+NETWORKID=<id>        set network identifier
//   AT+MODE=<n>              0=normal, 1=anchor, 2=tag (varies by FW)
//   AT+ANCHOR_SEND=<addr>    initiate ranging to address
//   +ANCHOR_RCV=<addr>,<dist>,<rssi>   incoming response
//
// If your modules use different syntax, edit UWB_AT_* defines below.
//
// ARCHITECTURE: non-blocking. Each tick we kick a ranging query on each
// anchor in parallel, then poll all 4 ports for responses. Modules respond
// asynchronously over their own UARTs.
#ifndef ARM_UWB_H
#define ARM_UWB_H
#include "config.h"
#include "state.h"

// AT command strings — edit if your firmware variant differs
#define UWB_AT_RESET     "AT+RESET\r\n"
#define UWB_AT_ADDRESS   "AT+ADDRESS="     // append number
#define UWB_AT_NETID     "AT+NETWORKID=ARM01\r\n"
#define UWB_AT_MODE_ANC  "AT+MODE=1\r\n"
#define UWB_AT_MODE_TAG  "AT+MODE=2\r\n"
#define UWB_AT_RANGE     "AT+ANCHOR_SEND=99\r\n"   // 99 = drone tag address

#define UWB_NUM_ANCHORS 4   // 3 base + 1 tip

struct UwbAnchor {
  HardwareSerial *port;
  uint16_t addr;            // 1..4
  bool isTip;
  bool ready;               // got OK during init
  float lastRange;          // inches (converted from cm)
  uint32_t lastRangeMs;
  float quality;            // EMA of valid responses, 0..1
  char rxBuf[64];
  uint8_t rxLen;
  uint32_t lastQueryMs;
  uint8_t initStep;         // 0..N for init state machine
  uint32_t initStepMs;
  bool initDone;
};

extern UwbAnchor uwbAnchors[UWB_NUM_ANCHORS];

inline void uwb_writeStr(HardwareSerial *port, const char *s) {
  while (*s) { port->write(*s++); }
}

inline void uwb_initBegin() {
  // Configure ports
  uwbAnchors[0].port = &Serial1;
  uwbAnchors[1].port = &Serial2;
  uwbAnchors[2].port = &Serial6;
  uwbAnchors[3].port = &Serial4;
  uwbAnchors[0].addr = 1; uwbAnchors[0].isTip = false;
  uwbAnchors[1].addr = 2; uwbAnchors[1].isTip = false;
  uwbAnchors[2].addr = 3; uwbAnchors[2].isTip = false;
  uwbAnchors[3].addr = 4; uwbAnchors[3].isTip = true;
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    uwbAnchors[i].port->begin(UWB_BAUD);
    uwbAnchors[i].rxLen = 0;
    uwbAnchors[i].quality = 0;
    uwbAnchors[i].ready = false;
    uwbAnchors[i].initStep = 0;
    uwbAnchors[i].initStepMs = 0;
    uwbAnchors[i].initDone = false;
    uwbAnchors[i].lastRange = 0;
    uwbAnchors[i].lastRangeMs = 0;
  }
  Serial.println("[uwb] init begin");
}

// Non-blocking init step machine. Sends one AT command per anchor per tick,
// waits for quiescent period before next, marks initDone when done.
inline void uwb_pumpInit() {
  uint32_t now = millis();
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    UwbAnchor &a = uwbAnchors[i];
    if (a.initDone) continue;
    if (now - a.initStepMs < 200) continue;   // pace AT commands
    char buf[40];
    switch (a.initStep) {
      case 0: uwb_writeStr(a.port, UWB_AT_RESET); break;
      case 1:
        snprintf(buf, sizeof(buf), "%s%u\r\n", UWB_AT_ADDRESS, a.addr);
        uwb_writeStr(a.port, buf);
        break;
      case 2: uwb_writeStr(a.port, UWB_AT_NETID); break;
      case 3:
        uwb_writeStr(a.port, a.isTip ? UWB_AT_MODE_ANC : UWB_AT_MODE_ANC);
        // Both tip and base are anchors; the drone-side module is the tag
        break;
      default:
        a.initDone = true;
        a.ready = true;
        Serial.print("[uwb] anchor "); Serial.print(a.addr);
        Serial.println(" init done");
        continue;
    }
    a.initStep++;
    a.initStepMs = now;
  }
}

// Parse a complete line from a module. Looks for "+ANCHOR_RCV=...".
// Format guess: "+ANCHOR_RCV=<addr>,<dist_cm>,<rssi>" — edit if yours differs.
inline bool uwb_parseLine(UwbAnchor &a, const char *line) {
  // Look for ANCHOR_RCV pattern
  const char *p = strstr(line, "ANCHOR_RCV=");
  if (!p) return false;
  p += 11;  // skip "ANCHOR_RCV="
  // skip address field
  const char *comma1 = strchr(p, ',');
  if (!comma1) return false;
  comma1++;
  // distance is in cm — convert to inches
  float distCm = atof(comma1);
  if (distCm <= 0 || distCm > 5000) return false;   // sanity (50m max)
  float distIn = distCm * 0.393701f;
  a.lastRange = distIn;
  a.lastRangeMs = millis();
  // EMA quality update (got a valid response = bump up)
  a.quality = 0.9f * a.quality + 0.1f;
  if (a.quality > 1) a.quality = 1;
  return true;
}

// Pump bytes from each port into rxBuf, parse complete lines.
inline void uwb_pumpRx() {
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    UwbAnchor &a = uwbAnchors[i];
    while (a.port->available()) {
      char c = (char)a.port->read();
      if (c == '\r') continue;
      if (c == '\n') {
        a.rxBuf[a.rxLen] = 0;
        if (a.rxLen > 0) uwb_parseLine(a, a.rxBuf);
        a.rxLen = 0;
      } else if (a.rxLen < (int)sizeof(a.rxBuf) - 1) {
        a.rxBuf[a.rxLen++] = c;
      } else {
        a.rxLen = 0;   // overflow, reset
      }
    }
  }
}

// Kick a ranging query on all anchors at UWB_QUERY_HZ.
inline void uwb_kickRanging() {
  static uint32_t lastTickMs = 0;
  uint32_t now = millis();
  uint32_t period = 1000UL / UWB_QUERY_HZ;
  if (now - lastTickMs < period) return;
  lastTickMs = now;
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    UwbAnchor &a = uwbAnchors[i];
    if (!a.initDone) continue;
    uwb_writeStr(a.port, UWB_AT_RANGE);
    a.lastQueryMs = now;
  }
}

// Mark stale anchors as low-quality.
inline void uwb_pumpStale() {
  uint32_t now = millis();
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    UwbAnchor &a = uwbAnchors[i];
    if (now - a.lastRangeMs > UWB_TIMEOUT_MS) {
      a.quality *= 0.95f;       // decay
      if (now - a.lastRangeMs > UWB_REINIT_MS && a.initDone) {
        a.initStep = 0;
        a.initDone = false;
        a.ready = false;
        Serial.print("[uwb] anchor "); Serial.print(a.addr); Serial.println(" stale, re-init");
      }
    }
  }
}

inline bool uwb_anchorAlive(int i) {
  if (i < 0 || i >= UWB_NUM_ANCHORS) return false;
  UwbAnchor &a = uwbAnchors[i];
  return a.ready && (millis() - a.lastRangeMs < UWB_TIMEOUT_MS);
}

inline void uwb_update() {
  uwb_pumpInit();
  uwb_pumpRx();
  uwb_kickRanging();
  uwb_pumpStale();
}

inline void uwb_status() {
  for (int i = 0; i < UWB_NUM_ANCHORS; i++) {
    UwbAnchor &a = uwbAnchors[i];
    Serial.print("[uwb] A"); Serial.print(a.addr);
    if (a.isTip) Serial.print("(tip)");
    Serial.print(" ready="); Serial.print(a.ready);
    Serial.print(" range="); Serial.print(a.lastRange, 2);
    Serial.print(" age="); Serial.print(millis() - a.lastRangeMs);
    Serial.print(" q="); Serial.println(a.quality, 2);
  }
}

#endif
