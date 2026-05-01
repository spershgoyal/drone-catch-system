// config.h — pins, constants, anchor positions, EEPROM layout. Teensy 4.1.
#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
#include <Arduino.h>

#define ARM_FW_VERSION "v4.0.0"
#define ARM_FW_DATE    "2026-04-30"

// ---- motor / sensor pins (Teensy 4.1) ----
#define BASE_RPWM 2
#define BASE_LPWM 29
#define SHLD_RPWM 3
#define SHLD_LPWM 5
#define ELBW_RPWM 14
#define ELBW_LPWM 35
#define BASE_POT_PIN 38
#define SHLD_POT_PIN 21
#define ELBW_POT_PIN 20
#define SERVO_ROLL_PIN 22
#define SERVO_PITCH_PIN 23
#define MAG_PIN 33                 // moved from 16 in v3p_4 to free Serial4

// ---- UWB UART assignments ----
// Serial1 (RX=0, TX=1)   anchor A1
// Serial2 (RX=7, TX=8)   anchor A2
// Serial4 (RX=16, TX=17) tip anchor
// Serial6 (RX=25, TX=24) anchor A3
#define UWB_BAUD 115200
#define UWB_QUERY_HZ 10
#define UWB_TIMEOUT_MS 500
#define UWB_REINIT_MS 5000

// ---- arm geometry (inches) ----
#define L1 13.75f
#define L2 17.00f
#define REACH_MAX (L1 + L2)
#define SHOULDER_OFFSET_Z 3.0f
#define MAG_TIP_OFFSET 3.0f

// ---- anchor positions (placeholders, runtime-overridable) ----
#define ANCHOR_COUNT 3
const float ANCHOR_POS_DEFAULT[ANCHOR_COUNT][3] = {
  { -5.0f,  -5.0f, -3.0f },   // A1
  { -5.0f,  10.0f, -3.0f },   // A2
  { 16.0f,   6.0f,  8.0f },   // A3
};
// tip boom local frame: [perp out from forearm, along forearm, perp z]
const float TIP_ANCHOR_LOCAL_DEFAULT[3] = { 4.0f, -2.0f, 0.0f };

// ---- joint limits (deg) ----
const float JOINT_MIN[3] = { -180.0f, -80.0f, -110.0f };
const float JOINT_MAX[3] = {  180.0f,  80.0f,  110.0f };
#define SHLD_FORBID_LO -10.0f
#define SHLD_FORBID_HI  10.0f
#define LIMIT_SLOW_ZONE 18.0f
#define LIMIT_SLOW_PWM 28
#define Z_FLOOR 0.0f
#define Z_FLOOR_MARGIN 0.5f

// ---- motion ----
#define POS_MARGIN_PCT 3.0f
#define POS_MARGIN_MIN_DEG 0.6f
#define POS_MARGIN_TIGHT_DEG 0.3f
#define SCURVE_ACCEL_FRAC 0.25f
#define SCURVE_DECEL_FRAC 0.25f
#define WRONG_WAY_CHECK_MS 250UL
#define WRONG_WAY_THRESH 8.0f

// ---- slip + stall ----
#define SLIP_DETECT_DEG 3.0f
#define SLIP_DETECT_MS 50UL
#define STALL_TIMEOUT_MS 600UL
#define STALL_THRESH_RAW 4

// ---- tilt v2 ----
#define TILT_DEADBAND 2.0f
#define TILT_KP 2.5f
#define TILT_KD 0.4f
#define TILT_NEAR_DAMP 4.0f
#define TILT_NEAR_DEG 5.0f
#define TILT_LOOKAHEAD_MS 50.0f
#define TILT_TARGET_ALPHA 0.6f
#define TILT_OMEGA_ALPHA 0.3f
#define TILT_UPDATE_MS 10UL

// ---- catch FSM ----
#define CATCH_TRACK_HOLDOFF_Z 8.0f
#define CATCH_STAGE_HOLDOFF_Z 4.0f
#define CATCH_COMMIT_TIP_DIST 1.0f
#define CATCH_LOCK_VARIANCE 1.5f
#define CATCH_LOST_TIMEOUT_MS 2000UL
#define CATCH_STAGE_TIMEOUT_MS 5000UL
#define CATCH_COMMIT_TIMEOUT_MS 1500UL
#define CATCH_INTEGRITY_THRESH 4.0f
#define CATCH_INTEGRITY_MS 200UL
#define CATCH_HOLD_AFTER_MAG_MS 800UL
#define CATCH_SETTLE_MS 500UL
#define CATCH_AUTO_DISARM_MS 30000UL
#define CATCH_PWM_TRACK 40
#define CATCH_PWM_STAGE 30
#define CATCH_PWM_COMMIT 70
#define CATCH_PWM_SECURED 30

// ---- IK cost factors ----
#define IK_COST_LIMIT_VIO 1000.0f
#define IK_COST_FORBID_ZONE 50.0f
#define IK_COST_ELBOW_FLIP 200.0f
#define IK_COST_CONTINUITY 0.05f
#define IK_COST_FORBID_TRAJ 30.0f

// ---- telemetry ----
#define TELEMETRY_INTERVAL_MS_NORMAL 50UL
#define TELEMETRY_INTERVAL_MS_CATCH  20UL
#define DEBUG_PRINT_MS 400UL

// ---- defaults ----
const int   POT_AT_0_DEFAULT[3]      = { 2061, 1900, 1745 };
const float COUNTS_PER_DEG_DEFAULT[3] = { 8.556f, 14.861f, -15.750f };
#define ROLL_LEVEL_DEFAULT 90
#define PITCH_LEVEL_DEFAULT 120
#define MAG_PITCH_BIAS_DEFAULT -30.0f

// ---- EEPROM ----
const uint16_t EEPROM_ADDR_BASE = 0;
const uint32_t EEPROM_MAGIC     = 0x4A4F4634;  // "JOF4"

// ---- mode enum (used everywhere) ----
enum Mode : uint8_t { MANUAL=0, DEBUG_MODE=1, HEIGHT=2, XYZ_MODE=3, DANCE_MODE=4, TILT=5, CATCH_MODE=6 };

// ---- catch FSM enum ----
enum CatchState : uint8_t {
  CATCH_IDLE=0, CATCH_SEARCHING=1, CATCH_TRACKING=2, CATCH_STAGING=3,
  CATCH_COMMITTING=4, CATCH_CATCHING=5, CATCH_SECURED=6, CATCH_STOWING=7,
  CATCH_ABORT=8, CATCH_ERROR=9
};

// ---- IK result ----
enum IKResult : uint8_t { IK_OK=0, IK_TOO_FAR=1, IK_TOO_CLOSE=2, IK_NO_VALID=3, IK_BELOW_FLOOR=4 };

#endif
