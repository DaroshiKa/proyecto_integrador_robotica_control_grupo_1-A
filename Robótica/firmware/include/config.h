#pragma once
#include <Arduino.h>

// =====================================================================
//  MECHANICS  (from the BOM / report)
// =====================================================================
constexpr float MOTOR_ENC_CPR   = 64.0f;          // counts per MOTOR-shaft rev, x4 decoding
constexpr float GEARBOX_RATIO   = 50.0f;          // CHECK the exact ratio on the Pololu page
constexpr float PULLEY_RATIO    = 100.0f / 20.0f; // 20T motor pulley -> 100T joint pulley = 5
constexpr float COUNTS_PER_JOINT_REV = MOTOR_ENC_CPR * GEARBOX_RATIO * PULLEY_RATIO; // 16000
constexpr float DEG_PER_COUNT   = 360.0f / COUNTS_PER_JOINT_REV;                     // 0.0225 deg

constexpr float JOINT_LIMIT_DEG = 90.0f;          // mechanical limit of J1 and J2 (+/-)

// Flip to -1 if a positive PWM makes the encoder count DOWN (then the loop would run away).
constexpr int ENC_SIGN_J1 = +1;
constexpr int ENC_SIGN_J2 = +1;

// =====================================================================
//  PINS  -- DO NOT GUESS THESE. Trace each one on the board (unpowered!)
//  and fill them in. Pins you said are in use:
//  Vin, 3V3, D26, D25, D33, D32, D13, D27, D18, D19
//  Rules: avoid GPIO 6-11 (flash) and be careful with 0, 2, 5, 12, 15 (boot pins).
//         34-39 are input-only and have NO internal pull-up.
// =====================================================================
#define PINS_VERIFIED false      // set to true ONLY after you checked every pin below

// MC33926: IN1/IN2 per motor (PWM goes on one of them, the other stays low)
constexpr int J1_IN1 = -1, J1_IN2 = -1;
constexpr int J2_IN1 = -1, J2_IN2 = -1;
// Encoders (A/B per motor). Must be 3.3 V levels at the ESP32 pin.
constexpr int J1_ENC_A = -1, J1_ENC_B = -1;
constexpr int J2_ENC_A = -1, J2_ENC_B = -1;
// Optional driver enable (-1 if the board ties it permanently). Check the MC33926 board
// docs for EN / D1 / D2 polarity before using it.
constexpr int DRIVER_EN_PIN = -1;

// =====================================================================
//  CONTROL
// =====================================================================
constexpr int   CONTROL_HZ  = 1000;
constexpr int   PWM_FREQ_HZ = 20000;   // MC33926 accepts up to 20 kHz; above hearing range
constexpr int   PWM_BITS    = 10;
constexpr float DERIV_ALPHA = 0.1f;    // low-pass on the measured velocity (0..1, smaller = smoother)

// Start-up defaults (units: duty per degree, per (deg*s), per (deg/s)). Only a starting point.
constexpr float DEF_KP = 0.05f, DEF_KI = 0.0f, DEF_KD = 0.001f;
constexpr float DEF_UMAX = 0.25f;      // max duty at boot: low on purpose. Raise with 'u'.
