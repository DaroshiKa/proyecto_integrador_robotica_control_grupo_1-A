#pragma once
#include <Arduino.h>

// =====================================================================
//  MECANICA  (BOM / informe)
// =====================================================================
constexpr float MOTOR_ENC_CPR   = 64.0f;          // cuentas por vuelta del MOTOR, decodificacion x4
constexpr float GEARBOX_RATIO   = 50.0f;          // confirmar la relacion exacta en la ficha de Pololu
constexpr float PULLEY_RATIO    = 100.0f / 20.0f; // polea 20T (motor) -> 100T (articulacion) = 5
constexpr float COUNTS_PER_JOINT_REV = MOTOR_ENC_CPR * GEARBOX_RATIO * PULLEY_RATIO; // 16000
constexpr float DEG_PER_COUNT   = 360.0f / COUNTS_PER_JOINT_REV;                     // 0.0225 deg

constexpr float JOINT_LIMIT_DEG = 90.0f;          // limite de J1 y J2 (+/-)

// Poner -1 si con duty positivo los grados salen negativos (prueba 'o' en main.cpp).
constexpr int ENC_SIGN_J1 = +1;
constexpr int ENC_SIGN_J2 = +1;

// =====================================================================
//  PINES (mapeados a mano sobre la placa)
//  MC33926: direccion por INV, velocidad por PWM en el pin D2 del driver.
//  IN1=3V3, IN2=GND, EN=3V3, D1=GND en la placa: no los maneja el ESP32.
//  Ojo: con D2 en bajo el driver queda en tri-state (el motor gira libre).
//  GPIO 34/35/36(VP)/39(VN) son solo entrada y NO tolerantes a 5 V.
// =====================================================================
#define PINS_VERIFIED true

constexpr int J1_DIR = 26, J1_PWM = 25;      // driver 1: INV, D2
constexpr int J2_DIR = 33, J2_PWM = 32;      // driver 2: INV, D2

constexpr int J1_ENC_A = 35, J1_ENC_B = 34;  // blanco, amarillo
constexpr int J2_ENC_A = 39, J2_ENC_B = 36;  // blanco (VN), amarillo (VP)

// Finales de carrera: contacto NO entre el pin y GND -> activo en BAJO (INPUT_PULLUP)
constexpr int J1_LIM_R = 27, J1_LIM_L = 13;
constexpr int J2_LIM_R = 19, J2_LIM_L = 18;

// =====================================================================
//  CONTROL
// =====================================================================
constexpr int   CONTROL_HZ  = 1000;
constexpr int   PWM_FREQ_HZ = 20000;   // el MC33926 acepta hasta 20 kHz
constexpr int   PWM_BITS    = 10;
constexpr float DERIV_ALPHA = 0.1f;    // filtro pasa-bajas de la velocidad medida

// Valores de arranque (duty por grado, por (grado*s), por (grado/s)). Solo un punto de partida.
constexpr float DEF_KP = 0.05f, DEF_KI = 0.0f, DEF_KD = 0.001f;
constexpr float DEF_UMAX = 0.25f;      // duty maximo al arrancar: bajo a proposito
