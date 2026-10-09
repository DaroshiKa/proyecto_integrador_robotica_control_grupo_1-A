// SCARA Estacion 2 -- bring-up de J1 y J2 con el cableado real
//   encoders en cuadratura + drivers MC33926 + un PID de posicion por articulacion,
//   sintonizable por serial. Sin ROS todavia.
//
// Comandos serial (115200, terminados en Enter):
//   j <1|2>            selecciona articulacion
//   e <0|1>            deshabilita / habilita (arranca DESHABILITADA)
//   t <deg>            angulo objetivo, limitado a +/-JOINT_LIMIT_DEG
//   g <kp> <ki> <kd>   ganancias de la articulacion seleccionada
//   u <0..1>           duty maximo
//   o <duty>           PRUEBA en lazo abierto: duty (max 0.3) durante 300 ms, solo si esta deshabilitada
//   z                  declara la posicion actual como 0 deg
//   v                  alterna el stream: ms,objetivo,posicion,duty
//   p                  estado
#include <Arduino.h>
#include "config.h"

// ------------------------------------------------------------------ Encoder
struct Encoder {
  int pinA = -1, pinB = -1;
  volatile int32_t count = 0;
  volatile uint8_t prev = 0;
};
static const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

void IRAM_ATTR encISR(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  uint8_t curr = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->count += QEM[(e->prev << 2) | curr];
  e->prev = curr;
}

// ------------------------------------------------------------------ Motor (MC33926)
// Direccion: pin INV. Velocidad: PWM en el pin D2 (D2 en bajo = tri-state, sin freno).
struct Motor { int dir, pwm, ch; };

void motorBegin(const Motor& m) {
  pinMode(m.dir, OUTPUT); digitalWrite(m.dir, LOW);
  ledcSetup(m.ch, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(m.pwm, m.ch);
  ledcWrite(m.ch, 0);
}
void motorWrite(const Motor& m, float u) {          // u en [-1, 1]
  u = constrain(u, -1.0f, 1.0f);
  if (u != 0.0f) digitalWrite(m.dir, u < 0 ? HIGH : LOW);
  ledcWrite(m.ch, (uint32_t)(fabsf(u) * ((1 << PWM_BITS) - 1)));
}

// ------------------------------------------------------------------ Joint
struct Joint {
  Encoder enc;
  Motor   mot;
  int     encSign;
  int     limR, limL;
  volatile bool  enabled = false, limitActive = false, tripped = false;
  volatile float targetDeg = 0, kp = DEF_KP, ki = DEF_KI, kd = DEF_KD, umax = DEF_UMAX;
  volatile float openDuty = 0;
  volatile uint32_t openUntil = 0;
  float integ = 0, prevPos = 0, dFilt = 0;
  volatile float posDeg = 0, duty = 0;
};
Joint joints[2];
int sel = 0;
bool streaming = false;

void jointBegin(Joint& j, int a, int b, int dir, int pwm, int ch, int sign, int limR, int limL) {
  j.enc.pinA = a; j.enc.pinB = b; j.encSign = sign;
  j.limR = limR; j.limL = limL;
  j.mot = {dir, pwm, ch};
  motorBegin(j.mot);                                  // primero el motor: salida en 0
  pinMode(limR, INPUT_PULLUP); pinMode(limL, INPUT_PULLUP);
  pinMode(a, INPUT); pinMode(b, INPUT);               // 34/35/36/39 no tienen pull-up interno
  j.enc.prev = (digitalRead(a) << 1) | digitalRead(b);
  attachInterruptArg(a, encISR, &j.enc, CHANGE);
  attachInterruptArg(b, encISR, &j.enc, CHANGE);
}
inline float jointPosDeg(const Joint& j) { return j.encSign * (float)j.enc.count * DEG_PER_COUNT; }

// ------------------------------------------------------------------ Tarea de control (nucleo 1)
void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const float dt = 1.0f / CONTROL_HZ;
  for (;;) {
    for (auto& j : joints) {
      float pos = jointPosDeg(j);
      j.posDeg = pos;

      j.limitActive = (digitalRead(j.limR) == LOW) || (digitalRead(j.limL) == LOW);
      if (j.enabled && j.limitActive) { j.enabled = false; j.tripped = true; }

      if (!j.enabled) {
        float u = (millis() < j.openUntil) ? j.openDuty : 0.0f;   // solo la prueba 'o' mueve en deshabilitado
        motorWrite(j.mot, u); j.duty = u; j.integ = 0; j.prevPos = pos; continue;
      }

      float err  = j.targetDeg - pos;
      float vel  = (pos - j.prevPos) / dt;            // derivada sobre la MEDICION
      j.prevPos  = pos;
      j.dFilt   += DERIV_ALPHA * (vel - j.dFilt);

      float uRaw = j.kp * err + j.integ - j.kd * j.dFilt;
      float u    = constrain(uRaw, -j.umax, j.umax);
      if (u == uRaw || (uRaw * err) < 0) j.integ += j.ki * err * dt;   // anti-windup

      motorWrite(j.mot, u);
      j.duty = u;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
  }
}

// ------------------------------------------------------------------ Comandos serial
void printStatus() {
  Joint& j = joints[sel];
  Serial.printf("J%d en=%d lim=%d target=%.2f pos=%.2f deg duty=%.3f | kp=%.4f ki=%.4f kd=%.4f umax=%.2f | counts=%ld\n",
                sel + 1, (int)j.enabled, (int)j.limitActive, j.targetDeg, j.posDeg, j.duty,
                j.kp, j.ki, j.kd, j.umax, (long)j.enc.count);
}

void handleLine(char* s) {
  Joint& j = joints[sel];
  float a, b, c; int k;
  switch (s[0]) {
    case 'j': if (sscanf(s + 1, "%d", &k) == 1 && k >= 1 && k <= 2) sel = k - 1; break;
    case 'e': if (sscanf(s + 1, "%d", &k) == 1) {
                if (k) {
                  if (j.limitActive) { Serial.println("Un final de carrera esta activo: mueve la articulacion a mano para liberarlo."); return; }
                  j.targetDeg = jointPosDeg(j); j.prevPos = j.targetDeg; j.integ = 0; j.enabled = true;
                } else { j.enabled = false; }
              } break;
    case 't': if (sscanf(s + 1, "%f", &a) == 1) j.targetDeg = constrain(a, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG); break;
    case 'g': if (sscanf(s + 1, "%f %f %f", &a, &b, &c) == 3) { j.kp = a; j.ki = b; j.kd = c; j.integ = 0; } break;
    case 'u': if (sscanf(s + 1, "%f", &a) == 1) j.umax = constrain(a, 0.0f, 1.0f); break;
    case 'z': j.enc.count = 0; j.targetDeg = 0; j.prevPos = 0; j.integ = 0; break;
    case 'v': streaming = !streaming; break;
    case 'o':
      if (sscanf(s + 1, "%f", &a) == 1) {
        if (j.enabled) { Serial.println("Deshabilita primero (e 0)."); return; }
        a = constrain(a, -0.3f, 0.3f);
        float p0 = jointPosDeg(j);
        j.openDuty = a; j.openUntil = millis() + 300;
        delay(450);
        float dp = jointPosDeg(j) - p0;
        Serial.printf("Prueba J%d duty=%+.2f -> cambio %+.2f deg\n", sel + 1, a, dp);
        if (fabsf(dp) < 0.5f) Serial.println("Casi no se movio: sube un poco el duty o revisa 12 V, D2 y encoder.");
        else if ((a > 0) != (dp > 0)) Serial.println("Signo invertido: cambia ENC_SIGN de esta articulacion a -1 en config.h.");
        else Serial.println("Signo correcto: duty positivo sube la posicion.");
      } break;
    case 'p': break;
    default:  return;
  }
  printStatus();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!PINS_VERIFIED) {
    Serial.println("PINS_VERIFIED en false en config.h: los motores quedan apagados.");
    for (;;) delay(1000);
  }
  jointBegin(joints[0], J1_ENC_A, J1_ENC_B, J1_DIR, J1_PWM, 0, ENC_SIGN_J1, J1_LIM_R, J1_LIM_L);
  jointBegin(joints[1], J2_ENC_A, J2_ENC_B, J2_DIR, J2_PWM, 1, ENC_SIGN_J2, J2_LIM_R, J2_LIM_L);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  Serial.println("Listo. Las articulaciones arrancan DESHABILITADAS. Selecciona con 'j 1', prueba el signo con 'o 0.15'.");
}

void loop() {
  static char buf[64]; static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; handleLine(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
  for (int i = 0; i < 2; i++) {
    if (joints[i].tripped) { joints[i].tripped = false; Serial.printf("J%d: final de carrera activado, articulacion deshabilitada.\n", i + 1); }
  }
  static uint32_t lastPrint = 0;
  if (streaming && millis() - lastPrint >= 20) {
    lastPrint = millis();
    Joint& j = joints[sel];
    Serial.printf("%lu,%.2f,%.2f,%.3f\n", (unsigned long)millis(), j.targetDeg, j.posDeg, j.duty);
  }
  delay(1);
}
