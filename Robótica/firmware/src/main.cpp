// SCARA Estacion 2 -- Stage 1-2 bring-up
//   drivers (MC33926) + quadrature encoders + one position PID per joint,
//   tuned from the serial monitor. No ROS yet: first prove the joint works.
//
// Serial commands (115200, newline-terminated):
//   j <1|2>            select joint
//   e <0|1>            disable / enable selected joint (boots DISABLED)
//   t <deg>            target angle, clamped to +/-JOINT_LIMIT_DEG
//   g <kp> <ki> <kd>   set gains of the selected joint
//   u <0..1>           max duty of the selected joint
//   z                  declare the current position as 0 deg (arm must be physically at 0)
//   v                  toggle streaming: ms,target_deg,pos_deg,duty
//   p                  print status
#include <Arduino.h>
#include "config.h"

// ------------------------------------------------------------------ Encoder
// x4 decoding: an interrupt on BOTH channels, a table gives -1/0/+1 per transition.
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

// ------------------------------------------------------------------ Motor
// MC33926: PWM on IN1 = one direction, PWM on IN2 = the other (the other input stays low).
struct Motor { int in1, in2, ch1, ch2; };

void motorBegin(const Motor& m) {
  ledcSetup(m.ch1, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(m.in1, m.ch1);
  ledcSetup(m.ch2, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(m.in2, m.ch2);
  ledcWrite(m.ch1, 0); ledcWrite(m.ch2, 0);
}
void motorWrite(const Motor& m, float u) {          // u in [-1, 1]
  u = constrain(u, -1.0f, 1.0f);
  uint32_t duty = (uint32_t)(fabsf(u) * ((1 << PWM_BITS) - 1));
  ledcWrite(m.ch1, u >= 0 ? duty : 0);
  ledcWrite(m.ch2, u <  0 ? duty : 0);
}

// ------------------------------------------------------------------ Joint
struct Joint {
  Encoder enc;
  Motor   mot;
  int     encSign;
  volatile bool  enabled = false;
  volatile float targetDeg = 0, kp = DEF_KP, ki = DEF_KI, kd = DEF_KD, umax = DEF_UMAX;
  float integ = 0, prevPos = 0, dFilt = 0;
  volatile float posDeg = 0, duty = 0;               // for printing
};
Joint joints[2];
int sel = 0;
bool streaming = false;

void jointBegin(Joint& j, int a, int b, int in1, int in2, int ch1, int ch2, int sign) {
  j.enc.pinA = a; j.enc.pinB = b; j.encSign = sign;
  j.mot = {in1, in2, ch1, ch2};
  pinMode(a, INPUT); pinMode(b, INPUT);              // external pull-ups/level shifting are on the board
  j.enc.prev = (digitalRead(a) << 1) | digitalRead(b);
  attachInterruptArg(a, encISR, &j.enc, CHANGE);
  attachInterruptArg(b, encISR, &j.enc, CHANGE);
  motorBegin(j.mot);
}
inline float jointPosDeg(const Joint& j) { return j.encSign * (float)j.enc.count * DEG_PER_COUNT; }

// ------------------------------------------------------------------ Control task (core 1)
// Runs at a fixed period, independent of anything the serial/WiFi side is doing.
void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const float dt = 1.0f / CONTROL_HZ;
  for (;;) {
    for (auto& j : joints) {
      float pos = jointPosDeg(j);
      j.posDeg = pos;
      if (!j.enabled) { motorWrite(j.mot, 0); j.duty = 0; j.integ = 0; j.prevPos = pos; continue; }

      float err  = j.targetDeg - pos;
      float vel  = (pos - j.prevPos) / dt;           // deg/s, derivative on the MEASUREMENT (no kick on new targets)
      j.prevPos  = pos;
      j.dFilt   += DERIV_ALPHA * (vel - j.dFilt);

      float uRaw = j.kp * err + j.integ - j.kd * j.dFilt;
      float u    = constrain(uRaw, -j.umax, j.umax);
      // anti-windup: integrate only if not saturated, or if the error pulls us out of saturation
      if (u == uRaw || (uRaw * err) < 0) j.integ += j.ki * err * dt;

      motorWrite(j.mot, u);
      j.duty = u;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
  }
}

// ------------------------------------------------------------------ Serial commands
void printStatus() {
  Joint& j = joints[sel];
  Serial.printf("J%d en=%d target=%.2f pos=%.2f deg duty=%.3f | kp=%.4f ki=%.4f kd=%.4f umax=%.2f | counts=%ld\n",
                sel + 1, (int)j.enabled, j.targetDeg, j.posDeg, j.duty,
                j.kp, j.ki, j.kd, j.umax, (long)j.enc.count);
}

void handleLine(char* s) {
  Joint& j = joints[sel];
  float a, b, c; int k;
  switch (s[0]) {
    case 'j': if (sscanf(s + 1, "%d", &k) == 1 && k >= 1 && k <= 2) sel = k - 1; break;
    case 'e': if (sscanf(s + 1, "%d", &k) == 1) {
                if (k) { j.targetDeg = jointPosDeg(j); j.prevPos = j.targetDeg; j.integ = 0; j.enabled = true; }
                else   { j.enabled = false; }
              } break;
    case 't': if (sscanf(s + 1, "%f", &a) == 1) j.targetDeg = constrain(a, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG); break;
    case 'g': if (sscanf(s + 1, "%f %f %f", &a, &b, &c) == 3) { j.kp = a; j.ki = b; j.kd = c; j.integ = 0; } break;
    case 'u': if (sscanf(s + 1, "%f", &a) == 1) j.umax = constrain(a, 0.0f, 1.0f); break;
    case 'z': j.enc.count = 0; j.targetDeg = 0; j.prevPos = 0; j.integ = 0; break;
    case 'v': streaming = !streaming; break;
    case 'p': break;                                   // status printed below
    default:  return;
  }
  printStatus();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!PINS_VERIFIED) {
    Serial.println("Pins not verified: fill them in include/config.h and set PINS_VERIFIED to true. Motors stay off.");
    for (;;) delay(1000);
  }
  if (DRIVER_EN_PIN >= 0) { pinMode(DRIVER_EN_PIN, OUTPUT); digitalWrite(DRIVER_EN_PIN, HIGH); }
  jointBegin(joints[0], J1_ENC_A, J1_ENC_B, J1_IN1, J1_IN2, 0, 1, ENC_SIGN_J1);
  jointBegin(joints[1], J2_ENC_A, J2_ENC_B, J2_IN1, J2_IN2, 2, 3, ENC_SIGN_J2);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  Serial.println("Ready. Joints boot DISABLED. Select with 'j 1', enable with 'e 1'.");
}

void loop() {
  static char buf[64]; static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; handleLine(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
  static uint32_t lastPrint = 0;
  if (streaming && millis() - lastPrint >= 20) {
    lastPrint = millis();
    Joint& j = joints[sel];
    Serial.printf("%lu,%.2f,%.2f,%.3f\n", (unsigned long)millis(), j.targetDeg, j.posDeg, j.duty);
  }
  delay(1);
}
