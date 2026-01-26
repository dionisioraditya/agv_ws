#include <Arduino.h>

// ================= ENCODER CONFIG =================
// Sesuaikan pin dengan wiring kamu

// Encoder motor kiri
const byte ENC_L_A = 2;   // interrupt
const byte ENC_L_B = 4;

// Encoder motor kanan
const byte ENC_R_A = 20;   // interrupt default 3
const byte ENC_R_B = 21;   // default 5

volatile long posL = 0;   // count encoder kiri
volatile long posR = 0;   // count encoder kanan

// ================= MOTOR / DRIVER CONFIG =================
// Dua pin PWM per motor (BTS7960)

// Motor kiri
const int PWM_L_FWD = 6;   // arah MAJU kiri
const int PWM_L_REV = 7;   // arah MUNDUR kiri

// Motor kanans
const int PWM_R_FWD = 3;   // arah MAJU kanan  default 9
const int PWM_R_REV = 5;  // arah MUNDUR kanan default 10

// +1 = forward, -1 = backward
int motorDirL = +1;
int motorDirR = +1;

// ================= ENCODER & TIMING CONST =================
const float COUNTS_PER_REV   = 16800.0;   // sesuaikan (PPR * 4 * rasio gearbox)
const float SAMPLE_PERIOD    = 0.01;      // 10 ms
const unsigned long SAMPLE_US = (unsigned long)(SAMPLE_PERIOD * 1e6);

// ================= CONTROL VARS =================
float targetL = 0.0f;      // target rad/s (magnitudo) kiri
float targetR = 0.0f;      // target rad/s (magnitudo) kanan
float wL = 0.0f;           // kecepatan nyata kiri (rad/s, magnitudo)
float wR = 0.0f;           // kecepatan nyata kanan

// PID gains (sama untuk kiri & kanan, bisa dibedakan kalau perlu)
float Kp  = 5.0f;
float Ki  = 2.0f;
float Kd  = 0.1f;
float Kff = 10.4f;         // feedforward

// PID internal (kiri)
float integL   = 0.0f;
float prevErrL = 0.0f;
float pwmL     = 0.0f;     // 0..255

// PID internal (kanan)
float integR   = 0.0f;
float prevErrR = 0.0f;
float pwmR     = 0.0f;

// batas integral (anti-windup)
const float INTEG_MAX = 50.0f;
const float INTEG_MIN = -50.0f;

// batas perubahan pwm tiap siklus (soft start)
const float PWM_STEP_MAX = 5.0f;

// timing
unsigned long lastSampleMicros = 0;
long lastPosL = 0;
long lastPosR = 0;

// ================== ENCODER ISR ==================
void readEncoderL() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  if (A == B) posL++;
  else        posL--;
}

void readEncoderR() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  if (A == B) posR++;
  else        posR--;
}

// ================== SET MOTOR FUNCTIONS ==================
// value: 0..255 (magnitudo), arah pakai motorDirL/R

void setMotorLeft(float value) {
  value = constrain(value, 0.0f, 255.0f);
  if (motorDirL > 0) {
    analogWrite(PWM_L_FWD, (int)value);
    analogWrite(PWM_L_REV, 0);
  } else {
    analogWrite(PWM_L_FWD, 0);
    analogWrite(PWM_L_REV, (int)value);
  }
}

void setMotorRight(float value) {
  value = constrain(value, 0.0f, 255.0f);
  if (motorDirR > 0) {
    analogWrite(PWM_R_FWD, (int)value);
    analogWrite(PWM_R_REV, 0);
  } else {
    analogWrite(PWM_R_FWD, 0);
    analogWrite(PWM_R_REV, (int)value);
  }
}

// ================== COMMAND PARSER ==================
void handleToken(String tok) {
  tok.trim();
  if (tok.length() < 3) return;

  char side = toupper(tok[0]);   // 'R' atau 'L'
  char dir  = toupper(tok[1]);   // 'F' atau 'B'
  float v = tok.substring(2).toFloat();  // angka setelah RF / LF
  if (v < 0) v = -v;             // pastikan positif

  if (side == 'L') {
    motorDirL = (dir == 'F') ? +1 : -1;
    targetL   = v;
  } else if (side == 'R') {
    motorDirR = (dir == 'F') ? +1 : -1;
    targetR   = v;
  }
}

// ================== SETUP ==================
void setup() {
  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, CHANGE);

  // Motor driver pins
  pinMode(PWM_L_FWD, OUTPUT);
  pinMode(PWM_L_REV, OUTPUT);
  pinMode(PWM_R_FWD, OUTPUT);
  pinMode(PWM_R_REV, OUTPUT);

  setMotorLeft(0);
  setMotorRight(0);

  Serial.begin(115200);
}

// ================== MAIN LOOP ==================
void loop() {
  // ---- Baca command, contoh: "RF1 LF1" ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Pecah jadi token dipisah spasi
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx == -1) {
      // hanya satu perintah
      handleToken(line);
    } else {
      String t1 = line.substring(0, spaceIdx);
      String t2 = line.substring(spaceIdx + 1);
      handleToken(t1);
      handleToken(t2);
    }
  }

  unsigned long now = micros();
  if (now - lastSampleMicros >= SAMPLE_US) {
    lastSampleMicros += SAMPLE_US;

    // ----- 1. Hitung kecepatan dari encoder -----
    long pL, pR;
    noInterrupts();
    pL = posL;
    pR = posR;
    interrupts();

    long dL = pL - lastPosL;
    long dR = pR - lastPosR;
    lastPosL = pL;
    lastPosR = pR;

    wL = (dL / COUNTS_PER_REV) * 2.0f * PI / SAMPLE_PERIOD;
    wR = (dR / COUNTS_PER_REV) * 2.0f * PI / SAMPLE_PERIOD;
    if (wL < 0) wL = -wL;
    if (wR < 0) wR = -wR;

    // ----- 2. PID motor kiri -----
    float errL = targetL - wL;
    float P_L = Kp * errL;

    if (fabs(targetL) > 0.01f) {
      integL += errL * SAMPLE_PERIOD;
      if (integL > INTEG_MAX) integL = INTEG_MAX;
      if (integL < INTEG_MIN) integL = INTEG_MIN;
    } else {
      integL = 0.0f;
    }
    float I_L = Ki * integL;

    float D_L = Kd * (errL - prevErrL) / SAMPLE_PERIOD;
    prevErrL = errL;

    float uL = Kff * targetL + P_L + I_L + D_L;
    float targetPwmL = constrain(uL, 0.0f, 255.0f);

    if (targetPwmL > pwmL + PWM_STEP_MAX)
      pwmL += PWM_STEP_MAX;
    else if (targetPwmL < pwmL - PWM_STEP_MAX)
      pwmL -= PWM_STEP_MAX;
    else
      pwmL = targetPwmL;

    pwmL = constrain(pwmL, 0.0f, 255.0f);

    // ----- 3. PID motor kanan -----
    float errR = targetR - wR;
    float P_R = Kp * errR;

    if (fabs(targetR) > 0.01f) {
      integR += errR * SAMPLE_PERIOD;
      if (integR > INTEG_MAX) integR = INTEG_MAX;
      if (integR < INTEG_MIN) integR = INTEG_MIN;
    } else {
      integR = 0.0f;
    }
    float I_R = Ki * integR;

    float D_R = Kd * (errR - prevErrR) / SAMPLE_PERIOD;
    prevErrR = errR;

    float uR = Kff * targetR + P_R + I_R + D_R;
    float targetPwmR = constrain(uR, 0.0f, 255.0f);

    if (targetPwmR > pwmR + PWM_STEP_MAX)
      pwmR += PWM_STEP_MAX;
    else if (targetPwmR < pwmR - PWM_STEP_MAX)
      pwmR -= PWM_STEP_MAX;
    else
      pwmR = targetPwmR;

    pwmR = constrain(pwmR, 0.0f, 255.0f);

    // ----- 4. Kirim ke motor -----
    setMotorLeft(pwmL);
    setMotorRight(pwmR);

    // ----- 5. Serial Plotter -----
    // value1 = targetL (signed sesuai arah)
    // value2 = wL
    // value3 = targetR (signed)
    // value4 = wR
    float tL_signed = (motorDirL > 0) ? targetL : -targetL;
    float tR_signed = (motorDirR > 0) ? targetR : -targetR;

    Serial.print(tL_signed); Serial.print(" ");
    Serial.print(wL);        Serial.print(" ");
    Serial.print(tR_signed); Serial.print(" ");
    Serial.println(wR);
  }
}
