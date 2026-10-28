#include "FastInterruptEncoder.h"

/* =========================
   IG32 OPTICAL ENCODER
   ========================= */
#define OPT_A 25
#define OPT_B 26

Encoder optEnc(OPT_A, OPT_B, SINGLE, 200);
const float OPT_CPR_1X = 232464.0 / 4.0;
const float GEAR_RATIO = 30.0;

/* =========================
   GENERIC QUADRATURE ENCODER
   ========================= */
#define GEN_A 4
#define GEN_B 16

volatile long genCount = 0;
volatile uint8_t genLastState = 0;
const float GEN_CPR = 9048.0;

/* =========================
   TIMING
   ========================= */
unsigned long lastTime = 0;
long lastOptCount = 0;
long lastGenCount = 0;

/* =========================
   GENERIC ENCODER ISR
   ========================= */
void IRAM_ATTR genEncoderISR() {
  uint8_t a = digitalRead(GEN_A);
  uint8_t b = digitalRead(GEN_B);

  uint8_t state = (a << 1) | b;
  uint8_t transition = (genLastState << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      genCount++;
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      genCount--;
      break;
  }
  genLastState = state;
}

/* =========================
   SETUP
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(500);

  /* --- Optical encoder --- */
  if (!optEnc.init()) {
    Serial.println("Optical encoder FAILED");
    while (1);
  }

  /* --- Generic encoder --- */
  pinMode(GEN_A, INPUT_PULLUP);
  pinMode(GEN_B, INPUT_PULLUP);
  genLastState = (digitalRead(GEN_A) << 1) | digitalRead(GEN_B);

  attachInterrupt(digitalPinToInterrupt(GEN_A), genEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GEN_B), genEncoderISR, CHANGE);

  lastTime = millis();
  Serial.println("ENCODERS READY");
}

/* =========================
   LOOP
   ========================= */
void loop() {
  optEnc.loop();   // REQUIRED

  if (millis() - lastTime >= 200) {

    /* -------- Optical -------- */
    long optCount = optEnc.getTicks();
    long optDelta = optCount - lastOptCount;

    float optRPM = (optDelta * 300.0) / OPT_CPR_1X;
    float optAngle = (optCount * 360.0) / (OPT_CPR_1X * GEAR_RATIO);

    /* -------- Generic -------- */
    noInterrupts();
    long genCnt = genCount;
    interrupts();

    long genDelta = genCnt - lastGenCount;

    float genRPM = (genDelta * 300.0) / GEN_CPR;
    float genAngle = (genCnt * 360.0) / GEN_CPR;

    /* -------- Output -------- */
    Serial.print("OPT | Count: ");
    Serial.print(optCount);
    Serial.print(" RPM: ");
    Serial.print(optRPM, 2);
    Serial.print(" Angle: ");
    Serial.print(optAngle, 2);

    Serial.print("GEN | Count: ");
    Serial.print(genCnt);
    Serial.print(" RPM: ");
    Serial.print(genRPM, 2);
    Serial.print(" Angle: ");
    Serial.println(genAngle, 2);

    lastOptCount = optCount;
    lastGenCount = genCnt;
    lastTime = millis();
  }
}
