#include "FastInterruptEncoder.h"

/* =========================================================
   OPTICAL ENCODER — FastInterruptEncoder
   Encoder + gearbox CPR = 232,464
   Library already handles decoding → DO NOT divide by 4
   Pins: 25 (A), 26 (B)
   ========================================================= */
#define OPT_A 25
#define OPT_B 26

Encoder optEnc(OPT_A, OPT_B, SINGLE, 200);
const float OPT_CPR = 232464.0 / 4.0;
  // FIXED: no /4

/* =========================================================
   MAGNETIC ENCODER — Manual Quadrature Decode
   Encoder CPR = 9,048 (already full quadrature)
   Pins: 21 (A), 19 (B)
   ========================================================= */
#define MAG_A 21
#define MAG_B 19

volatile long magCount = 0;
volatile uint8_t magLastState = 0;
const float MAG_CPR = 9048.0;

/* Quadrature ISR for magnetic encoder */
void IRAM_ATTR magneticISR() {
  uint8_t a = digitalRead(MAG_A);
  uint8_t b = digitalRead(MAG_B);

  uint8_t state = (a << 1) | b;
  uint8_t transition = (magLastState << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      magCount++;
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      magCount--;
      break;
  }

  magLastState = state;
}

/* =========================================================
   AS5600 — Absolute Magnetic Encoder (Analog Output)
   Pin: GPIO 34 (ADC1)
   ADC: 0–4095 → 0–360°
   ========================================================= */
#define AS5600_OUT 34

/* =========================================================
   SETUP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  delay(500);

  // Optical encoder init
  optEnc.init();

  // Magnetic encoder init
  pinMode(MAG_A, INPUT_PULLUP);
  pinMode(MAG_B, INPUT_PULLUP);
  magLastState = (digitalRead(MAG_A) << 1) | digitalRead(MAG_B);
  attachInterrupt(digitalPinToInterrupt(MAG_A), magneticISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MAG_B), magneticISR, CHANGE);

  // AS5600 ADC setup
  analogReadResolution(12); // 0–4095
}

/* =========================================================
   LOOP — Synchronized angle sampling & printing
   ========================================================= */
void loop() {

  // Always service FastInterruptEncoder
  optEnc.loop();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {

    /* ---------- Optical encoder angle ---------- */
    long optCount = optEnc.getTicks();
    float optAngle =
      (optCount * 360.0) / (OPT_CPR * 30.0);

    /* ---------- Magnetic encoder angle ---------- */
    long magCountSnapshot;
    noInterrupts();
    magCountSnapshot = magCount;
    interrupts();

    float magAngle =
      (magCountSnapshot * 360.0) / (MAG_CPR * 30.0);

    /* ---------- AS5600 angle ---------- */
    int adc = analogRead(AS5600_OUT);
    float as5600Angle =
      (adc * 360.0) / 4095.0;

    /* ---------- Single clean print ---------- */
    Serial.print("Optical: ");
    Serial.print(optAngle, 2);
    Serial.print(" deg | Magnetic: ");
    Serial.print(magAngle, 2);
    Serial.print(" deg | AS5600: ");
    Serial.print(as5600Angle, 2);
    Serial.println(" deg");

    lastPrint = millis();
  }
}
