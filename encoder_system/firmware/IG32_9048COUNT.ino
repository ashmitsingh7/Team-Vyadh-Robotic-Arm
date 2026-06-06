#define ENC_A 4
#define ENC_B 16

volatile long count = 0;
volatile int direction = 0;
volatile uint8_t lastState = 0;

unsigned long lastTime = 0;
long lastCount = 0;

const float CPR = 9048.0;   // counts per motor revolution

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);

  uint8_t state = (a << 1) | b;
  uint8_t transition = (lastState << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      count++;
      direction = +1;
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      count--;
      direction = -1;
      break;
  }

  lastState = state;
}

void setup() {
  Serial.begin(9600);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 100) {   // 100 ms update
    long currentCount;
    int dirCopy;

    noInterrupts();
    currentCount = count;
    dirCopy = direction;
    interrupts();

    long delta = currentCount - lastCount;

    /* ---- RPM (motor side) ---- */
    float rpm = (delta * 600.0) / CPR;

    /* ---- Angle (degrees, motor side) ---- */
    float angle_deg = (currentCount * 360.0) / CPR;

    const char *dirStr;
    if (delta > 0) dirStr = "CW";
    else if (delta < 0) dirStr = "CCW";
    else dirStr = "STOP";

    Serial.print("Count: ");
    Serial.print(currentCount);

    Serial.print(" | RPM: ");
    Serial.print(rpm, 2);

    Serial.print(" | Dir: ");
    Serial.print(dirStr);

    Serial.print(" | Angle (deg): ");
    Serial.println(angle_deg, 2);

    lastCount = currentCount;
    lastTime = now;
  }
}
