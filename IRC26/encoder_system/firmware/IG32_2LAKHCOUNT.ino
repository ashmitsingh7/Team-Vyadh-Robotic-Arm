#include "FastInterruptEncoder.h"

#define ENC_A 25
#define ENC_B 26

// 1× decoding
Encoder enc(ENC_A, ENC_B, SINGLE, 200);

unsigned long lastTime = 0;
long lastCount = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("BOOT OK");

  if (!enc.init()) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    Serial.println("Encoder init FAILED");
    while (1);
  }

  pinMode(ENC_B, INPUT);   // use B for direction

  Serial.println("Encoder init OK");
}

void loop() {
  enc.loop();   // REQUIRED

  if (millis() - lastTime >= 200) {
    long count = enc.getTicks();
    long delta = count - lastCount;

    /* ---- Direction from delta ---- */
    const char *dir;
    if (delta > 0) dir = "CW";
    else if (delta < 0) dir = "CCW";
    else dir = "STOP";

    /* ---- Encoder CPR (1×) ---- */
    const float CPR_1X = 232464.0 / 4.0;

    /* ---- RPM (motor side) ---- */
    float rpm = (delta * 300.0) / CPR_1X;

    /* ---- Output angle (after 30:1 gearbox) ---- */
    float angle_deg = (count * 360.0) / (CPR_1X * 30.0);

    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" | RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Dir: ");
    Serial.print(dir);
    Serial.print(" | Angle (deg): ");
    Serial.println(angle_deg, 2);

    lastCount = count;
    lastTime = millis();
  }

}
