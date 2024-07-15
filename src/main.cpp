#include "main.h"

void setup()
{
  Serial.end();
  Serial.begin(921600);
  delay(1000);
  Serial.println("Robot Holonome Firmware");
  delay(1000);

  // Sets the two pins as Outputs
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(stepPinM2, OUTPUT);
  pinMode(dirPinM2, OUTPUT);
  pinMode(stepPinM3, OUTPUT);
  pinMode(dirPinM3, OUTPUT);
}

void loop()
{

  delay(500);

  digitalWrite(dirPinM1, HIGH);
  digitalWrite(dirPinM2, LOW);

  for (long x = 0; x < 100 * MM_PER_STEP; x++)
  {
    digitalWrite(stepPinM1, HIGH);
    digitalWrite(stepPinM2, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinM1, LOW);
    digitalWrite(stepPinM2, LOW);
    delayMicroseconds(500);
  }
  delay(500);

  digitalWrite(dirPinM1, LOW);
  digitalWrite(dirPinM2, HIGH);

  for (long x = 0; x < 100 * MM_PER_STEP; x++)
  {
    digitalWrite(stepPinM1, HIGH);
    digitalWrite(stepPinM2, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinM1, LOW);
    digitalWrite(stepPinM2, LOW);
    delayMicroseconds(500);
  }
  delay(1000);

  digitalWrite(dirPinM2, HIGH);
  digitalWrite(dirPinM3, LOW);

  for (long x = 0; x < 100 * MM_PER_STEP; x++)
  {
    digitalWrite(stepPinM2, HIGH);
    digitalWrite(stepPinM3, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinM2, LOW);
    digitalWrite(stepPinM3, LOW);
    delayMicroseconds(500);
  }
  delay(500);

  digitalWrite(dirPinM2, LOW);
  digitalWrite(dirPinM3, HIGH);

  for (long x = 0; x < 100 * MM_PER_STEP; x++)
  {
    digitalWrite(stepPinM2, HIGH);
    digitalWrite(stepPinM3, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinM2, LOW);
    digitalWrite(stepPinM3, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
