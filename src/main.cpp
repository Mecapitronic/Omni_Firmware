#include "main.h"

megaAVR_PWM* stepper;

void setSpeed(int speed)
{
  if (speed == 0)
  {
    // Use DC = 0 to stop stepper
    stepper->setPWM(STEP_PIN, 1000, 0);
  }
  else
  {
    //  Set the frequency of the PWM output and a duty cycle of 50%
    digitalWrite(DIR_PIN, (speed < 0));
    stepper->setPWM(STEP_PIN, abs(speed), 50);
  }
}

void setup()
{
  Serial.end();
  Serial.begin(921600);
  delay(1000);
  Serial.println("Robot Holonome Firmware");
  delay(1000);

  double theta = 150 * PI / 180;      // Angle Ref X+
  double theta1 = theta + 0;          // Angle for motor 1
  double theta2 = theta + 2 * PI / 3; // Angle for motor 2 (120 degrees in radians)
  double theta3 = theta + 4 * PI / 3; // Angle for motor 3 (240 degrees in radians)

  // Sets the two pins as Outputs
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(stepPinM2, OUTPUT);
  pinMode(dirPinM2, OUTPUT);
  pinMode(stepPinM3, OUTPUT);
  pinMode(dirPinM3, OUTPUT);
  
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.print(F("\nStarting PWM_StepperControl on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_PWM_VERSION);
  
  // Create PWM object and passed just a random frequency of 500
  // The duty cycle is how you turn the motor on and off
  stepper = new megaAVR_PWM(STEP_PIN, 500, 0);
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


  for (long x = 0; x <= 5000; x+=10)
  {

    setSpeed(x);
    delay(2);
  }
  for (long x = 5000; x >= 0; x-=10)
  {

    setSpeed(x);
    delay(2);
  }

    setSpeed(0);
/*
  setSpeed(2000);
  delay(3000);

  // Stop before reversing
  setSpeed(0);
  delay(3000);

  // Reversing
  setSpeed(-5000);
  delay(3000);

  // Stop before reversing
  setSpeed(0);
  delay(3000);
*/
}
