#include "main.h"

ESP32_FAST_PWM *stepper1;
ESP32_FAST_PWM *stepper2;
ESP32_FAST_PWM *stepper3;

// Motion parameters
// https://poivron-robotique.fr/Robot-holonome-localisation-partie-1.html
// 2  Y  1
//    o  X (angle ref X+)
//    3
float theta1 = PI / 6;     // Angle for motor 1 : 30° PI/6
float theta2 = PI * 5 / 6; // Angle for motor 2 : 150° PI*5/6
float theta3 = PI * 3 / 2; // Angle for motor 3 : 270° PI*3/2

OpticalTrackingOdometrySensor otos;

// Timer Settings
static const TickType_t timer_delay_1 = 5 / portTICK_PERIOD_MS; // period of "tic" (ref time for robot motion asserv)
static TimerHandle_t timer_handle_1 = NULL;
static const TickType_t timer_delay_2 = 20 / portTICK_PERIOD_MS; // period of update teleplot
static TimerHandle_t timer_handle_2 = NULL;

// Test variables
float consigne_position = 0;
float consigne_vitesse = 0;
float commande_position = 0;

//******************************************************* TIMER 5ms => ASSERV **************************************************************** */
void timerCallback1(TimerHandle_t xTimer)
{
  // TIMER 1
  // do NON BLOCKING stuff
  otos.Update();
  updateOdometry();
}

//******************************************************* TIMER 20ms **************************************************************** */
void timerCallback2(TimerHandle_t xTimer)
{
  // TIMER 2
  // do NON BLOCKING stuff
  //otos.Teleplot();
}

//******************************************************* SETUP **************************************************************** */
void setup()
{
  ESP32_Helper::Initialisation();
  println("Temperature is : ", temperatureRead());
  println("Robot Holonome Firmware");

  // Sets the two pins as Outputs
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  digitalWrite(dirPinM1, LOW);

  pinMode(stepPinM2, OUTPUT);
  pinMode(dirPinM2, OUTPUT);
  digitalWrite(dirPinM2, LOW);

  pinMode(stepPinM3, OUTPUT);
  pinMode(dirPinM3, OUTPUT);
  digitalWrite(dirPinM3, LOW);

  print(F("\nStarting ESP32_PWM_StepperControl on "));
  println(ARDUINO_BOARD);
  println(ESP32_FAST_PWM_VERSION);

  stepper1 = new ESP32_FAST_PWM(stepPinM1, 500, 0, 2, 8); // pin, frequency = 500 Hz, dutycycle = 0 %, channel, resolution = 8
  if (stepper1)
  {
    stepper1->setPWM();
  }

  stepper2 = new ESP32_FAST_PWM(stepPinM2, 500, 0, 4, 8);
  if (stepper2)
  {
    stepper2->setPWM();
  }

  stepper3 = new ESP32_FAST_PWM(stepPinM3, 500, 0, 6, 8);
  if (stepper3)
  {
    stepper3->setPWM();
  }

  otos.Initialisation();

  // Create a timer
  timer_handle_1 = xTimerCreate(
      "Timer 1",       // Name of timer
      timer_delay_1,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)0,       // Timer ID
      timerCallback1); // Callback function
  xTimerStart(timer_handle_1, portMAX_DELAY);

  timer_handle_2 = xTimerCreate(
      "Timer 2",       // Name of timer
      timer_delay_2,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)1,       // Timer ID
      timerCallback2); // Callback function
  xTimerStart(timer_handle_2, portMAX_DELAY);
}

int x = 1;

//******************************************************* LOOP *****************************************************************/
void loop()
{
  
  setRobotPosition(0, 50, 0); // avance 50mm
  while(consigne_position != 0); // attendre reset consigne quand consigne atteinte

  setRobotPosition(50, 0, 0);
  while(consigne_position != 0);

  setRobotPosition(0, -50, 0);
  while(consigne_position != 0);
  
  setRobotPosition(-50, 0, 0);
  while(consigne_position != 0);
  

  setRobotPosition(0, 0, 60);
  while(consigne_position != 0);

  setRobotPosition(0, 0, -60);
  while(consigne_position != 0);

  if (ESP32_Helper::HasWaitingCommand())
  {
    Command cmd = ESP32_Helper::GetCommand();

    otos.HandleCommand(cmd);
  }
}
//**************************************************************************************************************************/

// Update position and velocity commands according to setpoint and max settings
void updateOdometry() // TODO: changer de nom, c'est pas odometry, voir ancien code
{
  if (commande_position >= consigne_position) // arrêt moteurs et reset consigne quand consigne atteinte
  {
    // stop all motors
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    setMotorSpeed(3, 0);
    commande_position = 0;
    consigne_position = 0;
    consigne_vitesse = 0;
  }
  else
  {
    commande_position += abs(consigne_vitesse); // estimation position cumulée, intégration vitesse
  }
}

// Set the robot center position : linear x and y in mm, angular omega in °
// TODO: coordonnées absolues, implémenter aussi sur y et theta, gérer correctement les abs et sign pour sens de marche...
// TODO: en fait, on est un peu en train de refaire les fonctions MOVE...!
void setRobotPosition(float dx, float dy, float theta)
{
  
  println("dx : ", dx);
  println("dy : ", dy);
  println("theta : ", theta);
  consigne_position = (abs(dx) + abs(dy) + abs(theta)) * UNIT_PER_MM;
  println("consigne_position : ", consigne_position);

  //SetRobotSpeed(dx*SIGN(dx), dy*SIGN(dy), 0); // 50mm/s, avec sens de marche
  SetRobotSpeed(dx, dy, theta);
}

// Set speed in mm/s of one specific motor
void setMotorSpeed(int motor_ID, float speed_mms)
{
  // convert speed in mm/s to frequency in step/s
  float freq = speed_mms * STEP_PER_MM;

  if (motor_ID == 1)
  {
    if (freq == 0)
    {
      // stop motor
      stepper1->setPWM();
    }
    else
    {
      //  Set the frequency of the PWM output and a duty cycle of 50%
      digitalWrite(dirPinM1, (freq > 0));
      stepper1->setPWM(stepPinM1, abs(freq), 50);
    }
  }
  else if (motor_ID == 2)
  {
    if (freq == 0)
    {
      // stop motor
      stepper2->setPWM();
    }
    else
    {
      //  Set the frequency of the PWM output and a duty cycle of 50%
      digitalWrite(dirPinM2, (freq > 0));
      stepper2->setPWM(stepPinM2, abs(freq), 50);
    }
  }
  else if (motor_ID == 3)
  {
    if (freq == 0)
    {
      // stop motor
      stepper3->setPWM();
    }
    else
    {
      //  Set the frequency of the PWM output and a duty cycle of 50%
      digitalWrite(dirPinM3, (freq > 0));
      stepper3->setPWM(stepPinM3, abs(freq), 50);
    }
  }
}

// Set the robot center speeds : linear x and y in mm/s, angular omega in °/s
// https://poivron-robotique.fr/Robot-holonome-lois-de-commande.html
// 2  Y  1
//    o  X
//    3
void SetRobotSpeed(float Vx, float Vy, float omega)
{
  println("Vx : ",Vx);
  println("Vy : ",Vy);
  println("omega : ",omega);
  // calcul consigne vitesse en unité interne robot
  consigne_vitesse = ((Vx+Vy+omega) * UNIT_PER_MM) / TIMER_ASSERV_FREQ;
  println("consigne_vitesse : ",consigne_vitesse);

  // preliminary calculations
  float speed_ang = WHEEL_DISTANCE * radians(omega); // d*ωz
  float speed_vx = Vx / 2;                           // 1/2*Vx
  float speed_vy = SQRT3_2 * Vy;                     // √3/2*Vy

  // Speeds calculations for each motor
  float v1 = speed_vx - speed_vy - speed_ang; // V1 = 1/2*Vx − √3/2*Vy − d*ωz
  float v2 = speed_vx + speed_vy - speed_ang; // V2 = 1/2*Vx + √3/2*Vy − d*ωz
  float v3 = -Vx - speed_ang;                  // V3 = -Vx − d*ωz

  println("v1 : ",v1);
  println("v2 : ",v2);
  println("v3 : ",v3);

  setMotorSpeed(1, v1);
  setMotorSpeed(2, v2);
  setMotorSpeed(3, v3);
}


void functionChrono(int nbrLoop)
{
  unsigned long startChrono = micros();
  for (int i = 0; i < nbrLoop; i++)
  {
    // function or code to loop
  }
  unsigned long endChrono = micros();
  unsigned long deltaChrono = endChrono - startChrono;

  unsigned long chrono = deltaChrono / nbrLoop;
  Serial.print("Chrono from ");
  Serial.print(nbrLoop);
  Serial.print(" loop is : ");
  Serial.print(deltaChrono);
  Serial.print(" µs total or ");
  Serial.print(deltaChrono / 1000);
  Serial.print(" ms total.    ");
  Serial.print(chrono);
  Serial.print(" µs/func or ");
  Serial.print(chrono / 1000);
  Serial.print(" ms/func.");
  Serial.println();
}