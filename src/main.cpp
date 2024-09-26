#include "main.h"

Stepper stepper;
OpticalTrackingOdometrySensor otos;

// Motion parameters
// https://poivron-robotique.fr/Robot-holonome-localisation-partie-1.html
// 2  Y  1
//    o  X (angle ref X+)
//    3
float theta1 = PI / 6;     // Angle for motor 1 : 30° PI/6
float theta2 = PI * 5 / 6; // Angle for motor 2 : 150° PI*5/6
float theta3 = PI * 3 / 2; // Angle for motor 3 : 270° PI*3/2


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
  println(ARDUINO_BOARD);
  println("Temperature is : ", temperatureRead());
  //println("Frequency  CPU : ", getCpuFrequencyMhz());

  println("Robot Holonome Firmware");

  otos.Initialisation();
  stepper.Initialisation();

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

    if (cmd.cmd.startsWith("Help"))
    {
      otos.PrintCommandHelp();
      stepper.PrintCommandHelp();
    }
    otos.HandleCommand(cmd);
    stepper.HandleCommand(cmd);
  }
}
//**************************************************************************************************************************/

// Update position and velocity commands according to setpoint and max settings
void updateOdometry() // TODO: changer de nom, c'est pas odometry, voir ancien code
{
  if (commande_position >= consigne_position) // arrêt moteurs et reset consigne quand consigne atteinte
  {
    // stop all motors
    stepper.SetMotorsSpeed(0, 0, 0);
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

// Set the robot center linear and angular speeds : 
// - lin_speed_mms : speed of the linear motion, in mm/s
// - lin_direction_rad : angle direction of the linear motion, in radians   [direction => where is moving, trajectory]
// - ang_speed_deg : speed of the angular motion, in °/s                    [orientation => where is facing]
// Polar coordinate for better vectorial control, only one linear PID combining x and y motions.
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