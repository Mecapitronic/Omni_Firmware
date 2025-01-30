#include "main.h"

LedRGB rgb;
MotorController motor;
OpticalTrackingOdometrySensor otos;
t_robot robot;
Motion lin; // linear motion in system unit
Motion ang; // angular motion in system unit => radians
PointF2D goTo = {500, 500, 0};

// Timer Settings
static const TickType_t timer_delay_1 = (1000 / TIMER_ASSERV_FREQ) / portTICK_PERIOD_MS; // period of robot motion asserv = TIMER_ASSERV_FREQ Hz
static TimerHandle_t timer_handle_1 = NULL;
static bool timer_enable_1 = false;

const bool simulation = false;

const int anticipation_mm = 1;
const int anticipation_unit = MM_TO_UNIT(anticipation_mm);
const int anticipation_deg = 1;
const int anticipation_deg_unit = DEG_TO_UNIT(anticipation_deg);

//******************************************************* SETUP **************************************************************** */
void setup()
{
  ESP32_Helper::Initialisation();
  println("Board : ", String(ARDUINO_BOARD));
  print("Arduino Version : ", ESP_ARDUINO_VERSION_MAJOR);
  print(".", ESP_ARDUINO_VERSION_MINOR);
  println(".", ESP_ARDUINO_VERSION_PATCH);
  println("ESP IDF Version : ", String(esp_get_idf_version()));
  // print("ESP IDF Version : ",ESP_IDF_VERSION_MAJOR);  print(".",ESP_IDF_VERSION_MINOR);  println(".",ESP_IDF_VERSION_PATCH);
  println("Temperature : ", temperatureRead(), " deg Celsius");
  println("Frequency CPU : ", getCpuFrequencyMhz(), " MHz");
  println();

  println("Robot Holonome Firmware");

  rgb.Initialisation();
  otos.Initialisation();
  motor.Initialisation(MotorController::OMNIDIRECTIONAL_3_MOTORS);
motor.centerToWheel1 = CENTER_WHEEL_DISTANCE;
  motor.centerToWheel2 = CENTER_WHEEL_DISTANCE;
  motor.centerToWheel3 = CENTER_WHEEL_DISTANCE;

  // Init start zone => team color ?
  robot.location.x = goTo.x;
  robot.location.y = goTo.y;
  robot.orientation = goTo.h;
  otos.SetPosition(robot.location.x, robot.location.y, robot.orientation);

  // Init Motion
  lin.Initialisation(speed_lin_max, accel_lin_max, jerk_lin);
  ang.Initialisation(speed_ang_max, accel_ang_max, jerk_ang);

  SetRobotPosition(goTo.x, goTo.y, goTo.h);

  // Create a timer
  timer_handle_1 = xTimerCreate(
      "Timer 1",       // Name of timer
      timer_delay_1,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)0,       // Timer ID
      timerCallback1); // Callback function
  xTimerStart(timer_handle_1, portMAX_DELAY);

  timer_enable_1 = true;

  // Serial.print("FreeRTOS heap remaining ");Serial.print(xPortGetFreeHeapSize());Serial.println(" bytes");
}

unsigned long startChrono = 0;
unsigned long endChrono = 0;
unsigned long deltaChrono = 0;
unsigned long teleplotChrono = 0;
unsigned long rgbChrono = 0;
int nbrLoop = 0;

//******************************************************* TIMER 5ms => ASSERV **************************************************************** */
void timerCallback1(TimerHandle_t xTimer)
{
  if (timer_enable_1)
  {
    // startChrono = micros();
    //  TIMER 1
    //  do NON BLOCKING stuff
    updateOdometry();
    SetRobotPosition(goTo.x, goTo.y, goTo.h);

    if (lin.position.setpoint > anticipation_unit || lin.position.setpoint < -anticipation_unit)
    {
      // Anti-lock => Restart from the real position if too much speed difference
      //! DO WE NEED IT ?
      // if ((abs(lin.velocity.real - lin.velocity.command) > ANTI_LOCK_SPEED_LIN) )
      //{
      //   resetRamp(&(lin));
      // }
      lin.Update();
    }
    else
    {
      lin.Reset_Ramp();
    }

    if (ang.position.setpoint > anticipation_deg_unit || ang.position.setpoint < -anticipation_deg_unit)
    {
      // Anti-lock => Restart from the real position if too much speed difference
      //! DO WE NEED IT ?
      // if ((abs(ang.velocity.real - ang.velocity.command) > ANTI_LOCK_SPEED_ANG) )
      //{
      //   resetRamp(&(ang));
      // }
      ang.Update();
    }
    else
    {
      ang.Reset_Ramp();
    }

    motor.Update(lin.velocity.command, robot.direction, ang.velocity.command);

    // else
    //{
    //! RESET ASSERV ?
    // motor.Update(0,0,0);
    //}
  }
}

// Odométrie
void updateOdometry()
{
  if (!simulation)
  {
    otos.Update();
  }
  else
  {
        float motor1_speed = motor.GetMotorSpeed(1);
    float motor2_speed = motor.GetMotorSpeed(2);
    float motor3_speed = motor.GetMotorSpeed(3);

    // Simulation Calcul Vitesse OK
    // Calcul des composantes x, y et ang à partir des vitesses des moteurs
    float v_x = (motor1_speed + motor2_speed - 2 * motor3_speed) / 3;
    float v_y = (motor2_speed - motor1_speed) * INV_SQRT3;
    float v_ang = degrees(-(motor1_speed + motor2_speed + motor3_speed) / (3 * CENTER_WHEEL_DISTANCE));

    otos.acceleration.x = v_x - otos.velocity.x;
    otos.acceleration.y = v_y - otos.velocity.y;
    otos.acceleration.h = v_ang - otos.velocity.h;

    otos.velocity.x = v_x;
    otos.velocity.y = v_y;
    otos.velocity.h = v_ang;

    // Mise à jour des positions en fonction des vitesses
    otos.position.x += v_x * timer_delay_1 / 1000;
    otos.position.y += v_y * timer_delay_1 / 1000;
    otos.position.h += v_ang * timer_delay_1 / 1000;
  }

  robot.location.x = otos.position.x;
  robot.location.y = otos.position.y;
  robot.orientation = otos.position.h;

  lin.position.real = sqrtf(robot.location.x * robot.location.x + robot.location.y * robot.location.y);
  ang.position.real = robot.orientation;

  lin.velocity.real = sqrtf(otos.velocity.x * otos.velocity.x + otos.velocity.y * otos.velocity.y);
  ang.velocity.real = otos.velocity.h;
  
  lin.acceleration.real = sqrtf(otos.acceleration.x * otos.acceleration.x + otos.acceleration.y * otos.acceleration.y);
  ang.acceleration.real = otos.acceleration.h;
  }

//******************************************************* LOOP *****************************************************************/

void loop()
{
  startChrono = micros();

  if (startChrono - teleplotChrono > 1000 * 20) // Update every 20ms
  {
    teleplotChrono = startChrono;

    static PointF2D lastPosition = {0, 0, 0};

    lastPosition.x = otos.position.x;
    lastPosition.y = otos.position.y;
    lastPosition.h = otos.position.h;
    teleplot("Position", lastPosition);
    teleplot("Orient", lastPosition.h);
    println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");

    lin.Teleplot("lin");
    // ang.Teleplot("ang");

    teleplot("v1", motor.GetMotorSpeed(1));
    teleplot("v2", motor.GetMotorSpeed(2));
    teleplot("v3", motor.GetMotorSpeed(3));

    // println();
  }

  if (startChrono - rgbChrono > 1000 * 100) // Update every 100ms
  {
    rgbChrono = startChrono;
    rgb.Update();
      }

  if (ESP32_Helper::HasWaitingCommand())
  {
    Command cmd = ESP32_Helper::GetCommand();

    if (cmd.cmd.startsWith("Help"))
    {
      otos.PrintCommandHelp();
      motor.PrintCommandHelp();
    }
    otos.HandleCommand(cmd);
    motor.HandleCommand(cmd);

    if (cmd.cmd == ("RobotPosition") && cmd.size == 3)
    {
      // RobotPosition:510;510;0
      // RobotPosition:50;50;0
      // RobotPosition:0;0;45
      // RobotPosition:0;0;0
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = cmd.data[2];
      print("Robot go to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
    }
  }

  endChrono = micros();
  deltaChrono += endChrono - startChrono;
  nbrLoop++;

  if (nbrLoop == 1)
  {
    // Serial.print(deltaChrono);
    // Serial.println(" µs/func (1)");
  }
  if (nbrLoop >= 1000)
  {
    // Serial.print(deltaChrono/1000);
    // Serial.println(" µs/func (1000)");
    nbrLoop = 0;
    deltaChrono = 0;
  }
}
//**************************************************************************************************************************/

// Set the robot center position : linear x and y in mm, angular omega in °
// TODO: coordonnées absolues, implémenter aussi sur y et theta, gérer correctement les abs et sign pour sens de marche...
// TODO: en fait, on est un peu en train de refaire les fonctions MOVE...!
void SetRobotPosition(float x, float y, float theta)
{
  // println("x : ", x);
  // println("y : ", y);
  // println("theta : ", theta);

  int32 dx = x - robot.location.x;
  int32 dy = y - robot.location.y;
  robot.direction = atan2(dy, dx); // en valeurs relatives ! //! WTF ?
  dx = abs(dx);
  dy = abs(dy);
  lin.position.setpoint = MM_TO_UNIT(Approx_Distance(dx, dy)); // en valeurs absolus !

  // aller en θ => ça c'est l'orientation du robot, différent de la direction de déplacement pour un holonome !
  ang.position.setpoint = DEG_TO_UNIT(theta - robot.orientation); // orientation absolu en degrés = 0°
}
