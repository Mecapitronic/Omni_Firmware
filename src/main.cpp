#include "main.h"

LedRGB rgb;
LedRGB ring;
MotorController motor;
OpticalTrackingOdometrySensor otos;
t_robot robot;
Motion lin; // linear motion in system unit
Motion ang; // angular motion in system unit => radians
PointF2D goTo = {500, 500, 0};

const bool simulation = false;

//******************************************************* SETUP **************************************************************** */
void setup()
{
  ESP32_Helper::Initialisation();
  delay(3000);
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

  pinMode(SWITCH_PIN, INPUT);
  pinMode(TEAM_PIN, INPUT);
  pinMode(BAU_PIN, INPUT);
  pinMode(START_PIN, INPUT);

  rgb.Initialisation(1, RGB_BUILTIN);
  //ring.Initialisation(36, WS2812_LED);
  otos.Initialisation();
  motor.Initialisation(MotorController::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

  // Init start zone => team color ?
  robot.location.x = goTo.x;
  robot.location.y = goTo.y;
  robot.orientation = goTo.h;
  otos.SetPosition(robot.location.x, robot.location.y, robot.orientation);

  // Init Motion
  lin.Initialisation(speed_lin_mms_max, accel_lin_mms2_max, jerk_lin);
  ang.Initialisation(speed_ang_rads_max, accel_ang_rads2_max, jerk_ang);

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
//  do NON BLOCKING stuff
void timerCallback1(TimerHandle_t xTimer)
{
  if (timer_enable_1)
  {
    //Odometrie
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

    robot.location.x = otos.position.x; // mm
    robot.location.y = otos.position.y; // mm
    robot.orientation = otos.position.h; // deg

    lin.position.real = sqrtf(robot.location.x * robot.location.x + robot.location.y * robot.location.y);
    ang.position.real = radians(robot.orientation);

    lin.velocity.real = sqrtf(otos.velocity.x * otos.velocity.x + otos.velocity.y * otos.velocity.y);
    ang.velocity.real = radians(otos.velocity.h);

    //lin.acceleration.real = sqrtf(otos.acceleration.x * otos.acceleration.x + otos.acceleration.y * otos.acceleration.y);
    //ang.acceleration.real = radians(otos.acceleration.h);

    SetRobotPosition(goTo.x, goTo.y, goTo.h);

    if ((lin.position.setpoint > anticipation_mm) || (lin.position.setpoint < -anticipation_mm))
    {
      lin.Update();
    }
    else
    {
      lin.Reset_Ramp();
    }

    if ((ang.position.setpoint > anticipation_deg) || (ang.position.setpoint < -anticipation_deg))
    {
      ang.Update();
    }
    else
    {
      ang.Reset_Ramp();
    }

    motor.Update(lin.velocity.command, robot.direction, ang.velocity.command);
  }
}


//******************************************************* LOOP *****************************************************************/

void loop()
{
  startChrono = micros();

  if (startChrono - teleplotChrono > 1000 * 20) // Update every 20ms
  {
    teleplotChrono = startChrono;
    teleplot("Position", otos.position);
    teleplot("Orient", otos.position.h);
    println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");

    lin.Teleplot("lin");
    // ang.Teleplot("ang");

    teleplot("v1", motor.GetMotorSpeed(1));
    teleplot("v2", motor.GetMotorSpeed(2));
    teleplot("v3", motor.GetMotorSpeed(3));
  }

  if (startChrono - rgbChrono > 1000 * 100) // Update every 100ms
  {
    rgbChrono = startChrono;
    rgb.Update();
    //ring.Update();
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
      SetRobotPosition(goTo.x, goTo.y, goTo.h);
      print("Robot go to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
    }
    if (cmd.cmd == ("SetPosition") && cmd.size == 3)
    {
      // SetPosition:500;500;0
      timer_enable_1 = false;
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = cmd.data[2];
      
      robot.location.x = goTo.x;
      robot.location.y = goTo.y;
      robot.orientation = goTo.h;
      otos.SetPosition(robot.location.x, robot.location.y, robot.orientation);
      print("Robot set to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
      timer_enable_1 = true;
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

// Set the robot center position
// Input : linear x and y in mm, angular heading in °
// Output : Setpoint, direction
void SetRobotPosition(float x_mm, float y_mm, float h_deg)
{
  // println("x : ", x);
  // println("y : ", y);
  // println("theta : ", theta);

  float dx = x_mm - robot.location.x;
  float dy = y_mm - robot.location.y;

  // Sens du vecteur vitesse lineaire
  robot.direction = atan2(dy, dx); // en valeurs relatives ! 

  // Norme du vecteur vitesse lineaire (toujours positif, distance euclidienne)
   lin.position.setpoint = sqrtf(dx*dx + dy*dy); 
  //dx = abs(dx);
  //dy = abs(dy);
  //lin.position.setpoint = MM_TO_UNIT(Approx_Distance(dx, dy)); // en valeurs absolus !

  // aller en θ => ça c'est l'orientation du robot, différent de la direction de déplacement pour un holonome !
  // /!\ peut être négatif, contrairement à lin.position.setpoint
  ang.position.setpoint = radians(h_deg - robot.orientation); // orientation absolu en degrés = 0°
}
