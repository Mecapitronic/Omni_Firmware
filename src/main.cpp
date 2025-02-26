#include "main.h"

/****************************************************************************************
 * Variables
 ****************************************************************************************/
LedRGB led_builtin;
LedRGB led_ring;
Motor motor;

Motion linear;
Motion angular;

Robot robot;
//Robot adversaire;

OpticalTrackingOdometrySensor otos;

PoseF goTo = {500, 500, 0}; // TODO: pourquoi parfois il démarre en 0;0 ? c'est réinitialisé à 0 par défaut au lieu de ces valeurs ?

bool mode_test = false;

unsigned long startChrono = 0;
unsigned long endChrono = 0;
unsigned long deltaChrono = 0;
unsigned long teleplotChrono = 0;
unsigned long rgbChrono = 0;
int nbrLoop = 0;

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

  // Init IHM
  pinMode(SWITCH_PIN, INPUT);
  pinMode(TEAM_PIN, INPUT);
  pinMode(BAU_PIN, INPUT);
  pinMode(START_PIN, INPUT);

  mode_test = digitalRead(SWITCH_PIN); // Mode TEST ou OK

  led_builtin.Initialisation(1, RGB_BUILTIN);
  //led_ring.Initialisation(36, WS2812_LED);

  // Init sensors
  otos.Initialisation();

  // Init motors
  motor.Initialisation(Motor::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

  // Init Motion
  linear.Initialisation(1500, 1000);                   // Linear max speed 1500 mm/s and acceleration 1000 mm/s²
  angular.Initialisation(radians(500), radians(1000)); // Angular max speed 500 °/s and acceleration 1000 °/s²
  // Init end position tolerance
  linear.SetMargin(1);            // 1 mm
  angular.SetMargin(radians(1));  // 1 deg

  // Initial pose
  // Init start zone => team color ?
  robot.SetPose(500,500,radians(0));

  // Reset odometry
  otos.SetPose(robot.x, robot.y, robot.h); 

  // Init trajectory
  Trajectory::Initialisation(&linear, &angular, &robot);

  // Init Path Planning
  Mapping::Initialize_Map(TEAM_A);
  Obstacle::Initialize_Obstacle();
  Mapping::Initialize_Passability_Graph();
  
  // Create a timer => for motion
  timer_handle_1 = xTimerCreate(
      "Timer 1",       // Name of timer
      timer_delay_1,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)0,       // Timer ID
      timerCallback1); // Callback function
  xTimerStart(timer_handle_1, portMAX_DELAY);

  EnableTimerMotion();

  // Serial.print("FreeRTOS heap remaining ");Serial.print(xPortGetFreeHeapSize());Serial.println(" bytes");
}

//******************************************************* TIMER 5ms => MOTION **************************************************************** */
//  do NON BLOCKING stuff
void timerCallback1(TimerHandle_t xTimer)
{
  if (TimerMotionIsEnable())
  {
    // Odometry Update
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
      // Calcul des composantes x, y et angular à partir des vitesses des moteurs
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

    // Actual position update
    robot.SetPose(otos.position.x, otos.position.y, otos.position.h);

    Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);

    // Actual velocity update, in global field reference
    linear.velocity_actual = Norm2D(otos.velocity.x, otos.velocity.y);
    angular.velocity_actual = otos.velocity.h;

    // Trajectory update => error update
    Trajectory::Update();

    // Motion update
    linear.Update();
    angular.Update();

    // Motor update => in local robot reference
    motor.Update(linear.velocity_command, linear.direction, angular.velocity_command);
  }
}

//******************************************************* LOOP *****************************************************************/
void loop()
{
  // delay(3000);
  // // Test trajectoires
  //  goTo.x = 500;
  //  goTo.y = 1000; // avance 500
  //  goTo.h = 0;
  //  SetRobotPosition(goTo.x, goTo.y, goTo.h);
  //  delay(5000);
  // //while(linear.isRunning || angular.isRunning) ;//doWhileWaiting();


  startChrono = micros();

  if (startChrono - teleplotChrono > 1000 * 1000) // Update every 100ms
  {
    teleplotChrono = startChrono;
    teleplot("Position", robot);
    teleplot("Orient", robot.h);
    teleplot("Direction", linear.direction);
    //println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");
    otos.Teleplot();
    linear.Teleplot("linear");
    angular.Teleplot("angular");

    teleplot("v1", motor.GetMotorSpeed(1));
    teleplot("v2", motor.GetMotorSpeed(2));
    teleplot("v3", motor.GetMotorSpeed(3));
  }

  if (startChrono - rgbChrono > 1000 * 100) // Update every 100ms
  {
    rgbChrono = startChrono;
    led_builtin.Update();
    //led_ring.Update();
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

    if (cmd.cmd == ("GoToPose") && cmd.size == 3)
    {
      // GoToPose:500;500;0
      // GoToPose:50;50;0
      // GoToPose:0;0;45
      // GoToPose:0;0;0
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = cmd.data[2];
      Trajectory::GoToPose(goTo.x, goTo.y, goTo.h ,linear.speed_max , 0);
      print("Robot go to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
    }
    if (cmd.cmd == ("SetPose") && cmd.size == 3)
    {
      // SetPose:500;500;0
      DisableTimerMotion();
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = cmd.data[2];
      robot.SetPose(goTo.x,goTo.y,goTo.h);
      otos.SetPose(robot.x, robot.y, robot.h);
      print("Robot set to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
      EnableTimerMotion();
    }
    if(cmd.cmd == ("PF") && cmd.size == 1)
    {
      // PathFinding
      // PF:5
      Mapping::Set_End_Vertex(cmd.data[0]);
      if (PathFinding::Path_Planning())
      {
        println("PF Found");
      }
      else
      {
        print("PF Not Found");
      }
    }
    
    if(cmd.cmd == ("VertexList") && cmd.size == 0)
    {
      Mapping::PrintVertexList();
    }
    if(cmd.cmd == ("SegmentList") && cmd.size == 0)
    {
      Mapping::PrintSegmentList();
    }
    if(cmd.cmd == ("CircleList") && cmd.size == 0)
    {
      Mapping::PrintCircleList();
    }
    if(cmd.cmd == ("MappingList") && cmd.size == 0)
    {
      Mapping::PrintVertexList();
      Mapping::PrintSegmentList();
      Mapping::PrintCircleList();
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



