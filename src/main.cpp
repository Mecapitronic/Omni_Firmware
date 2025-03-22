#include "main.h"

/****************************************************************************************
 * Variables
 ****************************************************************************************/
TimerThread timerMotion;

LedRGB led_builtin;
LedRGB led_ring;
Motor motor;

Motion linear;
Motion angular;

Robot robot;
// Robot adversaire;

OpticalTrackingOdometrySensor otos;

Team team;
Mode mode;

PoseF goTo = {500, 500, 0}; // TODO: pourquoi parfois il démarre en 0;0 ? c'est réinitialisé à 0 par défaut au lieu de ces valeurs ?

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
  pinMode(PIN_MODE, INPUT);
  pinMode(PIN_TEAM, INPUT);
  pinMode(PIN_BAU, INPUT);
  pinMode(PIN_START, INPUT);

  // TODO : boucle de lecture dans la loop de démarrage
  digitalRead(PIN_MODE) == LOW ? mode = Mode::Match : mode = Mode::Test;
  digitalRead(PIN_TEAM) == LOW ? team = Team::Jaune : team = Team::Bleue;

  led_builtin.Initialisation(1, PIN_RGB_LED);
  // led_ring.Initialisation(36, PIN_WS2812_LED);

  // Init sensors
  otos.Initialisation();

  // Init motors
  motor.Initialisation(Motor::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

  // Init Motion
  linear.Initialisation(1500, 1000);                   // Linear max speed 1500 mm/s and acceleration 1000 mm/s²
  angular.Initialisation(radians(500), radians(1000)); // Angular max speed 500 °/s and acceleration 1000 °/s²
  // Init end position tolerance
  linear.SetMargin(1);           // 1 mm
  angular.SetMargin(radians(1)); // 1 deg

  // Initial pose
  // Init start zone => team color ?
  robot.SetPose(500, 500, radians(0));

  // Reset odometry
  otos.SetPose(robot.x, robot.y, robot.h);

  // Init trajectory
  Trajectory::Initialisation(&linear, &angular, &robot);

  // Init Path Planning
  Mapping::Initialize_Map(team);
  Obstacle::Initialize_Obstacle();
  Mapping::Initialize_Passability_Graph();
  Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
  Mapping::Update_Passability_Graph();
  
  // Create a timer => for motion
  timerMotion = TimerThread(timerMotionCallback, "Timer Motion", (1000 * Motion::dt_motion) / portTICK_PERIOD_MS);
  timerMotion.Start();

  TaskThread Task1 = TaskThread(TaskLidar, "TaskLidar", 20000, 1, 1);

  // Serial.print("FreeRTOS heap remaining ");Serial.print(xPortGetFreeHeapSize());Serial.println(" bytes");
}

//******************************************************* TIMER 5ms => MOTION **************************************************************** */
//  do NON BLOCKING stuff
void timerMotionCallback(TimerHandle_t xTimer)
{
  if (timerMotion.IsEnable())
  {
    timerMotion.Running(true);
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
      float v_ang = (-(motor1_speed + motor2_speed + motor3_speed) / (3 * CENTER_WHEEL_DISTANCE));

      otos.acceleration.x = v_x - otos.velocity.x;
      otos.acceleration.y = v_y - otos.velocity.y;
      otos.acceleration.h = v_ang - otos.velocity.h;

      otos.velocity.x = v_x;
      otos.velocity.y = v_y;
      otos.velocity.h = v_ang;

      // Mise à jour des positions en fonction des vitesses
      otos.position.x += v_x * timerMotion.Period() / 1000;
      otos.position.y += v_y * timerMotion.Period() / 1000;
      otos.position.h += v_ang * timerMotion.Period() / 1000;
    }

    // Actual position update
    robot.SetPose(otos.position.x, otos.position.y, otos.position.h);

    //Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
    //Mapping::Update_Passability_Graph();

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
  
  timerMotion.Running(false);
}

//******************************************************* TASK => LIDAR *************************************************************** */
//  do NON BLOCKING stuff
void TaskLidar(void *pvParameters)
{
  Serial.println("Start TaskLidar");
  SERIAL_LIDAR.begin(230400, SERIAL_8N1, RX1, TX1);

  unsigned char trame[7];
  uint16_t cursor = 0;

  while (1)
  {
    PoseF p = robot.GetPoseF();
    // Starting char : '!'
    SERIAL_LIDAR.write(0x21);

    // Robot X
    SERIAL_LIDAR.write((int)p.x % 256);
    SERIAL_LIDAR.write((int)p.x >> 8);

    // Robot Y
    SERIAL_LIDAR.write((int)p.y % 256);
    SERIAL_LIDAR.write((int)p.y >> 8);

    // Robot Angle * 100
    int angle = (int)(degrees(p.h) * 100);
    SERIAL_LIDAR.write(angle % 256);
    SERIAL_LIDAR.write(angle >> 8);

    // Ending char : '\n'
    SERIAL_LIDAR.write(0x0A);
    //println("Lidar sent : ", p);

    while (SERIAL_LIDAR.available())
    {
      char data = SERIAL_LIDAR.read();

      if (data == 0x21 && cursor == 0)
      {
        trame[cursor++] = data;
      }
      else if (cursor > 0)
      {
        trame[cursor++] = data;
        if (cursor >= 7)
        {
          if (data == 0x0A)
          {
            Point p;
            int header = trame[0];
            int num = trame[1];
            p.x = trame[3] << 8 | trame[2];
            p.y = trame[5] << 8 | trame[4];
            int footer = trame[6];
            // if(num>=0 && num<MAX_OBSTACLE && IsInMap(p))
            //     Add_Obstacle_Cart(num, p.x, p.y);
            if (p.x != 0 && p.y != 0)
            {
              print("Lidar received : ", num);
              println(" ", p);
            }
            cursor = 0;
            // break;
          }
          cursor = 0;
        }
      }
    }
    vTaskDelay(100);
  }
  Serial.println("End TaskLidar");
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
    teleplot("Orient", degrees(robot.h));
    Mapping::PrintVertex0();
    //// teleplot("Direction", linear.direction);
    // println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");
    //// otos.Teleplot();
    //// linear.Teleplot("linear");
    //// angular.Teleplot("angular");

    //// teleplot("v1", motor.GetMotorSpeed(1));
    //// teleplot("v2", motor.GetMotorSpeed(2));
    //// teleplot("v3", motor.GetMotorSpeed(3));
  }

  if (startChrono - rgbChrono > 1000 * 100) // Update every 100ms
  {
    rgbChrono = startChrono;
    led_builtin.Update();
    // led_ring.Update();
  }

  if (ESP32_Helper::HasWaitingCommand())
  {
    Command cmd = ESP32_Helper::GetCommand();

    otos.HandleCommand(cmd);
    motor.HandleCommand(cmd);

    if (cmd.cmd == "Help")
    {
      otos.PrintCommandHelp();
      motor.PrintCommandHelp();
    }
    else if (cmd.cmd == "GoToPose" && cmd.size == 3)
    {
      // GoToPose:500;500;90
      // GoToPose:500;500;0
      // GoToPose:50;50;0
      // GoToPose:0;0;45
      // GoToPose:0;0;0
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = radians(cmd.data[2]);
      print("Robot go to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
      Trajectory::GoToPose(goTo.x, goTo.y, goTo.h, linear.speed_max, 0);
    }
    else if (cmd.cmd == "SetPose" && cmd.size == 3)
    {
      // SetPose:2000:200:9000
      // SetPose:500;500;0
      // SetPose:1500;1000;0
      timerMotion.WaitForDisable();
      goTo.x = cmd.data[0];
      goTo.y = cmd.data[1];
      goTo.h = radians(cmd.data[2]);
      print("Robot set to x=", goTo.x);
      print(" y=", goTo.y);
      print(" h=", goTo.h);
      println();
      robot.SetPose(goTo.x, goTo.y, goTo.h);
      otos.SetPose(robot.x, robot.y, robot.h);
      timerMotion.Enable();
    }
    else if (cmd.cmd == "PF" && cmd.size == 1)
    {
      // PathFinding
      // PF:5
      Mapping::Set_End_Vertex(cmd.data[0]);
      Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
      Mapping::Update_Passability_Graph();

      if (PathFinding::Path_Planning())
      {
        println("PF Found");
      }
      else
      {
        print("PF Not Found");
      }
    }
    else if (cmd.cmd == "VertexList")
    {
      Mapping::PrintVertexList();
    }
    else if (cmd.cmd == "SegmentList")
    {
      Mapping::PrintSegmentList();
    }
    else if (cmd.cmd == "CircleList")
    {
      Mapping::PrintCircleList();
    }
    else if (cmd.cmd == "MappingList")
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
