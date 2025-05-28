#include "main.h"

/****************************************************************************************
 * Variables
 ****************************************************************************************/
TimerThread timerMotion;

LedRGB led_ring;
Motor motor;

Motion linear;
Motion angular;

Robot robot;

OpticalTrackingOdometrySensor otos;

void setup()
{
  pinMode(PIN_EN_MCU, OUTPUT);
  digitalWrite(PIN_EN_MCU, LOW);

  ESP32_Helper::Initialisation();
  delay(3000);
  println("Board : ", String(ARDUINO_BOARD));
  print("Arduino Version : ", ESP_ARDUINO_VERSION_MAJOR);
  print(".", ESP_ARDUINO_VERSION_MINOR);
  println(".", ESP_ARDUINO_VERSION_PATCH);
  println("ESP IDF Version : ", String(esp_get_idf_version()));
  println("Temperature : ", temperatureRead(), " deg Celsius");
  println("Frequency CPU : ", getCpuFrequencyMhz(), " MHz");
  println();

  println("Robot Holonome Firmware");

  // Init IHM
  IHM::InitIHM();

  ServoAX12::Initialisation();

  led_ring.Initialisation(&robot);

  Lidar::Initialisation(&robot);

  // Init sensors
  otos.Initialisation(simulation);
  if (!otos.IsConnected() && !simulation)
  {
    // error = true;
  }

  // Init motors
  motor.Initialisation(Motor::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

  // Init Motion
  // Linear max speed 1500 mm/s and acceleration 1000 mm/s²
  linear.Initialisation(1500, 1000);
  // Angular max speed 500 °/s and acceleration 1000 °/s²
  angular.Initialisation(radians(500), radians(1000));
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
  Mapping::Initialize_Map(IHM::team);
  Obstacle::Initialize_Obstacle();
  Mapping::Initialize_Passability_Graph();
  Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
  Mapping::Update_Passability_Graph();

  // Create a timer => for motion
  timerMotion = TimerThread(timerMotionCallback, "Timer Motion",
                            (1000 * Motion::dt_motion) / portTICK_PERIOD_MS);
  timerMotion.Start();

  // Put at least the 1 Tick delay, this is needed so the watchdog doesn't trigger
  TaskThread(TaskTeleplot, "TaskTeleplot", 10000, 10, 0);
  TaskThread(TaskUpdate, "TaskUpdate", 10000, 15, 1);
  TaskThread(TaskHandleCommand, "TaskHandleCommand", 20000, 5, 1);
  TaskThread(TaskMatch, "TaskMatch", 20000, 10, 1);

  // Send to PC all the mapping data
  ESP32_Helper::HandleCommand(Command("UpdateMapping"));
}

void loop()
{
  // HACK Vérifier qu'on n'utilise pas les serialEvent !!!
  // C:\Users\xxx\.platformio\packages\framework-arduinoespressif32\cores\esp32\main.cpp
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/main.cpp
  vTaskDelete(NULL); // Supprime immédiatement le task Arduino "loop"
}

// TIMER 5ms => MOTION
// do NON BLOCKING stuff
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
      // Calcul des composantes x, y et angular à partir des vitesses des
      // moteurs
      float v_x = (motor1_speed + motor2_speed - 2 * motor3_speed) / 3;
      float v_y = (motor2_speed - motor1_speed) * INV_SQRT3;
      float v_ang = (-(motor1_speed + motor2_speed + motor3_speed) /
                     (3 * CENTER_WHEEL_DISTANCE));

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

    // Actual velocity update, in global field reference
    linear.velocity_actual = Norm2D(otos.velocity.x, otos.velocity.y);
    angular.velocity_actual = otos.velocity.h;

    // Trajectory update => error update
    Trajectory::Update();

    // Motion update
    linear.Update();
    angular.Update();

    // Motor update => in local robot reference
    motor.Update(linear.velocity_command, linear.direction,
                 angular.velocity_command);
  }

  timerMotion.Running(false);
}

void TaskTeleplot(void *pvParameters)
{
  int lastMatchTime = 0;
  println("Start TaskTeleplot");
  Timeout robotPosTimeOut, mapTimeOut;
  robotPosTimeOut.Start(100);
  mapTimeOut.Start(200);
  Chrono chrono("Teleplot", 1000);

  while (true)
  {
    chrono.Start();
    if (robotPosTimeOut.IsTimeOut())
    {
      teleplot("Position", robot);
      teleplot("Orient", degrees(robot.h));

      // Countdown
      if (lastMatchTime != (int)(Match::getMatchTimeSec()))
      {
        println("Match Time : ", (int)(Match::getMatchTimeSec()));
        lastMatchTime = (int)(Match::getMatchTimeSec());
      }
      // teleplot("Direction", linear.direction)
      // println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");
      // otos.Teleplot();
      // linear.Teleplot("linear");
      // angular.Teleplot("angular");

      // teleplot("v1", motor.GetMotorSpeed(1));
      // teleplot("v2", motor.GetMotorSpeed(2));
      // teleplot("v3", motor.GetMotorSpeed(3));
      // teleplot("Servo_Up Pos", ServoAX12::Servo_Up.position);
      // teleplot("Servo_Left Pos", ServoAX12::Servo_Left.position);
      // teleplot("Servo_Up Cmd", ServoAX12::Servo_Up.command_position);
      // teleplot("Servo_Left Cmd", ServoAX12::Servo_Left.command_position);
    }

    if (mapTimeOut.IsTimeOut())
    {
      Obstacle::PrintObstacleList();
      ServoAX12::TeleplotPosition();
    }
    if (chrono.Check())
    {
      // println("Chrono " + chrono.name + " : ", chrono.elapsedTime / chrono.loopNbr, "
      // µs/loop");
    }
    vTaskDelay(10);
  }
}

void TaskUpdate(void *pvParameters)
{
  println("Start TaskUpdate");
  Chrono chrono("Update", 1000);
  while (true)
  {
    chrono.Start();
    Match::updateMatch();
    IHM::UpdateBAU();
    IHM::Blink();
    led_ring.update();

    // take some time to update the servo, maybe move it elsewhere
    ServoAX12::Update();
    // functionChrono(1);
    //  functionChrono(1000);
    //   delay(3000);
    //   // Test trajectoires
    //    goTo.x = 500;
    //    goTo.y = 1000; // avance 500
    //    goTo.h = 0;
    //    SetRobotPosition(goTo.x, goTo.y, goTo.h);
    //    delay(5000);
    //   //while(linear.isRunning || angular.isRunning) ;//doWhileWaiting();

    // Enable or disable Communication
    if (IHM::switchMode == 0 && (Printer::IsEnable())) // || Wifi_Helper::IsEnable()))
    {
      println("Disable Com");
      Printer::EnablePrinter(Enable::ENABLE_FALSE);
      Wifi_Helper::EnableWifi(Enable::ENABLE_FALSE);
      Printer::teleplotUDPEnable = Enable::ENABLE_FALSE;
      Lidar::disableComLidar = true;
    }
    else if (IHM::switchMode == 1 && (!Printer::IsEnable())) // || !Wifi_Helper::IsEnable()))
    {
      Printer::EnablePrinter(Enable::ENABLE_TRUE);
      Wifi_Helper::EnableWifi(Enable::ENABLE_TRUE);
      Printer::teleplotUDPEnable = Enable::ENABLE_TRUE;
      println("Enable Com");
      Lidar::enableComLidar = true;
    }
    if (chrono.Check())
    {
      // println("Chrono " + chrono.name + " : ", chrono.elapsedTime / chrono.loopNbr, "
      // µs/loop");
    }
    vTaskDelay(10);
  }
}

void TaskHandleCommand(void *pvParameters)
{
  println("Start TaskHandleCommand");
  Chrono chrono("HandleCommand", 1000);
  while (true)
  {
    chrono.Start();
    if (ESP32_Helper::HasWaitingCommand())
    {
      Command cmd = ESP32_Helper::GetCommand();

      otos.HandleCommand(cmd);
      motor.HandleCommand(cmd);
      if (cmd.cmd.startsWith("AX12"))
      {
        ServoAX12::HandleCommand(cmd);
      }

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
        Pose goTo = Pose(cmd.data[0], cmd.data[1], radians(cmd.data[2]));
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
        Pose goTo = Pose(cmd.data[0], cmd.data[1], radians(cmd.data[2]));
        print("Robot set to x=", goTo.x);
        print(" y=", goTo.y);
        print(" h=", goTo.h);
        println();
        robot.SetPose(goTo.x, goTo.y, goTo.h);
        otos.SetPose(robot.x, robot.y, robot.h);
        Trajectory::Reset();
        timerMotion.Enable();
      }
      else if (cmd.cmd == "TrajReset")
      {
        Trajectory::Reset();
      }
      else if (cmd.cmd == "UpdateMapping")
      {
        Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
        Mapping::Update_Passability_Graph();
        Mapping::PrintVertexList();
        Mapping::PrintSegmentList();
        Mapping::PrintCircleList();
        Obstacle::PrintObstacleList();
        println("RobotRadius:", ROBOT_RADIUS);
        println("RobotMargin:", ROBOT_MARGIN);
      }
      else if (cmd.cmd == "PF")
      {
        bool result = false;
        // PathFinding
        // PF:5
        // PF:500:1000:5
        if (cmd.size == 1)
        {
          result = PathFinding::PathFinding((int16_t)robot.x, (int16_t)robot.y,
                                            cmd.data[0]);
        }
        else if (cmd.size == 3)
        {
          result =
              PathFinding::PathFinding(cmd.data[0], cmd.data[1], cmd.data[2]);
        }
        if (result)
        {
          println("PF Found");
        }
        else
        {
          print("PF Not Found");
        }
      }
      else if (cmd.cmd == "Nav" && cmd.size == 1)
      {
        Trajectory::Navigate_To_Vertex(cmd.data[0], linear.speed_max, 0);
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
      else if (cmd.cmd == "ObstacleList")
      {
        Obstacle::PrintObstacleList();
      }
      else if (cmd.cmd == "AddObs" && cmd.size == 3)
      {
        // AddObs:0:500:1000
        int num = cmd.data[0];
        Point p;
        p.x = cmd.data[1];
        p.y = cmd.data[2];
        // Obstacle::queueObstacle.Send(Circle(p, num));
        Obstacle::Add_Obstacle(num, p);
        Mapping::Update_Passability_Obstacle();
        Obstacle::PrintObstacleList();
      }
      else if (cmd.cmd == "RemoveObstacle" && cmd.size == 1)
      {
        int num = cmd.data[0];
        // Obstacle::queueObstacle.Send(Circle(0, 0, num));
        Obstacle::Add_Obstacle(num, {0, 0});
        Mapping::Update_Passability_Obstacle();
        Obstacle::PrintObstacleList();
      }
    }
    if (chrono.Check())
    {
      // println("Chrono " + chrono.name + " : ", chrono.elapsedTime / chrono.loopNbr, "
      // µs/loop");
    }
    vTaskDelay(10); // Allow other tasks to run
  }
}

void TaskMatch(void *pvParameters)
{
  println("Start TaskMatch");
  Chrono chrono("Match", 1000);
  while (true)
  {
    chrono.Start();
    // Attente du démarrage du match par la tirette
    if (Match::matchState == State::MATCH_WAIT)
    {
      IHM::UpdateHMI();
      Mapping::Initialize_Map(IHM::team);
      // Disable Motor & Servo Power in Match mode during waiting
      if (Match::matchMode == Enable::ENABLE_TRUE)
      {
        digitalWrite(PIN_EN_MCU, LOW);
        timerMotion.WaitForDisable();
        Mapping::Initialize_Map(IHM::team);
        Point p = Mapping::Get_Vertex_Point(1);
        robot.SetPose(p.x, p.y, 0);
        otos.SetPose(p.x, p.y, 0);
        Trajectory::Reset();
        timerMotion.Enable();
      }
      else
        digitalWrite(PIN_EN_MCU, HIGH);
    }

    // Match en cours
    if (Match::matchState == State::MATCH_BEGIN)
    {
      timerMotion.WaitForDisable();
      Mapping::Initialize_Map(IHM::team);
      Point p = Mapping::Get_Vertex_Point(1);
      robot.SetPose(p.x, p.y, 0);
      otos.SetPose(p.x, p.y, 0);
      Trajectory::Reset();
      timerMotion.Enable();

      Match::matchState = State::MATCH_RUN;
      Match::printMatch();
      // Enable Motor & Servo Power
      digitalWrite(PIN_EN_MCU, HIGH);
      ServoAX12::Bas();
      ServoAX12::Prise();
    }

    // Démarrage du robot
    if (Match::matchState == State::MATCH_RUN)
    {
      // Enable Motor & Servo Power
      digitalWrite(PIN_EN_MCU, HIGH);

      Trajectory::Navigate_To_Vertex(6, linear.speed_max, 0);
      Point p = Mapping::Get_Vertex_Point(6);
      Trajectory::TranslateToPosition(p.x, p.y + 140, linear.speed_max, 0);
      Trajectory::Navigate_To_Vertex(1, linear.speed_max, 0);
      Trajectory::RotateToOrientation(radians(180), 150, 0);
      ServoAX12::Depose();
      delay(1000); // Attente de 1 sec pour la dépose

      Trajectory::Navigate_To_Vertex(10, linear.speed_max, 0);
      Trajectory::Navigate_To_Vertex(11, linear.speed_max, 0);

      Match::stopMatch();
      // TODO 5 sec avant la fin du match aller dans la zone de back stage
    }

    // Arrêt du robot
    if (Match::matchState == State::MATCH_STOP)
    {
      // Wait for end of match
    }

    // Fin du match
    if (Match::matchState == State::MATCH_END)
    {
      // Disable Motor & Servo Power
      digitalWrite(PIN_EN_MCU, LOW);
      IHM::useBlink = false;
      // Disable Motion timer
      timerMotion.WaitForDisable();
      motor.Update(0, 0, 0);
      ServoAX12::StopAllServo();
    }
    if (chrono.Check())
    {
      // println("Chrono " + chrono.name + " : ", chrono.elapsedTime / chrono.loopNbr, "
      // µs/loop");
    }
    vTaskDelay(10);
  }
}

//**************************************************************************************************************************/
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
  print("Chrono from ", nbrLoop, " loop");
  print(" is : ", deltaChrono, " µs total");
  print(" = ", deltaChrono / 1000, " ms total.");
  print(" or ", chrono, " µs/func ");
  print(" = ", chrono / 1000, " ms/func.");
  println();
}
