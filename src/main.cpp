#include "main.h"

LedRGB rgb;
LedRGB ring;
MotorController motor;
OpticalTrackingOdometrySensor otos;
t_robot robot;
Motion lin; // linear motion in system unit
Motion ang; // angular motion in system unit => radians
PointF2D goTo = {500, 500, 0}; // TODO: pourquoi parfois il démarre en 0;0 ? c'est réinitialisé à 0 par défaut au lieu de ces valeurs ?

float last_position_x = 0;
float last_position_y = 0;
float last_position_h = 0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_h = 0;

const bool simulation = false;
bool mode_test = false;

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

  mode_test = digitalRead(SWITCH_PIN); // Mode TEST ou OK

  rgb.Initialisation(1, RGB_BUILTIN);
  //ring.Initialisation(36, WS2812_LED);
  otos.Initialisation();
  motor.Initialisation(MotorController::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

  // Init start zone => team color ?
  goTo.x = 500;
  goTo.y = 500;
  goTo.h = 0;
  robot.position.x = goTo.x;
  robot.position.y = goTo.y;
  robot.orientation = goTo.h;
  otos.SetPosition(robot.position.x, robot.position.y, robot.orientation); 

  // Init Motion
  lin.Initialisation(speed_lin_mms_max, accel_lin_mms2_max, jerk_lin);
  ang.Initialisation(speed_ang_rads_max, accel_ang_rads2_max, jerk_ang);

  lin.SetTolerance(1);          // 1 mm
  ang.SetTolerance(radians(1)); // 1 deg

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

    // TODO: voir la config OTOS pour passer en référentiel robot setSignalProcessConfig ? en fait non !
    robot.position.x = otos.position.x; // mm
    robot.position.y = otos.position.y; // mm
    robot.orientation = otos.position.h; // deg // TODO: ajouter unité dans nom de variable pour éviter erreurs... ex: orientation_rad
    float robot_rad = radians(robot.orientation); 

    //     // Conversion des vitesses du repère robot vers le repère global => OTOS déjà en référentiel global !
    // float vx_global = otos.velocity.x * cosf(robot_rad) - otos.velocity.y * sinf(robot_rad);
    // float vy_global = otos.velocity.x * sinf(robot_rad) + otos.velocity.y * cosf(robot_rad);
    // Calcul de la vitesse linéaire en global
    //lin.velocity.actual = sqrtf(vx_global * vx_global + vy_global * vy_global);

    // FIXME: ne plus utiliser les valeurs de velocity du capteur !! non nulle même immobile, et si on désactive l'accelero, la position sera moins précise.

    lin.velocity.actual = sqrtf(otos.velocity.x * otos.velocity.x + otos.velocity.y * otos.velocity.y); // vitesses dans le référentiel global
    //  velocity_x = (robot.position.x - last_position_x) / dt_asserv;
    //  velocity_y = (robot.position.y - last_position_y) / dt_asserv; 
    //  velocity_h = (robot.orientation - last_position_h) / dt_asserv;
     // On obtient la vitesse en mm / periode d'asserv, il faut convertir en mm/s
     //lin.velocity.actual = sqrtf(velocity_x * velocity_x + velocity_y * velocity_y);
    // TODO:  tester une moyenne glissante de la vitesse actuelle, ou le max des n dernieres valeurs

    last_position_x = robot.position.x;
    last_position_y = robot.position.y;
    last_position_h = robot.orientation;

    ang.velocity.actual = radians(otos.velocity.h);
    //ang.velocity.actual = radians(velocity_h); 


    // Error update
    SetRobotPosition(goTo.x, goTo.y, goTo.h); // TODO: à mettre dans motion update

    // Motion update
    lin.Update();
    ang.Update();
    //lin.velocity.command = lin.position.error; // test sans trapeze
    //ang.velocity.command = ang.position.error;
    
    // *** Découplage de la translation et de la rotation *** => à l'air similaire à robot.direction = atan2(dy, dx) - radians(robot.orientation); 
    // Décomposition des vitesses globales dans le référentiel du terrain
    // lin velocity est toujours positif, et le signe de la commande de vitesse est déjà donné par la direction
    float x_speed_global = lin.velocity.command * cos(robot.direction); // robot.direction est en radians
    float y_speed_global = lin.velocity.command * sin(robot.direction);

    // Projection des translations dans le référentiel du robot
    //float robot_rad = radians(robot.orientation); // robot.orientation est en degrés // TODO: mettre en rad pour uniformiser
    float x_speed = cos(robot_rad) * x_speed_global + sin(robot_rad) * y_speed_global;
    float y_speed = -sin(robot_rad) * x_speed_global + cos(robot_rad) * y_speed_global;

    // Motor update
    motor.Update(x_speed, y_speed, ang.velocity.command); // vitesses dans le référentiel robot
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
  // //while(lin.isRunning || ang.isRunning) ;//doWhileWaiting();
  //  goTo.x = 500;
  //  goTo.y = 500;
  //  goTo.h = 0;
  // // SetRobotPosition(goTo.x, goTo.y, goTo.h);
  //  delay(5000);
  // //while(lin.isRunning || ang.isRunning);// doWhileWaiting();
  // goTo.x = 0;
  // goTo.y = 500;
  // goTo.h = -180;
  // SetRobotPosition(goTo.x, goTo.y, goTo.h);
  // delay(5000);
  // //while(lin.isRunning || ang.isRunning);// doWhileWaiting();
  // goTo.x = 500;
  // goTo.y = 500;
  // goTo.h = 90;
  // SetRobotPosition(goTo.x, goTo.y, goTo.h);
  // delay(5000);
  // //while(lin.isRunning || ang.isRunning);// doWhileWaiting();

  startChrono = micros();

  if (startChrono - teleplotChrono > 1000 * 100) // Update every 20ms
  {
    teleplotChrono = startChrono;
    teleplot("Position", robot.position);
    teleplot("Orient", robot.orientation);
    teleplot("Direction", robot.direction);
    //println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");
    otos.Teleplot();
    lin.Teleplot("lin");
    ang.Teleplot("ang");

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
      // RobotPosition:500;500;0
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
      
      robot.position.x = goTo.x;
      robot.position.y = goTo.y;
      robot.orientation = goTo.h;
      otos.SetPosition(robot.position.x, robot.position.y, robot.orientation);
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

// Set the robot center position //TODO: passer cette fonction dans motion, avec la partie erreur dans update
// Input : linear setpoint is a point x and y in mm, angular setpoint heading in ° => dans le repère du terrain
// Output : linear direction, lin and ang positions errors => dans le repère du robot
void SetRobotPosition(float x_mm, float y_mm, float h_deg)
{
  // Ecarts de position en coordonnées
  float dx = x_mm - robot.position.x;
  float dy = y_mm - robot.position.y;

  // Direction du point cible (Sens du vecteur vitesse lineaire) => dans le repère du robot
  // atan2(dy, dx) : direction à prendre dans le référentiel global terrain (par rapport à l'axe x), ex: déplacement sur +Y => +90°
  // radians(robot.orientation) : orientation du robot dans le référentiel global terrain, ex: robot tourné d'un quart à droite donc sur axe x => -90°
  // robot.direction : direction à prendre dans le référentiel du robot, ex: +90 - -90 = +180° donc déplacement sur -X en réf robot 
  robot.direction = atan2(dy, dx); //- radians(robot.orientation); 

  // Normalisation de l'angle entre [-π, π] 
  while(robot.direction > PI ) robot.direction -= TWO_PI;
  while(robot.direction < -PI ) robot.direction += TWO_PI;

  // Ecart de distance au point cible (Norme du vecteur vitesse lineaire, toujours positif car distance euclidienne)
  lin.position.error = sqrtf(dx*dx + dy*dy);

  // Ecart d'orientation au cap cible (différent de la direction de déplacement pour un holonome)
  ang.position.error = radians(h_deg - robot.orientation);

  // Normalisation de l'angle entre [-π, π] 
  while(ang.position.error > PI ) ang.position.error -= TWO_PI;
  while(ang.position.error < -PI ) ang.position.error += TWO_PI;

}

// Normalisation de l'angle de direction entre [-π, π] 
// Ajout de PI : Décale l'intervalle de normalisation de [−π,π] vers [0,2π], Assure que les valeurs négatives passent en positifs avant le modulo.
// Modulo TWO_PI : Ramène l'angle dans l'intervalle [0,2π], en supprimant les multiples entiers de 2π.
// Soustraction de PI : Ramène l'intervalle de [0,2π] vers [−π,π].
// float NormalizeAngle(float angle)
// {
//     return fmodf(angle + PI, TWO_PI) - PI;
// }

