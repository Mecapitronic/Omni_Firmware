/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Motion.h"

/****************************************************************************************
 * Initialize setpoint
 ****************************************************************************************/
void Motion::Initialisation(int32 speedMax, int32 accelMax, int32 jerkMax)
{
  jerk.setpoint = jerkMax;
  jerk.command = 0;
  jerk.real = 0;
  jerk.pivot = 0;

  acceleration.setpoint = accelMax;
  acceleration.command = 0;
  acceleration.real = 0;
  acceleration.pivot = 0;

  velocity.setpoint = speedMax;
  velocity.command = 0;
  velocity.real = 0;
  velocity.pivot = 0;

  position.setpoint = 0;
  position.command = 0;
  position.real = 0;
  position.pivot = 0;
}

/****************************************************************************************
 * Filter the motion setpoint to get trapezoidal acceleration command
 ****************************************************************************************/
void Motion::Update()
{
  int speed_final = 0; // vitesse à atteindre à la fin du trapèze, pas forcément nulle !
  int velocity_setpoint = velocity.setpoint;
  int acceleration_setpoint = acceleration.setpoint;

  //? https://wiki.droids-corp.org/articles/a/v/e/Aversive/Modules/Control_system/Filters/Quadramp_derivate.html
  //************************** consigne position => commande vitesse ***************************/
  // Distance de décélération, point de bascule de la vitesse |Vfinal^2-Vreal^2|/2Amax (attention: l'acceleration doit être non nulle !)
  if (acceleration_setpoint != 0)
  {
    position.pivot = abs((speed_final * speed_final) - (velocity.real * velocity.real)) / (2 * acceleration_setpoint);

    if (jerk.setpoint != 0) // normally var set to a const <> 0
    {
      position.pivot += (abs(velocity.real) * acceleration_setpoint) / (2 * jerk.setpoint);
    }
  }
  else
  {
    position.pivot = 0;
  }
  // changement de consigne de vitesse => vitesse finale
  if (abs(position.setpoint - position.command) <= position.pivot)
  {
    velocity_setpoint = speed_final;
  }

  // Sens de la vitesse
  if (position.setpoint < position.command)
  {
    velocity_setpoint = -velocity_setpoint;
  }

  //************************** consigne vitesse => commande acceleration ***************************/
  // Vitesse de "décélération", point de bascule de l'accélération
  if (jerk.setpoint != 0)
  {
    velocity.pivot = abs(acceleration.real * acceleration.real) / (2 * jerk.setpoint);
  }
  // Changement de consigne d'acceleration => acceleration nulle
  if (abs(velocity_setpoint - velocity.command) <= velocity.pivot)
  {
    acceleration_setpoint = 0;
  }

  // Sens de l'acceleration
  if (velocity.setpoint < velocity.command)
  {
    acceleration.setpoint = -acceleration_setpoint;
  }

  // Acceleration = jerk integration
  if (acceleration.command < acceleration_setpoint)
  {
    acceleration.command += jerk.setpoint;
    if (acceleration.command > acceleration_setpoint)
    {
      acceleration.command = acceleration_setpoint;
    }
  }
  else if (acceleration.command > acceleration_setpoint)
  {
    acceleration.command -= jerk.setpoint;
    if (acceleration.command < acceleration_setpoint)
    {
      acceleration.command = acceleration_setpoint;
    }
  }

  // velocity = acceleration integration
  if (velocity.command < velocity_setpoint)
  {
    velocity.command += acceleration.command;
    if (velocity.command > velocity_setpoint)
    {
      velocity.command = velocity_setpoint;
    }
  }
  else if (velocity.command > velocity_setpoint)
  {
    velocity.command -= acceleration.command;
    if (velocity.command < velocity_setpoint)
    {
      velocity.command = velocity_setpoint;
    }
  }

  // Position = velocity integration
  if (position.command < position.setpoint)
  {
    position.command += velocity.command;
    if (position.command > position.setpoint)
    {
      position.command = position.setpoint;
    }
  }
  else if (position.command > position.setpoint)
  {
    position.command += velocity.command; // plus *(1)
    if (position.command < position.setpoint)
    {
      position.command = position.setpoint;
    }
  }
}

/****************************************************************************************
 * Re-initialize command position, speed and acc
 ****************************************************************************************/
void Motion::Reset_Ramp()
{
  position.command = position.real;
  velocity.command = 0;
  acceleration.command = 0;
  jerk.command = 0; // not used
}

/****************************************************************************************
 * Setup position setpoint
 ****************************************************************************************/
void Motion::Setpoint_Position(int32 newPosition)
{
  position.setpoint = newPosition;
}

/****************************************************************************************
 * Return OK if position setpoint is reached
 ****************************************************************************************/
boolean Motion::Check_Position()
{
  return ((position.command == position.setpoint) ? OK : NOK);
}

/****************************************************************************************
 * Teleplot motion
 ****************************************************************************************/

void Motion::Teleplot(String name)
{
  teleplot(name + " pos.setP", position.setpoint);
  teleplot(name + " pos.com", position.command);
  teleplot(name + " pos.real", position.real);
  teleplot(name + " pos.piv", position.pivot);

  teleplot(name + " vel.setP", velocity.setpoint);
  teleplot(name + " vel.com", velocity.command);
  teleplot(name + " vel.real", velocity.real);
  teleplot(name + " vel.piv", velocity.pivot);

  teleplot(name + " acc.setP", acceleration.setpoint);
  teleplot(name + " acc.com", acceleration.command);
  teleplot(name + " acc.real", acceleration.real);
  teleplot(name + " acc.piv", acceleration.pivot);

  //teleplot(name + " jerk.setP",jerk.setpoint);
  //teleplot(name + " jerk.com",jerk.command);
}
