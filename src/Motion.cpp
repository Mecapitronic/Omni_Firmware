/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Motion.h"

/****************************************************************************************
 * Initialize setpoint
 ****************************************************************************************/
void Motion::Initialisation(float speedMax, float accelMax, float jerkMax)
{
  isRunning = false;

  jerk.setpoint = 0;
  jerk.error = 0;
  jerk.command = 0;
  jerk.actual = 0;
  jerk.deceleration = 0;
  jerk.max = jerkMax;

  acceleration.setpoint = accelMax;
  acceleration.error = 0;
  acceleration.command = 0;
  acceleration.actual = 0;
  acceleration.deceleration = 0;
  acceleration.max = accelMax;

  velocity.setpoint = 0;
  velocity.error = 0;
  velocity.command = 0;
  velocity.actual = 0;
  velocity.deceleration = 0;
  velocity.max = speedMax;

  position.setpoint = 0;
  position.error = 0;
  position.command = 0;
  position.actual = 0;
  position.deceleration = 0;
  position.max = 0;
}

/****************************************************************************************
 * Adaptative trapezoidal controller //TODO: fonction plus générique avec x, dérivée et dérivée seconde
 // ref : https://poivron-robotique.fr/Consigne-de-vitesse.html
 ****************************************************************************************/
void Motion::Update()
{
  isRunning = true;
  float speed_final = 0; // vitesse à atteindre à la fin du trapèze, pas forcément nulle !
  //acceleration.setpoint = acceleration.max; // peut être simplifier, si l'accel ne varie pas
  //float position_setpoint = position.setpoint; // distance restante
  //float velocity_setpoint = velocity.setpoint;
  //float acceleration_setpoint = acceleration.setpoint;

  // Vérification de l'arrivée
  if (fabsf(position.error) >= tolerance)
  {/*
      // Étape 1 : Calcul de la distance de décélération
      // Distance de décélération, point de bascule de la vitesse |Vreal^2-Vfinal^2|/(2*accel_max) (attention: l'acceleration doit être non nulle !)
      if (acceleration.max != 0)
      {
        position.deceleration = fabsf((velocity.actual * velocity.actual) - (speed_final * speed_final)) / (2 * acceleration.max); // toujours positif

        //if (jerk.setpoint != 0) // normally var set to a const <> 0
        //{
        // position.deceleration += (abs(velocity.actual) * acceleration_setpoint) / (2 * jerk.setpoint);
        //}
      }
      else
      {
        position.deceleration = INFINITY; // théoriquement = l'infinie
      }

      // Étape 2 : Calcul de la vitesse cible
      if (fabsf(position.error) < position.deceleration) // comparaison en distance absolue (norme du vecteur)
      {
        // Décélération nécessaire : consigne vitesse = sqrt(Vfinal^2 + 2 * acceleration_max * distance_restante)
        velocity.setpoint = sqrtf((speed_final * speed_final) + 2 * acceleration.max * fabsf(position.error)); // toujours positif
        //velocity.setpoint = speed_final;
      }
      else
      {
        // Accélération ou vitesse constante : consigne = max
        velocity.setpoint = velocity.max; // toujours positif
      }*/
      // formule simplifiée 
      velocity.setpoint = fmin(sqrtf((speed_final * speed_final) + 2 * acceleration.max * fabsf(position.error)), velocity.max);

      // la consigne de vitesse prend le même signe que l'erreur de position, pour compenser dans le bon sens
      velocity.setpoint = copysignf(velocity.setpoint, position.error); 

      // Étape 3 : Ajustement / lissage de la vitesse / intégration de l'accélération
      // on va dans la direction de la consigne dans tous les cas
      if (velocity.actual < velocity.setpoint)
      {
        // Augmentation de la vitesse
        //velocity.command = fmin(velocity.actual + (acceleration.max * dt_asserv), velocity.setpoint);
        velocity.command = fmin(velocity.command + (acceleration.max * dt_asserv), velocity.setpoint);
      }
      else
      {
        // Réduction de la vitesse
        //velocity.command = fmax(velocity.actual - (acceleration.max * dt_asserv), velocity.setpoint);
        velocity.command = fmax(velocity.command - (acceleration.max * dt_asserv), velocity.setpoint);
      }
  //     // Limitation pour éviter un saut trop brutal //TODO: pourcentage
  //     float ecart_command_actual = velocity.command - velocity.actual;
  //     if (abs(ecart_command_actual) > 20) // seuil ecart vitesse mm/s
  //     {
  //       velocity.command  = velocity.actual + copysign(20, ecart_command_actual);
  //     }
  }
  else // Arrivée à destination
  {
    Stop();
  }
}

/****************************************************************************************
 * Stop motion : Reset commands
 ****************************************************************************************/
void Motion::Stop()
{
  isRunning = false;
  //position.command = position.actual; // not used
  velocity.command = 0;
  acceleration.command = 0;
  //jerk.command = 0; // not used
}

/****************************************************************************************
 * Setup position setpoint
 ****************************************************************************************/
void Motion::Setpoint_Position(float newPosition)
{
  position.setpoint = newPosition;
}

/****************************************************************************************
 * Set position tolerance, in mm for lin, in rad for ang
 ****************************************************************************************/
void Motion::SetTolerance(float mm_or_rad)
{
  tolerance = mm_or_rad;
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
  //teleplot(name + " pos.setP", position.setpoint);
  teleplot(name + " pos.error", position.error);
  //teleplot(name + " pos.com", position.command);
  //teleplot(name + " pos.actual", position.actual);
  teleplot(name + " pos.decel", position.deceleration);

  teleplot(name + " vel.setP", velocity.setpoint);
  //teleplot(name + " vel.error", velocity.error);
  teleplot(name + " vel.com", velocity.command);
  teleplot(name + " vel.actual", velocity.actual);
  //teleplot(name + " vel.decel", velocity.deceleration);

  //teleplot(name + " acc.setP", acceleration.setpoint);
  //teleplot(name + " acc.com", acceleration.command);
  //teleplot(name + " acc.actual", acceleration.actual);
  //teleplot(name + " acc.piv", acceleration.deceleration);

  //teleplot(name + " jerk.setP",jerk.setpoint);
  //teleplot(name + " jerk.com",jerk.command);
}
