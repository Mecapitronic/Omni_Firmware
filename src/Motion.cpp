/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Motion.h"

/****************************************************************************************
 * Initialize setpoint
 ****************************************************************************************/
void Motion::Initialisation(float speedMax, float accelMax, float jerkMax)
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
  float speed_final = 0; // vitesse à atteindre à la fin du trapèze, pas forcément nulle !
  float position_setpoint = position.setpoint; // distance restante
  float velocity_setpoint = velocity.setpoint;
  float acceleration_setpoint = acceleration.setpoint;

  //****************** Nouvelle méthode : trapèze adaptatif (chatGPT) *************************
  // Étape 1 : Calcul de la distance restante
    //distance_remaining <- calculate_distance(current_position, target_position)

    // Étape 2 : Calcul de la distance de décélération
    //deceleration_distance <- (current_velocity^2) / (2 * max_acceleration)
    // Distance de décélération, point de bascule de la vitesse |Vfinal^2-Vreal^2|/2Amax (attention: l'acceleration doit être non nulle !)
    if (acceleration_setpoint != 0)
    {
      position.pivot = fabsf((speed_final * speed_final) - (velocity.real * velocity.real)) / (2 * acceleration_setpoint);

      //if (jerk.setpoint != 0) // normally var set to a const <> 0
      //{
       // position.pivot += (abs(velocity.real) * acceleration_setpoint) / (2 * jerk.setpoint);
      //}
    }
    else
    {
      position.pivot = 0;
    }

    // Étape 3 : Calcul de la vitesse cible
    //if distance_remaining <= deceleration_distance:
        // Décélération nécessaire
      //  target_velocity <- sqrt(2 * max_acceleration * distance_remaining)
    //else:
        // Accélération ou vitesse constante
      //  target_velocity <- max_velocity
    if (fabsf(position_setpoint) <= position.pivot) 
    {
      velocity_setpoint = sqrt(2 * acceleration_setpoint * fabsf(position_setpoint));
    }

    // Étape 4 : Ajustement de la vitesse
    //if target_velocity > current_velocity:
        // Augmentation de la vitesse
        //new_velocity <- min(current_velocity + max_acceleration * dt, target_velocity)
    //else:
        // Réduction de la vitesse
        //new_velocity <- max(current_velocity - max_acceleration * dt, target_velocity)

    if (velocity_setpoint > velocity.real)
    {
      //l'accélération appliquée est trop faible par rapport aux frottements ou autres résistances (inertie, seuil minimum du moteur, etc.). 
      //Comme la vitesse initiale est nulle, la vitesse commandée reste trop basse pour provoquer un mouvement.
         // Étape 4.1 : Appliquer un boost temporaire si la vitesse est trop faible
      //if (velocity.real < 1000 && acceleration_setpoint > 0) {
        //  acceleration_setpoint = 500;
      //}
      // Étape 4.2 : Calculer la nouvelle vitesse commandée
      //if (velocity.real <= 0) acceleration_setpoint = acceleration_setpoint * 10;
      velocity.command = fmin(velocity.real + acceleration_setpoint, velocity_setpoint);

      // Étape 4.3 : Imposer une vitesse minimale pour initier le mouvement
      //if (velocity.command > 0 && velocity.command < 700) {
        //  velocity.command = 700;
      //}
    }
    else
    {
      velocity.command = fmax(velocity.real - acceleration_setpoint, velocity_setpoint);
    }
    // prise en compte d'une consigne négative => changer la direction de la vitesse
    if (position_setpoint < 0) velocity.command = -velocity.command;

    // Étape 5 : Calcul de la nouvelle position
    //direction <- normalize_vector(subtract_vectors(target_position, current_position))
    //displacement <- multiply_vector(direction, new_velocity * dt)
    //new_position <- add_vectors(current_position, displacement)

    // Étape 6 : Mise à jour des états
    //current_velocity <- new_velocity
    //current_position <- new_position

    // Étape 7 : Vérification de l'arrivée
    //if distance_remaining < TOLERANCE:
      //  current_velocity <- 0  // Arrêter le robot
       // stop_timer()           // Désactiver l'asservissement si nécessaire

/* //****************** Ancienne méthode : point pivot (Microb Technology) *************************
  //? https://wiki.droids-corp.org/articles/a/v/e/Aversive/Modules/Control_system/Filters/Quadramp_derivate.html
  //************************** consigne position => commande vitesse ***************************
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

  //************************** consigne vitesse => commande acceleration ***************************
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
*/
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
void Motion::Setpoint_Position(float newPosition)
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

  //teleplot(name + " acc.setP", acceleration.setpoint);
  //teleplot(name + " acc.com", acceleration.command);
  //teleplot(name + " acc.real", acceleration.real);
  //teleplot(name + " acc.piv", acceleration.pivot);

  //teleplot(name + " jerk.setP",jerk.setpoint);
  //teleplot(name + " jerk.com",jerk.command);
}
