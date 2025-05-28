/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Motion.h"

/****************************************************************************************
 * Initialize
 ****************************************************************************************/
void Motion::Initialisation(float speedMax, float accelMax)
{
    isRunning = false;

    accel_max = accelMax;
    speed_max = speedMax;
    speed_limit = speed_max;
    speed_final = 0;
    velocity_actual = 0;
    velocity_command = 0;
    position_error = 0;
    position_margin = 0;
}

/****************************************************************************************
 * Motion Update : Must be called on every speed control loop => motion timer
 ****************************************************************************************/
void Motion::Update()
{
    isRunning = true;

    // Check end of motion
    if (fabsf(position_error) >= position_margin)
    {
        TrapezoidalProfile();
        AntiOverspeed();
    }
    else
    {
        Stop();
    }
}

/****************************************************************************************
 * Trapezoidal velocity controller for smoothly motion profile, based on kinematic motion
 *laws : distance_deceleration = |velocity_real^2 - velocity_final^2| / (2 * accel_max)
 * -OR- velocity_setpoint = sqrt(velocity_final^2 + 2 * accel_max * distance_remaining)
 ****************************************************************************************/
void Motion::TrapezoidalProfile()
{
    // Speed setpoint
    float velocity_setpoint =
        sqrtf((speed_final * speed_final) + 2 * accel_max * fabsf(position_error));

    // Speed limitation (in any case, setpoint is limited by the absolute speed max)
    velocity_setpoint = fmin(velocity_setpoint, fmin(speed_limit, speed_max));

    // Velocity (speed with sign)
    velocity_setpoint = copysignf(velocity_setpoint, position_error);

    // Velocity smoothing by integration of acceleration
    if (velocity_actual < velocity_setpoint)
    {
        // Velocity increase
        velocity_command =
            fmin(velocity_command + (accel_max * dt_motion), velocity_setpoint);
    }
    else
    {
        // Velocity decrease
        velocity_command =
            fmax(velocity_command - (accel_max * dt_motion), velocity_setpoint);
    }
}

/****************************************************************************************
 * Anti Overspeed : Limit the velocity command to avoid overspeed or runaway
 ****************************************************************************************/
void Motion::AntiOverspeed()
{
    // TODO : sortir les variables et ajuster la fonction...
    float delta_v_min = 50.0; // Augmentation minimale par cycle (mm/s)
    float k = 0.5;            // Facteur d’augmentation relative

    // Max velocity delta
    float seuil_vit = fabsf(velocity_actual * k) + delta_v_min;
    float delta_vit = velocity_command - velocity_actual;

    // Velocity command limitation
    if (fabsf(delta_vit) > seuil_vit)
    {
        velocity_command = velocity_actual + copysign(seuil_vit, delta_vit);
    }
}

/****************************************************************************************
 * Stop motion : Reset commands
 ****************************************************************************************/
void Motion::Stop()
{
    isRunning = false;
    velocity_command = 0;
}

/****************************************************************************************
 * Set end position tolerance : in mm for linear, in radian for angular
 ****************************************************************************************/
void Motion::SetMargin(float margin_mm_or_rad)
{
    position_margin = margin_mm_or_rad;
}

/****************************************************************************************
 * Return OK if position setpoint is reached
 ****************************************************************************************/
// boolean Motion::Check_Position()
// {
//   return (position.command == true);
// }

/****************************************************************************************
 * Teleplot motion
 ****************************************************************************************/
void Motion::Teleplot(String name)
{
    teleplot(name + " pos.error", position_error);
    teleplot(name + " vel.com", velocity_command);
    teleplot(name + " vel.actual", velocity_actual);
}
