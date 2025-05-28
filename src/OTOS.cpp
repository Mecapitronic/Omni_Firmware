#include "OTOS.h"

using namespace Printer;

void OpticalTrackingOdometrySensor::Initialisation(bool simulation)
{
    println("Init QwiicOTOS");

    // Normal speed is 100 000
    // With higher speed, instructions on I2C take less time
    Wire.begin(SDA, SCL, 400000UL);

    // Same as :
    // Wire.setPins(SDA, SCL);
    // Wire.setClock(400000UL);
    // Wire.begin();

    int retryConnect = 0;
    // Attempt to begin the sensor
    connected = myOtos.begin();
    while (!connected && retryConnect < 5 && !simulation)
    {
        println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
        connected = myOtos.begin();
        retryConnect++;
    }

    if (connected)
    {
        println("OTOS connected!");

        println("Ensure the OTOS is flat and stationary, then enter any key to calibrate the IMU");

        // Clear the serial buffer
        // while (available()) read();
        // Wait for user input
        // while (!available());

        println("Calibrating IMU ...");

        // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
        myOtos.calibrateImu();

        println("IMU calibration done !");

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setAngularScalar(0.992); // 0.992
        myOtos.setLinearScalar(1.00);   // 1.035

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
        // myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
        myOtos.setAngularUnit(kSfeOtosAngularUnitRadians); // /!\ OTOS in radians !
        // myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        sfe_otos_pose2d_t offset = {0, 0, 0};
        myOtos.setOffset(offset);

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        sfe_otos_pose2d_t currentPose = {0, 0, 0};
        myOtos.setPosition(currentPose);

        sfTkError_t error;
        sfe_otos_signal_process_config_t config;
        error = myOtos.getSignalProcessConfig(config);
        if (error != 0)
            print("Error get Signal Process Config : ", error);
        else
        {
            // println("Signal Process Config :");
            // println("enVar : ", config.enVar);
            // println("enRot : ", config.enRot);
            // println("enAcc : ", config.enAcc);
            // println("enLut : ", config.enLut);
        }
    }
}

bool OpticalTrackingOdometrySensor::IsConnected()
{
  return connected;
}

void OpticalTrackingOdometrySensor::Update()
{
  if (connected)
  {
    sfTkError_t error;
    sfe_otos_pose2d_t myPosition;
    sfe_otos_pose2d_t myVelocity;
    sfe_otos_pose2d_t myAcceleration;
    // Get the latest position, which includes the x and y coordinates, plus the
    // heading angle
    // sfe_otos_pose2d_t myPosition;
    // error = myOtos.getPosition(myPosition);
    // if (error != 0)
    //    print("Error Pos : ", error);

    // Create structs for velocity, and acceleration
    // sfe_otos_pose2d_t myVelocity;
    // error = myOtos.getVelocity(myVelocity);
    // if (error != 0)
    //    print("Error Vel : ", error);

    // sfe_otos_pose2d_t myAcceleration;
    // error = myOtos.getAcceleration(myAcceleration);
    // if (error != 0)
    //     print("Error Acc : ", error);

    // If Velocity and Acceleration are not needed, use getPosition to decrease blocking
    // time currently blocking time of getPosVelAcc with 400 000 speed : 600µS
    error = myOtos.getPosVelAcc(myPosition, myVelocity, myAcceleration);
    if (error != 0)
    {
      print("Error getPosVelAcc : ", error);
    }
    else
    {
      position.x = myPosition.x * 1000;
      position.y = myPosition.y * 1000;
      position.h = myPosition.h;
      velocity.x = myVelocity.x * 1000;
      velocity.y = myVelocity.y * 1000;
      velocity.h = myVelocity.h;
      acceleration.x = myAcceleration.x * 1000;
      acceleration.y = myAcceleration.y * 1000;
      acceleration.h = myAcceleration.h;
    }
  }
}

void OpticalTrackingOdometrySensor::HandleCommand(Command cmd)
{
    // if (cmd.cmd == "Otos")
    //{
    //  Otos:0;0
    //    print("Otos : ", cmd.data[0]);
    //}
}

const void OpticalTrackingOdometrySensor::PrintCommandHelp()
{
    Printer::println("OTOS Command Help :");
    Printer::println("! No Command yet !");
    // Printer::println(" > Otos:[int]");
    // Printer::println("      [int] ");
    // Printer::println();
}

void OpticalTrackingOdometrySensor::SetPose(float x, float y, float h)
{
    sfTkError_t error;
    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();

    // After resetting the tracking, the OTOS will report that the robot is at
    // the origin. If your robot does not start at the origin, or you have
    // another source of location information (eg. vision odometry), you can set
    // the OTOS location to match and it will continue to track from there.
    sfe_otos_pose2d_t currentPose = {x / 1000, y / 1000, h};

    if (connected)
    {
        int retrySetPose = 0;
        error = myOtos.setPosition(currentPose);
        while (error != 0 && retrySetPose < 3)
        {
            println("OTOS Error SetPose : ", error);
            delay(100);
            error = myOtos.setPosition(currentPose);
            retrySetPose++;
        }
    }
    // Force position, but will be override when update occurs
    position.x = x;
    position.y = y;
    position.h = h;
    Update();
}

void OpticalTrackingOdometrySensor::Teleplot()
{
    // teleplot("X", myPosition.x*1000);
    // teleplot("Y", myPosition.y*1000);
    // teleplot("H", myPosition.h);

    // teleplot("VX", myVelocity.x * 1000);
    // teleplot("VY", myVelocity.y * 1000);
    // teleplot("VH", myVelocity.h);

    // teleplot("AX", myAcceleration.x * 1000);
    // teleplot("AY", myAcceleration.y * 1000);
    // teleplot("AH", myAcceleration.h);

    /*
        // Print measurement
        println();
        println("Position:");
        print("X (Meters): ");
        println(myPosition.x, 4);
        print("Y (Meters): ");
        println(myPosition.y, 4);
        print("Heading (Degrees): ");
        println(myPosition.h, 4);

        // Print velocity
        println();
        println("Velocity:");
        print("X (Meters/sec): ");
        println(vel.x, 4);
        print("Y (Meters/sec): ");
        println(vel.y, 4);
        print("Heading (Degrees/sec): ");
        println(vel.h, 4);

        // Print acceleration
        println();
        println("Acceleration:");
        print("X (Meters/sec^2): ");
        println(acc.x, 4);
        print("Y (Meters/sec^2): ");
        println(acc.y, 4);
        print("Heading (Degrees/sec^2): ");
        println(acc.h, 4);
    */
}
