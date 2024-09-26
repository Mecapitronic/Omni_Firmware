#include "Stepper.h"

ESP32_FAST_PWM *stepper1;
ESP32_FAST_PWM *stepper2;
ESP32_FAST_PWM *stepper3;

void Stepper::Initialisation()
{
    println("Initialisation Stepper");

    // Sets the two pins as Outputs
    pinMode(stepPinM1, OUTPUT);
    pinMode(dirPinM1, OUTPUT);
    digitalWrite(dirPinM1, LOW);

    pinMode(stepPinM2, OUTPUT);
    pinMode(dirPinM2, OUTPUT);
    digitalWrite(dirPinM2, LOW);

    pinMode(stepPinM3, OUTPUT);
    pinMode(dirPinM3, OUTPUT);
    digitalWrite(dirPinM3, LOW);

    print(F("\nStarting ESP32_PWM_StepperControl"));
    println(ESP32_FAST_PWM_VERSION);

    // TODO test with Resolution < 8 ?
    stepper1 = new ESP32_FAST_PWM(stepPinM1, 500, 0, 2, 8); // pin, frequency = 500 Hz, dutyCycle = 0 %, channel with independent timer, resolution = 8
    if (stepper1)
    {
        stepper1->setPWM();
    }

    stepper2 = new ESP32_FAST_PWM(stepPinM2, 500, 0, 4, 8);
    if (stepper2)
    {
        stepper2->setPWM();
    }

    stepper3 = new ESP32_FAST_PWM(stepPinM3, 500, 0, 6, 8);
    if (stepper3)
    {
        stepper3->setPWM();
    }
}

void Stepper::Update()
{
    // nothing to do, PWM works on their own
}

void Stepper::HandleCommand(Command cmd)
{

    if (cmd.cmd.startsWith("Stepper"))
    {
        if (cmd.cmd == ("StepperMotor") && cmd.size == 2)
        {
            // Stepper:1;500
            print("Stepper : ", cmd.data[0], " with ");
            println("Freq : ", cmd.data[1]);
            SetMotorSpeed(cmd.data[0], cmd.data[1]);
        }
        else if (cmd.cmd == ("StepperMotors") && cmd.size == 3)
        {
            // Stepper:200;500;300
            println("Stepper 1 with freq : ", cmd.data[0]);
            println("Stepper 2 with freq : ", cmd.data[1]);
            println("Stepper 3 with freq : ", cmd.data[3]);
            SetMotorsSpeed(cmd.data[0], cmd.data[1], cmd.data[2]);
        }
        else
        {
            println("Not a Stepper Command ");
        }
    }
}

void Stepper::PrintCommandHelp()
{
    Printer::println("Stepper Command Help :");
    Printer::println(" > StepperMotor:[int];[int]");
    Printer::println("      [int] number of motor, [int] frequency in Hz");
    Printer::println(" > StepperMotors:[int];[int];[int]");
    Printer::println("      [int] frequency of motor 1, frequency of motor 2, frequency of motor 3 in Hz");
    Printer::println();
}

void Stepper::SetMotorSpeed(int motor_ID, float speed_mms)
{
    // TODO STEP_PER_MM
    // convert speed in mm/s to frequency in step/s
    float freq = speed_mms; //* STEP_PER_MM;

    if (motor_ID == 1)
    {
        // TODO freq min = dead zone
        if (freq == 0)
        {
            // stop motor
            stepper1->setPWM();
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM1, (freq > 0));
            stepper1->setPWM(stepPinM1, abs(freq), 50);
        }
    }
    else if (motor_ID == 2)
    {
        if (freq == 0)
        {
            // stop motor
            stepper2->setPWM();
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM2, (freq > 0));
            stepper2->setPWM(stepPinM2, abs(freq), 50);
        }
    }
    else if (motor_ID == 3)
    {
        if (freq == 0)
        {
            // stop motor
            stepper3->setPWM();
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM3, (freq > 0));
            stepper3->setPWM(stepPinM3, abs(freq), 50);
        }
    }
}

void Stepper::SetMotorsSpeed(float speed_1_mms, float speed_2_mms, float speed_3_mms)
{
    SetMotorSpeed(1, speed_1_mms);
    SetMotorSpeed(2, speed_2_mms);
    SetMotorSpeed(3, speed_3_mms);
}