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

    print("Starting ESP32_PWM_StepperControl : ");
    println(ESP32_FAST_PWM_VERSION);

    // pin, frequency = 500 Hz, dutyCycle = 0 %, choose channel with independent timer, resolution = 13 to go up to 8k Hz
    stepper1 = new ESP32_FAST_PWM(stepPinM1, 500, 0, 2, BIT_RESOLUTION);
    if (stepper1)
    {
        stepper1->setPWM();
    }

    stepper2 = new ESP32_FAST_PWM(stepPinM2, 500, 0, 4, BIT_RESOLUTION);
    if (stepper2)
    {
        stepper2->setPWM();
    }

    stepper3 = new ESP32_FAST_PWM(stepPinM3, 500, 0, 6, BIT_RESOLUTION);
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
        else if (cmd.cmd == ("StepperMotor") && cmd.size == 3)
        {
            // StepperMotor:100;100;100
            println("Stepper 1 with freq : ", cmd.data[0]);
            println("Stepper 2 with freq : ", cmd.data[1]);
            println("Stepper 3 with freq : ", cmd.data[2]);
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
    // convert speed in mm/s to frequency in step/s
    float speed_step_s = speed_mms * MOTOR_STEP_PER_MM;
    //print("speed_mms:",speed_mms);
    int freq = (int)fmin(FREQ_MAX_STEPPER,fabs(speed_step_s));
    bool direction = (speed_mms > 0.0);
    //println(" freq:",freq);
    if (motor_ID == 1)
    {
        if (freq == 0 || freq < FREQ_MIN_STEPPER)
        {
            // stop motor do not use freq 0 : IntegefromrDivideByZero in ledc.c line 318 with ESP IDF Version : v4.4.7-dirty
            // use : duty cycle = 0 to put pin to LOW
            stepper1->setPWM_Int(stepPinM1, FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM1, direction);
            stepper1->setPWM_Int(stepPinM1, freq, dutyCycle50);
        }
    }
    else if (motor_ID == 2)
    {
        if (freq == 0 || freq < FREQ_MIN_STEPPER)
        {
            stepper2->setPWM_Int(stepPinM2, FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM2, direction);
            stepper2->setPWM_Int(stepPinM2, freq, dutyCycle50);
        }
    }
    else if (motor_ID == 3)
    {
        if (freq == 0 || freq < FREQ_MIN_STEPPER)
        {
            stepper3->setPWM_Int(stepPinM2, FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM3, direction);
            stepper3->setPWM_Int(stepPinM3, freq, dutyCycle50);
        }
    }
}

void Stepper::SetMotorsSpeed(float speed_1_mms, float speed_2_mms, float speed_3_mms)
{
    SetMotorSpeed(1, speed_1_mms);
    SetMotorSpeed(2, speed_2_mms);
    SetMotorSpeed(3, speed_3_mms);
}

float Stepper::GetMotorSpeed(int motor_ID)
{
    bool direction;
    if (motor_ID == 1)
    {
        if (stepper1->getActualDutyCycle() == 0)
            return 0;
        direction = digitalRead(dirPinM1);
        if (direction)
            return stepper1->getActualFreq() * MM_PER_STEP_MOTOR;
        else
            return -stepper1->getActualFreq() * MM_PER_STEP_MOTOR;
    }
    else if (motor_ID == 2)
    {
        if (stepper1->getActualDutyCycle() == 0)
            return 0;
        direction = digitalRead(dirPinM2);
        if (direction)
            return stepper2->getActualFreq() * MM_PER_STEP_MOTOR;
        else
            return -stepper2->getActualFreq() * MM_PER_STEP_MOTOR;
    }
    else if (motor_ID == 3)
    {
        if (stepper1->getActualDutyCycle() == 0)
            return 0;
        direction = digitalRead(dirPinM3);
        if (direction)
            return stepper3->getActualFreq() * MM_PER_STEP_MOTOR;
        else
            return -stepper3->getActualFreq() * MM_PER_STEP_MOTOR;
    }
    return 0;
}
