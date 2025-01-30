#include "MotorController.h"
//https://lastminuteengineers.com/esp32-pwm-tutorial/

ESP32_FAST_PWM *stepper1;
ESP32_FAST_PWM *stepper2;
ESP32_FAST_PWM *stepper3;

void MotorController::Initialisation()
{
    println("Initialisation MotorController");

    // Sets the two pins as Outputs
    pinMode(stepPinM1, OUTPUT);
    pinMode(dirPinM1, OUTPUT);
    digitalWrite(stepPinM1, LOW);
    digitalWrite(dirPinM1, LOW);

    pinMode(stepPinM2, OUTPUT);
    pinMode(dirPinM2, OUTPUT);
    digitalWrite(stepPinM2, LOW);
    digitalWrite(dirPinM2, LOW);

    pinMode(stepPinM3, OUTPUT);
    pinMode(dirPinM3, OUTPUT);
    digitalWrite(stepPinM3, LOW);
    digitalWrite(dirPinM3, LOW);

    print("Starting ESP32_FAST_PWM: ");
    println(ESP32_FAST_PWM_VERSION);

    // pin, frequency = 500 Hz, dutyCycle = 0 %, choose channel with independent timer, resolution to go up to desire frequency
    stepper1 = new ESP32_FAST_PWM(stepPinM1, 500, 0, 2, BIT_RESOLUTION);

    stepper2 = new ESP32_FAST_PWM(stepPinM2, 500, 0, 4, BIT_RESOLUTION);

    stepper3 = new ESP32_FAST_PWM(stepPinM3, 500, 0, 6, BIT_RESOLUTION);
}

void MotorController::Update()
{
    // nothing to do, PWM works on their own
}

void MotorController::HandleCommand(Command cmd)
{

    if (cmd.cmd.startsWith("Motor"))
    {
        if (cmd.cmd == ("Motor") && cmd.size == 3)
        {
            // Motor:1;1000;50
            // Motor:1;1;50
            if(cmd.data[2] >= 0 && cmd.data[2] <= 100)
            {
                print("Motor ", cmd.data[0]);
                print(" with freq ", cmd.data[1], " Hz");
                println(" with duty ", cmd.data[2], " %");
                if(cmd.data[0]==1)
                    stepper1->setPWM(cmd.data[1],cmd.data[2]);
                if(cmd.data[0]==2)
                    stepper2->setPWM(cmd.data[1],cmd.data[2]);
                if(cmd.data[0]==3)
                    stepper3->setPWM(cmd.data[1],cmd.data[2]);
            }
        }
        else
        {
            println("Not a Motor Command ");
        }
    }
}

void MotorController::PrintCommandHelp()
{
    Printer::println("MotorController Command Help :");
    Printer::println(" > Motor:[int];[int];[int]");
    Printer::println("      [int] number of motor, frequency of motor in Hz, duty cycle of motor between 0 and 100");
    Printer::println();
}

void MotorController::SetMotorSpeed(int motor_ID, float speed_mms)
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
            stepper1->setPWM(FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM1, direction);
            stepper1->setPWM(freq, 50);
        }
    }
    else if (motor_ID == 2)
    {
        if (freq == 0 || freq < FREQ_MIN_STEPPER)
        {
            stepper2->setPWM(FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM2, direction);
            stepper2->setPWM(freq, 50);
        }
    }
    else if (motor_ID == 3)
    {
        if (freq == 0 || freq < FREQ_MIN_STEPPER)
        {
            stepper3->setPWM(FREQ_MIN_STEPPER, 0);
        }
        else
        {
            //  Set the frequency of the PWM output and a duty cycle of 50%
            digitalWrite(dirPinM3, direction);
            stepper3->setPWM(freq, 50);
        }
    }
}
/*
void MotorController::SetMotorsSpeed(float speed_1_mms, float speed_2_mms, float speed_3_mms)
{
    SetMotorSpeed(1, speed_1_mms);
    SetMotorSpeed(2, speed_2_mms);
    SetMotorSpeed(3, speed_3_mms);
}*/

float MotorController::GetMotorSpeed(int motor_ID)
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


void MotorController::test_ledc()
{
    int PIN = 18;
    int channel = 0;
    int dutycycle = 4095;
    int SOC_LEDC_TIMER_BIT_WIDTH = 16;
    pinMode(PIN, OUTPUT);
    ledcSetup(channel, 1000, 14);
    ledcAttachPin(PIN, channel);
    //ledc_set_duty(0, 0, _dutycycle);
    
    uint32_t min_frequency;
    uint32_t max_frequency;
    uint32_t frequency;
    uint32_t successful_frequency;
    uint32_t max_freq_array[SOC_LEDC_TIMER_BIT_WIDTH];
    uint32_t min_freq_array[SOC_LEDC_TIMER_BIT_WIDTH];

    // Find Max Frequency
    for (uint8_t resolution = 1; resolution <= SOC_LEDC_TIMER_BIT_WIDTH; ++resolution)
    {
        
        ledcSetup(channel, 1000, resolution);
        max_freq_array[resolution - 1] = 0;
        min_frequency = 0;
        max_frequency = UINT32_MAX;
        successful_frequency = 0;
        while (min_frequency != max_frequency && min_frequency + 1 != max_frequency)
        {
            frequency = min_frequency + ((max_frequency - min_frequency) / 2);
            println("frequency max ",frequency);
            if (ledcChangeFrequency(channel, frequency, resolution))
            //if(ledc_set_freq((ledc_mode_t)0, (ledc_timer_t) 0, frequency) == ESP_OK)
            {
                min_frequency = frequency;
                successful_frequency = frequency;
            }
            else
            {
                max_frequency = frequency;
            }
        } // while not found the maximum
        max_freq_array[resolution - 1] = successful_frequency;
    } // for all resolutions

    // Find Min Frequency
    for (uint8_t resolution = 1; resolution <= SOC_LEDC_TIMER_BIT_WIDTH; ++resolution)
    {
        ledcSetup(channel, 1000, resolution);
        min_freq_array[resolution - 1] = 0;
        min_frequency = 0;
        max_frequency = max_freq_array[resolution - 1];
        successful_frequency = max_frequency;
        while (min_frequency != max_frequency && min_frequency + 1 != max_frequency)
        {
            frequency = min_frequency + ((max_frequency - min_frequency) / 2);
            println("frequency min ",frequency);
            if (ledcChangeFrequency(channel, frequency, resolution))
            //if(ledc_set_freq((ledc_mode_t)0, (ledc_timer_t) 0, frequency) == ESP_OK)
            {
                max_frequency = frequency;
                successful_frequency = frequency;
            }
            else
            {
                min_frequency = frequency;
            }
        } // while not found the maximum
        min_freq_array[resolution - 1] = successful_frequency;
    } // for all resolutions

    printf("Bit resolution | Min Frequency [Hz] | Max Frequency [Hz]\n");
    for (uint8_t r = 1; r <= SOC_LEDC_TIMER_BIT_WIDTH; ++r)
    {
        size_t max_len = std::to_string(UINT32_MAX).length();
        printf(
            "            %s%d |         %s%lu |         %s%lu\n", std::string(2 - std::to_string(r).length(), ' ').c_str(), r,
            std::string(max_len - std::to_string(min_freq_array[r - 1]).length(), ' ').c_str(), min_freq_array[r - 1],
            std::string(max_len - std::to_string(max_freq_array[r - 1]).length(), ' ').c_str(), max_freq_array[r - 1]);
    }
    ledcDetachPin(PIN);
}



void MotorController::test_ledc2()
{
    ESP32_FAST_PWM *stepper0;

    uint8_t resolution = 14;
    int dutycycle = 50;
    int freq = 500;
    uint8_t pin = 20;
    uint8_t channel = 0;
    
    stepper0 = new ESP32_FAST_PWM(pin, freq, dutycycle, channel, resolution);
    for (uint32_t freq = 1; freq <= 20; freq++)
    {
      stepper0->setPWM(freq, 50);
      delay(1);
    }
    println();
}