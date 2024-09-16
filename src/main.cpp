#include "main.h"

ESP32_FAST_PWM *stepper1;
ESP32_FAST_PWM *stepper2;
ESP32_FAST_PWM *stepper3;

// Motion parameters
OpticalTrackingOdometrySensor otos;

// Timer Settings
static const TickType_t timer_delay_1 = 5 / portTICK_PERIOD_MS; // period of "tic" (ref time for robot motion asserv)
static TimerHandle_t timer_handle_1 = NULL;
static const TickType_t timer_delay_2 = 20 / portTICK_PERIOD_MS; // period of update teleplot
static TimerHandle_t timer_handle_2 = NULL;

void timerCallback1(TimerHandle_t xTimer)
{
  // TIMER 1
  // do NON BLOCKING stuff
  otos.Update();
}

void timerCallback2(TimerHandle_t xTimer)
{
  // TIMER 2
  // do NON BLOCKING stuff
  otos.Teleplot();
}

//******************************************************* SETUP **************************************************************** */
void setup()
{
  ESP32_Helper::Initialisation();
  println("Temperature is : ", temperatureRead());
  println("Robot Holonome Firmware");

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

  print(F("\nStarting ESP32_PWM_StepperControl on "));
  println(ARDUINO_BOARD);
  println(ESP32_FAST_PWM_VERSION);

  stepper1 = new ESP32_FAST_PWM(stepPinM1, 500, 0, 1, 8); // pin, frequency = 500 Hz, dutycycle = 0 %, channel, resolution = 8 ? 10 ? 12 ?
  if (stepper1)
  {
    stepper1->setPWM();
  }

  stepper2 = new ESP32_FAST_PWM(stepPinM2, 500, 0, 2, 8);
  if (stepper2)
  {
    stepper2->setPWM();
  }

  stepper3 = new ESP32_FAST_PWM(stepPinM3, 500, 0, 3, 8);
  if (stepper3)
  {
    stepper3->setPWM();
  }

  otos.Initialisation();

  // Create a timer
  timer_handle_1 = xTimerCreate(
      "Timer 1",       // Name of timer
      timer_delay_1,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)0,       // Timer ID
      timerCallback1); // Callback function
  xTimerStart(timer_handle_1, portMAX_DELAY);

  timer_handle_2 = xTimerCreate(
      "Timer 2",       // Name of timer
      timer_delay_2,   // Period of timer (in ticks)
      pdTRUE,          // Auto-reload
      (void *)0,       // Timer ID
      timerCallback2); // Callback function
  xTimerStart(timer_handle_2, portMAX_DELAY);
}

int x = 1;

//******************************************************* LOOP *****************************************************************/
void loop()
{
  setMotorSpeed(1, 2000);
  setMotorSpeed(2, -2000);
  delay(2000);

  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  setMotorSpeed(3, 0);
  // delay(1000);

  setMotorSpeed(1, -2000);
  setMotorSpeed(2, 2000);
  delay(2000);

  // stop all motors
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  setMotorSpeed(3, 0);
  delay(2000);

  if (ESP32_Helper::HasWaitingCommand())
  {
    Command cmd = ESP32_Helper::GetCommand();

    otos.HandleCommand(cmd);
  }
}
//**************************************************************************************************************************/

// Set speed in mm/s of one specific motor
void setMotorSpeed(int motor_ID, float speed_mms)
{
  // convert speed in mm/s to frequency in step/s
  float freq = speed_mms / STEP_PER_MM;

  if (motor_ID == 1)
  {
    if (freq == 0)
    {
      // stop motor
      stepper1->setPWM();
    }
    else
    {
      //  Set the frequency of the PWM output and a duty cycle of 50%
      digitalWrite(dirPinM1, (freq < 0));
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
      digitalWrite(dirPinM2, (freq < 0));
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
      digitalWrite(dirPinM3, (freq < 0));
      stepper3->setPWM(stepPinM3, abs(freq), 50);
    }
  }
}
