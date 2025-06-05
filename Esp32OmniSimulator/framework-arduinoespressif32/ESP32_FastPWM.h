
/****************************************************************************************************************************
  ESP32_FastPWM.h
  For ESP32, ESP32_S2, ESP32_S3, ESP32_C3 boards with ESP32 core v2.0.0+
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/ESP32_FastPWM
  Licensed under MIT license

  Version: 1.1.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      31/10/2022 Initial coding for ESP32, ESP32_S2, ESP32_C3 boards with ESP32 core v2.0.0+
  1.0.1   K Hoang      22/01/2023 Add `PWM_StepperControl` example
  1.1.0   K Hoang      25/01/2023 Add `PWM_manual` example and function
*****************************************************************************************************************************/

#pragma once

#ifndef ESP32_FAST_PWM_HPP
#define ESP32_FAST_PWM_HPP

#if !defined( ESP32 )
  #error This code is designed to run on ESP32 platform, not Arduino nor ESP8266! Please check your Tools->Board setting.
#endif

#if ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_ESP32S2_THING_PLUS || ARDUINO_MICROS2 || \
      ARDUINO_METRO_ESP32S2 || ARDUINO_MAGTAG29_ESP32S2 || ARDUINO_FUNHOUSE_ESP32S2 || \
      ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM || ARDUINO_ADAFRUIT_QTPY_ESP32S2)
#define FAST_PWM_USING_ESP32_S2         true
#elif ( defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32_S3_BOX) || defined(ARDUINO_TINYS3) || \
        defined(ARDUINO_PROS3) || defined(ARDUINO_FEATHERS3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM) || \
        defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM))
#define FAST_PWM_USING_ESP32_S3         true
#elif ( ARDUINO_ESP32C3_DEV )
#define FAST_PWM_USING_ESP32_C3         true
#elif defined(ESP32)
#define FAST_PWM_USING_ESP32            true
#else
#error This code is ready to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#if defined(ARDUINO)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif

#ifndef ESP32_FAST_PWM_VERSION
  #define ESP32_FAST_PWM_VERSION           F("ESP32_FastPWM v1.1.1")

  #define ESP32_FAST_PWM_VERSION_MAJOR     1
  #define ESP32_FAST_PWM_VERSION_MINOR     1
  #define ESP32_FAST_PWM_VERSION_PATCH     1

  #define ESP32_FAST_PWM_VERSION_INT       1001001
#endif

#include "PWM_Generic_Debug.h"

#include "driver/ledc.h"

#define INVALID_ESP32_PIN         255

////////////////////////////////////////


/*
   ESP32 LEDC Chan to Group/Channel/Timer Mapping
** ledc: 0  => Group: 0, Channel: 0, Timer: 0
** ledc: 1  => Group: 0, Channel: 1, Timer: 0
** ledc: 2  => Group: 0, Channel: 2, Timer: 1
** ledc: 3  => Group: 0, Channel: 3, Timer: 1
** ledc: 4  => Group: 0, Channel: 4, Timer: 2
** ledc: 5  => Group: 0, Channel: 5, Timer: 2
** ledc: 6  => Group: 0, Channel: 6, Timer: 3
** ledc: 7  => Group: 0, Channel: 7, Timer: 3
** ledc: 8  => Group: 1, Channel: 0, Timer: 0
** ledc: 9  => Group: 1, Channel: 1, Timer: 0
** ledc: 10 => Group: 1, Channel: 2, Timer: 1
** ledc: 11 => Group: 1, Channel: 3, Timer: 1
** ledc: 12 => Group: 1, Channel: 4, Timer: 2
** ledc: 13 => Group: 1, Channel: 5, Timer: 2
** ledc: 14 => Group: 1, Channel: 6, Timer: 3
** ledc: 15 => Group: 1, Channel: 7, Timer: 3
*/


// Max resolution is 20-bit
// Resolution 65536 (16-bit) for lower frequencies,  OK @ 1K
// Resolution  4096 (12-bit) for lower frequencies,  OK @ 10K
// Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
// Resolution  256  ( 8-bit) for higher frequencies, OK @ 100K, 200K
// Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
#define MAX_PWM_RESOLUTION_BIT      SOC_LEDC_TIMER_BIT_WIDE_NUM
#define MAX_PWM_RESOLUTION          (1 << SOC_LEDC_TIMER_BIT_WIDE_NUM)

////////////////////////////////////////


#ifdef SOC_LEDC_SUPPORT_HS_MODE
  #define _LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM << 1)
#else
  #define _LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif

static uint8_t channels_resolution[_LEDC_CHANNELS] = {0};

class ESP32_FAST_PWM
{
  public:

    // Default 8-bit for higher freqs
    // dutycycle = 0.0f - 100.0f %
    // frequency != 0 Hz
    ESP32_FAST_PWM(const uint8_t& pin, const float& frequency, const float& dutycycle,
                   const uint8_t& channel = 0, const float& resolution = 8)
    {
      if (resolution <= SOC_LEDC_TIMER_BIT_WIDE_NUM)
        _resolution = resolution;
      else
      {
        _resolution = SOC_LEDC_TIMER_BIT_WIDE_NUM;
        PWM_LOGERROR1(F("ESP32_FastPWM: _resolution too big, reset to"), SOC_LEDC_TIMER_BIT_WIDE_NUM);
      }

      if (channel <= _LEDC_CHANNELS)
        _channel    = (ledc_channel_t) channel;
      else
      {
        _channel    = (ledc_channel_t) _LEDC_CHANNELS;
        PWM_LOGERROR1(F("ESP32_FastPWM: _channel too big, reset to"), _LEDC_CHANNELS);
      }

      if (channels_resolution[_channel] == 0)
      {
        PWM_LOGDEBUG3(F("ESP32_FastPWM: new _channel ="), _channel, F("with resolution of"), _resolution);
        channels_resolution[_channel] = _resolution;
      }
      else if (channels_resolution[_channel] != _resolution)
      {
        PWM_LOGDEBUG3(F("ESP32_FastPWM: Change resolution of used _channel ="), _channel, F("to"), _resolution);
        channels_resolution[_channel] = _resolution;
      }

      _group      = (ledc_mode_t) (_channel / 8);
      PWM_LOGDEBUG1(F("ESP32_FastPWM: _group "), _group);
      _timer      = (_channel / 2) % 4;
      PWM_LOGDEBUG1(F("ESP32_FastPWM: _timer "), _timer);

      _pin        = pin;
      PWM_LOGDEBUG1(F("ESP32_FastPWM: _pin "), _pin);
      _frequency  = frequency;
      PWM_LOGDEBUG1(F("ESP32_FastPWM: _frequency "), _frequency);

      _dutycycle  = round(remap(dutycycle, 0, 100.0f, 0, (1 << _resolution)));
      PWM_LOGDEBUG1(F("ESP32_FastPWM: _dutycycle ="), _dutycycle);

      pinMode(_pin, OUTPUT);
      digitalWrite(_pin, LOW);

      if(ledcSetup(_channel, _frequency, _resolution) == 0)
      {
        PWM_LOGDEBUG(F("ESP32_FastPWM: setup ledc failed !"));
      }
      else
      {
        PWM_LOGDEBUG(F("ESP32_FastPWM: setup ledc done."));
      }
      ledcAttachPin(_pin, _channel);
      ledcWrite(_channel, 0);
    }

    ///////////////////////////////////////////

    ~ESP32_FAST_PWM();

    ///////////////////////////////////////////
    ///////////////////////////////////////////

  private:

    // dutycycle from 0-(1 << _resolution) for 0%-100%
    bool setPWM_Int(const float& frequency, const uint32_t& dutycycle)
    {
      //ledc_stop(_group,_channel,0);
      // !!! need ledcWrite or update to turn it on again !!! 
      
      if(frequency <= 0)
      {
        _frequency = 0;
        ledcWrite(_channel, 0);
      }

      if(dutycycle <= 0)
      {
        _dutycycle = 0;
        ledcWrite(_channel, 0);
      }
      
      if(frequency > 0 && dutycycle > 0)
      {
        // Reprogram freq if necessary
        if (frequency != _frequency)
        {
          PWM_LOGDEBUG3(F("setPWM_Int: change frequency from"), _frequency, F("to"), frequency);

          _frequency  = frequency;

          // To avoid glitch when changing frequency
          // esp_err_t ledc_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz);
          ledc_set_freq(_group, (ledc_timer_t) _timer, frequency);     
        }

        // Reprogram duty if necessary
        if (dutycycle != _dutycycle)
        {
          PWM_LOGDEBUG5(F("setPWM_Int: change dutycycle from"), _dutycycle, F("to"), dutycycle,F(", DC % ="), dutycycle * 100.0f / (1 << _resolution) );
          
          _dutycycle = dutycycle;
          
          if (_dutycycle <= 0)
          {
            ledcWrite(_channel, 0);
          }
          else if (_dutycycle >= (1 << _resolution))
          {
            ledcWrite(_channel, (1 << _resolution)-1);
          }
          else
          {
            ledcWrite(_channel, _dutycycle);
          }
        }
      }

      return true;
    }
    
    long remap(long x, long in_min, long in_max, long out_min, long out_max) {
        const long run = in_max - in_min;
        if(run == 0){
            //log_e("remap(): Invalid input range, min == max");
            return -1; // AVR returns -1, SAM returns 0
        }
        const long rise = out_max - out_min;
        const long delta = x - in_min;
        return (delta * rise) / run + out_min;
    }

    ///////////////////////////////////////////

  public:

    ///////////////////////////////////////////

    // dutycycle = 0%-100%
    bool setPWM(const float& frequency, const float& dutycycle)
    {
      uint32_t duty = round(remap(dutycycle, 0, 100.0f, 0, (1 << _resolution)));

      PWM_LOGDEBUG3(F("setPWM: dutycycle ="), duty, F(", frequency ="), frequency);

      return setPWM_Int(frequency, duty);
    }

    ///////////////////////////////////////////

    bool setPWM_Frequency(const float& frequency)
    {
      PWM_LOGDEBUG1(F("setPWM_Frequency: frequency ="), frequency);

      return setPWM_Int(frequency, _dutycycle);
    }

    ///////////////////////////////////////////

    // dutycycle = 0%-100%
    bool setPWM_Dutycycle(const float& dutycycle)
    {
      uint32_t duty = round(remap(dutycycle, 0, 100.0f, 0, (1 << _resolution)));

      PWM_LOGDEBUG1(F("setPWM_Dutycycle: dutycycle ="), duty);

      return setPWM_Int(_frequency, duty);
    }

    ///////////////////////////////////////////

    // dutycycle = 0%-100%
    bool setPWM_Period(const float& period_us, const float& dutycycle)
    {
      uint32_t duty = round(remap(dutycycle, 0, 100.0f, 0, (1 << _resolution)));

      PWM_LOGDEBUG3(F("setPWM_Period: dutycycle ="), duty, F(", period_us ="), period_us);

      return setPWM_Int(round(1000000.0f / period_us), duty);
    }

    ///////////////////////////////////////////

    inline float getActualDutyCycle()
    {
      return ( (float) _dutycycle * 100 / (1 << _resolution) );
    }

    ///////////////////////////////////////////

    inline float getActualFreq()
    {
      return _frequency;
    }

    ///////////////////////////////////////////

    inline float getPWMPeriod()
    {
      return (1000000.0f / _frequency);
    }

    ///////////////////////////////////////////

    inline uint32_t get_freq_CPU()
    {
      return F_CPU;
    }

    ///////////////////////////////////////////

    inline uint32_t getPin()
    {
      return _pin;
    }
    
    ///////////////////////////////////////////

    inline uint32_t getResolution()
    {
      return _resolution;
    }

    ///////////////////////////////////////////////////////////////////

  private:

    float       _frequency;

    // dutycycle from 0-(1 << _resolution) for 0%-100%
    // The range of the _dutycycle values passed to functions depends on selected _resolution and should be from 0 to (2 ** duty_resolution).
    // For example, if the selected duty resolution is 10, then the duty cycle values can range from 0 to 1024. This provides the resolution of ~ 0.1%.
    uint32_t    _dutycycle;
    //////////

    uint8_t     _pin;

#define INVALID_TIMER     255

    ledc_channel_t  _channel;
    ledc_mode_t     _group;
    uint8_t         _timer = INVALID_TIMER;
    uint8_t         _resolution;      // bit size

    ///////////////////////////////////////////
};

///////////////////////////////////////////


#endif    // ESP32_FAST_PWM_HPP

