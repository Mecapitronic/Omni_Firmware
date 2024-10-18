
    // dutycycle from 0-65535 for 0%-100%
    bool setPWM_Int(const uint8_t& pin, const float& frequency, const uint16_t& dutycycle)
    {
      // Convert to new resolution
      if ( _resolution < 16 )
        _dutycycle = dutycycle >> (16 - _resolution);
      else if ( _resolution > 16 )
        _dutycycle = dutycycle << (_resolution - 16);

      PWM_LOGDEBUG3(F("setPWM_Int: _dutycycle ="), _dutycycle,
                    F(", DC % ="), _dutycycle * 100.0f / (1 << _resolution) );

      // Reprogram freq if necessary
      if ( frequency != _frequency)
      {
        PWM_LOGDEBUG3(F("setPWM_Int: change frequency to"), frequency, F("from"), _frequency);

        _frequency  = frequency;

        // To avoid glitch when changing frequency
        // esp_err_t ledc_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz);
        if(frequency !=0)
          ledc_set_freq(_group, (ledc_timer_t) _timer, frequency);
        else
          ledc_stop(_group,_channel,0);

      }

      if(frequency !=0)
      {
            if (dutycycle == 0)
            {
              digitalWrite(pin, LOW);
            }
            else if (dutycycle >= (MAX_COUNT_16BIT - 1) )
            {
              digitalWrite(pin, HIGH);
            }
            else
            {
              ledcWrite(_channel, _dutycycle);
            }
      }
      return true;
    }
