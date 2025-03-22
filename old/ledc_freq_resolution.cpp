


    int PIN = 18;
    int channel = 8;
    int dutycycle = 4095;
    int SOC_LEDC_TIMER_BIT_WIDTH = 16;
    pinMode(PIN, OUTPUT);
    ledcSetup(channel, 100, 12);
    ledcAttachPin(PIN, channel);
    
    uint32_t min_frequency;
    uint32_t max_frequency;
    uint32_t frequency;
    uint32_t successful_frequency;
    uint32_t max_freq_array[SOC_LEDC_TIMER_BIT_WIDTH];
    uint32_t min_freq_array[SOC_LEDC_TIMER_BIT_WIDTH];

    // Find Max Frequency
    for (uint8_t resolution = 1; resolution <= SOC_LEDC_TIMER_BIT_WIDTH; ++resolution)
    {
        max_freq_array[resolution - 1] = 0;
        min_frequency = 0;
        max_frequency = UINT32_MAX;
        successful_frequency = 0;
        while (min_frequency != max_frequency && min_frequency + 1 != max_frequency)
        {
            frequency = min_frequency + ((max_frequency - min_frequency) / 2);
            if (ledcChangeFrequency(channel, frequency, resolution))
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
        min_freq_array[resolution - 1] = 0;
        min_frequency = 0;
        max_frequency = max_freq_array[resolution - 1];
        successful_frequency = max_frequency;
        while (min_frequency != max_frequency && min_frequency + 1 != max_frequency)
        {
            frequency = min_frequency + ((max_frequency - min_frequency) / 2);
            if (ledcChangeFrequency(channel, frequency, resolution))
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

    // Find all Frequency for specific resolution
    uint8_t resolution = 13;    
    println("dutycycle : ",dutycycle);
    ledcWrite(channel, dutycycle);
    uint32_t freq =0;
    uint32_t freqRet=0;
    for (uint32_t freq = min_freq_array[resolution - 1]; freq <= max_freq_array[resolution - 1]; freq++)
    {

      freqRet = ledcChangeFrequency(channel, freq, resolution);
          print("",freq);println(",",freqRet);
          delay(1);
    }
    println();
    
    ledcDetachPin(PIN);