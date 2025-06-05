#ifndef Arduino_h
#define Arduino_h

#define ARDUINO_USB_MODE 1
#define ARDUINO_USB_CDC_ON_BOOT 1
#define SIMULATOR

#include <Windows.h>
#include <chrono>

#include "HardwareSerial.h"
#include "WString.h"
#include <pins_arduino.h>

typedef uint32_t TickType_t;

#define PI 3.1415926535897932384626433832795
#define M_PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
// #define EULER 2.718281828459045235360287471352

#define ARDUINO_BOARD "Visual Studio Simulator"

#define F_CPU 240000000L

#define CONFIG_FREERTOS_HZ  1000
#define configTICK_RATE_HZ  ( CONFIG_FREERTOS_HZ )
#define portTICK_PERIOD_MS  ( ( TickType_t ) 1000 / configTICK_RATE_HZ )

#define LOW               0x0
#define HIGH              0x1

//GPIO FUNCTIONS
#define INPUT             0x01
#define OUTPUT            0x03 


#include "esp_arduino_version.h"
#include "esp_idf_version.h"

#define CONFIG_DISABLE_HAL_LOCKS true

/*
#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

//Interrupt Modes
#define RISING    0x01
#define FALLING   0x02
#define CHANGE    0x03
#define ONLOW     0x04
#define ONHIGH    0x05
#define ONLOW_WE  0x0C
#define ONHIGH_WE 0x0D

#define DEFAULT 1
#define EXTERNAL 0

#ifndef __STRINGIFY
#define __STRINGIFY(a) #a
#endif
*/

// can't define max() / min() because of conflicts with C++
/*
#define _min(a,b) ((a)<(b)?(a):(b))
#define _max(a,b) ((a)>(b)?(a):(b))
#define _abs(x) ((x)>0?(x):-(x))  // abs() comes from STL
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  // round() comes from STL
*/
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))


// Get time stamp in milliseconds.
/*uint64_t millis()
{
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();
        return ms;
}*/

// Get time stamp in microseconds.
/*uint64_t micros()
{
        uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();
        return us;
}*/

// Get time stamp in nanoseconds.
/*uint64_t nanos()
{
        uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();
        return ns;
}*/

#define vTaskDelay(x) EspClass::timerSleep(x / 1000)
#define delay vTaskDelay

#define millis() EspClass::getTime() / 1e3
#define micros() EspClass::getTime()

#define temperatureRead() EspClass::TemperatureRead()
#define getCpuFrequencyMhz() EspClass::GetCpuFrequencyMhz()

#define pinMode(x,y) EspClass::PinMode(x,y)
#define digitalWrite(x,y) EspClass::DigitalWrite(x,y)
#define digitalRead(x) EspClass::DigitalRead(x)


#define LOW 0x0
#define HIGH 0x1

// GPIO FUNCTIONS
#define INPUT 0x01
// Changed OUTPUT from 0x02 to behave the same as Arduino pinMode(pin,OUTPUT)
// where you can read the state of pin even when it is set as OUTPUT
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG 0xC0

// Interrupt Modes
#define DISABLED 0x00
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define ONLOW 0x04
#define ONHIGH 0x05
#define ONLOW_WE 0x0C
#define ONHIGH_WE 0x0D

class EspClass
{
   public:
    EspClass() {}
    ~EspClass() {}
    void restart();

    static float TemperatureRead()
    {
        return (float)37.2;
    }

    static uint32_t GetCpuFrequencyMhz()
    {
        return 240;
    }

    static void PinMode(uint8_t pin, uint8_t mode) {}
    static void DigitalWrite(uint8_t pin, uint8_t val) {}
    static int DigitalRead(uint8_t pin) { return 0; }

    static void timerSleep(double seconds);
    static void startTime();
    static unsigned long getTime();
};

extern EspClass ESP;

void setup();
void loop();

#endif
