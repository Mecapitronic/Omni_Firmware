/********** INCLUDE *********/
#include "HardwareSerial.h"

HardwareSerial::HardwareSerial() {};
HardwareSerial::~HardwareSerial() {};

void HardwareSerial::end() {};
// negative Pin value will keep it unmodified
// can be called after or before begin()
bool HardwareSerial::setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin)
{
    // map logical pins to GPIO numbers
    //rxPin = digitalPinToGPIONumber(rxPin);
    //txPin = digitalPinToGPIONumber(txPin);
    //ctsPin = digitalPinToGPIONumber(ctsPin);
    //rtsPin = digitalPinToGPIONumber(rtsPin);
    return true;
}

void HardwareSerial::setRxBufferSize(int size) {};
void HardwareSerial::setTxBufferSize(int size) {};
void HardwareSerial::begin(int baud_speed) {};
void HardwareSerial::print() {};
void HardwareSerial::print(const char* str) { if(str != NULL) myprintf(str); };
void HardwareSerial::print(String str) {
	if (str != "")
	{
		string s = str.c_str();
		myprintf(s);
	}
};
void HardwareSerial::print(int i) { if (i != NULL) myprintf(i); };
void HardwareSerial::println() { myprintf('\n'); };
void HardwareSerial::println(const char* str) { print(str); println(); };
void HardwareSerial::println(String str) { print(str); println(); };
void HardwareSerial::println(int i) { print(i); println(); };
int HardwareSerial::available() { return bytes; };

char HardwareSerial::read()
{
	char c = incoming[0];
	incoming.erase(0,1);
	bytes = incoming.length();
	return c;
};
void HardwareSerial::write(const char* str, int length)
{
	for (int i = 0; i < length; i++)
	{
		char c = str[i];
		myprintf(c);
	} 
};
void HardwareSerial::write(const char c)
{
     myprintf(c);
};

HardwareSerial Serial = HardwareSerial();
HardwareSerial Serial0 = HardwareSerial();
HardwareSerial Serial1 = HardwareSerial();
HardwareSerial Serial2 = HardwareSerial();