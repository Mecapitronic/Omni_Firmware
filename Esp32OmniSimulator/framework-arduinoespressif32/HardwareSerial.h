#ifndef HARDWARE_SERIAL_H
#define HARDWARE_SERIAL_H

#include "Arduino.h"
#include "WString.h"
#include <iostream>

using namespace std;

#ifdef _WINDLL
//when in dll mode, we print over the OutputDebugString
void __cdecl myprintf(string str)
{
	OutputDebugString(str.c_str());
}
void __cdecl myprintf(const char* format, ...)
{
	char    buf[4096], * p = buf;
	va_list args;
	int     n;

	va_start(args, format);
	n = _vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
	va_end(args);

	p += (n < 0) ? sizeof buf - 3 : n;

	while (p > buf && isspace(p[-1]))
		*--p = '\0';

	*p++ = '\r';
	*p++ = '\n';
	*p = '\0';

	OutputDebugString(buf);
}

#else
// When in Console mode, we print over it
#define myprintf cout<<
#endif

static uint8_t s_uart_debug_nr = 0; // UART number for debug output

class HardwareSerial
{

protected:
    uint8_t _uart_nr;
    size_t _rxBufferSize;
    size_t _txBufferSize;
    uint32_t _baudrate;

public:
	int bytes = 0;
	string incoming = "";

	HardwareSerial(uint8_t uart_nr);
    ~HardwareSerial();

	void end();
    bool setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin = -1, int8_t rtsPin = -1);
	void setRxBufferSize(int size);
	void setTxBufferSize(int size);
	void begin(int baud_speed);
	int available();
	char read();

    size_t write(const char c);
	size_t write(const char* str, int length);

	//size_t print(const char* str);
	size_t print(String str);
	size_t print(int i);
	size_t println();
	//size_t println(const char* str);
	size_t println(String str);
	size_t println(int i);
};

extern HardwareSerial Serial;
extern HardwareSerial Serial0;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

#endif