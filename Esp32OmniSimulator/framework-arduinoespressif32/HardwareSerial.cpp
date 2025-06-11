/********** INCLUDE *********/
#include "HardwareSerial.h"

HardwareSerial::HardwareSerial(uint8_t uart_nr) : _uart_nr(uart_nr) {};
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

void HardwareSerial::setRxBufferSize(int size)
{
    _rxBufferSize = size;
};

void HardwareSerial::setTxBufferSize(int size)
{
    _txBufferSize = size;
};

void HardwareSerial::begin(int baud_speed)
{
    _baudrate = baud_speed;
};

int HardwareSerial::available()
{
    return incoming.length();
};

char HardwareSerial::read()
{
    char c = incoming[0];
    incoming.erase(0, 1);
    return c;
};

size_t HardwareSerial::write(const char c)
{
    if (_uart_nr == s_uart_debug_nr)
    {
        outgoing += c;

        if (c == '\n')
        {
            if (connected)
            {
                int iResult = send(*socketClient, outgoing.c_str(), (int)strlen(outgoing.c_str()), 0);
                if (iResult == SOCKET_ERROR)
                {
                    printf("send failed with error: %d\n", WSAGetLastError());
                }
                else
                {
                    outgoing = "";
                }
            }
        }

        myprintf(c);
    }
    return 1;
};

size_t HardwareSerial::write(const char *str, int length)
{
    int n = 0;
    for (int i = 0; i < length; i++)
    {
        char c = str[i];
        n += write(c);
    }
    return n;
};
//
//size_t HardwareSerial::print(const char *str)
//{
//    if (str != NULL)
//        myprintf(str);
//};

size_t HardwareSerial::print(String str)
{
    int n = 0;
    for (int i = 0; i < str.length(); i++)
    {
        char c = str[i];
        n += write(c);
    }
    return n;
};

size_t HardwareSerial::print(int i)
{
    if (i != NULL)
        return print(String(i));
    return 0;
};

size_t HardwareSerial::println()
{
    return write('\n');
};

//size_t HardwareSerial::println(const char *str)
//{
//    print(str);
//    println();
//};

size_t HardwareSerial::println(String str)
{
    return print(str) + println();
};

size_t HardwareSerial::println(int i)
{
    return print(i) + println();
};

HardwareSerial Serial = HardwareSerial(0);
HardwareSerial Serial0 = HardwareSerial(1);
HardwareSerial Serial1 = HardwareSerial(2);
HardwareSerial Serial2 = HardwareSerial(3);