#ifndef _FIRMWARE_H_
#define _FIRMWARE_H_

#ifdef _VISUAL_STUDIO
#else
#endif
#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>

#define boolean bool

// include for client TCP
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")
#pragma comment(lib, "AdvApi32.lib")

#define HAVE_STRUCT_TIMESPEC
#include <time.h>

#pragma comment(lib, "winmm.lib")
#pragma comment( user, "Compiled on " __DATE__ " at " __TIME__ )

#ifndef _WINDLL
#include <iostream>
#endif

#include <pthread.h>

//#include "HardwareSerial.h"

#include <Windows.h>
#include <chrono>

#include <string>
#include <vector>

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#ifndef _USE_OLD_OSTREAMS
using namespace std;
#endif

#include "main.h"

SOCKET socketClient;

//extern "C"
//{

#ifdef _WINDLL
	#define dll __declspec(dllexport)
	dll int Firmware(void);
#else
	#define dll
	#define Firmware main
#endif

	dll void AbortSimulator(void);
//}

#endif