/********** INCLUDE *********/
#include "Esp32OmniSimulator.h"

/********** VARIABLES *******/
// http://franckh.developpez.com/tutoriels/posix/pthreads/
pthread_t pthread_SETUP;
pthread_t pthread_LOOP;
pthread_t pthread_uart;

bool arret = false;

static void *thread_SETUP(void *p_data)
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    myprintf("Start thread SETUP\n");
    EspClass::startTime();
    setup();
    myprintf("End thread SETUP\n");
    return NULL;
}

static void *thread_LOOP(void *p_data)
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    myprintf("Start thread LOOP\n");
    while (!arret)
    {
        loop();
    }
    myprintf("End thread LOOP\n");
    return NULL;
}

static void* thread_uart(void* p_data)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	myprintf("Start thread Uart\n");

    const char *ip = "192.168.137.1";
    const int port = 20240;
    // ------------------------------------------------------------------------------------------
    // initialize Winsock
    // MAKEWORD(2, 2) is a version,
    // and wsaData will be filled with initialized library information.
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        printf("WSAStartup() error!");
    else
        printf("WSAStartup() succeed.\n");

    // create client socket
    // Protocol Family: PF_INET, PF_INET6, PF_LOCAL, PF_PACKET, PF_IPX
    // Type: SOCK_STREAM, SOCK_DGRAM
    // PF_INET means IPv4, and SOCK_STREAM means TCP.
    SOCKET socketClient = socket(PF_INET, SOCK_STREAM, 0);
    if (socketClient == INVALID_SOCKET)
        printf("socket() error!");
    else
        printf("socket() succeed.\n");
    
    // create server address
    SOCKADDR_IN serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET; // IPv4
    serv_addr.sin_addr.s_addr = inet_addr(ip);
    serv_addr.sin_port = htons(port);
    // AF_INET also means IPv4.htonl and htons convert host-ordered port and address to
    // network order.

    struct addrinfo *result = NULL, *ptr = NULL, hints;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    int iResult = getaddrinfo(ip, to_string(port).c_str(), &hints, &result);
    if (iResult != 0)
    {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        //return 1;
    }

    // connect to server
    printf("ready to connect. \n");
    iResult = connect(socketClient, (SOCKADDR *)&serv_addr, sizeof(serv_addr));
    if (iResult == SOCKET_ERROR)
        printf("connect() error!");
    else
        printf("connect() succeed.\n");

    Serial.socketClient = &socketClient;
    Serial.connected = true;

    // Send an initial buffer
    const char *sendbuf = "!!! Omni Simulator !!!\n";
    iResult = send(socketClient, sendbuf, (int)strlen(sendbuf), 0);
    if (iResult == SOCKET_ERROR)
    {
        printf("send failed with error: %d\n", WSAGetLastError());
    }
    printf(sendbuf);
    printf("Bytes Sent: %ld\n", iResult);
    // ------------------------------------------------------------------------------------------

    // read data
    const int length = 64;
    char buff[length];
    for (size_t i = 0; i < length; i++)
    {
        buff[i] = 0;
    }
    int str_len = 0;

    while (!arret)
    {
        // to show that TCP transportation doesn't have boundary,
        // we can read data multiple time.
        int read_len = 0;
        int idx = 0;
        while (read_len = recv(socketClient, &buff[idx++], 1, 0))
        {
            //printf("reading one time...[%d]\n", idx);
            if (read_len == -1)
            {
                printf("read() error!");
                idx = 0;
                continue;
            }
            str_len += read_len;
            if (buff[idx - 1] == '\n')
            {
                Serial.incoming = std::string(&buff[0], idx);
                for (size_t i = 0; i < idx; i++)
                {
                    buff[i] = 0;
                }
                idx = 0;
            }
        }
        printf("read() succeed. str_len=%d. message from server: %s\n", str_len, buff);
    }
    myprintf("End thread Uart\n");
    return NULL;
}

void AbortSimulator(void)
{
    arret = true;
    int ret = -1;
    myprintf("Abort des Thread ! \n");
    vTaskDelay(1000);
    ret = pthread_cancel(pthread_SETUP);
    ret = pthread_join(pthread_SETUP, NULL);
    ret = pthread_cancel(pthread_LOOP);
    ret = pthread_join(pthread_LOOP, NULL);
    myprintf("Abort des Thread OK :) \n");
}

int Firmware(void)
{
    myprintf("Starting Esp32OmniSimulator !\n");

    int ret = 0;
    
    arret = false;
    // Lancement des thread Interruption, Setup, Loop et UART reception/transmission

    ret = pthread_create(&pthread_uart, NULL, thread_uart, NULL);
    pthread_setname_np(pthread_uart, "UART");
    if (!ret)
    {
        ret = pthread_create(&pthread_SETUP, NULL, thread_SETUP, NULL);
        pthread_setname_np(pthread_SETUP, "SETUP");

        // Wait for Setup to finish then start Loop
        ret = pthread_join(pthread_SETUP, NULL);
        if (!ret)
        {
            ret = pthread_create(&pthread_LOOP, NULL, thread_LOOP, NULL);
            pthread_setname_np(pthread_LOOP, "LOOP");
            if (!ret)
            {
                myprintf("Lancement des thread OK !\n");
            }
        }
    }

#ifndef _WINDLL
    string in = "";
    myprintf("STARTING");
    myprintf(" !\n");
    while (in != "exit" || in != "quit" || in != "close")
    {
        cin >> in;
        // SendUART(in.c_str());
        Serial.incoming = in + '\n';
    }
    AbortSimulator();
#endif

    ret = pthread_join(pthread_LOOP, NULL);
    AbortSimulator();

    closesocket(socketClient);
    WSACleanup();

    return ret;

}
