// Implements a mechanism for displaying and controling different variables via websockets

#ifndef WSDATA_H
#define WSDATA_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

struct wsDataStore {
    char * name;
    int type;
    union {
        float * f;
        int  * i;
        void  * v;
    } var;
};

// TODO: These are related, need to make them connected
#define MAX_VAR (64)
#define MAX_PACKET_SIZE (256)   //TODO set


class WsData {
public:
    WsData();
    void add( float * f, char * name);
    void add( int * i, char * name);
    void sendNames(AsyncWebSocketClient * client);
    void sendData(AsyncWebSocketClient * client);
    void processPacket(AsyncWebSocketClient * client, uint8_t *data, size_t len);
    void updateVars(uint8_t *data, size_t len);
private:
    wsDataStore vars[MAX_VAR];
    int currentIndex = 0;
};

#endif // WSDATA_H
