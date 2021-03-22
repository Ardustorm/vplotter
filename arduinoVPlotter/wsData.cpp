#include "wsData.h"


WsData::WsData() {
}

void WsData::add( float * f, char * name) {
    if (currentIndex < MAX_VAR) {
        vars[currentIndex].name = name;
        vars[currentIndex].type = 'f';
        vars[currentIndex].var.f = f;
        currentIndex++;
    }
}

void WsData::add( int * i, char * name) {
    if (currentIndex < MAX_VAR) {
        vars[currentIndex].name = name;
        vars[currentIndex].type = 'i';
        vars[currentIndex].var.i = i;
        currentIndex++;

    }
}


/**
 * Sends a binary WS packet containing variable numbers followed by a null terminated name.
 * This function will send multiple packets if necessary.
 *
 * If a variable's name is too long to fit in a packet, it is skipped
 * example packet: ['S', 0, 'v','a','r','1','\0', 1, 'v','a','r','2','\0', ...]
 **/
void WsData::sendNames(AsyncWebSocketClient * client) {
    char out[MAX_PACKET_SIZE];               // TODO
    out[0] = 'S';
    int i = 1;

        Serial.printf("Start of function\n");
    for (int varNum=0; varNum < currentIndex; varNum++) {
        int nameLength = strlen(vars[varNum].name);
        Serial.printf("Name length[%d]: %d\n", varNum, nameLength);

        // Check if we have room for var #, var name, & null termination
        if((i + 1 + nameLength + 1 ) >= MAX_PACKET_SIZE) { // no room:
            Serial.printf("Sending packet\n");
            // send buff and reset
            client->binary(out,i);
            i=0;
            // Check again since name could be too large. if so, skip
            if((i + 1 + nameLength + 1 ) >= MAX_PACKET_SIZE) {
                // TODO: handle large names more gracefully? send debug message?
                Serial.printf("Packet too large\n");
                continue;
            }
        }
        Serial.printf("About to pack buffer[%d]\n", i);
        out[i] = varNum;
        memcpy(&out[i+1], (void *) vars[varNum].name, nameLength);
        out[i+nameLength+1] = '\0';
        i += 1 + nameLength + 1;
    }
    // send out
                Serial.printf("Sending packet end\n");

    client->binary(out,i);
}

void WsData::sendData(AsyncWebSocketClient * client) {
    char out[1 + MAX_VAR * 5];
    out[0] = 'D';
    int i = 0;
    while (i < currentIndex) {
        out[i*5 + 1] = (char) i;
        memcpy(&out[i*5+2], (void *)vars[i].var.f, sizeof(float)); //todo check that it works for floats and ints
        i++;
    }
    // send out
    client->binary(out,i*5+2);
}


void WsData::processPacket(AsyncWebSocketClient * client, uint8_t *data, size_t len){
    if(len == 0) { return; }

    switch(data[0]) {
    case 'S':
        Serial.printf("Setup request\n");
        sendNames(client);
        // TODO: eventually we will check for more data/settings here
        break;
    case 'D':
        sendData(client);
        // TODO: eventually we will check this to see if we need to update any variables
        break;
    default:
        Serial.printf("Unknown packet type: %c\n", data[0]);
        break;
    }

}
