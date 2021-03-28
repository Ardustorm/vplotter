#include <ArduinoOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>

#include "wsData.h"
#include "motor.h"

WsData wsData;


float duty = 0;
int32_t position = 0;

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Motor motA = Motor(13, 12,      // motor
                   33, 32,      // encoder
                   500); // frequency

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
    if(type == WS_EVT_DATA){
        AwsFrameInfo * info = (AwsFrameInfo*)arg;
        String msg = "";
        if(info->final && info->index == 0 && info->len == len){
            //the whole message is in a single frame and we got all of it's data
            if(info->opcode == WS_BINARY){
                wsData.processPacket(client, data, len);
            } else {
                Serial.printf("\nNon-Binary WS packet received\n ");
            }
        } else {
            Serial.printf("\nERR: WS data split in multiple frames! This functionality has been removed. "
                          "Either add it back in, or make frames smaller\n\n");
            // github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/ESP_AsyncFSBrowser/ESP_AsyncFSBrowser.ino#L35
        }
    }
}




// const char* ssid = "*******";
// const char* password = "*******";
#include "secrets.h"            //contains ssid and password, can use above instead
const char * hostName = "vplotter";
const char* http_username = "admin";
const char* http_password = "admin";

void setup(){
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(hostName);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("STA: Failed!\n");
        WiFi.disconnect(false);
        delay(1000);
        WiFi.begin(ssid, password);
    }

    ArduinoOTA.setHostname(hostName);
    ArduinoOTA.begin();

    MDNS.addService("http","tcp",80);

    SPIFFS.begin();

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);



    // To access the editor go to url/edit
    server.addHandler(new SPIFFSEditor(SPIFFS, http_username,http_password));


    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/plain", String(ESP.getFreeHeap()));
        });

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

    server.onNotFound([](AsyncWebServerRequest *request){
            Serial.printf("NOT_FOUND: ");
            Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

            request->send(404);
        });
    server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
            if(!index)
                Serial.printf("UploadStart: %s\n", filename.c_str());
            Serial.printf("%s", (const char*)data);
            if(final)
                Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
        });
    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            if(!index)
                Serial.printf("BodyStart: %u\n", total);
            Serial.printf("%s", (const char*)data);
            if(index + len == total)
                Serial.printf("BodyEnd: %u\n", total);
        });
    server.begin();


    wsData.add(&duty, "Duty Cycle");
    wsData.add(&position, "Position");


}

void loop(){
    ArduinoOTA.handle();
    ws.cleanupClients();

    motA.setDuty(duty);
    position = motA.position();
}
