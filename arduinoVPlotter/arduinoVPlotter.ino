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
float position = 0;
float velocityOut = 0;
float velocityKP = 0.1;

// SKETCH BEGIN
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Motor motA = Motor(13, 12,      // motor
                   33, 32,      // encoder
                   200); // frequency

static SemaphoreHandle_t timer_sem;

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

    wsData.add(&velocityOut, "Velocity (rev/sec)");
    wsData.add(&velocityKP, "Velocity Kp");

    // initTimer();


    esp_timer_init();
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic",
        // .skip_unhandled_events = 0
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* Start the timers */
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1953));

    xTaskCreatePinnedToCore(sample_timer_task, "sample_timer", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);

}

void loop(){
    ArduinoOTA.handle();
    ws.cleanupClients();


    // motA.setDuty(duty);

    // motA.setVelocity(duty);
    // motA.testControl(velocityKP);
    // position = motA.getPosition();
    // velocityOut = motA.getVelocity();
    // position = getPosition(0);
    // velocityOut = getVelocity(0);

}






hw_timer_t * timer = NULL;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer(){
   isrCounter++;
  // lastIsrAt = micros();
  // velocityControlLoop(micros());

   static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
   if( xHigherPriorityTaskWoken) {
       portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
   }


}

void initTimer() {
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every  (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 500, true);

  // Start an alarm
  timerAlarmEnable(timer);
}



static void periodic_timer_callback(void* arg)
{
    static int64_t i= 0;
    static int64_t offset= 0;
    static int64_t last= 0;

    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    // return (PCNT_H_LIM_VAL * motorOverflow[encoderNum]) + count;


    int64_t time_since_boot = esp_timer_get_time();

    if(i == 0) {
        offset = time_since_boot;
        last = time_since_boot;
    }
    // float time = ((float)time_since_boot - (i)*2000.0 - offset) / 1000;
    float time = (time_since_boot - last) - 1953;
    printf("%f\t %d\n", time, count);
    i++;
    last = time_since_boot;
}





void sample_timer_task(void *param)
{
    timer_sem = xSemaphoreCreateBinary();

    // timer group init and config goes here (timer_group example gives code for doing this)
    initTimer();

    // timer_isr_register(timer_group, timer_idx, timer_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    static int64_t last= esp_timer_get_time();
    int64_t offset= esp_timer_get_time();
    int64_t time_since_boot;
    int64_t max =0;
    while (1) {
        xSemaphoreTake(timer_sem, portMAX_DELAY);
        time_since_boot = esp_timer_get_time();

        int64_t time = (time_since_boot - last);
        // sample sensors via i2c here
        if((time - 500) > 25)
            printf("\t%lld\n", (time -500));
        // push sensor data to another queue, or send to a socket...

        if( time > max) {
            max = time;
            printf("%lld\n", (max - 500));
        }

        last= time_since_boot;
    }
}

void IRAM_ATTR timer_isr_handler(void *param)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // TIMERG0.hw_timer[timer_idx].update = 1;
    // any other code required to reset the timer for next timeout event goes here

    xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
    }
}
