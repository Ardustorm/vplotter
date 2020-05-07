/* Pulse counter module - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


#include "wifi.h"
#include "motor.h"


void testDebug(void *arg);

void app_main()
{


   /* ######################### WIFI ################################# */
#ifdef CONFIG_ESPHTTPD_USE_ESPFS
      espFsInit((void*)(image_espfs_start));
      printf("\nUsing ESPFS\n");
#endif // CONFIG_ESPHTTPD_USE_ESPFS

      tcpip_adapter_init();
      httpdInit();
      init_wifi(false); // Supply false for STA mode
      esp_wifi_connect();
      /* xTaskCreate(websocketBcast, "wsbcast", 3000, NULL, 3, NULL); */

      printf("\nReady: %s\t%s\n", __DATE__, __TIME__);

   /* ############################################################## */

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    /* pcnt_example_init(); */
    encoderInit(PCNT_UNIT_0, 33, 32);
    encoderInit(PCNT_UNIT_1, 26, 25);

    /* Motor */
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, NULL, 5, NULL);

    xTaskCreate(testDebug, "test_debug", 4096, NULL, 5, NULL);

    /* xTaskCreate(encoderEventsTask, "encodeEventsTask", 4096, NULL, 5, NULL); */

}


void testDebug(void *arg) {

   while(1) {
      /* sprintf(debugBuf, "d encoder %d, motorPosition %d \n",  encoderCount(), motorPosition ); */
      /* wsDebug(debugBuf); */
      printf("encoder0: %d \t encoder1: %d\n", encoderCount(PCNT_UNIT_0), encoderCount(PCNT_UNIT_1));
      vTaskDelay(1000/portTICK_RATE_MS);
   }

}
