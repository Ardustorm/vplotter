#ifndef __WIFI_H__
#define __WIFI_H__
#include "sdkconfig.h"

#include <libesphttpd/esp.h>
#include "libesphttpd/httpd.h"


#ifdef CONFIG_ESPHTTPD_USE_ESPFS
#include "espfs.h"
#include "espfs_image.h"
#include "libesphttpd/httpd-espfs.h"
#endif // CONFIG_ESPHTTPD_USE_ESPFS


#include "libesphttpd/cgiwifi.h"
#include "libesphttpd/cgiflash.h"
#include "libesphttpd/auth.h"
#include "libesphttpd/captdns.h"
#include "libesphttpd/cgiwebsocket.h"
#include "libesphttpd/httpd-freertos.h"
#include "libesphttpd/route.h"


#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "freertos/event_groups.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "tcpip_adapter.h"



/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID      CONFIG_EXAMPLE_WIFI_SSID
#define EXAMPLE_WIFI_PASS      CONFIG_EXAMPLE_WIFI_PASSWORD




#define LISTEN_PORT     80u
#define MAX_CONNECTIONS 32u






/* Task that broadcasts time */
void websocketBcast(void *arg);


void httpdInit();
void init_wifi(bool modeAP);


/* send a string over websockets */
void wsDebug( char * str);

/* register variables to allow changing via websockets */
void wsRegisterVariable( void * ptr, char type, char * name);

#endif	/* WIFI_H */
