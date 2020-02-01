
#include "wifi.h"


#define TAG "user_main"

char my_hostname[16] = "esphttpd";


static void ota_server_task(void * param);  /* OTA */

static char connectionMemory[sizeof(RtosConnType) * MAX_CONNECTIONS];
static HttpdFreertosInstance httpdFreertosInstance;


void wsDebug( char * str) {
   cgiWebsockBroadcast(&httpdFreertosInstance.httpdInstance,
			  "/websocket/ws.cgi", str, strlen(str),
			  WEBSOCK_FLAG_NONE);

}

/* 
   struct:
   - name
   - address
   - type

 */
/* how long of a string is allowed to identify a variable */
#define WS_MAX_VAR_NAME (32)
/* Maximum number of variables that can be registered */
#define WS_MAX_VAR_NUMBER (16)


typedef struct {
   void * ptr;
   char type;
   char name[WS_MAX_VAR_NAME];
} WsVariableEntry;


/* Store variables */
WsVariableEntry wsStoredVariables[WS_MAX_VAR_NUMBER] = {0};

/* 
   ptr must be statically allocated (either global of static)
   type such as:
      c   : char
      d/i : int
      u   : unsigned int
      l   : long
      f   : float
   name: the string to display to user and to use as id.
 */
void wsRegisterVariable( void * ptr, char type, char * name) {
   char debugBuf[64];
   sprintf(debugBuf, "d %s 0 \n",  name );
   wsDebug(debugBuf);

   
   int i = 0;
   while(i < WS_MAX_VAR_NUMBER) {
      if( wsStoredVariables[i].ptr == NULL) {
	 wsStoredVariables[i].ptr = ptr;
	 wsStoredVariables[i].type = type;
	 strncpy(wsStoredVariables[i].name, name, WS_MAX_VAR_NAME);
	 return;
      }
      i++;
   }
   printf("\n### FAILED TO REGISTER VARIABLE\n\n");

}

/* given a variable, and a string representing the data, set the variable
   to the new value */
void setStoredVariable( WsVariableEntry variable, char * data) {
   if( variable.type == 'l' ) {
      *(int32_t*)(variable.ptr) = strtol( data, NULL, 10);
      /* printf("##SETVARIABLE %s to %d\n", variable.name, *(int32_t*)(variable.ptr) ); */
   } else if( variable.type == 'f') {
      *(float*)(variable.ptr) = strtof( data, NULL);
      /* printf("##SETVARIABLE %s to %f\n", variable.name, *(float*)(variable.ptr) ); */
   } else {
      printf("### UNSUPPORTED 'setStoredVariable' CALL!\n");
   }
}


/* If the packet starts with a 'v ' then this function is called.
   If there is only the v, then we reply with the registered variables
   using the wsReplyRegisteredVariables function, otherwise we try
   and evaluate the packet and set the correct variable.
   Example packet:
   'v varName 100'
 */
void processVariablePacket(Websock *ws, char * data, int len) {
   char * varName = NULL;
   char *saveptr = NULL;		/* for strtok */
   char buf[64];

   strncpy(buf, data, len);
   buf[len] = '\0';
   data = buf;
   
   if( len <= 2 ) {		/* Empty packet, reply with registered */
      wsReplyRegisteredVariables(ws);

   } else {
      data += 2;		/* remove starting 'v ' */
      len -= 2;
      varName = strtok_r(data, " ", &saveptr);

      int i = 0;
      while(i < WS_MAX_VAR_NUMBER && wsStoredVariables[i].ptr != NULL) { /* check all stored variables */
	 if( strcmp(wsStoredVariables[i].name, varName) == 0 ) {
	    setStoredVariable( wsStoredVariables[i], strtok_r( NULL, ",", &saveptr ));
	    return;
	 }
	 i++;
      }
      printf("### UNSUPPORTED 'processVariablePacket', cant find: <%s>!\n", varName);
   }

}

/* if recieved a request for registerd variables, reply */
void wsReplyRegisteredVariables(Websock *ws) {
   char buf[(WS_MAX_VAR_NAME+10) * WS_MAX_VAR_NUMBER + 10];
   char tempBuf[32];
   strcpy(buf , "v ");

   int i = 0;
   while(i < WS_MAX_VAR_NUMBER && wsStoredVariables[i].ptr != NULL) {
      strncat(buf , wsStoredVariables[i].name, WS_MAX_VAR_NAME);
      /* variable */
      switch(wsStoredVariables[i].type) {
	 case 'i':
	    sprintf(tempBuf," %i, ",*(int *)wsStoredVariables[i].ptr);
	    strncat(buf, tempBuf, WS_MAX_VAR_NAME);
	    break;
	 case 'l':
	    sprintf(tempBuf," %ld, ",*(long *)wsStoredVariables[i].ptr);
	    strncat(buf, tempBuf, WS_MAX_VAR_NAME);
	    break;
	 case 'f':
	    sprintf(tempBuf," %f, ",*(float *)wsStoredVariables[i].ptr);
	    strncat(buf, tempBuf, WS_MAX_VAR_NAME);
	    break;
	 case 's':
	    strcat(buf , " ");
	    strncat(buf , (char *)wsStoredVariables[i].ptr, WS_MAX_VAR_NAME);
	    strcat(buf , ", ");
	    break;
	 default:
	    strcat(buf , " NOT IMPLEMENTED, ");
	    break;
      }
      i++;
   }
   strcat(buf , "\n");
   printf("sending: <%s>\n",buf);
   cgiWebsocketSend(&httpdFreertosInstance.httpdInstance,
		    ws, buf, strlen(buf), WEBSOCK_FLAG_NONE);
}

void setVariables(char * data) {
   /* parse first space
      seach registered variables for entry
      
    */
   
}



//Broadcast the uptime in seconds every second over connected websockets
void websocketBcast(void *arg) {
   static int ctr=0;
   char buff[128];
   while(1) {
      ctr++;
      sprintf(buff, "d minutes %d, seconds %d, const 3.14\n", ctr/60, ctr%60);
      cgiWebsockBroadcast(&httpdFreertosInstance.httpdInstance,
			  "/websocket/ws.cgi", buff, strlen(buff),
			  WEBSOCK_FLAG_NONE);

      vTaskDelay(10/portTICK_RATE_MS);
   }
}

//On reception of a message, send "You sent: " plus whatever the other side sent
static void myWebsocketRecv(Websock *ws, char *data, int len, int flags) {
   int i;
   char buff[128];
   sprintf(buff, "You sent: ");
   for (i=0; i<len; i++) buff[i+10]=data[i];
   buff[i+10]=0;

   if(data[0] == 'v') {		/* process request for variables */
      processVariablePacket(ws, data, len);
   }
   
   cgiWebsocketSend(&httpdFreertosInstance.httpdInstance,
		    ws, buff, strlen(buff), WEBSOCK_FLAG_NONE);
}

//Websocket connected. Install reception handler and send welcome message.
static void myWebsocketConnect(Websock *ws) {
   ws->recvCb=myWebsocketRecv;
   cgiWebsocketSend(&httpdFreertosInstance.httpdInstance,
		    ws, "Hi, Websocket!", 14, WEBSOCK_FLAG_NONE);
}

//On reception of a message, echo it back verbatim
void myEchoWebsocketRecv(Websock *ws, char *data, int len, int flags) {
   printf("EchoWs: echo, len=%d\n", len);
   cgiWebsocketSend(&httpdFreertosInstance.httpdInstance,
		    ws, data, len, flags);
}

//Echo websocket connected. Install reception handler.
void myEchoWebsocketConnect(Websock *ws) {
   printf("EchoWs: connect\n");
   ws->recvCb=myEchoWebsocketRecv;
}


CgiStatus ICACHE_FLASH_ATTR tplBuildInfo(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	if (strcmp(token, "build")==0) {
	   sprintf(buff, "Build Date: %s\tTime: %s\n", __DATE__, __TIME__);
	}
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}


/*
  This is the main url->function dispatching data struct.
  In short, it's a struct with various URLs plus their handlers. The handlers can
  be 'standard' CGI functions you wrote, or 'special' CGIs requiring an argument.
  They can also be auth-functions. An asterisk will match any url starting with
  everything before the asterisks; "*" matches everything. The list will be
  handled top-down, so make sure to put more specific rules above the more
  general ones. Authorization things (like authBasic) act as a 'barrier' and
  should be placed above the URLs they protect.
*/
HttpdBuiltInUrl builtInUrls[] = {
				 ROUTE_CGI_ARG("*", cgiRedirectApClientToHostname, "esp8266.nonet"),
				 ROUTE_REDIRECT("/", "/index.tpl"),

				 ROUTE_TPL("/index.tpl", tplBuildInfo),

				 ROUTE_REDIRECT("/wifi", "/wifi/wifi.tpl"),
				 ROUTE_REDIRECT("/wifi/", "/wifi/wifi.tpl"),
				 ROUTE_CGI("/wifi/wifiscan.cgi", cgiWiFiScan),
				 ROUTE_TPL("/wifi/wifi.tpl", tplWlan),
				 ROUTE_CGI("/wifi/connect.cgi", cgiWiFiConnect),
				 ROUTE_CGI("/wifi/connstatus.cgi", cgiWiFiConnStatus),
				 ROUTE_CGI("/wifi/setmode.cgi", cgiWiFiSetMode),
				 ROUTE_CGI("/wifi/startwps.cgi", cgiWiFiStartWps),
				 ROUTE_CGI("/wifi/ap", cgiWiFiAPSettings),

				 ROUTE_REDIRECT("/websocket", "/websocket/index.html"),
				 ROUTE_WS("/websocket/ws.cgi", myWebsocketConnect),
				 ROUTE_WS("/websocket/echo.cgi", myEchoWebsocketConnect),

				 ROUTE_FILESYSTEM(),

				 ROUTE_END()
};



static esp_err_t app_event_handler(void *ctx, system_event_t *event)
{
   switch(event->event_id) {
      case SYSTEM_EVENT_ETH_START:
	 tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_ETH, my_hostname);
	 break;
      case SYSTEM_EVENT_STA_START:
	 tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, my_hostname);
	 // esp_wifi_connect(); /* Calling this unconditionally would interfere with the WiFi CGI. */
	 break;
      case SYSTEM_EVENT_STA_GOT_IP:
	 {
	    tcpip_adapter_ip_info_t sta_ip_info;
	    wifi_config_t sta_conf;
	    /* xEventGroupSetBits(wifi_event_group, CONNECTED_BIT); /\* for OTA *\/ */
	    printf("~~~~~STA~~~~~" "\n");
	    if (esp_wifi_get_config(TCPIP_ADAPTER_IF_STA, &sta_conf) == ESP_OK) {
	       printf("ssid: %s" "\n", sta_conf.sta.ssid);
	    }

	    if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &sta_ip_info) == ESP_OK) {
	       printf("IP:" IPSTR "\n", IP2STR(&sta_ip_info.ip));
	       printf("MASK:" IPSTR "\n", IP2STR(&sta_ip_info.netmask));
	       printf("GW:" IPSTR "\n", IP2STR(&sta_ip_info.gw));
	    }
	    printf("~~~~~~~~~~~~~" "\n");

	    xTaskCreate(&ota_server_task, "ota_server_task", 8096, NULL, 5, NULL);
	    /* ota_server_start(); */
	 }
	 break;
      case SYSTEM_EVENT_STA_CONNECTED:
	 break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
	 /* This is a workaround as ESP32 WiFi libs don't currently
	    auto-reassociate. */
	 /* Skip reconnect if disconnect was deliberate or authentication      *\
	    \* failed.                                                            */
	 switch(event->event_info.disconnected.reason){
	    case WIFI_REASON_ASSOC_LEAVE:
	    case WIFI_REASON_AUTH_FAIL:
	       break;
	    default:
	       esp_wifi_connect();
	       break;
	 }
	 break;
      case SYSTEM_EVENT_AP_START:
	 {
	    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, my_hostname);
	 }
	 break;
      case SYSTEM_EVENT_AP_STACONNECTED:
	 ESP_LOGI(TAG, "station:" MACSTR" join,AID=%d",
		  MAC2STR(event->event_info.sta_connected.mac),
		  event->event_info.sta_connected.aid);
	 break;
      case SYSTEM_EVENT_AP_STADISCONNECTED:
	 ESP_LOGI(TAG, "station:" MACSTR"leave,AID=%d",
		  MAC2STR(event->event_info.sta_disconnected.mac),
		  event->event_info.sta_disconnected.aid);

	 wifi_sta_list_t sta_list;
	 ESP_ERROR_CHECK( esp_wifi_ap_get_sta_list(&sta_list));
	 break;
      case SYSTEM_EVENT_SCAN_DONE:

	 break;
      default:
	 break;
   }

   /* Forward event to to the WiFi CGI module */
   cgiWifiEventCb(event);

   return ESP_OK;
}


/* wait till we get an IP, then start OTA server */
static void ota_server_task(void * param)
{
    /* xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY); */
    ota_server_start();
    vTaskDelete(NULL);
}

void httpdInit() {

   httpdFreertosInit(&httpdFreertosInstance,
		     builtInUrls,
		     LISTEN_PORT,
		     connectionMemory,
		     MAX_CONNECTIONS,
		     HTTPD_FLAG_NONE);
   httpdFreertosStart(&httpdFreertosInstance);
}


//Simple task to connect to an access point
void init_wifi(bool modeAP) {
   esp_err_t result;

   result = nvs_flash_init();
   if(   result == ESP_ERR_NVS_NO_FREE_PAGES
	 || result == ESP_ERR_NVS_NEW_VERSION_FOUND)
      {
	 ESP_LOGI(TAG, "Erasing NVS");
	 nvs_flash_erase();
	 result = nvs_flash_init();
      }
   ESP_ERROR_CHECK(result);

   // Initialise wifi configuration CGI
   result = initCgiWifi();
   ESP_ERROR_CHECK(result);

   ESP_ERROR_CHECK( esp_event_loop_init(app_event_handler, NULL) );

   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
   ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

   //Go to station mode
   esp_wifi_disconnect();

   if(modeAP) {
      ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );

      wifi_config_t ap_config;
      strcpy((char*)(&ap_config.ap.ssid), "ESP");
      ap_config.ap.ssid_len = 3;
      ap_config.ap.channel = 1;
      ap_config.ap.authmode = WIFI_AUTH_OPEN;
      ap_config.ap.ssid_hidden = 0;
      ap_config.ap.max_connection = 3;
      ap_config.ap.beacon_interval = 100;

      esp_wifi_set_config(WIFI_IF_AP, &ap_config);
   }
   else {
      esp_wifi_set_mode(WIFI_MODE_STA);

      //Connect to the defined access point.
      wifi_config_t config;
      memset(&config, 0, sizeof(config));
      sprintf((char*)config.sta.ssid, EXAMPLE_WIFI_SSID);
      sprintf((char*)config.sta.password, EXAMPLE_WIFI_PASS);
      esp_wifi_set_config(WIFI_IF_STA, &config);
      esp_wifi_connect();
   }

   ESP_ERROR_CHECK( esp_wifi_start() );


}

