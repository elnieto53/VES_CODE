/*
 * udpManager.c
 *
 *  Created on: 28 sept. 2020
 *      Author: sreal
 */

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "transportLayer.h"
#include "wifiAP.h"
//#include <select.h>"

#include "wifiManager.h"


typedef struct  pkgData {//__attribute__((__packed__))
	uint8_t *data;
	uint16_t length;
	in_port_t port;
	in_addr_t ipAddress;
} pkgData;

typedef struct socketCallback_ {
	int sockfd;
	rxCallback callback;
} socketCallback_t;

static genList_t* socketCallbackList;

static QueueHandle_t txQueue;

#define WIFI_MAX_CONNECTION_RETRY  10

/* Event group bits mapping */
#define WIFI_CONNECTED_BIT 		BIT0
#define WIFI_SCAN_COMPLETE_BIT 	BIT1
#define WIFI_START_COMPLETE_BIT BIT2

#define WIFI_SCAN_LIST_MAX_SIZE 15

/* FreeRTOS event group to signal when we are connected*/
//static EventGroupHandle_t wifiEventGroup;

static const char *STA_TAG = "WiFi";
static const char *UDP_TAG = "UDP";

//static int connectRetryCounter = 0;

/*List of available access points. A lower index results into a higher connection priority
 * Examples added:
 * */
//wifi_sta_config_t registeredAP[] = {
//	{ .ssid = "Knight025_Net", .password = "pasatiempo", .pmf_cfg = { .capable = true, .required = false } },
//	{ .ssid = "AndroidAP_SRV", .password = "pasatiempo", .pmf_cfg = { .capable = true, .required = false } },
//};

//static void wifi_staInit(void);
//static void wifi_apInit(void);
//static void wifi_eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void udpSenderTask (void *pvParameters);
static void udpReceiverTask ();
static int buildFdSet(fd_set* rxSockets);
static retval_t initRXSockets(genList_t *socketDataList);
static retval_t deInitRXSockets(void);
static pkgData initPkgData(in_addr_t addr, in_port_t port, uint8_t *data, uint16_t dataLength);
static void deletePkgData(pkgData *pkg);

#if 1

retval_t transportLayerInit(wifi_mode_t mode, genList_t *rxSocketDataList){
	wifi_ap_record_t ap;
	wifi_ap_data_t apData;

	WiFi_Init(mode);

	if(mode == WIFI_MODE_APSTA || mode == WIFI_MODE_STA){
		if(WiFi_searchRegisteredAP(&ap, &apData) != RET_OK)
			ESP_LOGE(STA_TAG, "Could not find AP");
		ESP_LOGI(STA_TAG, "SSID detected: %s. RRSI measured: %i", apData.ssid, ap.rssi);
		if(WiFi_connectToAP(&apData) != RET_OK)
			ESP_LOGE(STA_TAG, "Could not connect to the AP");
	}

//	switch(mode){
//		case(WIFI_MODE_STA):
//			wifi_staInit();
//			break;
//		case(WIFI_MODE_AP):
//			wifi_apInit();
//			break;
//		default:
//			return RET_ERROR;
//			break;
//	}


	txQueue = xQueueCreate(7, sizeof(pkgData));
	xTaskCreate(udpSenderTask, "udpTX", 4096, NULL, 5, NULL);
	initRXSockets(rxSocketDataList);
	xTaskCreate(udpReceiverTask, "udpRX", 4096, rxSocketDataList, 5, NULL);
	return RET_OK;
}


static retval_t initRXSockets(genList_t *socketDataList){
	genListElement_t *currentEntry = socketDataList->tailElement;
	socketCallback_t *newSocketCallback;
	int err;

	struct sockaddr_in dest_addr = {
		.sin_addr.s_addr = htonl(INADDR_ANY),
		.sin_family = AF_INET,
	};

	socketCallbackList = genListInit();

	while(currentEntry != NULL){
		newSocketCallback = (socketCallback_t*)pvPortMalloc(sizeof(socketCallback_t));
		newSocketCallback->callback = ((socketData_t*)currentEntry->item)->callback;
		newSocketCallback->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //family: AF_INET; ip_protocol: IP

		dest_addr.sin_port = htons(((socketData_t*)currentEntry->item)->port);
		err = bind(newSocketCallback->sockfd, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (err < 0) {
			ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
			deInitRXSockets();
			return RET_ERROR;
		}
		ESP_LOGI(UDP_TAG, "Socket created with fd '%d' and bound to the port %d", newSocketCallback->sockfd, ((socketData_t*)currentEntry->item)->port);
		genListAdd(socketCallbackList, newSocketCallback);
		currentEntry = currentEntry->next;
	}

	return RET_OK;
}


static retval_t deInitRXSockets(void){
	genListElement_t *current = socketCallbackList->tailElement;

	while(current != NULL){
		close(((socketCallback_t*)current->item)->sockfd);
		current = current->next;
	}
	genListRemoveAndDeleteAll(socketCallbackList);

	return RET_OK;
}


static void udpSenderTask (void *pvParameters)
{
	struct sockaddr_in dest_addr = { .sin_family = AF_INET };
	pkgData txPkg;
	int txError;

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //family: AF_INET; ip_protocol: IP
	if (sock < 0) {
		ESP_LOGE(UDP_TAG, "Unable to create tx socket: errno %d", errno);
		vTaskDelete(NULL);
	}
	ESP_LOGI(UDP_TAG, "Socket created");

	while (1) {
		if(xQueueReceive(txQueue, &txPkg, portMAX_DELAY) != pdTRUE ){
			ESP_LOGE(UDP_TAG, "Error occurred in txQueue");
		}

		dest_addr.sin_port = txPkg.port;
		dest_addr.sin_addr.s_addr = txPkg.ipAddress;
		txError = sendto(sock, (txPkg.data), (size_t)(txPkg.length), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (txError < 0) {
			ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", txError);
		}
		deletePkgData(&txPkg);
		//ESP_LOGI(UDP_TAG, "Message sent");
	}
}

static void fd_zero_fix(int p)	{
        fd_set *_p;
        int _n;

        _p = (p);
        _n = _howmany(FD_SETSIZE, _NFDBITS);
        while (_n > 0)
                _p->__fds_bits[--_n] = 0;
}


static int buildFdSet(fd_set* rxSocketFdSet){
	genListElement_t* current = socketCallbackList->tailElement;
	int fdMax = 0;
	int currentSocket;

	//FD_ZERO(rxSocketFdSet);
	fd_zero_fix(rxSocketFdSet);
	while(current != NULL){
		currentSocket = ((socketCallback_t*) current->item)->sockfd;
		FD_SET(currentSocket, rxSocketFdSet);
		if(fdMax < currentSocket)
			fdMax = currentSocket;
		current = current->next;
	}
	return fdMax;
}

//static void buildFd(int fd, fd_set* fdset){
////	FD_ZERO(fdset);
//	FD_CLR(fd, fdset);
//	FD_SET(fd, fdset);
//}


static void udpReceiverTask (){
    uint8_t rxBuffer[128];
    char addr_str[128];
    genListElement_t *current;
    socketCallback_t* currentItem;
    fd_set rxSocketsFdSet;
    int length;
    int fdMax;

	struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
	socklen_t socklen = sizeof(source_addr);


	fdMax = buildFdSet(&rxSocketsFdSet);
	ESP_LOGI(UDP_TAG, "Max fd: %d", fdMax);
	while (1) {
		ESP_LOGI(UDP_TAG, "Reception started");

		/*Rebuild rxSockets before calling 'select' again*/
		buildFdSet(&rxSocketsFdSet);
		if(select(fdMax+1, &rxSocketsFdSet, NULL, NULL, NULL) < 0){
			ESP_LOGE(UDP_TAG, "Error occurred during reception (select)");
		}

		//ESP_LOGI(UDP_TAG, "Reception: changes detected in file descriptors...");

		current = socketCallbackList->tailElement;
		while(current != NULL){
			currentItem = (socketCallback_t*) current->item;
			if(FD_ISSET(currentItem->sockfd, &rxSocketsFdSet)){
				//ESP_LOGI(UDP_TAG, "Socket ready to receive, with fd: %d", currentItem->sockfd);
				length = recvfrom(currentItem->sockfd, rxBuffer, sizeof(rxBuffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
				if (length < 0){
					ESP_LOGE(UDP_TAG, "Error occurred during reception: errno %d", errno);
				}else{
					currentItem->callback(source_addr.sin_addr.s_addr, rxBuffer, length);
				}
				inet_ntoa_r(source_addr.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
				ESP_LOGI(UDP_TAG, "Package received from: %s, fd: '%d'", addr_str, currentItem->sockfd);

//				buildFd(currentItem->sockfd, &rxSocketsFdSet);
//				FD_SET(currentItem->sockfd, &rxSockets);
			}
			current = current->next;
		}
	}
}

#endif



#if 0
static int APMatch(wifi_sta_config_t* apList, wifi_ap_record_t* apRecords, uint16_t apNumber){
	int i = -1;
	while (apList[++i].ssid[0] != '\0'){
		for(int j = 0; j < apNumber; j++){
			if(strcmp((char*)&apRecords[j].ssid, (char*)&apList[i].ssid) == 0){
				return i;
			}
		}
	}
	return -1;
}


static void wifi_eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	ESP_LOGI(STA_TAG, "Entered event handler");

	/*STA Configuration*/
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//    	ESP_LOGI(STA_TAG, "Sending 'Start completed'...");
    	xEventGroupSetBits(wifiEventGroup, WIFI_START_COMPLETE_BIT);
    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE){
//    	ESP_LOGI(STA_TAG, "Sending 'Scan completed'...");
    	xEventGroupSetBits(wifiEventGroup, WIFI_SCAN_COMPLETE_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    	ESP_LOGI(STA_TAG, "Trying to reconnect...");
        if (connectRetryCounter < WIFI_MAX_CONNECTION_RETRY) {
            esp_wifi_connect();
            connectRetryCounter++;
            ESP_LOGI(STA_TAG, "retry to connect to the AP");
        }
        ESP_LOGI(STA_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(STA_TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        connectRetryCounter = 0;
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }

    /*AP Configuration*/
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
		ESP_LOGI(UDP_TAG, "station "MACSTR" join, AID=%d",
				 MAC2STR(event->mac), event->aid);
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
		ESP_LOGI(UDP_TAG, "station "MACSTR" leave, AID=%d",
				 MAC2STR(event->mac), event->aid);
	}
}


static void wifi_apInit(){
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_t* apHandle = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_eventHandler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(WIFI_SSID) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(UDP_TAG, "wifi_init_softap finished. SSID: %s password: %s channel: %d",
             WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}


static void wifi_staInit()
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifiEventGroup = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_eventHandler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_eventHandler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    EventBits_t bits = 0;
    uint16_t number = WIFI_SCAN_LIST_MAX_SIZE;
    wifi_ap_record_t ap_records[WIFI_SCAN_LIST_MAX_SIZE];
    int i = 0;
    while(!(bits & WIFI_CONNECTED_BIT)){
    	if(bits & WIFI_START_COMPLETE_BIT){
    		xEventGroupClearBits(wifiEventGroup, WIFI_START_COMPLETE_BIT);
    		ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, false));
    	}else if(bits & WIFI_SCAN_COMPLETE_BIT){
    		xEventGroupClearBits(wifiEventGroup, WIFI_SCAN_COMPLETE_BIT);
    		ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_records));
    		ESP_LOGI(STA_TAG, "%d LAN found", number);

    		i = APMatch(registeredAP, ap_records, number);
    		if(i >= 0){
    			ESP_LOGI(STA_TAG, "SSID %s detected. Connecting...", registeredAP[i].ssid);
    			wifi_config_t wifiConfig = {.sta = registeredAP[i] };
				ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiConfig) );
				esp_wifi_connect();
    		}else{
    			ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, false));
    		}
    	}

    	/*Wait until one of the bits are set*/
    	bits = xEventGroupWaitBits(wifiEventGroup,
					WIFI_CONNECTED_BIT | WIFI_SCAN_COMPLETE_BIT | WIFI_START_COMPLETE_BIT,
					pdFALSE,
					pdFALSE,
					portMAX_DELAY);
    }

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(STA_TAG, "connected to ap SSID:%s password:%s", registeredAP[i].ssid, registeredAP[i].password);
	}
}
#endif


retval_t sendPkg(in_addr_t addr, in_port_t port, uint8_t *data, uint16_t length){
	pkgData pkg = initPkgData(addr, port, data, length);
	char addr_str[128];
	inet_ntoa_r(addr, addr_str, sizeof(addr_str) - 1);
	if(xQueueSend(txQueue, &pkg, 0) != pdTRUE)
		return RET_ERROR;
	return RET_OK;
}


static pkgData initPkgData(in_addr_t addr, in_port_t port, uint8_t *data, uint16_t dataLength){
	pkgData retVal;
	retVal.ipAddress = addr;
	retVal.port = port;
	retVal.length = dataLength;
	retVal.data = (uint8_t*)pvPortMalloc(dataLength);
	memcpy(retVal.data, data, dataLength);
	return retVal;
}


static void deletePkgData(pkgData *pkg){
	vPortFree(pkg->data);
}



