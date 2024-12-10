/*
 * wifiManager.c
 *
 *  Created on: 19 abr. 2022
 *      Author: sreal
 */

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
//#include "tcpip_adapter.h"
#include <string.h>

#include <stdint.h>
#include "commonTypes.h"
#include "wifiManager.h"



wifi_ap_data_t registeredAP2[] = {
	{ .ssid = "Wifi_test", .password = "555777000111"},
	{ .ssid = "VES", .password = "pasatiempo"},
	{ .ssid = "Knight025_Net", .password = "pasatiempo" },
	{ .ssid = "MOVISTAR_91C0", .password = "5hfXn5cpu7hD4mK5MqFS"},
	{ .ssid = "B105_net", .password = "FiNaO?21" },
	{ .ssid = "Test_Implant", .password = "B105Testing" },
	{ .ssid = "AndroidAP_SRV", .password = "pasatiempo" },
	{ .ssid = "DESKTOP-SAGENI", .password = "60293fZw" },
	{ .ssid = "MiFibra-EAA9", .password = "TZtnDkr9" },
	{ .ssid = "vodafoneBA2168", .password = "DHLP6KAYJDENTUHE" },
	{ .ssid = "Linksys11276", .password = "B105core" },
	{ .ssid = "" },
};

wifi_config_t wifi_config = {
	.ap = {
		.ssid = "Test_Implant",
		.ssid_len = strlen("Test_Implant"),
		.channel = 1,
		.password = "B105Testing",
		.max_connection = 5,
		.authmode = WIFI_AUTH_WPA_WPA2_PSK
	},
};

wifi_sta_config_t staDefaultConfig = { .pmf_cfg = { .capable = true, .required = false } };

#define GET_SYS_TICKS(x)				x / portTICK_PERIOD_MS

#define WIFI_MAX_CONNECTION_RETRY  		10
#define WIFI_DEFAULT_TIMEOUT			10000/portTICK_PERIOD_MS//GET_SYS_TICKS(4000) //ms
#define WIFI_SCAN_TIMEOUT				10000/portTICK_PERIOD_MS// GET_SYS_TICKS(10000) //ms
#define WIFI_SCAN_LIST_MAX_SIZE 		15

static const char *WIFI_TAG = "WiFi";


/* Event group bits mapping */
#define WIFI_CONNECTED_BIT_ 			BIT0
#define WIFI_SCAN_COMPLETE_BIT_ 		BIT1
#define WIFI_START_AP_COMPLETE_BIT 		BIT2
#define WIFI_START_STA_COMPLETE_BIT 	BIT3
#define WIFI_STOP_AP_COMPLETE_BIT 		BIT4
#define WIFI_STOP_STA_COMPLETE_BIT 		BIT5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifiEventGroup;

static esp_event_handler_instance_t default_event_handler_instance;
esp_netif_t* ap_netif_handle;
esp_netif_t* sta_netif_handle;

static int connectRetryCounter = 0;

/*Event handlers*/
static int APMatch2(wifi_ap_data_t* apList, wifi_ap_record_t* apRecords, uint16_t apNumber);
static void wifi_startEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_stopEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_scanHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_connectionHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_disconnectionHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

static inline retval_t checkMask(EventBits_t mask, EventBits_t bits){
	return (mask & bits) == bits ? RET_OK : RET_ERROR;
}

retval_t WiFi_Init(wifi_mode_t mode){
	esp_netif_init();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);

	wifiEventGroup = xEventGroupCreate();
	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START, &wifi_startEventHandler, NULL, &default_event_handler_instance);
	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, &wifi_startEventHandler, NULL, &default_event_handler_instance);
	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &wifi_scanHandler, NULL, &default_event_handler_instance);
	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_disconnectionHandler, NULL, &default_event_handler_instance);
	esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_connectionHandler, NULL, &default_event_handler_instance);

	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_STOP, &wifi_stopEventHandler, NULL, &default_event_handler_instance);
	esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &wifi_stopEventHandler, NULL, &default_event_handler_instance);
	//ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_eventHandler, NULL, &default_event_handler_instance));

	if(esp_wifi_set_mode(mode) != ESP_OK){
		ESP_LOGE(WIFI_TAG, "Could not set WiFi mode");
		return RET_ERROR;
	}
	if(esp_wifi_start() != RET_OK){
		ESP_LOGE(WIFI_TAG, "Could not start WiFi module");
	}

	EventBits_t bits = 0;
	EventBits_t bitsToWaitFor = 0;
	switch(mode){
		case (WIFI_MODE_AP):
			bitsToWaitFor = WIFI_START_AP_COMPLETE_BIT;
			ESP_LOGI(WIFI_TAG, "Initializing AP mode...");
			break;
		case (WIFI_MODE_STA):
			bitsToWaitFor = WIFI_START_STA_COMPLETE_BIT;
			ESP_LOGI(WIFI_TAG, "Initializing STA mode...");
			break;
		case (WIFI_MODE_APSTA):
			bitsToWaitFor = WIFI_START_STA_COMPLETE_BIT | WIFI_START_AP_COMPLETE_BIT;
			ESP_LOGI(WIFI_TAG, "Initializing AP-STA mode...");
			break;
		default:
			ESP_LOGE(WIFI_TAG, "Error: not supported mode");
			break;
	}

	bits = xEventGroupWaitBits(wifiEventGroup, bitsToWaitFor, pdFALSE, pdFALSE, WIFI_DEFAULT_TIMEOUT);
	if((bits & bitsToWaitFor) != bitsToWaitFor){
		esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, default_event_handler_instance);
		ESP_LOGE(WIFI_TAG, "Could not configure WiFi mode");
		return RET_ERROR;
	}

	switch(mode){
		case (WIFI_MODE_AP):
			ap_netif_handle = esp_netif_create_default_wifi_ap();
			esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
			break;
		case(WIFI_MODE_STA):
			sta_netif_handle = esp_netif_create_default_wifi_sta();
			break;
		case(WIFI_MODE_APSTA):
			ap_netif_handle = esp_netif_create_default_wifi_ap();
			sta_netif_handle = esp_netif_create_default_wifi_sta();
			break;
		default:
			ESP_LOGE(WIFI_TAG, "Mode not supported");
			break;
	}

	ESP_LOGI(WIFI_TAG, "Wifi module initialized");
	return RET_OK;
}


static int APMatch2(wifi_ap_data_t* apList, wifi_ap_record_t* apRecords, uint16_t apNumber){
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

retval_t WiFi_connectToAP(wifi_ap_data_t* apData){
	EventBits_t bits;
	ESP_LOGI(WIFI_TAG, "Attempting to connect to the AP...");
	strcpy((char*)staDefaultConfig.ssid, (char*)apData->ssid);
	strcpy((char*)staDefaultConfig.password, (char*)apData->password);
	if(esp_wifi_stop() != ESP_OK)
		return RET_OK;
	if(esp_wifi_set_config(ESP_IF_WIFI_STA, (wifi_config_t*)&staDefaultConfig) != ESP_OK)
		return RET_ERROR;
	if(esp_wifi_start() != ESP_OK)
		return RET_ERROR;
	bits = xEventGroupWaitBits(wifiEventGroup, WIFI_START_STA_COMPLETE_BIT, pdFALSE, pdFALSE, WIFI_DEFAULT_TIMEOUT);
	if(checkMask(WIFI_START_STA_COMPLETE_BIT, bits) != RET_OK)
		return RET_ERROR;
	connectRetryCounter = 0;
	if(esp_wifi_connect() != ESP_OK)
		return RET_ERROR;
	bits = xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT_, pdFALSE, pdFALSE, WIFI_DEFAULT_TIMEOUT);
	return checkMask(WIFI_CONNECTED_BIT_, bits) ? RET_OK : RET_ERROR;
}


retval_t WiFi_searchRegisteredAP(wifi_ap_record_t* apRecord, wifi_ap_data_t* apData){
	EventBits_t bits = 0;
	uint16_t number = WIFI_SCAN_LIST_MAX_SIZE;
	wifi_ap_record_t ap_records[WIFI_SCAN_LIST_MAX_SIZE];
	int i = 0;
	esp_wifi_scan_start(NULL, false);
	while(1){
		if(bits & WIFI_SCAN_COMPLETE_BIT_){
			xEventGroupClearBits(wifiEventGroup, WIFI_SCAN_COMPLETE_BIT_);
			esp_wifi_scan_get_ap_records(&number, ap_records);
			ESP_LOGI(WIFI_TAG, "%d LAN found", number);
			i = APMatch2(registeredAP2, ap_records, number);
			if(i >= 0){
				ESP_LOGI(WIFI_TAG, "SSID %s detected", registeredAP2[i].ssid);
				*apRecord = ap_records[i];
				*apData = registeredAP2[i];
				return RET_OK;
			}else{
				ESP_LOGI(WIFI_TAG, "Pega");
				ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, false));
			}
		}

		/*Wait until one of the bits are set*/
		bits = xEventGroupWaitBits(wifiEventGroup, WIFI_SCAN_COMPLETE_BIT_, pdFALSE, pdFALSE, portMAX_DELAY);
	}
	return RET_OK;
}

/*Handlers*/

static void wifi_startEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	switch(event_id){
		case(WIFI_EVENT_STA_START):
			xEventGroupSetBits(wifiEventGroup, WIFI_START_STA_COMPLETE_BIT);
			break;
		case(WIFI_EVENT_AP_START):
			xEventGroupSetBits(wifiEventGroup, WIFI_START_AP_COMPLETE_BIT);
			break;
	}
}

static void wifi_stopEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	switch(event_id){
		case(WIFI_EVENT_STA_START):
			xEventGroupSetBits(wifiEventGroup, WIFI_STOP_STA_COMPLETE_BIT);
			break;
		case(WIFI_EVENT_AP_START):
			xEventGroupSetBits(wifiEventGroup, WIFI_STOP_AP_COMPLETE_BIT);
			break;
	}
}

static void wifi_scanHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	ESP_LOGI(WIFI_TAG, "Entered event handler");
	if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
		xEventGroupSetBits(wifiEventGroup, WIFI_SCAN_COMPLETE_BIT_);
	return;
}

static void wifi_connectionHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT_);
	ESP_LOGI(WIFI_TAG, "Connected!!");
}

static void wifi_disconnectionHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	if (connectRetryCounter < WIFI_MAX_CONNECTION_RETRY) {
		esp_wifi_connect();
		connectRetryCounter++;
		ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
	}else{
		ESP_LOGE(WIFI_TAG,"connect to the AP fail");
	}
}


