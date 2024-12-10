/*
 * networkManager.c
 *
 *  Created on: 14 oct. 2020
 *      Author: sreal
 */

#include "networkManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "batteryMonitor.h"
#include "haptics.h"
#include "moCap.h"

static const char *NM_TAG = "Network Manager";

static uint8_t id;

typedef enum networkCommand {
	GET_NET_DEVICE_DATA	= 0,
	SYNC_CLOCK			= 1,
	NET_DEVICE_DATA		= 2,
	AQ_SYNC_CLOCK		= 3
} networkCommand_t;


typedef struct __attribute__((__packed__)) newDevicePkg_ {
	uint8_t networkCommand;
	uint8_t deviceID;
	uint8_t hapticInterfaceAvalailable;
	uint8_t moCapAvailable;
	uint32_t batteryLevel;	/*Battery measurement in mV*/
	uint32_t synchronizedTimestamp;
} newDevicePkg_t;


typedef struct __attribute__((__packed__)) aqSyncPkg_ {
	uint8_t networkCommand;
	uint32_t timestamp;
} aqSyncPkg_t;


typedef struct netClock_{
	int offset;
	SemaphoreHandle_t mutex;
} netClock_t;

/*Static functions*/
void netClockInit();

/*Global variables*/
static netClock_t netClock;

//static uint8_t aqSyncPkg = AQ_SYNC_CLOCK;
static aqSyncPkg_t aqSyncPkg = {
	.networkCommand = AQ_SYNC_CLOCK,
	.timestamp = 0
};

static newDevicePkg_t netDevicePkg = {
	.networkCommand = NET_DEVICE_DATA,
	.deviceID = 0,
	.hapticInterfaceAvalailable = 0,
	.moCapAvailable = 1,
	.batteryLevel = 0,
	.synchronizedTimestamp = -1
};

retval_t networkManagerInit(uint8_t deviceID){
	netClockInit();
	batteryMonitorInit();
	id = deviceID;
	return RET_OK;
}

retval_t networkManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length){
	if(data[0] == GET_NET_DEVICE_DATA){
		netDevicePkg.deviceID = id;
		netDevicePkg.hapticInterfaceAvalailable = hapticsAvailable1() ? getHapticsChannel1() : 0;
		netDevicePkg.moCapAvailable = moCapEnabled() ? 1 : 0;
		netDevicePkg.batteryLevel = getBatteryVoltage();
		sendPkg(origin, htons(NETWORK_MANAGER_REMOTE_PORT), (uint8_t*)&netDevicePkg, sizeof(newDevicePkg_t));
	}
	if(data[0] == SYNC_CLOCK){
		int32_t timestampReceived = (int32_t)data[1] + (data[2]<<8) + (data[3]<<16) + (data[4]<<24);
		ESP_LOGI(NM_TAG, "Package length: %d. Timestamp received: %d", (int)length, (int)timestampReceived);
		setNetClock(timestampReceived);
		aqSyncPkg.timestamp = timestampReceived;
		netDevicePkg.synchronizedTimestamp = timestampReceived;
		sendPkg(origin, htons(NETWORK_MANAGER_REMOTE_PORT), (uint8_t*)&aqSyncPkg, sizeof(aqSyncPkg_t));
	}

	return RET_OK;
}


void netClockInit(){
	netClock.mutex = xSemaphoreCreateMutex();
	netClock.offset = 0;
	xSemaphoreGive(netClock.mutex);
}


int getNetClock(){
	int retval;
	xSemaphoreTake(netClock.mutex, portMAX_DELAY);
	retval = netClock.offset + (int)(esp_timer_get_time()/1000);
	xSemaphoreGive(netClock.mutex);
	return retval;
}


retval_t setNetClock(int32_t time){
	xSemaphoreTake(netClock.mutex, portMAX_DELAY);
	netClock.offset = time - esp_timer_get_time()/1000;
	xSemaphoreGive(netClock.mutex);
	return RET_OK;
}
