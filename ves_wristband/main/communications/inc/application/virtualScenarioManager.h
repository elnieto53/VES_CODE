/*
 * virtualScenarioManager.h
 *
 *  Created on: 14 oct. 2020
 *      Author: sreal
 */

#ifndef MAIN_COMMUNICATIONS_APPLICATION_VSMANAGER_H_
#define MAIN_COMMUNICATIONS_APPLICATION_VSMANAGER_H_

#include "virtualScenarioElements.h"
#include "transportLayer.h"
#include "commonTypes.h"
#include "quatOps.h"
#include "vector.h"

#define VIRTUAL_SCENARIO_MANAGER_REMOTE_PORT	11001
#define VIRTUAL_SCENARIO_MANAGER_HOST_PORT		11001
#define VIRTUAL_SCENARIO_SOCKET_DATA (socketData_t) { .port = VIRTUAL_SCENARIO_MANAGER_HOST_PORT, .callback = virtualScenarioManagerCallback }


typedef retval_t (*elementUpdateCallback)(uint8_t *data, uint16_t length);

typedef struct vsChannelHandle_ {
	uint8_t channel;
	genList_t* subscribedDevices;
	elementUpdateCallback callback;
	SemaphoreHandle_t mutex;
} vsChannelHandle;

typedef struct __attribute__((__packed__)) vsHeader_ {
	uint8_t command;
	uint8_t channel;
} vsHeader;

typedef struct __attribute__((__packed__)) vsUpdatePackage_ {
	vsHeader header;
	uint8_t* data;
} vsUpdatePackage;


retval_t virtualScenarioManagerInit();
retval_t virtualScenarioManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length);
vsChannelHandle* getChannelHandle(uint8_t channel);
retval_t subscribeDevice(in_addr_t deviceAddress, vsChannelHandle* channelHandle);
retval_t unsubscribeDevice(in_addr_t deviceAddress, vsChannelHandle* channelHandle);
//retval_t sendToSubscriptors(vsChannelHandle *channelHandle, pkgData *pkg);
retval_t sendUpdateToSubscriptors(vsChannelHandle *channelHandle, uint8_t *updateData, uint16_t length);

vsChannelHandle* addChannel(uint8_t channel, elementUpdateCallback callback);

#endif /* MAIN_COMMUNICATIONS_APPLICATION_VSMANAGER_H_ */
