/*
 * virtualScenarioManager.c
 *
 *  Created on: 14 oct. 2020
 *      Author: sreal
 */

#include "virtualScenarioManager.h"
#include "networkManager.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "deviceConfig.h"

static const char *VE_TAG = "Virtual Scenario";


#define MAX_SUBSCRIBED_DEVICES	5

typedef enum virtualScenarioCommands_ {
	SUBSCRIBE = 0,
	UNSUBSCRIBE = 1,
	ELEMENT_UPDATE = 2
} virtualScenarioCommands_t;


/*PRIVATE FUNCTIONS*/
static vsChannelHandle* vsChannelInit(uint8_t channel, elementUpdateCallback callback);
static retval_t vsChannelDeInit(vsChannelHandle *channelHandle);

SemaphoreHandle_t activeChannelsMutex;
genList_t* activeChannels;

retval_t virtualScenarioManagerInit(){
	activeChannels = genListInit();
	activeChannelsMutex = xSemaphoreCreateMutex();
	if((activeChannels == NULL) || (activeChannelsMutex == NULL)){
		ESP_LOGE(VE_TAG, "Could not initialize the Virtual Scenario Manager");
		return RET_ERROR;
	}
	//in_addr_t aux = ((u32_t)0x57DB71C4UL);
	//subscribeDevice(aux, getChannelHandle(MOCAP_CHANNEL));
	return RET_OK;
}

retval_t virtualScenarioManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length){
	vsHeader rxHeader = *((vsHeader*)data);
	vsChannelHandle *channelHandle;
//	ESP_LOGI(VE_TAG, "Packet received of length: %d, command %d, channel %d", length, rxHeader.command, rxHeader.channel);

	if((channelHandle = getChannelHandle(rxHeader.channel)) == NULL)
		return RET_ERROR;

	switch(rxHeader.command){
		case ELEMENT_UPDATE:
			ESP_LOGI(VE_TAG, "Update received from channel %d", rxHeader.channel);
			channelHandle->callback(data + sizeof(vsHeader), length - sizeof(vsHeader));
			break;
		case SUBSCRIBE:
			ESP_LOGI(VE_TAG, "Subscribing device to %d", rxHeader.channel);
			return subscribeDevice(origin, channelHandle);
			break;
		case UNSUBSCRIBE:
			ESP_LOGI(VE_TAG, "Unsubscribing device to %d", rxHeader.channel);
			return unsubscribeDevice(origin, channelHandle);
			break;
	}
	return RET_OK;
}

retval_t subscribeDevice(in_addr_t deviceAddress, vsChannelHandle* channelHandle){
	genListElement_t *current;
	xSemaphoreTake(channelHandle->mutex, portMAX_DELAY);
	current = channelHandle->subscribedDevices->tailElement;
	while(current != NULL){
		if(*((in_addr_t*)current->item) == deviceAddress){
			xSemaphoreGive(channelHandle->mutex);
			return RET_ERROR;
		}
		current = current->next;
	}
	in_addr_t *newDeviceAddress = (in_addr_t*)pvPortMalloc(sizeof(in_addr_t));
	*newDeviceAddress = deviceAddress;
	genListAdd(channelHandle->subscribedDevices, newDeviceAddress);
	xSemaphoreGive(channelHandle->mutex);
	ESP_LOGI(VE_TAG, "Device subscribed. Number of subscriptions: %d", (int)channelHandle->subscribedDevices->numElements);
	return RET_OK;
}

retval_t unsubscribeDevice(in_addr_t deviceAddress, vsChannelHandle* channelHandle){
	genListElement_t *current;
	xSemaphoreTake(channelHandle->mutex, portMAX_DELAY);
	current = channelHandle->subscribedDevices->tailElement;
	while(current != NULL){
		if((*((in_addr_t*)current->item)) == deviceAddress){
			if(genListRemoveAndDelete(channelHandle->subscribedDevices, current->item) != RET_OK){
				ESP_LOGE(VE_TAG, "Could not delete item");
				xSemaphoreGive(channelHandle->mutex);
				return RET_ERROR;
			}
			xSemaphoreGive(channelHandle->mutex);
			return RET_OK;
		}
		current = current->next;
	}
	xSemaphoreGive(channelHandle->mutex);
	ESP_LOGE(VE_TAG, "Match not found");
	return RET_ERROR;
}

vsChannelHandle* getChannelHandle(uint8_t channel){
	vsChannelHandle* channelHandle;
	genListElement_t *current;
	xSemaphoreTake(activeChannelsMutex, portMAX_DELAY);
	if(activeChannels->numElements == 0){
		xSemaphoreGive(activeChannelsMutex);
		return NULL;
	}
	current = activeChannels->tailElement;
	while(1){
		/*BEWARE!!!!!!!!! I enter an item shared between tasks, so maybe I should use the item's mutex...*/
		if(((vsChannelHandle*)current->item)->channel == channel){
			break;
		}
		if((current = current->next) == NULL){
			xSemaphoreGive(activeChannelsMutex);
			return NULL;
		}
	}
	channelHandle = (vsChannelHandle*)current->item;
	xSemaphoreGive(activeChannelsMutex);
	return channelHandle;
}


vsUpdatePackage* initVsUpdatePackage(uint16_t dataLength){
	return (vsUpdatePackage*)pvPortMalloc(sizeof(vsHeader) + dataLength);
}

retval_t deInitVsUpdatePackage(vsUpdatePackage *pkg){
	vPortFree(pkg);
	return RET_OK;
}

uint8_t* getDataPtr(vsUpdatePackage* pkg){
	return ((uint8_t*)pkg)+sizeof(vsHeader);
}

retval_t sendUpdateToSubscriptors(vsChannelHandle *channelHandle, uint8_t *updateData, uint16_t length){
	genListElement_t *current;
	uint16_t updatePkgLength = sizeof(vsHeader) + length;
	uint8_t *updatePkg = pvPortMalloc(updatePkgLength);
	vsHeader *header = (vsHeader*)updatePkg;
	header->command = ELEMENT_UPDATE;

	xSemaphoreTake(channelHandle->mutex, portMAX_DELAY);
	header->channel = channelHandle->channel;

	memcpy(updatePkg + sizeof(vsHeader), updateData, length);

	current = channelHandle->subscribedDevices->tailElement;
	xSemaphoreGive(channelHandle->mutex);

	while(current != NULL){
//		ESP_LOGI(VE_TAG, "Sending to %d", (*((in_addr_t*)current->item)));
		if(sendPkg(*((in_addr_t*)current->item), htons(VIRTUAL_SCENARIO_MANAGER_REMOTE_PORT), (uint8_t*)updatePkg, updatePkgLength) != RET_OK){
			vPortFree(updatePkg);
			ESP_LOGE(VE_TAG, "Could not send package: TX queue full");
			return RET_ERROR;
		}
		//ESP_LOGI("", "OK1");
		current = current->next;
	}
	vPortFree(updatePkg);
	return RET_OK;
}

vsChannelHandle* addChannel(uint8_t channel, elementUpdateCallback callback){
	vsChannelHandle *retval = vsChannelInit(channel, callback);

	xSemaphoreTake(activeChannelsMutex, portMAX_DELAY);
	if(genListAdd(activeChannels, retval) != RET_OK){
		xSemaphoreGive(activeChannelsMutex);
		return NULL;
	}
	xSemaphoreGive(activeChannelsMutex);

	return retval;
}

retval_t removeChannel(uint8_t channel){
	genListElement_t *current;

	xSemaphoreTake(activeChannelsMutex, portMAX_DELAY);
	current = activeChannels->tailElement;
	while(current != NULL){
		/*BEWARE!!!!!!!!! I enter an item shared between tasks, so maybe I should use the item's mutex...*/
		if(((vsChannelHandle*)current->item)->channel == channel){
			vsChannelDeInit((vsChannelHandle*)current->item);
			genListRemoveAndDelete(activeChannels, current);
			xSemaphoreGive(activeChannelsMutex);
			return RET_OK;
		}
		current = current->next;
	}
	xSemaphoreGive(activeChannelsMutex);
	return RET_ERROR;
}

static vsChannelHandle* vsChannelInit(uint8_t channel, elementUpdateCallback callback){
	vsChannelHandle *channelHandle = (vsChannelHandle*)pvPortMalloc(sizeof(vsChannelHandle));
	channelHandle->channel = channel;
	channelHandle->callback = callback;
	if((channelHandle->subscribedDevices = genListInit())== NULL){
		ESP_LOGE(VE_TAG, "Could not initialize the Virtual Scenario Manager");
		vPortFree(channelHandle);
		return NULL;
	}
	if((channelHandle->mutex = xSemaphoreCreateMutex()) == NULL){
		ESP_LOGE(VE_TAG, "Could not initialize the Virtual Scenario Manager");
		genListRemoveAndDeleteAll(channelHandle->subscribedDevices);
		vPortFree(channelHandle);
		return NULL;
	}

	return channelHandle;
}

static retval_t vsChannelDeInit(vsChannelHandle *channelHandle){
	vSemaphoreDelete(channelHandle->mutex);	/*BEWARE!!!!!!!!!!!!!!!!!!!!!!!!!  Probably this allows a race condition */
	vPortFree(channelHandle);
	return RET_OK;
}


