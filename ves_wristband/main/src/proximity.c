/*
 * proximity.c
 *
 *  Created on: 14 mar. 2022
 *      Author: sreal
 */
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include <math.h>
#include "virtualScenarioManager.h"
#include "networkManager.h"
#include "nvsManager.h"
#include "driver/gpio.h"

//#include "hc_sr04.h" //ToDo
#include "proximity.h"

#define SONAR_TRIGGER_GPIO	14//GPIO_NUM_14
#define SONAR_ECHO_GPIO		27//GPIO_NUM_27

static vsChannelHandle* channelHandle;
static proximityDataUpdate_t dataPkg;

retval_t sendData(uint64_t data);

retval_t initProximity(uint8_t channel, uint8_t frequency){
	if((channelHandle = addChannel(channel, NULL)) == NULL){
		return RET_ERROR;
	}

	/*BEWARE with frequency range (app default: 20-50 Hz)*/
//	initSonar(frequency, SONAR_ECHO_GPIO, SONAR_TRIGGER_GPIO, sendData);	//ToDo

	return RET_OK;
}

retval_t sendData(uint64_t data){
	//ESP_LOGI("OK", "Data measured: %llu", data);
	dataPkg.timestamp = getNetClock();
	dataPkg.distance = data;
	return sendUpdateToSubscriptors(channelHandle, (uint8_t*)&dataPkg, sizeof(proximityDataUpdate_t));
}


