/*
 * haptics.c
 *
 *  Created on: 19 nov. 2020
 *      Author: sreal
 */


#include "haptics.h"
#include "bsp.h"
#include <drv2605L/drv2605L.h>
#include <drv2605L/drv2605L_Const.h>
#include "esp_log.h"
#include "virtualScenarioManager.h"
#include "virtualScenarioElements.h"
#include "transportLayer.h"

static const char *HAPTICS_TAG = "Haptics";

static drv2605L_t* drv2605L_0 = NULL;
static drv2605L_t* drv2605L_1 = NULL;
static vsChannelHandle *hapChannelHandle1;
static vsChannelHandle *hapChannelHandle2;
static genList_t *acousticStimElements;
static bool available1 = false;
static bool available2 = false;
static uint8_t hapticsChannel1;
static uint8_t hapticsChannel2;

QueueHandle_t hapticQueue1;
QueueHandle_t hapticQueue2;

static retval_t hapticCallback1(uint8_t *data, uint16_t length);
static retval_t hapticCallback2(uint8_t *, uint16_t length);
void setVibrationAmplitude1(uint8_t value);
void setVibrationAmplitude2(uint8_t value);

static void hapticTask1();
static void hapticTask2();

retval_t hapticsInit(uint8_t channel1, uint8_t channel2){
	drv2605L_0 = drv2605L_Initialize(&i2c_line_0);
	drv2605L_1 = drv2605L_Initialize(&i2c_line_1);
	ESP_LOGI("HAPTICS", "Trying to fire element...");
//	if(drv2605L_init(drv2605L_0, I2C_ACTUATOR_1) != RET_OK){
//		drv2605L_delete(drv2605L_0);
//		return RET_ERROR;
//	}
//	if(drv2605L_init(drv2605L_1, I2C_ACTUATOR_1) != RET_OK){
//		drv2605L_delete(drv2605L_1);
//		return RET_ERROR;
//	}
	drv2605L_exitStandby(drv2605L_0);
	drv2605L_loadActuatorConfig(drv2605L_0, &DRV2605L_ACTUATOR1);
	drv2605L_setMode(drv2605L_0, DRV2605L_MODE_SELECTION1);
	drv2605L_selectEffectLibrary(drv2605L_0, DRV2605L_LIBRARY1);
	drv2605L_exitStandby(drv2605L_1);
	drv2605L_loadActuatorConfig(drv2605L_1, &DRV2605L_ACTUATOR1);
	drv2605L_setMode(drv2605L_1, DRV2605L_MODE_SELECTION1);
	drv2605L_selectEffectLibrary(drv2605L_1, DRV2605L_LIBRARY1);
	setVibrationAmplitude1(128);
	setVibrationAmplitude2(128);

	hapticQueue1 = xQueueCreate(10, sizeof(acStimUpdate_t));
	hapticQueue2 = xQueueCreate(10, sizeof(acStimUpdate_t));
	hapChannelHandle1 = addChannel(channel1, hapticCallback1);
	hapChannelHandle2 = addChannel(channel2, hapticCallback2);
	acousticStimElements = genListInit();
	hapticsChannel1 = channel1;
	hapticsChannel2 = channel2;
	available1 = true;
	available2 = true;


	xTaskCreate(hapticTask1, "hapticTask1", 4096, NULL, 5, NULL);
	xTaskCreate(hapticTask2, "hapticTask2", 4096, NULL, 5, NULL);

	return RET_OK;
}


bool hapticsAvailable1(){
	return available1;
}

uint8_t getHapticsChannel1(){
	return hapticsChannel1;
}

bool hapticsAvailable2(){
	return available2;
}

uint8_t getHapticsChannel2(){
	return hapticsChannel2;
}

void fireEffect1(drv2605L_Effect_t hapticEffect){
	ESP_LOGI("HAPTICS", "Firing now...");
	drv2605L_fireROMLibraryEffect(drv2605L_0, hapticEffect, false);
	drv2605L_fireROMLibraryEffect(drv2605L_1, hapticEffect, false); //Ñapa Pocha para que vayan los 2 motores a la vez
	return;
}

void setVibrationAmplitude1(uint8_t value){
	ESP_LOGI("HAPTICS", "Firing with value: %d", value);
	drv2605L_setRTPInput(drv2605L_0, value);
	drv2605L_setRTPInput(drv2605L_1, value); //Ñapa Pocha para que vayan los 2 motores a la vez
	return;
}

void fireEffect2(drv2605L_Effect_t hapticEffect){
	ESP_LOGI("HAPTICS", "Firing now...");
	drv2605L_fireROMLibraryEffect(drv2605L_1, hapticEffect, false);
	return;
}

void setVibrationAmplitude2(uint8_t value){
	ESP_LOGI("HAPTICS", "Firing with value: %d", value);
	drv2605L_setRTPInput(drv2605L_1, value);
	return;
}


//static void runHapticStimuli(acousticStimuli_t* hapticStimuli){
//	if()
//}

static retval_t hapticCallback1(uint8_t *data, uint16_t length){
	return xQueueSend(hapticQueue1, (acStimUpdate_t*)data, 0) == pdTRUE ? RET_OK : RET_ERROR;
}

static retval_t hapticCallback2(uint8_t *data, uint16_t length){
	return xQueueSend(hapticQueue2, (acStimUpdate_t*)data, 0) == pdTRUE ? RET_OK : RET_ERROR;
}

static void hapticTask1(){
	acStimUpdate_t stimuli;
	while(1){
		if(xQueueReceive(hapticQueue1, &stimuli, portMAX_DELAY) != pdTRUE ){
			ESP_LOGE(HAPTICS_TAG, "Error occurred in txQueue");
		}


		if((drv2605L_0 != NULL) && (stimuli.vibrating > 0)){
			setVibrationAmplitude1(stimuli.vibrating);
		}
		if((drv2605L_1 != NULL) && (stimuli.vibrating > 0)){
			setVibrationAmplitude1(stimuli.vibrating);
		}

		//	genListElement_t *current = acousticStimElements->tailElement;
		//
		//	while(current != NULL){
		//		if(((acousticStimuli_t*)current->item)->elementID == stimuli->elementID){
		//			((acousticStimuli_t*)current->item)->vibrating = stimuli->vibrating;
		//
		//			return RET_OK;
		//		}
		//		current = current->next;
		//	}
		//	acousticStimuli_t *newElement = (acousticStimuli_t*)pvPortMalloc(sizeof(acousticStimuli_t));
		//
		//	ESP_LOGI("Haptic Element","Element: %llu, length: %d", stimuli->elementID, length);
		//	if(stimuli->vibrating > 0){
		//		setVibrationAmplitude(stimuli->vibrating);
		////		testHaptics(stimuli->vibrating);
		//	}

	}
}

static void hapticTask2(){
	acStimUpdate_t stimuli;
	while(1){
		if(xQueueReceive(hapticQueue2, &stimuli, portMAX_DELAY) != pdTRUE ){
			ESP_LOGE(HAPTICS_TAG, "Error occurred in txQueue");
		}

		if((drv2605L_0 != NULL) && (stimuli.vibrating > 0)){
			setVibrationAmplitude2(stimuli.vibrating);
		}
		if((drv2605L_1 != NULL) && (stimuli.vibrating > 0)){
			setVibrationAmplitude2(stimuli.vibrating);
		}
	}
}


