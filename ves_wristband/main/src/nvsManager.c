/*
 * nvs.c
 *
 *  Created on: 6 dic. 2020
 *      Author: sreal
 */
#include <nvsManager.h>
#include "nvs.h"
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/semphr.h"
#include "esp_log.h"
//#include "esp_sleep.h"
#include "esp_err.h"


typedef union data64_{
	axis_t axis;
	struct {
		uint16_t u16_1;
		uint16_t u16_2;
		uint16_t u16_3;
		uint16_t u16_4;
	};
	uint64_t all;
}data64_t;


static const char *NVS_TAG = "NVS";

static SemaphoreHandle_t nvsSemaphore;
nvs_handle_t nvsHandle;

retval_t nvsManagerInit(){
	if(nvs_open("storage", NVS_READWRITE, &nvsHandle) != ESP_OK ){
		return RET_ERROR;
	}
	nvsSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(nvsSemaphore);
	return RET_OK;
}

retval_t nvsSetAxis(const char* key, axis_t data){
	xSemaphoreTake(nvsSemaphore, portMAX_DELAY);
	data64_t entry;
	entry.axis = data;

	if(nvs_set_u64(nvsHandle, key, entry.all) != ESP_OK){
		xSemaphoreGive(nvsSemaphore);
		return RET_ERROR;
	}
	xSemaphoreGive(nvsSemaphore);
	return RET_OK;
}

retval_t nvsGetAxis(const char* key, axis_t *out){
	data64_t entry;
	xSemaphoreTake(nvsSemaphore, portMAX_DELAY);
	if(nvs_get_u64(nvsHandle, key, &(entry.all)) != ESP_OK){
		xSemaphoreGive(nvsSemaphore);
		return RET_ERROR;
	}
	*out = entry.axis;
	xSemaphoreGive(nvsSemaphore);
	return RET_OK;
}
