/*
 * moCapManager.c
 *
 *  Created on: 7 dic. 2020
 *      Author: sreal
 */
#include "moCapManager.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"


typedef enum networkCommand {
	CALIBRATE_BODY_PART	 = 0,
	CALIBRATE_IMU_OFFSET = 1,
} noCapCommand_t;

typedef struct __attribute__((__packed__)) moCapPkgHeader_ {
	uint8_t command;
} moCapPkgHeader_t;

typedef struct __attribute__((__packed__)) bodyPartCalibData_ {
	quaternion initialRot;
	vector localRotRef;
	vector globalRotRef;
} bodyPartCalibData_t;

static const char *MOCAP_MANAGER_TAG = "MoCap Manager";
static MoCapHandler_t *hostBodyPart = NULL;

retval_t moCapManagerInit(){
	return RET_OK;
}

retval_t moCapManager_setHostBodyPart(MoCapHandler_t* moCapHandler){
	hostBodyPart = moCapHandler;
	return RET_OK;
}

retval_t moCapManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length){
	moCapPkgHeader_t rxHeader = *((moCapPkgHeader_t*)data);
	bodyPartCalibData_t recPkg;
	if(hostBodyPart == NULL)
		return RET_ERROR;

	switch(rxHeader.command){
		case CALIBRATE_BODY_PART:
			recPkg = *((bodyPartCalibData_t*)(data+sizeof(moCapPkgHeader_t)));
			ESP_LOGI(MOCAP_MANAGER_TAG, "Calibrating IMU...");
			moCapCalibratePose(hostBodyPart, recPkg.localRotRef, recPkg.globalRotRef, recPkg.initialRot);
			break;
		case CALIBRATE_IMU_OFFSET:
			moCapCalibrateOffset(hostBodyPart);
			break;
	}
	return RET_OK;
}
