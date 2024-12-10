/*
 * moCap.h
 *
 *  Created on: 23 oct. 2020
 *      Author: sreal
 */

#ifndef MAIN_MOCAP_H_
#define MAIN_MOCAP_H_

#include "virtualScenarioManager.h"
#include "transportLayer.h"
#include "commonTypes.h"
#include "quatOps.h"
#include "vector.h"
#include "lsm6dso_reg.h"


typedef struct MoCapHandler_ {
	quaternion orientation;			/*Device orientation*/
	quaternion orientationOffset;	/*Device orientation offset*/
	vector gravity;					/*Gravity vector (global coordinates)*/
//	void *privateData;
	axis_t lastRawGyro;				/*Private data*/
	axis_t lastRawAccel;			/*Private data*/
	axis_t gyroRawOffset;			/*Private data*/
	axis_t accelRawOffset;			/*Private data*/
	quaternion inicialRot;
	SemaphoreHandle_t mutex;
} MoCapHandler_t;

retval_t moCapInit(bodyPart_prefabID_t prefabID, uint8_t channel);
void moCapCalibratePose(MoCapHandler_t *moCapData, vector localRotRef, vector artRotRef, quaternion initialRot);
retval_t moCapCalibrateOffset(MoCapHandler_t *moCapData);
bool moCapEnabled(void);

#endif /* MAIN_MOCAP_H_ */
