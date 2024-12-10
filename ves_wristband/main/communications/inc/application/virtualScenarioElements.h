/*
 * virtualScenarioElements.h
 *
 *  Created on: 19 nov. 2020
 *      Author: sreal
 */

#ifndef MAIN_COMMUNICATIONS_APPLICATION_VIRTUALSCENARIOELEMENTS_H_
#define MAIN_COMMUNICATIONS_APPLICATION_VIRTUALSCENARIOELEMENTS_H_

#include "commonTypes.h"
#include "vector.h"
#include "quatOps.h"

typedef enum bodyPart_prefabID_ {
	HEAD 			= 0,
	TORSO 			= 1,
	RIGHT_ARM 		= 2,
	LEFT_ARM 		= 3,
	RIGHT_FOREARM	= 4,
	LEFT_FOREARM 	= 5
} bodyPart_prefabID_t;

typedef struct __attribute__((__packed__)) physPkg_ {
	unsigned long long elementID;
	uint8_t prefabID;
	int timestamp;
	vector position;
	quaternion orientation;
} physPkg;

typedef struct __attribute__((__packed__)) bodyPartUpdate_ {
	unsigned long long elementID;
	uint8_t prefabID;
	int timestamp;
	uint8_t bodyID;
	quaternion orientation;
} moCapOrientationUpdate;

typedef struct __attribute__((__packed__)) acStimUpdate_ {
	unsigned long long elementID;
	uint8_t prefabID;
	int timestamp;
	uint8_t vibrating;
} acStimUpdate_t;

typedef struct __attribute__((__packed__)) proximityDataUpdate_ {
	unsigned long long elementID;
	uint8_t prefabID;
	int timestamp;
	uint64_t distance;
} proximityDataUpdate_t;

#endif /* MAIN_COMMUNICATIONS_APPLICATION_VIRTUALSCENARIOELEMENTS_H_ */
