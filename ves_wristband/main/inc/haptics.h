/*
 * haptics.h
 *
 *  Created on: 19 nov. 2020
 *      Author: sreal
 */

#ifndef MAIN_INC_HAPTICS_H_
#define MAIN_INC_HAPTICS_H_

#include "commonTypes.h"
#include "stdBool.h"

#define HAPTICS_MAX_AMPLITUDE	255
#define HAPTICS_ZERO_AMPLITUDE	128

retval_t hapticsInit(uint8_t channel1, uint8_t channel2);
bool hapticsAvailable1();
uint8_t getHapticsChannel1();
void fireEffect1();
void setVibrationAmplitude1(uint8_t value);
bool hapticsAvailable2();
uint8_t getHapticsChannel2();
void fireEffect2();
void setVibrationAmplitude2(uint8_t value);


#endif /* MAIN_INC_HAPTICS_H_ */
