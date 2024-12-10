/*
 * lsm6dsox_util.h
 *
 *  Created on: 16 nov. 2020
 *      Author: sreal
 */

#ifndef MAIN_LSM6DSOX_LSM6DSOX_UTIL_H_
#define MAIN_LSM6DSOX_LSM6DSOX_UTIL_H_

#include "lsm6dso_reg.h"

typedef float_t (*IMUconversionFunc)(int16_t lsb);

IMUconversionFunc getGyrConversionData(lsm6dso_fs_g_t rangeMode);
IMUconversionFunc getAccConversionData(lsm6dso_fs_xl_t rangeMode);

float getGyrFrequency(lsm6dso_odr_g_t freq);
float getAccFrequency(lsm6dso_odr_xl_t freq);

#endif /* MAIN_LSM6DSOX_LSM6DSOX_UTIL_H_ */
