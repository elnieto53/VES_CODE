/*
 * lsm6dsox_util.c
 *
 *  Created on: 16 nov. 2020
 *      Author: sreal
 */
#include "lsm6dso_util.h"
#include "esp_log.h"
#include "lwip/err.h"

//float_t lsm6dsox_from_fs250_to_rps(int16_t lsb);
//float_t lsm6dsox_from_fs125_to_rps(int16_t lsb);
//float_t lsm6dsox_from_fs500_to_rps(int16_t lsb);
//float_t lsm6dsox_from_fs1000_to_rps(int16_t lsb);
//float_t lsm6dsox_from_fs2000_to_rps(int16_t lsb);
//
//float_t lsm6dsox_from_fs2_to_g(int16_t lsb);
//float_t lsm6dsox_from_fs16_to_g(int16_t lsb);
//float_t lsm6dsox_from_fs4_to_g(int16_t lsb);
//float_t lsm6dsox_from_fs8_to_g(int16_t lsb);


typedef struct gyrConversionData_ {
	lsm6dso_fs_g_t rangeMode;
	IMUconversionFunc conversionFunc;
} gyrConversionData_t;

typedef struct accConversionData_ {
	lsm6dso_fs_xl_t rangeMode;
	IMUconversionFunc conversionFunc;
} accConversionData_t;

gyrConversionData_t gyrConversionMatrix[] = {
	{ .rangeMode = LSM6DSO_250dps, .conversionFunc = lsm6dso_from_fs250_to_mdps },
	{ .rangeMode = LSM6DSO_125dps, .conversionFunc = lsm6dso_from_fs125_to_mdps },
	{ .rangeMode = LSM6DSO_500dps, .conversionFunc = lsm6dso_from_fs500_to_mdps },
	{ .rangeMode = LSM6DSO_1000dps, .conversionFunc = lsm6dso_from_fs1000_to_mdps },
	{ .rangeMode = LSM6DSO_2000dps, .conversionFunc = lsm6dso_from_fs2000_to_mdps },
	{ .rangeMode = 0, .conversionFunc = NULL }
};

accConversionData_t accConversionMatrix[] = {
	{ .rangeMode = LSM6DSO_2g, .conversionFunc = lsm6dso_from_fs2_to_mg },
	{ .rangeMode = LSM6DSO_16g, .conversionFunc = lsm6dso_from_fs16_to_mg },
	{ .rangeMode = LSM6DSO_4g, .conversionFunc = lsm6dso_from_fs4_to_mg },
	{ .rangeMode = LSM6DSO_8g, .conversionFunc = lsm6dso_from_fs8_to_mg },
	{ .rangeMode = 0, .conversionFunc = NULL }
};


typedef struct gyrFreqData_{
	lsm6dso_odr_g_t frequencyMode;
	float frequency;
}gyrFreqData_t;

typedef struct accFreqData_{
	lsm6dso_odr_xl_t frequencyMode;
	float frequency;
}accFreqData_t;

gyrFreqData_t gyrFreqMatrix[] = {
	{ .frequencyMode = LSM6DSO_XL_ODR_OFF, 	.frequency = 0},
	{ .frequencyMode = LSM6DSO_XL_ODR_12Hz5, 	.frequency = 12.5f},
	{ .frequencyMode = LSM6DSO_XL_ODR_26Hz, 	.frequency = 26.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_52Hz, 	.frequency = 52.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_104Hz, 	.frequency = 104.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_208Hz, 	.frequency = 208.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_417Hz, 	.frequency = 417.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_833Hz, 	.frequency = 833.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_1667Hz, 	.frequency = 1667.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_3333Hz, 	.frequency = 3333.0f},
	{ .frequencyMode = LSM6DSO_XL_ODR_6667Hz, 	.frequency = 6667.0f},
	{ .frequencyMode = -1, 	.frequency = 0},
};

accFreqData_t accFreqMatrix[] = {
	{ .frequencyMode = LSM6DSO_GY_ODR_OFF, 	.frequency = 0},
	{ .frequencyMode = LSM6DSO_GY_ODR_12Hz5, 	.frequency = 12.5f},
	{ .frequencyMode = LSM6DSO_GY_ODR_26Hz, 	.frequency = 26.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_52Hz, 	.frequency = 52.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_104Hz, 	.frequency = 104.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_208Hz, 	.frequency = 208.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_417Hz, 	.frequency = 417.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_833Hz, 	.frequency = 833.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_1667Hz, 	.frequency = 1667.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_3333Hz, 	.frequency = 3333.0f},
	{ .frequencyMode = LSM6DSO_GY_ODR_6667Hz, 	.frequency = 6667.0f},
	{ .frequencyMode = -1, 	.frequency = 0},
};


IMUconversionFunc getGyrConversionData(lsm6dso_fs_g_t rangeMode){
	int i = -1;
	while(gyrConversionMatrix[++i].conversionFunc != NULL){
		if(rangeMode == gyrConversionMatrix[i].rangeMode)
			return gyrConversionMatrix[i].conversionFunc;
	}
	ESP_LOGI("IMU Utils", "Conversion function not found");
	return NULL;
}
IMUconversionFunc getAccConversionData(lsm6dso_fs_xl_t rangeMode){
	int i = -1;
	while(accConversionMatrix[++i].conversionFunc != NULL){
		if(rangeMode == accConversionMatrix[i].rangeMode)
			return accConversionMatrix[i].conversionFunc;
	}
	ESP_LOGI("IMU Utils", "Conversion function not found");
	return NULL;
}

float getGyrFrequency(lsm6dso_odr_g_t freq){
	int i = -1;
	while(gyrFreqMatrix[++i].frequencyMode != -1){
		if(freq == gyrFreqMatrix[i].frequencyMode)
			return gyrFreqMatrix[i].frequency;
	}
	return -1;
}

float getAccFrequency(lsm6dso_odr_xl_t freq){
	int i = -1;
	while(accFreqMatrix[++i].frequencyMode != -1){
		if(freq == accFreqMatrix[i].frequencyMode)
			return accFreqMatrix[i].frequency;
	}
	return -1;
}
