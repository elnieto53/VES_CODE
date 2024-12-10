/*
 * esp32-i2cDriver.h
 *
 *  Created on: 23 feb 2024
 *      Author: nietoschoebel
 */

#ifndef MAIN_DRIVERS_BSP_INC_ESP32_I2CDRIVER_H_
#define MAIN_DRIVERS_BSP_INC_ESP32_I2CDRIVER_H_

#include <stdio.h>
#include "commonTypes.h"
#include "deviceConfig.h"
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hal/i2c_types.h"

typedef struct {
	i2c_port_t i2c_port;
	int sda_io_num;
	int scl_io_num;
	uint32_t clk_speed;
	bool enabled;
	SemaphoreHandle_t mutex;
}i2c_line_t;


int i2c_enabled(i2c_line_t* i2c_line);

retval_t i2c_master_Initialize(i2c_line_t* i2c_line);
retval_t i2c_master_Terminate(i2c_line_t* i2c_line);
retval_t i2c_master_Write(uint8_t devAddress, uint8_t regAddress, uint8_t *buf, uint16_t length, i2c_line_t* i2c_line);
retval_t i2c_master_Read(uint8_t devAddress, uint8_t regAddress, uint8_t *buf, uint16_t length, i2c_line_t* i2c_line);


#endif /* MAIN_DRIVERS_BSP_INC_ESP32_I2CDRIVER_H_ */
