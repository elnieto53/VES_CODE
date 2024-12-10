/*
 * esp32-i2cDriver.c
 *
 *  Created on: 23 feb 2024
 *      Author: nietoschoebel
 */

#include "esp32-i2cDriver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

int i2c_enabled(i2c_line_t* i2c_line){
	return i2c_line->enabled;
}

retval_t i2c_master_Initialize(i2c_line_t* i2c_line){
	if(i2c_enabled(i2c_line))
		return RET_REINIT;

	int i2c_master_port = i2c_line->i2c_port;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = i2c_line->sda_io_num,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = i2c_line->scl_io_num,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = i2c_line->clk_speed,
		.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	if (i2c_param_config(i2c_master_port, &conf) != ESP_OK) {
		return RET_ERROR;
	}
	if (i2c_driver_install(i2c_master_port, conf.mode,
							I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0) != ESP_OK) {
		return RET_ERROR;
	}
	i2c_line->mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(i2c_line->mutex);
    i2c_line->enabled = true;
    return RET_OK;
}

retval_t i2c_master_Terminate(i2c_line_t* i2c_line){
	if( i2c_driver_delete(i2c_line->i2c_port) != ESP_OK)
			return RET_ERROR;
	i2c_line->enabled = false;
	vSemaphoreDelete(i2c_line->mutex);
	return RET_OK;
}

retval_t i2c_master_Write(uint8_t devAddress, uint8_t regAddress, uint8_t *buf, uint16_t length, i2c_line_t* i2c_line){

	xSemaphoreTake(i2c_line->mutex, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (devAddress << 1) | WRITE_BIT, ACK_CHECK_EN);
//	ESP_LOGI("I2C Write", "Device address sent %X\n", devAddress);
	i2c_master_write_byte(cmd, regAddress, ACK_CHECK_EN);
//	ESP_LOGI("I2C Write", "Register sent %X\n", regAddress);
	i2c_master_write(cmd, buf, length, ACK_CHECK_EN);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(i2c_line->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	if ( err != ESP_OK) {
		xSemaphoreGive(i2c_line->mutex);
//		ESP_LOGE("I2C Write", "Error cmd begin: %d\n", err);
		return RET_ERROR;
	}

	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_line->mutex);
	return RET_OK;
}

retval_t i2c_master_Read(uint8_t devAddress, uint8_t regAddress, uint8_t *buf, uint16_t length, i2c_line_t* i2c_line){
	if (length == 0) {
		return RET_OK;
	}

	xSemaphoreTake(i2c_line->mutex, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (devAddress << 1), ACK_CHECK_EN);
//	ESP_LOGI("I2C Read", "Device address sent %d\n", devAddress);
	i2c_master_write_byte(cmd, regAddress, ACK_CHECK_EN);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (devAddress << 1) | READ_BIT, ACK_CHECK_EN);

	if (length > 1) {
		i2c_master_read(cmd, buf, length - 1, ACK_VAL);
//		ESP_LOGI("I2C Read", "Data received %X\n", *buf);
	}

	i2c_master_read_byte(cmd, buf + length - 1, NACK_VAL);
//	ESP_LOGI("I2C Read", "Data received %X\n", *(buf + length - 1));
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(i2c_line->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	if ( err != ESP_OK) {
		xSemaphoreGive(i2c_line->mutex);
		ESP_LOGE("I2C Read", "Error cmd begin: %d\n", err);
		return RET_ERROR;
	}

	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_line->mutex);
	return RET_OK;
}










