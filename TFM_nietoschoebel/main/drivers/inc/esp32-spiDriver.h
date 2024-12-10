/*
 * esp32-spiDriver.h
 *
 *  Created on: 19 jul 2024
 *      Author: nietoschoebel
 */

#ifndef MAIN_DRIVERS_INC_ESP32_SPIDRIVER_H_
#define MAIN_DRIVERS_INC_ESP32_SPIDRIVER_H_

#include <stdio.h>
#include "commonTypes.h"
#include "deviceConfig.h"
#include <stdbool.h>
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

typedef struct spi_line_t{
	spi_host_device_t host; ///< The SPI host used
	spi_dma_chan_t dma_chan;
	int cs_io;       ///< CS gpio number
	int mosi_io_num;
	int miso_io_num;
	int sclk_io_num;
	int quadwp_io_num;
	int quadhd_io_num;
    int enabled;
}spi_line_t;

int spi_enabled(spi_line_t* spi_line);

retval_t spi_Initialize(spi_line_t* spi_line, spi_device_handle_t* handle);
retval_t spi_Terminate(spi_line_t* spi_line, spi_device_handle_t* handle);

retval_t spi_Transmit(spi_device_handle_t* spi, uint8_t* data, size_t size);
retval_t spi_Recieve(spi_device_handle_t* spi, uint8_t* buffer, size_t size);
retval_t spi_Transmit_Recieve(spi_device_handle_t* spi, uint8_t* txData, uint8_t* rxData, size_t size);

#endif /* MAIN_DRIVERS_INC_ESP32_SPIDRIVER_H_ */
