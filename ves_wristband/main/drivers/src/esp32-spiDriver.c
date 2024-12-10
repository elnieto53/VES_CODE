/*
 * esp32-spiDriver.c
 *
 *  Created on: 19 jul 2024
 *      Author: nietoschoebel
 */

#include "esp32-spiDriver.h"
#include "esp_log.h"
#include "deviceConfig.h"
#include <string.h>  // Necesario para memset

int spi_enabled(spi_line_t* spi_line){
	return spi_line->enabled;
}

retval_t spi_Initialize(spi_line_t* spi_line, spi_device_handle_t* handle){
	if(spi_enabled(spi_line))
		return RET_REINIT;

	esp_err_t err;

	const spi_bus_config_t buscfg = {
		.miso_io_num = spi_line->mosi_io_num,
		.mosi_io_num = spi_line->mosi_io_num,
		.sclk_io_num = spi_line->sclk_io_num,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
//		.quadwp_io_num = spi_line->quadwp_io_num,
//		.quadhd_io_num = spi_line->quadhd_io_num,
//		.data4_io_num = -1,     ///< GPIO pin for spi data4 signal in octal mode, or -1 if not used.
//		.data5_io_num = -1,     ///< GPIO pin for spi data5 signal in octal mode, or -1 if not used.
//		.data6_io_num = -1,     ///< GPIO pin for spi data6 signal in octal mode, or -1 if not used.
//		.data7_io_num = -1,     ///< GPIO pin for spi data7 signal in octal mode, or -1 if not used.
//		.max_transfer_sz = 4092,
		.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
	};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SPI_MASTER_FREQ_HZ,           // Clock out at 1S MHz
		.mode = SPI_MODE,                                // SPI mode 0
		.spics_io_num = spi_line->cs_io,          // CS pin
		.duty_cycle_pos = 128,
		.queue_size = SPI_QUEUE_SIZE,             // We want to be able to queue 7 transactions at a time
		//.flags = SPI_DEVICE_HALFDUPLEX,
		.post_cb = NULL,
		.pre_cb = NULL,
	};

    // Initialize the SPI bus
	err = spi_bus_initialize(spi_line->host, &buscfg, spi_line->dma_chan);
	if (err != ESP_OK) {
		ESP_LOGE("SPI_TAG", "Error SPI bus Initialization: %d\n", err);
		return RET_ERROR;
	}
    // Attach the SPI device on the SPI bus
	err = spi_bus_add_device(spi_line->host, &devcfg, handle);
	if (err != ESP_OK) {
		ESP_LOGE("SPI_TAG", "Error SPI bus Device Add: %d\n", err);
		return RET_ERROR;
	}

    spi_line->enabled = true;
    return RET_OK;
}

retval_t spi_Terminate(spi_line_t* spi_line, spi_device_handle_t* handle){
    spi_line->enabled = false;
    if(spi_bus_remove_device(*handle) != ESP_OK){
    		return RET_ERROR;
    	}
	if(spi_bus_free(spi_line->host) != ESP_OK){
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t spi_Transmit(spi_device_handle_t* spi, uint8_t* data, size_t size){
    spi_transaction_t t;
	esp_err_t err;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.tx_buffer = data;
    t.rx_buffer = NULL;
    err = spi_device_transmit(*spi, &t);
	if (err != ESP_OK) {
		ESP_LOGE("SPI_TAG", "Error off transmition: %d\n", err);
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t spi_Recieve(spi_device_handle_t* spi, uint8_t* buffer, size_t size){
	spi_transaction_t t;
	esp_err_t err;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.rx_buffer = buffer;
    t.tx_buffer = NULL;
    err = spi_device_transmit(*spi, &t);
    if (err != ESP_OK) {
		ESP_LOGE("SPI_TAG", "Error off reception: %d\n", err);
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t spi_Transmit_Recieve(spi_device_handle_t* spi, uint8_t* txData, uint8_t* rxData, size_t size){
	spi_transaction_t t;
	esp_err_t err;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.rx_buffer = rxData;
    t.tx_buffer = txData;
    err = spi_device_transmit(*spi, &t);
    if (err != ESP_OK) {
		ESP_LOGE("SPI_TAG", "Error off reception: %d\n", err);
		return RET_ERROR;
	}
	return RET_OK;
}



