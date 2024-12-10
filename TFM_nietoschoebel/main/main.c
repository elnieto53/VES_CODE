/*
 * main.c
 *
 *  Created on: 22 feb 2024
 *      Author: nietoschoebel
 */

#include "bsp.h"
#include "quatOps.h"
#include "quaternion.h"
#include "deviceConfig.h"
#include "commonTypes.h"
#include "w25n01gv.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include <stdbool.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp32-i2cDriver.h"
#include "esp32-spiDriver.h"

uint8_t addlsm = 0x11;
uint8_t addbhi = 0x22;
uint8_t adddrv1 = 0x44;
uint8_t adddrv2 = 0x88;

void app_main(void)
{
//	LSM6DSO_Object_t* LSMHan;
//	LSMHan = LSM6DSO_Initialize();
//	LSM6DSO_Axes_t AccelerationLSM;
//	uint8_t id_LSM;
//	LSM6DSO_ACC_Enable(LSMHan);
//
//	struct bhy2_dev bhy2;
//	bhy2 = BHI360_Initialize();
//	uint8_t id_BHI;
//
//	drv2605L_t* drvDev0;
//	drvDev0 = drv2605L_Initialize(&i2c_line_0);
//	drv2605L_t* drvDev1;
//	drvDev1 = drv2605L_Initialize(&i2c_line_1);
////	drv2605L_delete(drvDev);
//	drv2605L_DeviceID_t id_DRV0;
//	drv2605L_DeviceID_t id_DRV1;
//
//    while (true) {
//    	id_LSM = 0;
//    	id_BHI = 0;
//    	id_DRV0 = 0;
//    	id_DRV1 = 0;
//    	ESP_LOGW("TEST", "While(1) reached\n");
//
//    	if(LSM6DSO_ReadID(LSMHan, &id_LSM) != LSM6DSO_OK)
//    		ESP_LOGE("TEST", "ReadID LSM error\n");
//    	if(LSM6DSO_ACC_GetAxes(LSMHan, &AccelerationLSM) != LSM6DSO_OK)
//    		ESP_LOGE("TEST", "GetAxes ACC LSM error\n");
//    	vTaskDelay(pdMS_TO_TICKS(1000));
//
//    	if(bhy2_get_product_id(&id_BHI, &bhy2) != BHY2_OK)
//    		ESP_LOGE("TEST", "ReadID BHI error\n");
//    	vTaskDelay(pdMS_TO_TICKS(1000));
//
//    	if(drv2605L_getDeviceID(drvDev0, &id_DRV0) != RET_OK)
//    		ESP_LOGE("TEST", "ReadID DRV2605L error in line 0\n");
//    	vTaskDelay(pdMS_TO_TICKS(1000));
//
//    	if(drv2605L_getDeviceID(drvDev1, &id_DRV1) != RET_OK)
//    		ESP_LOGE("TEST", "ReadID DRV2605L error in line 1\n");
//    	vTaskDelay(pdMS_TO_TICKS(1000));
//
//    	ESP_LOGI("TEST LSM ACC", "X = %ld, Y = %ld, Z = %ld\n", AccelerationLSM.x, AccelerationLSM.y, AccelerationLSM.z);
//    	ESP_LOGI("TEST", "OK, BHI = 0x%X\n",  id_BHI);
//    	ESP_LOGI("TEST", "OK,  DRV0 = 0x%X,  DRV1 = 0x%X\n",  (uint8_t)id_DRV0, (uint8_t)id_DRV1);
//
//    	if(id_DRV0 == DRV2605L_DEV_ID && id_DRV1 == DRV2605L_DEV_ID && id_BHI == BHI360_FUSER2_DEV_ID && id_LSM == LSM6DSO_DEV_ID){
//    		ESP_LOGI("TEST", "OK, DRV0 = 0x%X,  DRV1 = 0x%X, BHI = 0x%X, LSM = 0x%X\n", (uint8_t)id_DRV0, (uint8_t)id_DRV1, id_BHI, id_LSM);
//    		gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
//    		gpio_set_level(LED_PIN, 1);
//    		if(drv2605L_go(drvDev0) != RET_OK)
//    		    ESP_LOGE("TEST", "Motor 0 go error\n");
//    		if(drv2605L_go(drvDev1) != RET_OK)
//    		    ESP_LOGE("TEST", "Motor 1 go error\n");
////			 Esperar 1 segundo
//    		ESP_LOGI("TEST", "Encendiendo led en %d y motores\n", LED_PIN);
//			vTaskDelay(1000 / portTICK_PERIOD_MS);
//			// Apagar el LED
//			gpio_set_level(LED_PIN, 0);
//			if(drv2605L_stop(drvDev0) != RET_OK)
//				ESP_LOGE("TEST", "Motor 0 Stop error\n");
//			if(drv2605L_stop(drvDev1) != RET_OK)
//				ESP_LOGE("TEST", "Motor 1 Stop error\n");
////			 Esperar 1 segundo
//			ESP_LOGI("TEST", "Apagando led en %d y motores\n", LED_PIN);
//			vTaskDelay(1000 / portTICK_PERIOD_MS);
//    	}else{
//    		ESP_LOGE("TEST" ,"ERROR, DRV0 = 0x%X,  DRV1 = 0x%X, BHI = 0x%X, LSM = 0x%X\n", (uint8_t)id_DRV0, (uint8_t)id_DRV1, id_BHI, id_LSM);
//    	}
//    }

//	  //FLASH TEST
//
//	// Estructura de la línea SPI
//	spi_line_t spi_line = {
//		.host = SPI2_HOST, ///< The SPI host used
//		.dma_chan = SPI_DMA_CH_AUTO,
//		.cs_io = PIN_NUM_CS,
//		.quadwp_io_num = PIN_NUM_WP,
//		.quadhd_io_num = PIN_NUM_HD,
//		.mosi_io_num = PIN_NUM_MOSI,
//		.miso_io_num = PIN_NUM_MISO,
//		.sclk_io_num = PIN_NUM_CLK,
//	};
//
////	spi_Terminate(&spi_line);
//    // Inicializar la memoria flash
//    W25_Init(&spi_line);
//
//    while(1){
//    // Leer el ID JEDEC
//		uint8_t jedec_id[3];
//		int retval = W25_ReadJedecID(jedec_id, sizeof(jedec_id));
//		if (retval == RET_OK) {
//			ESP_LOGI("W25N01GV", "JEDEC ID: 0x%02X 0x%02X 0x%02X", jedec_id[0], jedec_id[1], jedec_id[2]);
//		} else {
//		    ESP_LOGE("W25N01GV", "Error al leer el JEDEC ID: %d", retval);
//		}
//		vTaskDelay(2000 / portTICK_PERIOD_MS);
//    }
//    // Escribir datos en la flash
//    uint8_t data_to_write[6] = {0xAA, 0xBB, 0xCC, 0xDD}; // Datos de ejemplo
//    retval = W25_WriteNoCheck(data_to_write, 0, sizeof(data_to_write));
//    if (retval == RET_OK) {
//        ESP_LOGI("W25N01GV", "Datos escritos correctamente");
//    } else {
//        ESP_LOGE("W25N01GV", "Error al escribir datos");
//    }
//
//    // Leer datos de la flash
//    uint8_t read_data[6] = {0};
//    retval = W25_Read(read_data, 0, sizeof(read_data));
//    if (retval == RET_OK) {
//        ESP_LOGI("W25N01GV", "Datos leídos correctamente");
//    } else {
//        ESP_LOGE("W25N01GV", "Error al leer datos");
//    }
//
//    // Comparar los datos leídos con los escritos
//    if (memcmp(data_to_write, read_data, sizeof(data_to_write)) == 0) {
//        ESP_LOGI("W25N01GV", "Verificación de datos exitosa");
//    } else {
//        ESP_LOGE("W25N01GV", "Error en la verificación de datos");
//    }
//
//    // Finaliza la prueba
//    ESP_LOGI("W25N01GV", "Prueba completa");


//    //quaternion TEST

//	LSM6DSO_Object_t* LSMHan;
//	LSMHan = LSM6DSO_Initialize();
//	LSM6DSO_Axes_t AccelerationLSM;
//	uint8_t id_LSM;
//	LSM6DSO_ACC_Enable(LSMHan);
//	quaternion LSM_quat;

//    struct bhy2_dev bhy2;
//    struct bhy2_data_quaternion quaternion_data;
//    bhy2 = BHI360_Initialize();
//	  BHI360_init_quaternion(1, 0, &quaternion_data, &bhy2);
//
//	if(LSM6DSO_ReadID(LSMHan, &id_LSM) != LSM6DSO_OK)
//		ESP_LOGE("TEST", "ReadID LSM error\n");
//    while (1) {
//    	BHI360_get_quaternion(&quaternion_data, &bhy2);
//    	if(LSM6DSO_ACC_GetAxes(LSMHan, &AccelerationLSM) != LSM6DSO_OK)
//    		ESP_LOGE("TEST", "GetAxes ACC LSM error\n");
//    	vector LSM_vec = {
//    			.x = (float)AccelerationLSM.x,
//				.y = (float)AccelerationLSM.y,
//				.z = (float)AccelerationLSM.z,
//    	};
//    	LSM_quat = ToQuaternion(LSM_vec);
//    	ESP_LOGI("LSM", "X = %f, Y = %f, Z = %f, W = %f", LSM_quat.x, LSM_quat.y, LSM_quat.z, LSM_quat.w);
//    	ESP_LOGI("TEST LSM ACC", "X = %ld, Y = %ld, Z = %ld", AccelerationLSM.x, AccelerationLSM.y, AccelerationLSM.z);
//    	ESP_LOGI("BHI", "X = %f, Y = %f, Z = %f, W = %f, Acc = %f\n", quaternion_data.x / 16384.0f,
//    			quaternion_data.y / 16384.0f, quaternion_data.z / 16384.0f, quaternion_data.w / 16384.0f,
//				((quaternion_data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
//    	vTaskDelay(pdMS_TO_TICKS(500));
//    }


	//DRV test

	drv2605L_t* drvDev0;
	drvDev0 = drv2605L_Initialize(&i2c_line_0);
	drv2605L_t* drvDev1;
	drvDev1 = drv2605L_Initialize(&i2c_line_1);
//	drv2605L_delete(drvDev);
	drv2605L_DeviceID_t id_DRV0;
	drv2605L_DeviceID_t id_DRV1;


	while(true){
		if(drv2605L_getDeviceID(drvDev0, &id_DRV0) != RET_OK)
			ESP_LOGE("TEST", "ReadID DRV2605L error in line 0\n");
		vTaskDelay(pdMS_TO_TICKS(1000));
		if(drv2605L_getDeviceID(drvDev1, &id_DRV1) != RET_OK)
			ESP_LOGE("TEST", "ReadID DRV2605L error in line 1\n");

		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
		gpio_set_level(LED_PIN, 1);
		if(drv2605L_go(drvDev0) != RET_OK)
			ESP_LOGE("TEST", "Motor 0 go error\n");
		if(drv2605L_go(drvDev1) != RET_OK)
			ESP_LOGE("TEST", "Motor 1 go error\n");
//		 Esperar 1 segundo
		ESP_LOGI("TEST", "Encendiendo led en %d y motores\n", LED_PIN);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
//		 Apagar el LED
		gpio_set_level(LED_PIN, 0);
//		if(drv2605L_stop(drvDev0) != RET_OK)
//			ESP_LOGE("TEST", "Motor 0 Stop error\n");
//		if(drv2605L_stop(drvDev1) != RET_OK)
//			ESP_LOGE("TEST", "Motor 1 Stop error\n");
//		 Esperar 1 segundo
		ESP_LOGI("TEST", "Apagando led en %d y motores\n", LED_PIN);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

////	Polling I2C Test
//
//	while(1){
//		ESP_LOGI("TEST", "While(1) reached\n");
//		if ( i2c_master_Initialize(&i2c_line_0) != 0) {
//				ESP_LOGE("i2c_0", "Init error");
//			}
//		if ( i2c_master_Initialize(&i2c_line_1) != 0) {
//				ESP_LOGE("i2c_1", "Init error");
//			}
//		ESP_LOGI("Test", "1");
////		i2c_master_Read(LSM6DSO_I2C_ADD, 0x0F, &addlsm, 1, &i2c_line_0);
////		vTaskDelay(pdMS_TO_TICKS(1000));
//		ESP_LOGI("Test", "2");
//		i2c_master_Read(BHI360_I2C_ADD, 0x2B, &addbhi, 1, &i2c_line_0);
//		vTaskDelay(pdMS_TO_TICKS(1000));
//		ESP_LOGI("Test", "3");
//		i2c_master_Read(DRV2605L_SLAVE_ADDRESS, 0x00, &adddrv1, 1, &i2c_line_0);
//		vTaskDelay(pdMS_TO_TICKS(1000));
//		ESP_LOGI("Test", "4");
//		i2c_master_Read(DRV2605L_SLAVE_ADDRESS, 0x00, &adddrv2, 1, &i2c_line_1);
//		vTaskDelay(pdMS_TO_TICKS(1000));
//		ESP_LOGI("i2c_test", "vals: 0x%X, 0x%X, 0x%X\n", addbhi, adddrv1, adddrv2);
//		i2c_master_Terminate(&i2c_line_0);
//		i2c_master_Terminate(&i2c_line_1);
//
//	}

////	SIMPLE LED TEST
//
//	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
//	while(1){
//		gpio_set_level(LED_PIN, 1);
//		ESP_LOGI("TEST", "Encendiendo %d", LED_PIN);
//		// Esperar 1 segundo
//		vTaskDelay(1000 / portTICK_PERIOD_MS);
//		// Apagar el LED
//		gpio_set_level(LED_PIN, 0);
//		ESP_LOGI("TEST", "Apagando %d", LED_PIN);
//		// Esperar 1 segundo
//		vTaskDelay(1000 / portTICK_PERIOD_MS);
//	}
}
