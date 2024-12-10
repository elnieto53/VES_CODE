/*
 * bsp.c
 *
 *  Created on: 12 mar 2024
 *      Author: nietoschoebel
 */

#include "bsp.h"
#include "deviceConfig.h"
#include "commonTypes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"

/*************************************************************************************/
/*COMMON*/
/*************************************************************************************/

int32_t GetTick (void){
	return xTaskGetTickCount();
}

/*************************************************************************************/
/*I2C Line 0*/
/*************************************************************************************/

int32_t I2C0_Initialize(void){				//OJO!!! El BHI360 y el LSM6DSO comparten linea de i2c (Proteger contra 2 inicializaciones)
	ESP_LOGI(I2C_TAG_0, "I2C Initialization\n");
	retval_t init = i2c_master_Initialize(&i2c_line_0);
	if(init != RET_OK){
		if(init == RET_REINIT)
			ESP_LOGW(I2C_TAG_0, "I2C line already initialized\n");
		else{
			ESP_LOGE(I2C_TAG_0, "Init error\n");
			return RET_ERROR;
		}
	}
	return RET_OK;
}

int32_t I2C0_Terminate(void){
	ESP_LOGI(I2C_TAG_0, "I2C Termination\n");
	if(i2c_master_Terminate(&i2c_line_0) != RET_OK){
		ESP_LOGE(I2C_TAG_0, "DeInit error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

/*************************************************************************************/
/*I2C Line 1*/
/*************************************************************************************/

int32_t I2C1_Initialize(void){				//OJO!!! El BHI360 y el LSM6DSO comparten linea de i2c (Proteger contra 2 inicializaciones)
	ESP_LOGI(I2C_TAG_1, "I2C Initialization\n");
	retval_t init = i2c_master_Initialize(&i2c_line_1);
	if(init != RET_OK){
		if(init == RET_REINIT)
			ESP_LOGW(I2C_TAG_1, "I2C line already initialized\n");
		else{
			ESP_LOGE(I2C_TAG_1, "Init error\n");
			return RET_ERROR;
		}
	}
	return RET_OK;
}


int32_t I2C1_Terminate(void){
	ESP_LOGI(I2C_TAG_1, "I2C Termination\n");
	if(i2c_master_Terminate(&i2c_line_1) != RET_OK){
		ESP_LOGE(I2C_TAG_1, "DeInit error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

/*************************************************************************************/
/*LSM6DSO*/
/*************************************************************************************/

static const char *LSM6DSO = "LSM6DSO";

static LSM6DSO_Object_t LSMHandler;
static LSM6DSO_IO_t LSMIO;

int32_t LSM6DSO_i2c_Write(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
//	ESP_LOGI(LSM6DSO, "I2C LSM6DSO Writing 0x%X in 0x%X \n",*pData, Reg);
	if(i2c_master_Write(Address, Reg, pData,  Length, &i2c_line_0) != RET_OK){
		ESP_LOGE(LSM6DSO, "Write error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

int32_t LSM6DSO_i2c_Read(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
//	ESP_LOGI(LSM6DSO, "I2C LSM6DSO Reading in 0x%X\n", Reg);
	if(i2c_master_Read(Address, Reg, pData,  Length, &i2c_line_0) != RET_OK){
		ESP_LOGE(LSM6DSO, "Read error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

LSM6DSO_Object_t* LSM6DSO_getHandler(void){
	return &LSMHandler;
}

LSM6DSO_Object_t* LSM6DSO_Initialize(void){
	LSMIO = (LSM6DSO_IO_t){
		.Init = I2C0_Initialize,
		.DeInit = I2C0_Terminate,
		.BusType = 0, 						//Bus i2c
		.Address = LSM6DSO_I2C_ADD,
		.ReadReg = LSM6DSO_i2c_Read,
		.WriteReg = LSM6DSO_i2c_Write,
		.GetTick = GetTick,
	};

	LSM6DSO_RegisterBusIO(&LSMHandler, &LSMIO);

	if(LSM6DSO_Init(&LSMHandler) != LSM6DSO_OK){
		ESP_LOGE(LSM6DSO, "Initialization error\n");
		return NULL;
	}

	return &LSMHandler;
}

/*************************************************************************************/
/*BHI360*/
/*************************************************************************************/

static const char *BHI360 = "BHI360";

struct bhy2_dev bhy2;

int8_t BHI360_i2c_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr){

	(void)intf_ptr;

//	ESP_LOGI(BHI360, "I2C BHI360 Writing 0x%X in 0x%X \n", *reg_data, reg_addr);
	if(i2c_master_Write(BHI360_I2C_ADD, reg_addr, (uint8_t *)reg_data, (uint16_t)length, &i2c_line_0) != RET_OK){
		ESP_LOGE(BHI360, "Write error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

int8_t BHI360_i2c_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr){

	(void)intf_ptr;

//	ESP_LOGI(BHI360, "I2C BHI360 Reading in 0x%X\n", reg_addr);
	if(i2c_master_Read(BHI360_I2C_ADD, reg_addr, reg_data,  length, &i2c_line_0) != RET_OK){
		ESP_LOGE(BHI360, "Read error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

void BHI360_DelayUs(uint32_t period_us, void *intf_ptr){

	(void)intf_ptr;

	uint32_t periodTicks = pdMS_TO_TICKS(period_us/1000);

	uint32_t now = GetTick();
	vTaskDelayUntil(&now, periodTicks);
}

struct bhy2_dev BHI360_Initialize(void){
	int8_t rslt;
	rslt = bhy2_init(BHY2_I2C_INTERFACE,
					 BHI360_i2c_Read,
					 BHI360_i2c_Write,
					 BHI360_DelayUs,
					 BHi360_RD_WR_LEN,
	                 NULL,
					 &bhy2);
	if(rslt != BHY2_OK){
		ESP_LOGE(BHI360, "Init error %d\n", rslt);
		return bhy2;
	}

	I2C0_Initialize();
	return bhy2;
}

/*************************************************************************************/
/*DRV2605L*/
/*************************************************************************************/

static const char *DRV2605L = "DRV2605L";

retval_t drv2605_init(i2c_line_t* i2c_line){
	if(i2c_line->i2c_port == I2C_PORT_0){
		I2C0_Initialize();
		i2c_line->mutex = i2c_line_0.mutex;
	}else if(i2c_line->i2c_port == I2C_PORT_1){
		I2C1_Initialize();
		i2c_line->mutex = i2c_line_1.mutex;
	}else{
		ESP_LOGE(DRV2605L, "That I2C line does not exist\n");
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t drv2605L_register_read(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf){
//	ESP_LOGI(DRV2605L, "I2C line %d, DRV2605L Reading in 0x%X\n", (uint8_t)drvDevice->i2c_line->i2c_port, regAddress);
	if(i2c_master_Read(drvDevice->devAddress, regAddress, buf,  1, drvDevice->i2c_line) != RET_OK){
		ESP_LOGE(DRV2605L, "Read error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t drv2605L_register_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t buf){
//	ESP_LOGI(DRV2605L, "I2C line %d, DRV2605L Reading in 0x%X\n", (uint8_t)drvDevice->i2c_line->i2c_port, regAddress);
	if(i2c_master_Write((uint8_t)drvDevice->devAddress, (uint8_t)regAddress, &buf,  1, drvDevice->i2c_line) != RET_OK){
		ESP_LOGE(DRV2605L, "Read error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

retval_t drv2605L_block_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf, uint16_t length){
//	ESP_LOGI(DRV2605L, "I2C line %d, DRV2605L Reading in 0x%X\n", (uint8_t)drvDevice->i2c_line->i2c_port, regAddress);
	if(i2c_master_Write((uint8_t)drvDevice->devAddress, (uint8_t)regAddress, buf,  length, drvDevice->i2c_line) != RET_OK){
		ESP_LOGE(DRV2605L, "Read error\n");
		return RET_ERROR;
	}
	return RET_OK;
}

drv2605L_t* drv2605L_Initialize(i2c_line_t* i2c_line){
	retval_t rslt;
	drv2605L_t* drvDev = drv2605L_create(i2c_line);
	rslt = drv2605L_init(drvDev, I2C_ACTUATOR_1);
	if(rslt != RET_OK){
		ESP_LOGE(DRV2605L, "Init error %d\n", rslt);
		return drvDev;
	}
	return drvDev;
}

/*************************************************************************************/
/*W25N01GV*/
/*************************************************************************************/






