/*
 * bsp.h
 *
 *  Created on: 12 mar 2024
 *      Author: nietoschoebel
 */

#ifndef MAIN_INC_BSP_H_
#define MAIN_INC_BSP_H_

#include "commonTypes.h"
#include "deviceConfig.h"
#include "esp32-i2cDriver.h"
#include "esp32-spiDriver.h"

#include "lsm6dso.h"
#include "bhi3.h"
#include "bhy2_parse.h"
#include "drv2605L.h"

#endif /* MAIN_INC_BSP_H_ */

/*************************************************************************************/
/*COMMON*/
/*************************************************************************************/

int32_t GetTick (void);

/*************************************************************************************/
/*I2C Line 0*/
/*************************************************************************************/

static i2c_line_t i2c_line_0 = {
	.i2c_port = I2C_PORT_0,
	.sda_io_num = I2C_MASTER_SDA_IO_0,
	.scl_io_num = I2C_MASTER_SCL_IO_0,
	.clk_speed = I2C_MASTER_FREQ_HZ_0,
	.enabled = false,
	.mutex = NULL,
};

int32_t I2C0_Initialize(void);
int32_t I2C0_Terminate(void);

/*************************************************************************************/
/*I2C Line 1*/
/*************************************************************************************/

static i2c_line_t i2c_line_1 = {
	.i2c_port = I2C_PORT_1,
	.sda_io_num = I2C_MASTER_SDA_IO_1,
	.scl_io_num = I2C_MASTER_SCL_IO_1,
	.clk_speed = I2C_MASTER_FREQ_HZ_1,
	.enabled = false,
	.mutex = NULL,
};

int32_t I2C1_Initialize(void);
int32_t I2C1_Terminate(void);

/*************************************************************************************/
/*LSM6DSO*/
/*************************************************************************************/

int32_t LSM6DSO_i2c_Write(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t LSM6DSO_i2c_Read(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length);
LSM6DSO_Object_t* LSM6DSO_getHandler(void);
LSM6DSO_Object_t* LSM6DSO_Initialize(void);

/*************************************************************************************/
/*BHI360*/
/*************************************************************************************/

int8_t BHI360_i2c_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t BHI360_i2c_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
struct bhy2_dev BHI360_Initialize(void);
void BHI360_DelayUs(uint32_t period_us, void *intf_ptr);

/*************************************************************************************/
/*DRV2605L*/
/*************************************************************************************/

retval_t drv2605_init(i2c_line_t* i2c_line);
retval_t drv2605L_register_read(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf);
retval_t drv2605L_register_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t buf);
retval_t drv2605L_block_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf, uint16_t length);
drv2605L_t* drv2605L_Initialize(i2c_line_t* i2c_line);
