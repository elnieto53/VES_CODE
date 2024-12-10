/*
 * config.h
 *
 *  Created on: 7 ene. 2022
 *      Author: sreal
 */

#ifndef DEVICE_CONFIG_H_
#define DEVICE_CONFIG_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "virtualScenarioElements.h"

/********************************/
/*VES device configuration*/
/********************************/
//#define DEVICE_ID		5						//Each device under the same network requires a unique ID
//#define BODY_PART 		LEFT_ARM				//Body part monitored (MoCap)
#define DEVICE_ID		1
#define BODY_PART 		TORSO

/*Channels*/
#define HAPTICS_CHANNEL1 			DEVICE_ID + 10
#define HAPTICS_CHANNEL2 			DEVICE_ID + 20
#define MOCAP_ORIENTATION_CHANNEL	3



/********************************/
/*Board selection */
/********************************/
/*Supported platforms*/
#define ADAFRUIT_FEATHER_HUZZAH_DEVBOARD 	1
#define AZDELIVERY_DEVBOARD 				2
#define LILYGO_T_WRISTBAND	 				3

/*Platform selection*/
#define PLATFORM 				ADAFRUIT_FEATHER_HUZZAH_DEVBOARD


/**************************************/
/*Board configuration - DO NOT MODIFY */
/**************************************/

#if PLATFORM == ADAFRUIT_FEATHER_HUZZAH_DEVBOARD

/*LED GPIO*/
#define NOTIF_LED				(uint8_t)GPIO_NUM_25//GPIO_NUM_25
/*BATTERY GPIO*/
#define BATTERY_ADC_CHANNEL		ADC_CHANNEL_0	//It uses GPIO36 (ADC_CHANNEL_0), which measures V_bat/2

/*Default I2C GPIO*/
#define DEFAULT_I2C_SDA			GPIO_NUM_21
#define DEFAULT_I2C_SCL			GPIO_NUM_22

#elif PLATFORM == AZDELIVERY_DEVBOARD

/*LED GPIO*/
#define NOTIF_LED				(uint8_t)GPIO_NUM_4
/*BATTERY GPIO*/
#define BATTERY_ADC_CHANNEL		ADC_CHANNEL_7	//It uses GPIO35 (ADC_CHANNEL_7), which measures V_bat/2

/*Default I2C config*/
#define DEFAULT_I2C_SDA			GPIO_NUM_33
#define DEFAULT_I2C_SCL			GPIO_NUM_32

#elif PLATFORM == LILYGO_T_WRISTBAND

#define NOTIF_LED				(uint8_t)GPIO_NUM_4
#define BATTERY_ADC_CHANNEL		ADC_CHANNEL_7	//It uses GPIO35 (ADC_CHANNEL_7), which measures V_bat/2

/*Default I2C config*/
#define DEFAULT_I2C_SDA			GPIO_NUM_21
#define DEFAULT_I2C_SCL			GPIO_NUM_22

#else
#error "Please select a valid PLATFORM"
#endif

/*************************************************************************************/
/*General Config*/
/*************************************************************************************/

#define LED_PIN GPIO_NUM_25

#define HEAD 0
#define TORSO 1
#define RIGHT_ARM 2
#define LEFT_ARM 3
#define RIGHT_FOREARM 4
#define LEFT_FOREARM 5

/*************************************************************************************/
/*I2C config*/
/*************************************************************************************/

#define I2C_PORT_0					I2C_NUM_0
#define I2C_MASTER_SDA_IO_0			GPIO_NUM_21
#define I2C_MASTER_SCL_IO_0			GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ_0		100000

#define I2C_PORT_1					I2C_NUM_1
#define I2C_MASTER_SDA_IO_1			GPIO_NUM_26
#define I2C_MASTER_SCL_IO_1			GPIO_NUM_27
#define I2C_MASTER_FREQ_HZ_1		100000

static const char *I2C_TAG_0 = "I2C_LINE_0";

static const char *I2C_TAG_1 = "I2C_LINE_1";

/*************************************************************************************/
/*SPI Config*/
/*************************************************************************************/
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14
#define PIN_NUM_CS   GPIO_NUM_15
#define PIN_NUM_WP   GPIO_NUM_2
#define PIN_NUM_HD   GPIO_NUM_4

#define SPI_MASTER_FREQ_HZ		(10 * 1000 * 1000 / 10)//todo: Config this

#define SPI_INPUT_DELAY_NS		0 //todo: Config this

#define SPI_MODE 				0

#define SPI_QUEUE_SIZE			7

static const char *SPI_TAG = "SPI_LINE";

/*************************************************************************************/
/*LSM6DSO config*/
/*************************************************************************************/

//#define LSM6DSO_I2C_ADD LSM6DSO_I2C_ADD_L
#define LSM6DSO_I2C_ADD LSM6DSO_I2C_ADD_H

#define LSM6DSO_CS		GPIO_NUM_20
#define LSM6DSO_INT1	GPIO_NUM_34
#define LSM6DSO_INT2	GPIO_NUM_37

#define LSM6DSO_DEV_ID 	0x6C

/*************************************************************************************/
/*BHI360 config*/
/*************************************************************************************/

#define BHI360_I2C_ADD_L 	0x28
#define BHI360_I2C_ADD_H 	0x29

#define BHI360_I2C_ADD 		BHI360_I2C_ADD_L

#define BHI360_FUSER2_DEV_ID 	0x89

#define BHi360_RD_WR_LEN 	256    /* Maximum read write length */

/*************************************************************************************/
/*DRV2605L config*/
/*************************************************************************************/

#define DRV2605L_SLAVE_ADDRESS 0x5A

#define DRV2605L_DEV_ID 0xE0

/*************************************************************************************/
/*FLASH config*/
/*************************************************************************************/


#endif /* MAIN_CONFIG_H_ */
