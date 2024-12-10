/*
 * batteryMonitor.c
 *
 *  Created on: 17 nov. 2020
 *      Author: sreal, based on Espressif ADC1 Example
 *      It uses GPIO34 (ADC_CHANNEL_7), which measures V_bat/2
 */

#include "batteryMonitor.h"
#include <stdio.h>
//#include <stdlib.h>
#include "deviceConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"


//#include "esp_timer.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static const char *BATTERY_MANAGER_TAG = "Battery Manager";

static esp_adc_cal_characteristics_t *adc_chars;

SemaphoreHandle_t adcMutex;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    	ESP_LOGI(BATTERY_MANAGER_TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    	ESP_LOGI(BATTERY_MANAGER_TAG, "Characterized using eFuse Vref");
    } else {
    	ESP_LOGI(BATTERY_MANAGER_TAG, "Characterized using Default Vref");
    }
}

void batteryMonitorInit(){
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    adcMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(adcMutex);

    //Configure ADC
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    ESP_LOGI(BATTERY_MANAGER_TAG, "Initialized");
    print_char_val_type(val_type);
}

uint32_t getBatteryVoltage()
{
	uint32_t adc_reading = 0;
	//Multisampling
	xSemaphoreTake(adcMutex, portMAX_DELAY);
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		adc_reading += adc1_get_raw((adc1_channel_t)BATTERY_ADC_CHANNEL);
	}
	adc_reading /= NO_OF_SAMPLES;
	//Convert adc_reading to voltage in mV
	uint32_t retval = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars)*2;
	xSemaphoreGive(adcMutex);
	ESP_LOGI(BATTERY_MANAGER_TAG, "Raw: %d\tVoltage: %dmV", (int)adc_reading, (int)retval);
	return retval;
}
