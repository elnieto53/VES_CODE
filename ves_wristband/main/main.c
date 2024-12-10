#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "deviceConfig.h"

#include "networkManager.h"
#include "transportLayer.h"
#include "virtualScenarioManager.h"
#include "moCapManager.h"
#include "moCap.h"
#include "haptics.h"
#include "bsp.h"
#include "quatOps.h"
#include "quaternion.h"
#include "deviceConfig.h"
#include "commonTypes.h"
#include "w25n01gv.h"
#include "proximity.h"

#include "nvsManager.h"

#include "wifiManager.h"

static const char *SERVER_TAG = "Network Manager Server";

void initLED(uint8_t gpio){
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = (1ULL<<gpio);
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
}

#if configGENERATE_RUN_TIME_STATS && configUSE_STATS_FORMATTING_FUNCTIONS
/* This example demonstrates how a human readable table of run time stats
information is generated from raw data provided by uxTaskGetSystemState().
The human readable table is written to pcWriteBuffer.  (see the vTaskList()
API function which actually does just this).
configGENERATE_RUN_TIME_STATS and configUSE_STATS_FORMATTING_FUNCTIONS
must both be defined as 1 for this function to be available. */
void vTaskGetRunTimeStats2( char *pcWriteBuffer )
{
TaskStatus_t *pxTaskStatusArray;
volatile UBaseType_t uxArraySize, x;
unsigned long ulStatsAsPercentage;
uint32_t ulTotalRunTime = 1;

   /* Make sure the write buffer does not contain a string. */
   *pcWriteBuffer = 0x00;

   /* Take a snapshot of the number of tasks in case it changes while this
   function is executing. */
   uxArraySize = uxTaskGetNumberOfTasks();

   /* Allocate a TaskStatus_t structure for each task.  An array could be
   allocated statically at compile time. */
   pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
//   ESP_LOGI(SERVER_TAG, "Number of task: %d", uxArraySize);

   if( pxTaskStatusArray != NULL )
   {
      /* Generate raw status information about each task. */
//	   ESP_LOGI(SERVER_TAG, "OK1");
       uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

      /* For percentage calculations. */
      ulTotalRunTime /= 100UL;

      /* Avoid divide by zero errors. */
      if( ulTotalRunTime > 0 )
      {
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
         for( x = 0; x < uxArraySize; x++ )
         {
            /* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;
            if( ulStatsAsPercentage > 0UL )
            {
               sprintf( pcWriteBuffer, "%s\t\t%lu%%\t\t%u\r\n",
                                 pxTaskStatusArray[x].pcTaskName,
                                 ulStatsAsPercentage,
								 pxTaskStatusArray[x].uxBasePriority
								 );
            }
            else
            {
               /* If the percentage is zero here then the task has
               consumed less than 1% of the total run time. */
               sprintf( pcWriteBuffer, "%s\t\t<1%%\t\t%u\r\n",
                                 pxTaskStatusArray[x].pcTaskName,
								 pxTaskStatusArray[x].uxBasePriority
								 );
            }
            pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
         }
      }

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
}

void log_TaskTimeStats(){
	char[300] text;
	vTaskGetRunTimeStats2(text);
	ESP_LOGI(SERVER_TAG, "Output: \r\n%s", text);
}

#endif


retval_t comProtocolInit(uint8_t deviceID){
	retval_t retval;
	if(virtualScenarioManagerInit() != RET_OK)
		return RET_ERROR;
	if(networkManagerInit(deviceID) != RET_OK)
		return RET_ERROR;
	if(moCapManagerInit() != RET_OK)
		return RET_ERROR;

	genList_t *rxSocketDataList = genListInit();
	genListAdd(rxSocketDataList, &NETWORK_MANAGER_SOCKET_DATA);
	genListAdd(rxSocketDataList, &MOCAP_SOCKET_DATA);
	genListAdd(rxSocketDataList, &VIRTUAL_SCENARIO_SOCKET_DATA);

	retval = transportLayerInit(WIFI_MODE_STA, rxSocketDataList);

	genListRemoveAll(rxSocketDataList);
	return retval;
}


void configInputButton(){
	gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 27;//GPIO_NUM_27;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}


void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	nvsManagerInit();

	APP_ERROR_CHECK(comProtocolInit(DEVICE_ID), "Could not init the application com layer");
	APP_ERROR_CHECK(moCapInit(BODY_PART, MOCAP_ORIENTATION_CHANNEL), "Could not init the motion capture system");

	if(hapticsInit(HAPTICS_CHANNEL1, HAPTICS_CHANNEL2) == RET_OK){
		setVibrationAmplitude1(HAPTICS_MAX_AMPLITUDE);
		setVibrationAmplitude2(HAPTICS_MAX_AMPLITUDE);
		vTaskDelay(200 / portTICK_PERIOD_MS);
		setVibrationAmplitude1(HAPTICS_ZERO_AMPLITUDE);
		setVibrationAmplitude2(HAPTICS_ZERO_AMPLITUDE);
	}

	initLED(NOTIF_LED);
    while (true) {
        gpio_set_level(NOTIF_LED, 1);
        //etVibrationAmplitude(HAPTICS_MAX_AMPLITUDE);
        		//vTaskDelay(200 / portTICK_PERIOD_MS);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        //setVibrationAmplitude(HAPTICS_ZERO_AMPLITUDE);
        gpio_set_level(NOTIF_LED, 0);
#if configGENERATE_RUN_TIME_STATS && configUSE_STATS_FORMATTING_FUNCTIONS
        log_TaskTimeStats();
#endif

        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}




