/*
 * moCap.c
 *
 *  Created on: 23 oct. 2020
 *      Author: sreal
 */
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "quatOps.h"
#include <math.h>
#include "vector.h"
#include "moCap.h"
#include "bsp.h"
#include "quaternion.h"
#include "lsm6dso_reg.h"
#include "lsm6dso_util.h"
#include "networkManager.h"
#include "moCapManager.h"
#include "nvsManager.h"
#include "deviceConfig.h"
#include "driver/gpio.h"


#define UPDATE_PERIOD			0.02 //Time period in s
#define DEFAULT_ORIENTATION		QUATERNION_IDENTITY
#define DEFAULT_GRAVITY			VECTOR_ZERO
#define GRAVITY_EXP_FILT_LAMBDA 0.3f

#define GPIO_INPUT_INT1     34  /*GPIO34 Connected to lsm6dso INT1*/
#define GPIO_INPUT_INT2     38	/*GPIO38 Connected to lsm6dso INT2*/
#define GPIO_INPUT_INTQ     37	/*GPIO38 Connected to bhi360 INT*/
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_INT2) | (1ULL<<GPIO_INPUT_INT1) | (1ULL<<GPIO_INPUT_INTQ))
#define ESP_INTR_FLAG_DEFAULT 0

#define NVS_ACCEL_RAW_OFFSET_KEY	"accelRawOffset"
#define NVS_GYRO_RAW_OFFSET_KEY		"gyroRawOffset"

#if BODY_PART == RIGHT_ARM || BODY_PART == RIGHT_FOREARM
    #define PART 2
#elif BODY_PART == LEFT_ARM || BODY_PART == LEFT_FOREARM
    #define PART 1
#else
    #define PART 0
#endif

//typedef struct moCapPrivateData_{
//	axis_t lastRawGyro;
//	axis_t lastRawAccel;
//} moCapPrivateData_t;

static const char *IMU_TAG = "IMU";

/*Function definitions*/
/*  Task*/
static void gyrManagerTask(void *pvParameters);
static void accManagerTask(void *pvParameters);
static void quatManagerTask(void *pvParameters);
static void updateTask(void *pvParameters);
/*  Auxiliary*/
static void initIMU(lsm6dso_fs_g_t gyrMode, lsm6dso_odr_g_t gyrFreq, lsm6dso_fs_xl_t accMode, lsm6dso_odr_xl_t accFreq);
static void initMoCapData(MoCapHandler_t *moCapData);
static void updateMoCapRotation(MoCapHandler_t *moCapData, quaternion rotationIncrement);
static void updateMoCapGravity(MoCapHandler_t *moCapData, vector rawLocalGravity, float lambda);
static quaternion getOrientation(MoCapHandler_t *moCapData);
static void periodic_timer_callback(void* arg);


static moCapOrientationUpdate moCapElementUpdate = {
	.elementID = 10,
	.prefabID = 0,
	.timestamp = 2,
	.bodyID = 1,
	.orientation = QUATERNION_IDENTITY
};

//typedef struct __attribute__((__packed__)) {
//	unsigned long long elementID;
//	uint8_t prefabID;
//	int timestamp;
//	uint8_t bodyID;
//	quaternion orientation;
//} moCapOrientationUpdate;

static struct bhy2_dev bhyQ;
static struct bhy2_data_quaternion quat_data;

static LSM6DSO_Object_t* LSMHan;
static SemaphoreHandle_t updateSemaphore;

static esp_timer_handle_t periodic_timer;		//Todo

/*IMU Data*/
static MoCapHandler_t defaultMoCapData;

static vsChannelHandle *channelHandle;

static IMUconversionFunc accConversionFunc;		//Todo
static IMUconversionFunc gyrConversionFunc;		//Todo
static float_t gyrFrequency;

static SemaphoreHandle_t gyrSemaph, accSemaph, quatSemaph;

static bool enable = false;

retval_t moCapInit(bodyPart_prefabID_t prefabID, uint8_t channel){
	updateSemaphore = xSemaphoreCreateBinary();

	moCapElementUpdate.prefabID = prefabID;
	moCapElementUpdate.elementID = prefabID; /*TEMPORAL. BEWARE: The ID should be unique in the channel*/

//	gyrSemaph = xSemaphoreCreateBinary();
//	accSemaph = xSemaphoreCreateBinary();
	quatSemaph = xSemaphoreCreateBinary();

	bhyQ = BHI360_Initialize();
	BHI360_init_quaternion(50, 0, &quat_data, &bhyQ);
	initIMU(LSM6DSO_1000dps, LSM6DSO_GY_ODR_104Hz, LSM6DSO_8g, LSM6DSO_XL_ODR_208Hz);	//Todo
	initMoCapData(&defaultMoCapData);

	uint8_t id_BHI;

	if(bhy2_get_product_id(&id_BHI, &bhyQ) != BHY2_OK)
		ESP_LOGE("TEST", "ReadID BHI error\n");
	if(id_BHI == BHI360_FUSER2_DEV_ID)
		ESP_LOGI("TEST", "OK, BHI = 0x%X\n", id_BHI);

//	xTaskCreate(gyrManagerTask, "gyrManagerTask", 4096, &defaultMoCapData, 10, NULL);
//	xTaskCreate(accManagerTask, "accManagerTask", 4096, &defaultMoCapData, 10, NULL);
	xTaskCreate(quatManagerTask, "quatManagerTask", 4096, &defaultMoCapData, 10, NULL);

	xTaskCreate(updateTask, "MoCapUP", 4096, &defaultMoCapData, 11, NULL);

	moCapManager_setHostBodyPart(&defaultMoCapData);

	esp_timer_create_args_t periodic_timer_args = {		//Todo
		.callback = &periodic_timer_callback,
		.name = "periodic"
	};

	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, UPDATE_PERIOD*1000000));

	ESP_LOGI(IMU_TAG, "Motion Capture initialized");

	if((channelHandle = addChannel(channel, NULL)) == NULL){
		ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
		ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
		return RET_ERROR;
	}

	ESP_LOGI(IMU_TAG, "Channel registered for moCap: %d" , channelHandle->channel);
	enable = true;

	return RET_OK;
}


static void gyrManagerTask(void *pvParameters){
	MoCapHandler_t *moCapData = (MoCapHandler_t*)pvParameters;
	vector eulerRotationIncrement;
	axis_t rawGyroData;

	while(1){
		if(xSemaphoreTake(gyrSemaph, portMAX_DELAY) != pdTRUE){
			ESP_LOGE(IMU_TAG, "Error with the gyrManagerTask");
		}
		lsm6dso_angular_rate_raw_get(&LSMHan->Ctx, (int16_t*)&rawGyroData);

//		ESP_LOGI(IMU_TAG, "Raw Gyro: (%d, %d, %d)", rawGyroData.x, rawGyroData.y, rawGyroData.z);

		xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
		moCapData->lastRawGyro = rawGyroData;
		rawGyroData.x -= moCapData->gyroRawOffset.x;
		rawGyroData.y -= moCapData->gyroRawOffset.y;
		rawGyroData.z -= moCapData->gyroRawOffset.z;
		xSemaphoreGive(moCapData->mutex);

		/*Get mdps, convert it to rps and get the angle increment*/
		eulerRotationIncrement.x = gyrConversionFunc(rawGyroData.z) / gyrFrequency * M_PI / 180000;
		eulerRotationIncrement.y = gyrConversionFunc(rawGyroData.y) / gyrFrequency * M_PI / 180000;
		eulerRotationIncrement.z = gyrConversionFunc(rawGyroData.x) / gyrFrequency * M_PI / 180000;

//		updateMoCapRotation(moCapData, ToQuaternion(eulerRotationIncrement));
	}
}

static void accManagerTask(void *pvParameters){
	MoCapHandler_t *moCapData = (MoCapHandler_t*)pvParameters;
	vector rawLocalGravity;
	axis_t rawAccelData;

	while(1){
		if(xSemaphoreTake(accSemaph, portMAX_DELAY) != pdTRUE){
			ESP_LOGE(IMU_TAG, "Error with the accManagerTask");
		}
		lsm6dso_acceleration_raw_get(&LSMHan->Ctx, (int16_t*)&rawAccelData);

		xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
		moCapData->lastRawAccel = rawAccelData;
		xSemaphoreGive(moCapData->mutex);

		/*Get gravity vector in g*/
		rawLocalGravity.x = accConversionFunc(rawAccelData.x)/1000;
		rawLocalGravity.y = accConversionFunc(rawAccelData.y)/1000;
		rawLocalGravity.z = accConversionFunc(rawAccelData.z)/1000;

//		ESP_LOGI(IMU_TAG, "(%f, %f, %f)", rawLocalGravity.x, rawLocalGravity.y, rawLocalGravity.z);

//		updateMoCapGravity(moCapData, rawLocalGravity, GRAVITY_EXP_FILT_LAMBDA);
	}
}


static void quatManagerTask(void *pvParameters){
	MoCapHandler_t *moCapData = (MoCapHandler_t*)pvParameters;
	quaternion aux_quat;

	while(1){
		if(xSemaphoreTake(quatSemaph, portMAX_DELAY) != pdTRUE){
			ESP_LOGE(IMU_TAG, "Error with the quatManagerTask");
		}

		BHI360_get_quaternion(&quat_data, &bhyQ);

		switch(PART){
			case 0:
				aux_quat.x = -quat_data.y; aux_quat.y = -quat_data.x; aux_quat.z = -quat_data.z; aux_quat.w = quat_data.w;
//				//Si se pone en la espalda cambiar por lo siguiente
//				aux_quat.x = quat_data.y; aux_quat.y = -quat_data.x; aux_quat.z = quat_data.z; aux_quat.w = quat_data.w;
				break;
			case 1:
				aux_quat.x = quat_data.x; aux_quat.y = -quat_data.z; aux_quat.z = quat_data.y; aux_quat.w = quat_data.w;
				break;
			case 2:
				aux_quat.x = -quat_data.x; aux_quat.y = -quat_data.z; aux_quat.z = -quat_data.y; aux_quat.w = quat_data.w;
				break;
		}

		updateMoCapRotation(moCapData, aux_quat);
	}
}

static void updateTask(void *pvParameters){
	MoCapHandler_t *moCapData = (MoCapHandler_t*)pvParameters;
	while(1){
		if(xSemaphoreTake(updateSemaphore, portMAX_DELAY) == pdTRUE){
			moCapElementUpdate.timestamp = getNetClock();
			moCapElementUpdate.orientation = getOrientation(moCapData);

			sendUpdateToSubscriptors(channelHandle, (uint8_t*)&moCapElementUpdate, sizeof(moCapOrientationUpdate));

//			moCapOrientationUpdate arg;
//			arg.bodyID = moCapElementUpdate.bodyID;
//			arg.elementID = moCapElementUpdate.elementID;
//			arg.orientation = moCapElementUpdate.orientation;
//			arg.prefabID = moCapElementUpdate.prefabID;
//			arg.timestamp = moCapElementUpdate.timestamp;
		}
	}
}

static void periodic_timer_callback(void* arg){
	xSemaphoreGive(updateSemaphore);
}

static void initMoCapData(MoCapHandler_t *moCapData){
	moCapData->orientation = DEFAULT_ORIENTATION;
	moCapData->gravity = DEFAULT_GRAVITY;
	moCapData->orientationOffset = QUATERNION_IDENTITY;
	moCapData->inicialRot = QUATERNION_IDENTITY;
	moCapData->lastRawAccel = (axis_t){.x = 0, .y = 0, .z = 0};
	moCapData->lastRawGyro = (axis_t){.x = 0, .y = 0, .z = 0};
	if(nvsGetAxis(NVS_ACCEL_RAW_OFFSET_KEY, &(moCapData->accelRawOffset)) != RET_OK){
		ESP_LOGE(IMU_TAG, "ARGFGSSGSGRTG");
		moCapData->accelRawOffset = (axis_t){.x = 0, .y = 0, .z = 0};
	}
	ESP_LOGI(IMU_TAG, "Accel offset loaded: (%d, %d, %d)", moCapData->accelRawOffset.x, moCapData->accelRawOffset.y, moCapData->accelRawOffset.z);
	if(nvsGetAxis(NVS_GYRO_RAW_OFFSET_KEY, &(moCapData->gyroRawOffset)) != RET_OK){
		ESP_LOGE(IMU_TAG, "ARGFGSSGSGRTG");
		moCapData->gyroRawOffset = (axis_t){.x = 0, .y = 0, .z = 0};
	}
	ESP_LOGI(IMU_TAG, "Gyro offset loaded: (%d, %d, %d)", moCapData->gyroRawOffset.x, moCapData->gyroRawOffset.y, moCapData->gyroRawOffset.z);

	moCapData->mutex = xSemaphoreCreateMutex();
	xSemaphoreGive(moCapData->mutex);
}

static void updateMoCapRotation(MoCapHandler_t *moCapData, quaternion rotationIncrement){
	xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
	/*Add the rotation increment*/
//	moCapData->orientation = normaliseQ(multiplicationQ(moCapData->orientation, rotationIncrement));
	moCapData->orientation = rotationIncrement;
	xSemaphoreGive(moCapData->mutex);
	return;
}

static void updateMoCapGravity(MoCapHandler_t *moCapData, vector rawLocalGravity, float lambda){
	xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
	/*Transform gravity from local to global coordinates and filter it*/
	vector rawGlobalGravity = transformDirection(rawLocalGravity, moCapData->orientation);
	moCapData->gravity.x = moCapData->gravity.x*(1-lambda) + rawGlobalGravity.x*lambda;
	moCapData->gravity.y = moCapData->gravity.y*(1-lambda) + rawGlobalGravity.y*lambda;
	moCapData->gravity.z = moCapData->gravity.z*(1-lambda) + rawGlobalGravity.z*lambda;
	moCapData->gravity = vector_normalize(moCapData->gravity);
	xSemaphoreGive(moCapData->mutex);
	return;
}

static quaternion getOrientation(MoCapHandler_t *moCapData){
	quaternion retval;
//	quaternion auxrot = {
////			.x = 0.71, .y = 0, .z = 0, .w = 0.71,		//LEFT
////			.x = 0, .y = -0.71, .z = 0.71, .w = 0,		//RIGHT
//			.x = 0, .y = 0, .z = 0, .w = 1,
//	};
	xSemaphoreTake(moCapData->mutex, portMAX_DELAY);

//	retval = multiplicationQ(fromToRotationQ(moCapData->gravity, VECTOR_DOWN), moCapData->orientation);

//	BHI360_get_quaternion(&quat_data, &bhyQ);

//	retval.x = quat_data.x; retval.y = quat_data.y; retval.z = quat_data.z; retval.w = quat_data.w;

//	retval = multiplicationQ(auxrot, moCapData->orientation);
//	retval = multiplicationQ(retval, moCapData->orientationOffset);
	retval = multiplicationQ(moCapData->orientationOffset, moCapData->orientation);
	retval = multiplicationQ(retval, moCapData->inicialRot);
//	ESP_LOGW("TEST", "(%f, %f, %f, %f)", retval.x, retval.y, retval.z, retval.w);
//	ESP_LOGI("TEST", "(%f, %f, %f, %f)", moCapData->orientationOffset.x, moCapData->orientationOffset.y, moCapData->orientationOffset.z, moCapData->orientationOffset.w);
	xSemaphoreGive(moCapData->mutex);
	return retval;
}

void moCapCalibratePose(MoCapHandler_t *moCapData, vector localRotRef, vector globalRotRef, quaternion initialRot){
	xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
	/*First the IMU 'down' vector is aligned with the gravity vector*/
	ESP_LOGI(IMU_TAG, "CALIBRATION: Vector gravity: (%f, %f, %f)", moCapData->gravity.x, moCapData->gravity.y, moCapData->gravity.z);
	ESP_LOGI(IMU_TAG, "CALIBRATION: Local reference: (%f, %f, %f)", localRotRef.x, localRotRef.y, localRotRef.z);
	ESP_LOGI(IMU_TAG, "CALIBRATION: Global reference: (%f, %f, %f)", globalRotRef.x, globalRotRef.y, globalRotRef.z);
	ESP_LOGI(IMU_TAG, "CALIBRATION: Initial Rotation reference: (%f, %f, %f)", initialRot.x, initialRot.y, initialRot.z);

//	BHI360_init_quaternion(50, 0, &quat_data, &bhyQ);
	/*Thereafter, the horizontal components of the device and world reference vectors are calculated*/
//	vector vecFrom = transformDirection(localRotRef, moCapData->orientation);
////	vecFrom.y = 0;
////	vecFrom.z = 1;
//	if(vector_module(vecFrom) < 0.1){
//		ESP_LOGE(IMU_TAG, "Calibration error: the device reference is not contained in the xz plane");
//		return;
//	}
//	vecFrom = vector_normalize(vecFrom);
//
//	vector vecTo = globalRotRef;
//	if(vector_module(vecFrom) < 0.1){
//		ESP_LOGE(IMU_TAG, "Calibration error: the global reference is not contained in the xz plane");
//		return;
//	}
//	vecTo = vector_normalize(vecTo);
//
////	ESP_LOGI(IMU_TAG, "Vector from: (%f, %f, %f), vector to: (%f, %f, %f)", vecFrom.x, vecFrom.y, vecFrom.z, vecTo.x, vecTo.y, vecTo.z);
//
//	ESP_LOGI(IMU_TAG, "CALIBRATION: VecFrom: (%f, %f, %f), VecTo: (%f, %f, %f)", vecFrom.x, vecFrom.y, vecFrom.z, vecTo.x, vecTo.y, vecTo.z);
//	/*Finally, the previous device and world reference vectors are calculated*/
//
//	moCapData->orientationOffset = normaliseQ(multiplicationQ(fromToRotationQ(vecFrom, vecTo), initialRot));

	moCapData->orientationOffset = normaliseQ(conjugateQ(moCapData->orientation));

	moCapData->inicialRot = initialRot;
//	moCapData->orientation = multiplicationQ(fromToRotationQ(vecFrom, vecTo), moCapData->orientation);
//	ESP_LOGI("TEST", "(%f, %f, %f, %f)", moCapData->orientation.x, moCapData->orientation.y, moCapData->orientation.z, moCapData->orientation.w);
//	moCapData->orientationOffset = normaliseQ(multiplicationQ(conjugateQ(moCapData->orientation), initialRot));
//	ESP_LOGI("TEST", "(%f, %f, %f, %f)", moCapData->orientationOffset.x, moCapData->orientationOffset.y, moCapData->orientationOffset.z, moCapData->orientationOffset.w);

	xSemaphoreGive(moCapData->mutex);
}

retval_t moCapCalibrateOffset(MoCapHandler_t *moCapData){
	ESP_LOGI(IMU_TAG, "OFFSET CALIBRATION: Gyro: (%d, %d, %d)", moCapData->lastRawGyro.x, moCapData->lastRawGyro.y, moCapData->lastRawGyro.z);
	ESP_LOGI(IMU_TAG, "OFFSET CALIBRATION: Accel: (%d, %d, %d)", moCapData->lastRawAccel.x, moCapData->lastRawAccel.y, moCapData->lastRawAccel.z);
	xSemaphoreTake(moCapData->mutex, portMAX_DELAY);
	moCapData->gyroRawOffset = moCapData->lastRawGyro;
	moCapData->accelRawOffset = moCapData->lastRawAccel;
	xSemaphoreGive(moCapData->mutex);
	if(nvsSetAxis(NVS_ACCEL_RAW_OFFSET_KEY, moCapData->accelRawOffset) != RET_OK)
		return RET_ERROR;
	if(nvsSetAxis(NVS_GYRO_RAW_OFFSET_KEY, moCapData->gyroRawOffset) != RET_OK)
		return RET_ERROR;
	ESP_LOGI(IMU_TAG, "Data saved");

	return RET_OK;
}

//static void IRAM_ATTR gpio_int1_handler(void* arg)	//Todo
//{
//    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	xSemaphoreGiveFromISR( accSemaph, &xHigherPriorityTaskWoken );
//}
//
//static void IRAM_ATTR gpio_int2_handler(void* arg)	//Todo
//{
//    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	xSemaphoreGiveFromISR( gyrSemaph, &xHigherPriorityTaskWoken );
//}

static void IRAM_ATTR gpio_intq_handler(void* arg)	//Todo
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( quatSemaph, &xHigherPriorityTaskWoken );
}

static void initGPIOInterrupt(){
	gpio_config_t io_conf;

	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
//    gpio_isr_handler_add(GPIO_INPUT_INT2, gpio_int2_handler, (void*) GPIO_INPUT_INT2);
//    //hook isr handler for specific gpio pin
//    gpio_isr_handler_add(GPIO_INPUT_INT1, gpio_int1_handler, (void*) GPIO_INPUT_INT1);

    gpio_isr_handler_add(GPIO_INPUT_INTQ, gpio_intq_handler, (void*) GPIO_INPUT_INTQ);
}

static void initIMU(lsm6dso_fs_g_t gyrMode, lsm6dso_odr_g_t gyrFreq, lsm6dso_fs_xl_t accMode, lsm6dso_odr_xl_t accFreq){
//	uint8_t whoAmI;
//	uint8_t rst;
//
//	LSMHan = LSM6DSO_Initialize();
//
//    ESP_LOGI(IMU_TAG, "For now it's ok\n");
//
//    gpio_set_direction(25/*GPIO_NUM_13*/, GPIO_MODE_OUTPUT);
//    lsm6dso_device_id_get(&LSMHan->Ctx, &whoAmI);
//    int level = 0;
//    while(whoAmI != LSM6DSO_ID){	//Todo
//    	gpio_set_level(25/*GPIO_NUM_13*/, level);
//    	level = !level;
//    	ESP_LOGE(IMU_TAG, "ARGHH It does not work! %d\n", whoAmI);
//    	return;
//    	//vTaskDelay(300 / portTICK_PERIOD_MS);
//    }
//
//    /*Reset*/
//    lsm6dso_reset_set(&LSMHan->Ctx, PROPERTY_ENABLE);
//    do{
//    	lsm6dso_reset_get(&LSMHan->Ctx, &rst);
//    } while(rst);
//
//
//    /* Disable I3C interface */
//    lsm6dso_i3c_disable_set(&LSMHan->Ctx, LSM6DSO_I3C_DISABLE);	//Todo
//
//    /* Enable Block Data Update */
//    lsm6dso_block_data_update_set(&LSMHan->Ctx, PROPERTY_ENABLE);
//
//    /* Set Output Data Rate */
//    lsm6dso_xl_data_rate_set(&LSMHan->Ctx, accFreq);
//    lsm6dso_gy_data_rate_set(&LSMHan->Ctx, gyrFreq);
//
//    /* Set range */
//    lsm6dso_xl_full_scale_set(&LSMHan->Ctx, accMode);
//    lsm6dso_gy_full_scale_set(&LSMHan->Ctx, gyrMode);
//
//    accConversionFunc = getAccConversionData(accMode);
//    gyrConversionFunc = getGyrConversionData(gyrMode);
//    gyrFrequency = getGyrFrequency(gyrFreq);
//
//    /* Configure filtering chain(No aux interface)
//     * Accelerometer - LPF1 + LPF2 path */
//    lsm6dso_xl_hp_path_on_out_set(&LSMHan->Ctx, LSM6DSO_LP_ODR_DIV_100);
//    lsm6dso_xl_filter_lp2_set(&LSMHan->Ctx, PROPERTY_ENABLE);

    initGPIOInterrupt();

//    /*Init INT1 and INT2, corresponding to 'acc data ready' and 'gyro data ready'*/
//    lsm6dso_pin_int1_route_t int1_route;
//    lsm6dso_pin_int1_route_get(&LSMHan->Ctx, &int1_route);
//    int1_route.drdy_xl = PROPERTY_ENABLE;
//    lsm6dso_pin_int1_route_set(&LSMHan->Ctx, int1_route);
//
//    lsm6dso_pin_int2_route_t int2_route;
//	lsm6dso_pin_int2_route_get(&LSMHan->Ctx, NULL, &int2_route);
//	int2_route.drdy_g = PROPERTY_ENABLE;
//	lsm6dso_pin_int2_route_set(&LSMHan->Ctx, NULL, int2_route);
}

bool moCapEnabled(void){
	return enable;
}
