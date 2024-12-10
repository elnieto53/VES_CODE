/*
 * commonTypes.h
 *
 *  Created on: 2 oct. 2020
 *      Author: sreal
 */

#ifndef MAIN_COMMONTYPES_H_
#define MAIN_COMMONTYPES_H_

#include "stdint.h"
#include <stdbool.h>

typedef enum retval_ {
	RET_OK = 0,
	RET_ERROR = 1,
	RET_TIMEOUT = 2,
	RET_REINIT = 3
} retval_t;

typedef struct __attribute__((__packed__)) axis_ {
	int16_t x;
	int16_t y;
	int16_t z;
} axis_t;

#define APP_ERROR_CHECK(x, msg) do {										\
	retval_t __err_rc = (x);												\
	if (__err_rc != RET_OK) {												\
		ESP_LOGE(SERVER_TAG, msg);				\
		while(1){ vTaskDelay(10 / portTICK_PERIOD_MS); }					\
	}																		\
} while(0)

#endif /* MAIN_COMMONTYPES_H_ */
