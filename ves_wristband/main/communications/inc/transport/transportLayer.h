/*
 * udpManager.h
 *
 *  Created on: 28 sept. 2020
 *      Author: sreal
 */

#ifndef MAIN_TRANSPORTLAYER_H_
#define MAIN_TRANSPORTLAYER_H_

#include <stdint.h>
#include "commonTypes.h"
#include "lwip/sockets.h"
#include "genList.h"
#include "esp_wifi.h"


typedef retval_t (*rxCallback)(in_addr_t origin, uint8_t *data, uint16_t length);

typedef struct rxSocketData_ {
	int port;				/*Binds rx socket to this port*/
	rxCallback callback;	/*Callback for when a pkg arrives*/
} socketData_t;

retval_t transportLayerInit(wifi_mode_t mode, genList_t *rxSocketDataList);
retval_t sendPkg(in_addr_t addr, in_port_t port, uint8_t *data, uint16_t length);

#endif /* MAIN_TRANSPORTLAYER_H_ */
