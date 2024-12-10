/*
 * networkManager.h
 *
 *  Created on: 14 oct. 2020
 *      Author: sreal
 */

#ifndef MAIN_NETWORKMANAGER_H_
#define MAIN_NETWORKMANAGER_H_

#include "transportLayer.h"
#include "commonTypes.h"


#define NETWORK_MANAGER_REMOTE_PORT 11000
#define NETWORK_MANAGER_HOST_PORT	11000
#define NETWORK_MANAGER_SOCKET_DATA (socketData_t) { .port = NETWORK_MANAGER_HOST_PORT, .callback = networkManagerCallback }


retval_t networkManagerInit(uint8_t deviceID);
int getNetClock();
retval_t setNetClock(int32_t time);
retval_t networkManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length);


#endif /* MAIN_NETWORKMANAGER_H_ */
