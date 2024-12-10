/*
 * moCapManager.h
 *
 *  Created on: 7 dic. 2020
 *      Author: sreal
 */

#ifndef MAIN_COMMUNICATIONS_INC_APPLICATION_MOCAPMANAGER_H_
#define MAIN_COMMUNICATIONS_INC_APPLICATION_MOCAPMANAGER_H_

#include "transportLayer.h"
#include "commonTypes.h"
#include "moCap.h"

#define MOCAP_MANAGER_HOST_PORT	11002
#define MOCAP_SOCKET_DATA (socketData_t) { .port = MOCAP_MANAGER_HOST_PORT, .callback = moCapManagerCallback }

retval_t moCapManagerInit();
retval_t moCapManager_setHostBodyPart(MoCapHandler_t* moCapHandler);
retval_t moCapManagerCallback(in_addr_t origin, uint8_t *data, uint16_t length);

#endif /* MAIN_COMMUNICATIONS_INC_APPLICATION_MOCAPMANAGER_H_ */
