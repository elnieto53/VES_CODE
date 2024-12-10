/*
 * nvs.h
 *
 *  Created on: 6 dic. 2020
 *      Author: sreal
 */

#ifndef MAIN_INC_NVSHANDLER_H_
#define MAIN_INC_NVSHANDLER_H_

#include "commonTypes.h"

retval_t nvsManagerInit();
retval_t nvsSetAxis(const char* key, axis_t data);
retval_t nvsGetAxis(const char* key, axis_t *out);


#endif /* MAIN_INC_NVSHANDLER_H_ */
