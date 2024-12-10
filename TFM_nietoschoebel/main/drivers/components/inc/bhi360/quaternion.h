/*
 * quaternion.h
 *
 *  Created on: 4 jul 2024
 *      Author: nietoschoebel
 */

#ifndef MAIN_DRIVERS_COMPONENTS_INC_BHI360_QUATERNION_H_
#define MAIN_DRIVERS_COMPONENTS_INC_BHI360_QUATERNION_H_

#include "commonTypes.h"

retval_t BHI360_init_quaternion(float sample_rate, uint32_t report_latency_ms, struct bhy2_data_quaternion *data, struct bhy2_dev *bhy2);
retval_t BHI360_deinit_quaternion(void);
retval_t BHI360_get_quaternion(struct bhy2_data_quaternion *data, struct bhy2_dev *bhy2);

#endif /* MAIN_DRIVERS_COMPONENTS_INC_BHI360_QUATERNION_H_ */
