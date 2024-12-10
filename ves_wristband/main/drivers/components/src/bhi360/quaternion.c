/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    quaternion.c
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bsp.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"
#include "quaternion.h"
#include "esp_log.h"

#include "BHI360.fw.h"

#define WORK_BUFFER_SIZE  256//2048

static Boolean firmware_upload = TRUE;

#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_GAMERV

//static const char *BHIQUAT = "BHI_QUAT";

static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);
static void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);

enum bhy2_intf intf;
uint8_t work_buffer[WORK_BUFFER_SIZE];

struct bhy2_data_quaternion *quaternion_data;

retval_t BHI360_init_quaternion(float sample_rate, uint32_t report_latency_ms, struct bhy2_data_quaternion *data, struct bhy2_dev *bhy2){
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    quaternion_data = data;

    setup_interfaces(); /* Perform a power on reset */

    rslt = bhy2_init(BHY2_I2C_INTERFACE, BHI360_i2c_Read, BHI360_i2c_Write, BHI360_DelayUs, BHY2_RD_WR_LEN, NULL, bhy2);

    print_api_error(rslt, bhy2);

    rslt = bhy2_soft_reset(bhy2);
    print_api_error(rslt, bhy2);

    rslt = bhy2_get_product_id(&product_id, bhy2);
    print_api_error(rslt, bhy2);

    /* Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID)
    {
        ESP_LOGE(BHI360, "Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
    }
    else
    {
    	ESP_LOGI(BHI360, "BHI360 found. Product ID read %X\r\n", product_id);
    }

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, bhy2);
    print_api_error(rslt, bhy2);
    rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, bhy2);
    print_api_error(rslt, bhy2);

    ESP_LOGI(BHI360, "Host interrupt control\r\n");
    ESP_LOGI(BHI360, "Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    ESP_LOGI(BHI360, "Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    ESP_LOGI(BHI360, "Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    ESP_LOGI(BHI360, "Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    ESP_LOGI(BHI360, "Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    ESP_LOGI(BHI360, "Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    ESP_LOGI(BHI360, "Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
    ESP_LOGI(BHI360, "Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, bhy2);
    print_api_error(rslt, bhy2);

    /* Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, bhy2);
    print_api_error(rslt, bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
    	if(firmware_upload){
			upload_firmware(boot_status, bhy2);

			rslt = bhy2_get_kernel_version(&version, bhy2);
			print_api_error(rslt, bhy2);
			if ((rslt == BHY2_OK) && (version != 0))
			{
				ESP_LOGI(BHI360, "Boot successful. Kernel version %u.\r\n", version);
			}

			firmware_upload = TRUE;
    	}

        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, bhy2);
        print_api_error(rslt, bhy2);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, bhy2);
        print_api_error(rslt, bhy2);
        rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, NULL, bhy2);
        print_api_error(rslt, bhy2);

        rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, bhy2);
        print_api_error(rslt, bhy2);
    }
    else
    {
    	ESP_LOGW(BHI360, "Host interface not ready. Exiting\r\n");

        close_interfaces();

        return RET_ERROR;
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhy2_update_virtual_sensor_list(bhy2);
    print_api_error(rslt, bhy2);

    rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, bhy2);
    print_api_error(rslt, bhy2);
    ESP_LOGI(BHI360, "Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

    return rslt;
}

retval_t BHI360_deinit_quaternion(void){
	close_interfaces();
	return RET_OK;
}

//retval_t BHI360_reset_quaternion(float sample_rate, uint32_t report_latency_ms, struct bhy2_dev *bhy2){
//	bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 0, report_latency_ms, bhy2);
//	bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, bhy2);
//	return RET_OK;
//}

retval_t BHI360_get_quaternion(struct bhy2_data_quaternion *data, struct bhy2_dev *bhy2){
    int8_t rslt;

	rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, bhy2);
	print_api_error(rslt, bhy2);

	*data = *quaternion_data;

//	ESP_LOGE("TEST", "(%d, %d, %d, %d)", data->x, data->y, data->z, data->w);

    return rslt;
}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhy2_data_quaternion data;
//    uint32_t s, ns;
    if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    *quaternion_data = data;
//
//    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */
//
//    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
//    s = (uint32_t)(timestamp / UINT64_C(1000000000));
//    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));
//
//    ESP_LOGI(BHIQUAT, "SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %.2f\r\n",
//           callback_info->sensor_id,
//           s,
//           ns,
//           data.x / 16384.0f,
//           data.y / 16384.0f,
//           data.z / 16384.0f,
//           data.w / 16384.0f,
//           ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
        	ESP_LOGI(BHI360, "%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
        	ESP_LOGI(BHI360, "%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
        	ESP_LOGI(BHI360, "%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
        	ESP_LOGI(BHI360, "%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
        	ESP_LOGI(BHI360, "%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
        	ESP_LOGI(BHI360, "%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
        	ESP_LOGI(BHI360, "%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
        	ESP_LOGI(BHI360, "%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
        	ESP_LOGI(BHI360, "%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
        	ESP_LOGI(BHI360, "%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
        	ESP_LOGI(BHI360, "%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
        	ESP_LOGI(BHI360, "%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
        	ESP_LOGI(BHI360, "%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
        	ESP_LOGI(BHI360, "%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
        	ESP_LOGI(BHI360, "%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
        	ESP_LOGI(BHI360, "%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
        	ESP_LOGI(BHI360, "%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
    	ESP_LOGE(BHI360, "%s\r\n", get_api_error(rslt));

        exit(0);
    }
}

static void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHY2_OK;


    ESP_LOGI(BHI360, "Loading firmware into RAM.\r\n");
    rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);

    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
    	ESP_LOGE(BHI360, "%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

    ESP_LOGI(BHI360, "Booting from RAM.\r\n");
    rslt = bhy2_boot_from_ram(dev);


    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
    	ESP_LOGE(BHI360, "%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}
