/*
 * wifiManager.h
 *
 *  Created on: 19 abr. 2022
 *      Author: sreal
 */

#ifndef MAIN_COMMUNICATIONS_INC_802_11_WIFIMANAGER_H_
#define MAIN_COMMUNICATIONS_INC_802_11_WIFIMANAGER_H_

#include "esp_wifi.h"
#include "commonTypes.h"

typedef struct {
    uint8_t ssid[32];      /**< SSID of target AP. */
    uint8_t password[64];  /**< Password of target AP. */
} wifi_ap_data_t;

retval_t WiFi_Init(wifi_mode_t mode);
retval_t WiFi_searchRegisteredAP(wifi_ap_record_t* apRecord, wifi_ap_data_t* apData);
retval_t WiFi_connectToAP(wifi_ap_data_t* apData);

#endif /* MAIN_COMMUNICATIONS_INC_802_11_WIFIMANAGER_H_ */
