/*
 * wifiAP.h
 *
 *  Created on: 12 ene. 2022
 *      Author: sreal
 */

#ifndef MAIN_COMMUNICATIONS_SRC_TRANSPORT_WIFIAP_H_
#define MAIN_COMMUNICATIONS_SRC_TRANSPORT_WIFIAP_H_

#include "wifiManager.h"

#define WIFI_SSID 		"Test_Implant"
#define WIFI_PASS		"B105Testing"
#define WIFI_CHANNEL	1
#define MAX_STA_CONN	5


/*List of available access points. A lower index results into a higher connection priority*/
wifi_sta_config_t registeredAP[] = {
	{ .ssid = "Knight025_Net", .password = "pasatiempo", .pmf_cfg = { .capable = true, .required = false } },
	{ .ssid = "Test_Implant", .password = "B105Testing", .pmf_cfg = { .capable = true, .required = false } },
	{ .ssid = "AndroidAP_SRV", .password = "pasatiempo", .pmf_cfg = { .capable = true, .required = false } },
	{ .ssid = "DESKTOP-SAGENI", .password = "60293fZw", .pmf_cfg = { .capable = true, .required = false } },
	{ .ssid = "MiFibra-EAA9", .password = "TZtnDkr9", .pmf_cfg = { .capable = true, .required = false }	},
	{ .ssid = "vodafoneBA2168", .password = "DHLP6KAYJDENTUHE", .pmf_cfg = { .capable = true, .required = false }	},
	{ .ssid = "Linksys11276", .password = "B105core", .pmf_cfg = { .capable = true, .required = false }	},
	{ .ssid = "B105_net", .password = "FiNaO?21", .pmf_cfg = { .capable = true, .required = false }	},
	{ .ssid = "" },
};

//wifi_ap_data_t registeredAP[] = {
//	{ .ssid = "Knight025_Net", .password = "pasatiempo" },
//	{ .ssid = "Test_Implant", .password = "B105Testing" },
//	{ .ssid = "AndroidAP_SRV", .password = "pasatiempo" },
//	{ .ssid = "DESKTOP-SAGENI", .password = "60293fZw" },
//	{ .ssid = "MiFibra-EAA9", .password = "TZtnDkr9" },
//	{ .ssid = "vodafoneBA2168", .password = "DHLP6KAYJDENTUHE" },
//	{ .ssid = "Linksys11276", .password = "B105core" },
//	{ .ssid = "B105_net", .password = "FiNaO?21" },
//	{ .ssid = "" },
//};


#endif /* MAIN_COMMUNICATIONS_SRC_TRANSPORT_WIFIAP_H_ */
