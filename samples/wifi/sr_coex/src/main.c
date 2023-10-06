/**
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief SR coexistence sample test bench
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include <nrfx_clock.h>
#include "zephyr_fmac_main.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define RSSI_INIT_VALUE 127

int main(void)
{
	int ret = 0;
	bool is_ant_mode_sep = IS_ENABLED(CONFIG_COEX_SEP_ANTENNAS);
	bool is_ble_central = IS_ENABLED(CONFIG_BT_ROLE_CENTRAL);
	bool is_wlan_server = IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER);
	bool is_zperf_udp = IS_ENABLED(CONFIG_WIFI_ZPERF_PROT_UDP);
	bool test_wlan = IS_ENABLED(CONFIG_TEST_TYPE_WLAN);
	bool test_ble = IS_ENABLED(CONFIG_TEST_TYPE_BLE);

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
	bool bt_external_antenna = IS_ENABLED(CONFIG_BT_EXTERNAL_ANTENNA);
#endif

	bool is_wifi_conn_scan =
		IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL) ||
		IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH) ||
		IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL) ||
		IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_TP_PERIPH) ||
		IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_SCAN_STABILITY) ||
		IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_SCAN_STABILITY);

#if !defined(CONFIG_COEX_SEP_ANTENNAS) && \
	!(defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP))
	BUILD_ASSERT("Shared antenna support is not available with nRF7002 shields");
#endif

	memset_context();
	
	wifi_net_mgmt_callback_functions();

	

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
	#if defined(CONFIG_NRF700X_BT_COEX)
		/* Configure SR side (nRF5340 side) switch in nRF7x */
		LOG_INF("Configure SR side (nRF5340 side) switch");
		ret = nrf_wifi_config_sr_switch(is_ant_mode_sep, bt_external_antenna);
		if (ret != 0) {
			LOG_ERR("Unable to configure SR side switch: %d", ret);
			goto err;
		}
	#endif /* CONFIG_NRF700X_BT_COEX */
#endif

#if defined(CONFIG_NRF700X_BT_COEX)
	/* Configure non-PTA registers of Coexistence Hardware */
	LOG_INF("Configuring non-PTA registers.");
	ret = nrf_wifi_coex_config_non_pta(is_ant_mode_sep);
	if (ret != 0) {
		LOG_ERR("Configuring non-PTA registers of CoexHardware FAIL");
		goto err;
	}
#endif /* CONFIG_NRF700X_BT_COEX */

#ifdef CONFIG_RUN_SPT_TESTS
	if (IS_ENABLED(CONFIG_WIFI_SCAN_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_SCAN_BLE_CON_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH)) {
		ret = wifi_scan_ble_connection(is_ant_mode_sep, test_ble, test_wlan,
		is_ble_central, is_wifi_conn_scan);
	}

	if (IS_ENABLED(CONFIG_WIFI_SCAN_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_SCAN_BLE_TP_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_CON_SCAN_BLE_TP_PERIPH)) {
		ret = wifi_scan_ble_tput(is_ant_mode_sep, test_ble, test_wlan,
		is_ble_central, is_wifi_conn_scan);
	}

	if (IS_ENABLED(CONFIG_WIFI_CON_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_CON_BLE_TP_PERIPH)) {
		ret = wifi_con_ble_tput(test_wlan, is_ant_mode_sep, test_ble, is_ble_central);
	}

	if (IS_ENABLED(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_PERIPH)) {
		ret = wifi_tput_ble_con(test_wlan, test_ble, is_ble_central,
		is_wlan_server, is_ant_mode_sep, is_zperf_udp);
	}

	if (IS_ENABLED(CONFIG_WIFI_TP_UDP_CLIENT_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_CLIENT_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_CLIENT_BLE_TP_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_CLIENT_BLE_TP_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_SERVER_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_SERVER_BLE_TP_CENTRAL) ||
	IS_ENABLED(CONFIG_WIFI_TP_UDP_SERVER_BLE_TP_PERIPH) ||
	IS_ENABLED(CONFIG_WIFI_TP_TCP_SERVER_BLE_TP_PERIPH)) {
		ret = wifi_tput_ble_tput(test_wlan, is_ant_mode_sep,
		test_ble, is_ble_central, is_wlan_server, is_zperf_udp);
	}
#endif
#ifdef CONFIG_RUN_ST_TESTS
	if (IS_ENABLED(CONFIG_WIFI_CON_BLE_CON_CENTRAL_STABILITY) ||
	IS_ENABLED(CONFIG_WIFI_CON_BLE_CON_PERIPH_STABILITY)) {
		ret = wifi_con_stability_ble_con_interference(test_wlan, test_ble, is_ble_central,
		is_ant_mode_sep);
	}

	if (IS_ENABLED(CONFIG_WIFI_CON_BLE_TP_CENTRAL_STABILITY) ||
	IS_ENABLED(CONFIG_WIFI_CON_BLE_TP_PERIPH_STABILITY)) {
		ret = wifi_con_stability_ble_tput_interference(test_wlan, is_ant_mode_sep, test_ble,
			is_ble_central);
	}

	if (IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_SCAN_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_SCAN_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_SCAN_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_SCAN_STABILITY)) {
		ret = ble_con_stability_wifi_scan_interference(is_ant_mode_sep, test_ble, test_wlan,
		is_ble_central, is_wifi_conn_scan);
	}

	if (IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_STABILITY)) {
		if (is_ble_central) {
			LOG_INF("Test case: ble_conn_central_wifi_con_stability");
		} else {
			LOG_INF("Test case: ble_conn_peripheral_wifi_con_stability");
		}
		ret = ble_con_stability_wifi_conn_interference(test_wlan, test_ble, is_ble_central,
		is_ant_mode_sep);
	}

	if (IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_UDP_CLIENT_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_TCP_CLIENT_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_UDP_CLIENT_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_TCP_CLIENT_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_UDP_SERVER_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_TCP_SERVER_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_UDP_SERVER_STABILITY) ||
	IS_ENABLED(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_TCP_SERVER_STABILITY)) {
		ret = ble_con_stability_wifi_tput_interference(test_wlan, test_ble,
		is_ble_central, is_wlan_server, is_ant_mode_sep, is_zperf_udp);
	}
#endif
#ifdef CONFIG_RUN_WST_TESTS

	if (IS_ENABLED(CONFIG_BLE_CON_CENTRAL_WIFI_SHUTDOWN) ||
	IS_ENABLED(CONFIG_BLE_CON_PERIPHERAL_WIFI_SHUTDOWN)) {
		ret = ble_con_wifi_shutdown(is_ble_central);
	}

	if (IS_ENABLED(CONFIG_BLE_TP_CENTRAL_WIFI_SHUTDOWN) ||
	IS_ENABLED(CONFIG_BLE_TP_PERIPH_WIFI_SHUTDOWN)) {
		ret = ble_tput_wifi_shutdown(is_ble_central);
	}
#endif
	/* common to all the above function calls */
	if (ret != 0) {
		LOG_INF("Test case failed");
		goto err;
	}

	LOG_INF("BLE Tx Power: %d", ble_txpower);
	if (wifi_rssi != RSSI_INIT_VALUE) {
		LOG_INF("WiFi RSSI: %d", wifi_rssi);
	} else {
		LOG_INF("WiFi RSSI: NA");
	}
	if (ble_rssi != RSSI_INIT_VALUE) {
		LOG_INF("BLE RSSI: %d", ble_rssi);
	} else {
		LOG_INF("BLE RSSI: NA");
	}
	LOG_INF("Test case(s) complete");

	return 0;

err:
	LOG_INF("Returning with error");
	return ret;
}
