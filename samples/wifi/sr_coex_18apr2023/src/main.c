/**
 * Copyright (c) 2022 Nordic Semiconductor ASA
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

//#include "common.h"
#include "main.h"

#include <nrfx_clock.h>

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
#include "zephyr_fmac_main.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
void print_test_params_info(bool test_wlan, bool test_ble,
				bool antenna_mode, bool ble_role, bool wifi_coex_enable,
				bool ble_coex_enable, bool coex_hardware_enable)
{
	if (test_wlan && test_ble) {
		if (coex_hardware_enable) {
			LOG_INF("Running WLAN and BLE tests with coex hardware enable");
		} else {
			LOG_INF("Running WLAN and BLE tests with coex hardware disable");
		}
	} else {
		if (test_wlan) {
			LOG_INF("Running WLAN only test");
		} else {
			LOG_INF("Running BLE only test");
		}
	}
	if (antenna_mode) {
		LOG_INF("Antenna mode : Separate antennas");
	} else {
		LOG_INF("Antenna mode : Shared antennas");
	}

	if (ble_role) {
		LOG_INF("BLE role : Central");
	} else {
		LOG_INF("BLE role : Peripheral");
	}

	if (wifi_coex_enable) {
		LOG_INF("WLAN posts requests to PTA");
	} else {
		LOG_INF("WLAN doesn't post requests to PTA");
	}

	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesn't post requests to PTA");
	}
	if (IS_ENABLED(CONFIG_RPU_ENABLE)) {
		LOG_INF("RPU enabled");
	} else {
		LOG_INF("RPU disabled");
	}
}


void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_IPV4_DHCP_BOUND:
		print_dhcp_ip(cb);
		break;
	default:
		break;
	}
}

void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				uint32_t mgmt_event, struct net_if *iface)
{
	const struct device *dev = iface->if_dev->dev;
	struct wifi_nrf_vif_ctx_zep *vif_ctx_zep = NULL;

	vif_ctx_zep = dev->data;

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	case NET_EVENT_WIFI_SCAN_RESULT:
		vif_ctx_zep->scan_in_progress = 0;
		handle_wifi_scan_result(cb);
		break;
	case NET_EVENT_WIFI_SCAN_DONE:
		handle_wifi_scan_done(cb);
		break;
	default:
		break;
	}
}
#endif

int main(void)
{

#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)
	#ifdef BLE_PEER_THROUGHPUT_TEST
		LOG_INF("BLE peer throughput test");
	#else
		LOG_INF("BLE peer connection-central test");
	#endif
	ble_peer_tp_conn();
#endif 

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)

		int ret = 0;
	bool wifi_coex_enable = IS_ENABLED(CONFIG_WIFI_COEX_ENABLE);
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);
	bool coex_hardware_enable = IS_ENABLED(CONFIG_COEX_HARDWARE_ENABLE);
	bool antenna_mode = IS_ENABLED(CONFIG_COEX_SEP_ANTENNAS);
	bool ble_role = IS_ENABLED(CONFIG_COEX_BT_CENTRAL);
	bool wlan_role = IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER);

		bool test_wlan = IS_ENABLED(CONFIG_TEST_TYPE_WLAN);
		bool test_ble = IS_ENABLED(CONFIG_TEST_TYPE_BLE);

	#if !defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP) && !defined(CONFIG_COEX_SEP_ANTENNAS)
		BUILD_ASSERT("Shared antenna support is not available with nRF7002 shields");
	#endif

		memset_context();

		net_mgmt_init_event_callback(&wifi_sta_mgmt_cb,
					wifi_mgmt_event_handler,
					WIFI_MGMT_EVENTS);

		net_mgmt_add_event_callback(&wifi_sta_mgmt_cb);

		net_mgmt_init_event_callback(&net_addr_mgmt_cb,
					net_mgmt_event_handler,
					NET_EVENT_IPV4_DHCP_BOUND);

		net_mgmt_add_event_callback(&net_addr_mgmt_cb);

	#ifdef CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT
		nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	#endif

		LOG_INF("Starting %s with CPU frequency: %d MHz", CONFIG_BOARD,
					SystemCoreClock/MHZ(1));
		k_sleep(K_SECONDS(1));

		print_test_params_info(test_wlan, test_ble, antenna_mode, ble_role,
					wifi_coex_enable, ble_coex_enable, coex_hardware_enable);

	#ifdef CONFIG_NRF700X_BT_COEX
		/* Configure SR side (nRF5340 side) switch in nRF7002 DK */
		LOG_INF("Configure SR side (nRF5340 side) switch");
		ret = nrf_wifi_config_sr_switch(antenna_mode);
		if (ret != 0) {
			LOG_ERR("Unable to configure SR side switch: %d", ret);
			goto err;
		}

		/* Configure Coexistence Hardware non-PTA registers */
		LOG_INF("Configuring non-PTA registers.");
		ret = nrf_wifi_coex_config_non_pta(antenna_mode);
		if (ret != 0) {
			LOG_ERR("Configuring non-PTA registers of CoexHardware FAIL");
			goto err;
		}
	#endif /* CONFIG_NRF700X_BT_COEX */

		/**if (!(IS_ENABLED(CONFIG_RPU_ENABLE))) {
		 *	LOG_INF("RPU disabled");
		 *	rpu_disable();
		 *}
		 */

		#ifdef WIFI_SCAN_BLE_CON_CENTRAL
			LOG_INF("Test case: wifi_scan_ble_conn_central");
			ret = wifi_scan_ble_conn_central(wifi_coex_enable, antenna_mode, test_ble,
					test_wlan, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_scan_ble_conn_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_SCAN_BLE_CON_PERIPH
			LOG_INF("Test case: wifi_scan_ble_conn_peripheral");
			ret = wifi_scan_ble_conn_peripheral(wifi_coex_enable, antenna_mode, test_ble,
					test_wlan, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_scan_ble_conn_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_SCAN_BLE_TP_CENTRAL
			LOG_INF("Test case: wifi_scan_ble_tput_central");
			ret = wifi_scan_ble_tput_central(wifi_coex_enable, antenna_mode,
					test_ble, test_wlan, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_scan_ble_tput_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_SCAN_BLE_TP_PERIPH
			LOG_INF("Test case: wifi_scan_ble_tput_peripheral");
			ret = wifi_scan_ble_tput_peripheral(wifi_coex_enable, antenna_mode,
					test_ble, test_wlan, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_scan_ble_tput_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_CON_BLE_CON_CENTRAL
			LOG_INF("Test case: wifi_con_ble_con_central");
			ret = wifi_con_ble_con_central(test_wlan, wifi_coex_enable, test_ble, ble_role,
					wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_con_ble_con_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_CON_BLE_CON_PERIPH
			LOG_INF("Test case: wifi_con_ble_con_peripheral");
			ret = wifi_con_ble_con_peripheral(test_wlan, wifi_coex_enable, test_ble,
					ble_role, wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_con_ble_con_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_CON_BLE_TP_CENTRAL
			LOG_INF("Test case: wifi_con_ble_tput_central");
			ret = wifi_con_ble_tput_central(test_wlan, wifi_coex_enable, antenna_mode,
					test_ble, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_con_ble_tput_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_CON_BLE_TP_PERIPH
			LOG_INF("Test case: wifi_con_ble_tput_peripheral");
			ret = wifi_con_ble_tput_peripheral(test_wlan, wifi_coex_enable, antenna_mode,
					test_ble, ble_role, wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_con_ble_tput_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_TP_CLIENT_BLE_CON_CENTRAL
			LOG_INF("Test case: wifi_tput_client_ble_con_central");
			ret = wifi_tput_client_ble_con_central(test_wlan, wifi_coex_enable, test_ble,
					ble_role, wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_client_ble_con_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_TP_CLIENT_BLE_CON_PERIPH
			LOG_INF("Test case: wifi_tput_client_ble_con_peripheral");
			ret = wifi_tput_client_ble_con_peripheral(test_wlan, wifi_coex_enable, test_ble,
					ble_role, wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_client_ble_con_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_TP_SERVER_BLE_CON_CENTRAL
			LOG_INF("Test case: wifi_tput_server_ble_con_central");
			ret = wifi_tput_server_ble_con_central(test_wlan, wifi_coex_enable, test_ble,
					ble_role, wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_server_ble_con_central fail");
				goto err;
			}
		#endif

		#ifdef WIFI_TP_SERVER_BLE_CON_PERIPH
			LOG_INF("Test case: wifi_tput_server_ble_con_peripheral");
			ret = wifi_tput_server_ble_con_peripheral(test_wlan, wifi_coex_enable, test_ble,
					ble_role,
					wlan_role, antenna_mode, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_server_ble_con_peripheral fail");
				goto err;
			}
		#endif

		#ifdef WIFI_TP_CLIENT_BLE_TP_CENTRAL
			if (!IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER) && IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
				LOG_INF(" Test case: wifi_tput_client_ble_tput_central");
				ret = wifi_tput_client_ble_tput_central(test_wlan, wifi_coex_enable,
						antenna_mode, test_ble, ble_role,
				wlan_role, coex_hardware_enable);
				if (ret != 0) {
					LOG_ERR("Running wifi_tput_client_ble_tput_central fail");
					goto err;
				}
			}
		#endif

		#ifdef WIFI_TP_CLIENT_BLE_TP_PERIPH
			if (!IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER) && !IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
				LOG_INF(" Test case: wifi_tput_client_ble_tput_peripheral");
				ret = wifi_tput_client_ble_tput_peripheral(test_wlan, wifi_coex_enable,
				antenna_mode, test_ble, ble_role, wlan_role, coex_hardware_enable);
				if (ret != 0) {
					LOG_ERR("Running wifi_tput_client_ble_tput_peripheral fail");
					goto err;
				}
			}
		#endif

		#ifdef WIFI_TP_SERVER_BLE_TP_CENTRAL
			if (IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER) && IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
				LOG_INF(" Test case: wifi_tput_server_ble_tput_central");
				ret = wifi_tput_server_ble_tput_central(test_wlan, wifi_coex_enable,
					antenna_mode, test_ble, ble_role, wlan_role, coex_hardware_enable);
				if (ret != 0) {
					LOG_ERR("Running wifi_tput_server_ble_tput_central fail");
					goto err;
				}
			}
		#endif

		#ifdef WIFI_TP_SERVER_BLE_TP_PERIPH
			if (IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER) && !IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
				LOG_INF(" Test case: wifi_tput_server_ble_tput_peripheral");
				ret = wifi_tput_server_ble_tput_peripheral(test_wlan, wifi_coex_enable,
						antenna_mode, test_ble, ble_role, wlan_role,
						coex_hardware_enable);
				if (ret != 0) {
					LOG_ERR("Running wifi_tput_server_ble_tput_peripheral fail");
					goto err;
				}
			}
		#endif

		return 0;

	err:
		LOG_INF("Returning with error");
		return ret;
#endif
}
