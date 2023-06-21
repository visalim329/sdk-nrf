/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "common.h"

#include <nrfx_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/net/zperf.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/socket.h>
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
#include <zephyr_coex.h>
#endif


/**
 *#define WIFI_SCAN_BLE_CON_CENTRAL
 *#define WIFI_SCAN_BLE_CON_PERIPH
 *
 *#define WIFI_SCAN_BLE_TP_CENTRAL
 *#define WIFI_SCAN_BLE_TP_PERIPH
 *
 *#define WIFI_CON_BLE_CON_CENTRAL
 *#define WIFI_CON_BLE_CON_PERIPH
 *
 *#define WIFI_CON_BLE_TP_CENTRAL
 *#define WIFI_CON_BLE_TP_PERIPH
 *
 *#define WIFI_TP_CLIENT_BLE_CON_CENTRAL
 *#define WIFI_TP_CLIENT_BLE_CON_PERIPH
 *
 *#define WIFI_TP_CLIENT_BLE_TP_CENTRAL
 *#define WIFI_TP_CLIENT_BLE_TP_PERIPH
 *
 *#define WIFI_TP_SERVER_BLE_CON_CENTRAL
 *#define WIFI_TP_SERVER_BLE_CON_PERIPH
 *
 *#define WIFI_TP_SERVER_BLE_TP_CENTRAL
 *#define WIFI_TP_SERVER_BLE_TP_PERIPH
 */
#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | \
				NET_EVENT_WIFI_DISCONNECT_RESULT)

static struct net_mgmt_event_callback wifi_sta_mgmt_cb;
static struct net_mgmt_event_callback net_addr_mgmt_cb;
/**
 * @brief Print Test Params Info
 *
 * @return No return value.
 */
void print_test_params_info(bool test_wlan, bool test_ble, bool antenna_mode,
		bool ble_role,
		bool wifi_coex_enable, bool ble_coex_enable, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi scan and BLE connection central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_scan_ble_conn_central(bool wifi_coex_enable, bool antenna_mode, bool test_ble,
		bool test_wlan, bool ble_role, bool wlan_role, bool coex_hardware_enable,
		bool wifi_connected_scan);
/**
 * @brief Function to test Wi-Fi scan and BLE connection peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable, bool antenna_mode,
		bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool wifi_connected_scan);
/**
 * @brief Function to test Wi-Fi scan and BLE throughput central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_scan_ble_tput_central(bool wifi_coex_enable, bool antenna_mode, bool test_ble,
		bool test_wlan, bool ble_role, bool wlan_role, bool coex_hardware_enable,
		bool wifi_connected_scan);
/**
 * @brief Function to test Wi-Fi scan and BLE throughput peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_scan_ble_tput_peripheral(bool wifi_coex_enable, bool antenna_mode,
		bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool wifi_connected_scan);
/**
 * @brief Function to test Wi-Fi connection and BLE connection central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_con_central(bool test_wlan, bool wifi_coex_enable, bool test_ble,
		bool ble_role, bool wlan_role, bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection and BLE connection peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_con_peripheral(bool test_wlan, bool wifi_coex_enable, bool test_ble,
		bool ble_role, bool wlan_role, bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection and BLE throughput central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_tput_central(bool test_wlan, bool wifi_coex_enable, bool antenna_mode,
		bool test_ble, bool ble_role, bool wlan_role, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection and BLE throughput peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_tput_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi throughput and BLE connection central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_client_ble_con_central(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput client and BLE
 * connection peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_client_ble_con_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput client and BLE
 * throughput central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_client_ble_tput_central(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput client and BLE
 * throughput peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_client_ble_tput_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput server and BLE
 * connection central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_server_ble_con_central(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput server and BLE
 * connection peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_server_ble_con_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput server and BLE
 * throughput central
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_server_ble_tput_central(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi throughput server and BLE
 * throughput peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_server_ble_tput_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test Wi-Fi connection stability with BLE connection central
 * as interference.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_con_central_regr(bool test_wlan, bool wifi_coex_enable, bool test_ble,
		bool ble_role, bool wlan_role, bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection stability with BLE connection peripheral
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_con_peripheral_regr(bool test_wlan, bool wifi_coex_enable, bool test_ble,
		bool ble_role, bool wlan_role, bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection stability with BLE throughput central
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_tput_central_regr(bool test_wlan, bool wifi_coex_enable, bool antenna_mode,
		bool test_ble, bool ble_role, bool wlan_role, bool coex_hardware_enable);
/**
 * @brief Function to test Wi-Fi connection stability with BLE throughput peripheral
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_con_ble_tput_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool antenna_mode, bool test_ble, bool ble_role, bool wlan_role,
		bool coex_hardware_enable);
/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi scan as
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_central_wifi_scan_regr(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable, bool wifi_connected_scan);
/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi connection
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_central_wifi_con_regr(bool test_wlan,
	bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
	bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi ping
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */

/* PENDING: ble_conn_central_wifi_ping_regr() */

/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi throughput
 * client as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_central_wifi_tput_client_regr(bool test_wlan,
	bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
	bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi throughput
 * peripheral as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_central_wifi_tput_server_regr(bool test_wlan,
	bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
	bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test BLE connection(peripheral) stability with Wi-Fi scan as 
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_peripheral_wifi_scan_regr(bool wifi_coex_enable,
	bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
	bool wlan_role, bool coex_hardware_enable, bool wifi_connected_scan);
/**
 * @brief Function to test BLE connection(peripheral) stability with Wi-Fi connection
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_peripheral_wifi_con_regr(bool test_wlan,
	bool wifi_coex_enable, bool test_ble, bool ble_role,
	bool wlan_role, bool antenna_mode, bool coex_hardware_enable);
/**
 * @brief Function to test BLE connection(peripheral) stability with Wi-Fi ping
 * as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */

/* PENDING: ble_conn_peripheral_wifi_ping_regr() */

/**
 * @brief Function to test BLE connection(central) stability with Wi-Fi throughput
 * client as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_peripheral_wifi_tput_client_regr(bool test_wlan,
	bool wifi_coex_enable, bool test_ble, bool ble_role,
	bool wlan_role, bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test BLE connection(peripheral) stability with Wi-Fi throughput
 * peripheral as interference
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_conn_peripheral_wifi_tput_server_regr(bool test_wlan, bool wifi_coex_enable,
	bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
	bool coex_hardware_enable, bool zperf_udp_or_tcp);
/**
 * @brief Function to test BLE connection(central) functionality with Wi-Fi shutdown
  *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_con_central_wifi_shutdown(bool test_ble, bool ble_role);
/**
 * @brief Function to test BLE connection(central) functionality with Wi-Fi shutdown
  *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_con_peripheral_wifi_shutdown(bool test_ble, bool ble_role);
/**
 * @brief Function to test BLE throughput(central) functionality with Wi-Fi shutdown
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_tput_central_wifi_shutdown(bool test_ble, bool ble_role);
/**
 * @brief Function to test BLE throughput(peripheral) functionality with Wi-Fi shutdown
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int ble_tput_periph_wifi_shutdown(bool test_ble, bool ble_role);

/**
 * @brief memset_context
 *
 * @return No return value.
 */
void memset_context(void);
/**
 * @brief CB for Wi-Fi connection result
 *
 * @return No return value.
 */
void handle_wifi_connect_result(struct net_mgmt_event_callback *cb);
/**
 * @brief CB for Wi-Fi disconnection result
 *
 * @return No return value.
 */
void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb);
/**
 * @brief CB for Wi-Fi scan result
 *
 * @return No return value.
 */
void handle_wifi_scan_result(struct net_mgmt_event_callback *cb);
/**
 * @brief CB for Wi-Fi scan done
 *
 * @return No return value.
 */
void handle_wifi_scan_done(struct net_mgmt_event_callback *cb);
/**
 * @brief print DHCP IP addredd assigned
 *
 * @return No return value.
 */
void print_dhcp_ip(struct net_mgmt_event_callback *cb);
/**
 * @brief Handle net management events
 *
 * @return No return value.
 */
void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface);
/**
 * @brief Handle Wi-Fi management events
 *
 * @return No return value.
 */
void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				uint32_t mgmt_event, struct net_if *iface);

#endif /* MAIN_H_ */
