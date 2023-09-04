/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef MAIN_H_
#define MAIN_H_

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

#include <zephyr_coex.h>


#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)



static struct net_mgmt_event_callback wifi_sta_mgmt_cb;
static struct net_mgmt_event_callback net_addr_mgmt_cb;

/**
 * @brief Print common test test parameters info
 *
 * @return No return value.
 */
void print_common_test_params(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
	bool is_ble_central);

/**
 * @brief Function to test Wi-Fi throughput client/server and BLE throughput central/peripheral
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_tput_ble_tput(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_zperf_udp);

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
void net_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
		struct net_if *iface);

/**
 * @brief Handle Wi-Fi management events
 *
 * @return No return value.
 */
void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
		struct net_if *iface);

#endif /* MAIN_H_ */
