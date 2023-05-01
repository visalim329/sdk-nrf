/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_COEX_TEST_FUNCTIONS_
#define BT_COEX_TEST_FUNCTIONS_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#include <bluetooth/scan.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_coex_test_func, CONFIG_LOG_DEFAULT_LEVEL);

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

/* For net_sprint_ll_addr_buf */
#include "net_private.h"
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
#include <zephyr_coex.h>
#include <zephyr_coex_struct.h>
#endif

#include "bt_utils.h"
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
#include "zephyr_fmac_main.h"
#endif

#define MAX_SSID_LEN 32
#define WIFI_CONNECTION_TIMEOUT 30

#define PRINT_WIFI_SCAN_RESULT
#define PRINT_WIFI_CONN_RESULT
#define DEMARCATE_TEST_START

#define HIGHEST_CHANNUM_24G 14

extern bool ble_periph_connected;
uint8_t wait_for_ble_central_run;

static uint32_t wifi_scan_cnt_24g;
static uint32_t wifi_scan_cnt_5g;
static uint32_t wifi_scan_cmd_cnt;

static uint8_t wait_for_wifi_client_start;

extern uint32_t ble_conn_success_cnt;
extern uint32_t ble_conn_fail_cnt;

static uint32_t wifi_conn_success_cnt;
static uint32_t wifi_conn_fail_cnt;

static struct sockaddr_in in4_addr_my = {
	.sin_family = AF_INET,
	.sin_port = htons(CONFIG_NET_CONFIG_PEER_IPV4_PORT),
};

static uint32_t scan_result_count;
/**
 * @brief BT throughtput tets run
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void run_bt_benchmark(void);
/**
 * @brief commnad to start wifi scan
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int cmd_wifi_scan(void);
/**
 * @brief Call wifi connection event
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_connection(bool test_wlan, bool wifi_coex_enable,
				bool antenna_mode);
/**
 * @brief configure PTA registers
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int config_pta(bool wifi_coex_enable, bool antenna_mode, bool ble_role,
				bool wlan_role);
/**
 * @brief Start wi-fi traffic for zperf udp upload or configure
 * zperf traffic for udp download
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int run_wifi_traffic(bool test_wlan);
/**
 * @brief start BLE traffic using thread start
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void start_ble_traffic(bool test_ble, bool ble_role);
/**
 * @brief check if iperf traffic is complete
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void check_wifi_traffic(bool test_wlan);
/**
 * @brief run BLE traffic using thread join
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void run_ble_traffic(bool test_ble, bool ble_role);
/**
 * @brief Disconnect Wi-Fi
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void disconnect_wifi(bool test_wlan);
/**
 * @brief Disconnect BLE
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void disconnect_ble(bool test_ble, bool ble_role);

static struct {
	uint8_t connected :1;
	uint8_t disconnect_requested: 1;
	uint8_t _unused : 6;
} context;

K_SEM_DEFINE(wait_for_next, 0, 1);
K_SEM_DEFINE(udp_callback, 0, 1);
struct wifi_iface_status status = { 0 };
uint32_t repeat_scan = 1;

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
K_THREAD_DEFINE(run_bt_traffic,
		CONFIG_WIFI_THREAD_STACK_SIZE,
		run_bt_benchmark,
		NULL,
		NULL,
		NULL,
		CONFIG_WIFI_THREAD_PRIORITY,
		0,
		K_TICKS_FOREVER);
#endif

/**
 * @brief Print Wi-Fi status
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int cmd_wifi_status(void);
/**
 * @brief Initailise wifi arguments in variables
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int __wifi_args_to_params(struct wifi_connect_req_params *params);
/**
 * @brief Request connection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_connect(void);
/**
 * @brief Request disconnection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_disconnect(void);
/**
 * @brief parse IPv4 address
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int parse_ipv4_addr(char *host, struct sockaddr_in *addr);
/**
 * @brief wait for next wifi event
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wait_for_next_event(const char *event_name, int timeout);
/**
 * @brief CB for UDP download results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void udp_download_results_cb(enum zperf_status status,
			   struct zperf_results *result,
			   void *user_data);
/**
 * @brief CB for UDP upload results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void udp_upload_results_cb(enum zperf_status status,
			struct zperf_results *result,
			void *user_data);
/**
 * @brief CB for TCP download results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void tcp_download_results_cb(enum zperf_status status,
			   struct zperf_results *result,
			   void *user_data);
/**
 * @brief CB for TCP upload results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void tcp_upload_results_cb(enum zperf_status status,
			struct zperf_results *result,
			void *user_data);

#endif /* BT_COEX_TEST_FUNCTIONS_ */
