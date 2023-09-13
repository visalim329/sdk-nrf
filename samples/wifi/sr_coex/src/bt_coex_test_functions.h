/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_COEX_TEST_FUNCTIONS_
#define BT_COEX_TEST_FUNCTIONS_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#include <zephyr_coex.h>
#include <zephyr_coex_struct.h>
#include "bt_utils.h"
#include "zephyr_fmac_main.h"
#define MAX_SSID_LEN 32
#define WIFI_CONNECTION_TIMEOUT 30 /* in seconds */
#define WIFI_DHCP_TIMEOUT 10 /* in seconds */

#define DEMARCATE_TEST_START

#define RSSI_INIT_VALUE 127

#define HIGHEST_CHANNUM_24G 14

#define KSLEEP_WIFI_CON_2SEC K_SECONDS(2)
#define KSLEEP_WIFI_DISCON_2SEC K_SECONDS(2)
#define KSLEEP_WIFI_CON_10MSEC K_MSEC(10)
#define KSLEEP_WIFI_DISCON_10MSEC K_MSEC(10)
#define KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC K_SECONDS(1)
#define KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC K_SECONDS(1)
#define KSLEEP_ADV_START_1SEC K_SECONDS(1)
#define KSLEEP_SCAN_START_1SEC K_SECONDS(1)
#define KSLEEP_WHILE_DISCON_CENTRAL_2SEC K_SECONDS(2)


static uint32_t wifi_scan_cnt_24g;
static uint32_t wifi_scan_cnt_5g;
extern uint32_t wifi_scan_cmd_cnt;

static uint32_t wifi_conn_attempt_cnt;
static uint32_t wifi_conn_success_cnt;
static uint32_t wifi_conn_fail_cnt;
static uint32_t wifi_conn_timeout_cnt;
static uint32_t wifi_dhcp_timeout_cnt;

static uint32_t wifi_disconn_attempt_cnt;
static uint32_t wifi_disconn_success_cnt;
static uint32_t wifi_disconn_fail_cnt;
static uint32_t wifi_disconn_no_conn_cnt;

static uint32_t wifi_conn_cnt_stability;
static uint32_t wifi_disconn_cnt_stability;

static uint8_t wait4_peer_wifi_client_to_start_tp_test;

extern bool ble_periph_connected;
extern bool ble_central_connected;
uint8_t wait4_peer_ble2_start_connection;

extern uint32_t ble_supervision_timeout;

extern uint32_t ble_connection_attempt_cnt;
extern uint32_t ble_connection_success_cnt;

extern uint32_t ble_disconnection_attempt_cnt;
extern uint32_t ble_disconnection_success_cnt;
extern uint32_t ble_disconnection_fail_cnt;
extern uint32_t ble_discon_no_conn_cnt;
extern uint32_t ble_discon_no_conn;

extern uint32_t ble_disconn_cnt_stability;

static struct net_mgmt_event_callback wifi_sta_mgmt_cb;
static struct net_mgmt_event_callback net_addr_mgmt_cb;

static struct sockaddr_in in4_addr_my = {
	.sin_family = AF_INET,
	.sin_port = htons(CONFIG_NET_CONFIG_PEER_IPV4_PORT),
};

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

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

static uint32_t scan_result_count;

/**
 * @brief Disable RPU.
 *
 * @return None
 */
void rpu_disable(void);

/**
 * @brief BT throughtput test run
 *
 * @return None
 */
void run_bt_benchmark(void);

/**
 * @brief BT connection test run
 *
 * @return None
 */
void run_bt_connection_test(void);

/**
 * @brief Wi-Fi scan test run
 *
 * @return None
 */
void run_wifi_scan_test(void);

/**
 * @brief Wi-Fi connection test run
 *
 * @return None
 */
void run_wifi_conn_test(void);

/**
 * @brief commnad to start Wi-Fi scan
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int cmd_wifi_scan(void);
/**
 * @brief Call Wi-Fi connection event
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_connection(void);
/**
 * @brief configure PTA registers
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int config_pta(bool is_ant_mode_sep, bool is_ble_central,
				bool is_wlan_server);
/**
 * @brief Start wi-fi traffic for zperf udp upload or configure
 * zperf traffic for udp download
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int run_wifi_traffic_udp(void);

/**
 * @brief Start wi-fi traffic for zperf tcp upload or configure
 * zperf traffic for tcp download
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int run_wifi_traffic_tcp(void);

/**
 * @brief start BLE connection/traffic using thread start
 *
 * @return None
 */
void start_ble_activity(void);

/**
 * @brief start Wi-Fi scan using thread start
 *
 * @return None
 */
void start_wifi_activity(void);

/**
 * @brief check if iperf traffic is complete
 *
 * @return None
 */
void check_wifi_traffic(void);
/**
 * @brief Run BLE traffic using thread join
 *
 * @return None
 */
void run_ble_activity(void);
/**
 * @brief Disconnect Wi-Fi
 *
 * @return None
 */
void wifi_disconnection(void);
/**
 * @brief Exit BLE throughput test
 *
 * @return None
 */
void exit_bt_throughput_test(void);

static struct {
	uint8_t connected :1;
	uint8_t disconnect_requested: 1;
	uint8_t _unused : 6;
} context;

K_SEM_DEFINE(wait_for_next, 0, 1);
K_SEM_DEFINE(udp_tcp_callback, 0, 1);

struct wifi_iface_status status = { 0 };
uint32_t repeat_wifi_scan = 1;


#if defined(CONFIG_WIFI_SCAN_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_SCAN_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_CON_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_CON_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_CON_BLE_TP_CENTRAL_STABILITY) || \
	defined(CONFIG_WIFI_CON_BLE_TP_PERIPH_STABILITY) || \
	defined(CONFIG_BLE_TP_CENTRAL_WIFI_SHUTDOWN) || \
	defined(CONFIG_BLE_TP_PERIPH_WIFI_SHUTDOWN)

	#define ENABLE_BLE_TRAFFIC_TEST

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

#if defined(CONFIG_WIFI_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_PERIPH) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_UDP_CLIENT_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_UDP_SERVER_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_TCP_CLIENT_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_TP_TCP_SERVER_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_UDP_CLIENT_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_UDP_SERVER_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_TCP_CLIENT_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_TP_TCP_SERVER_STABILITY) || \
	defined(CONFIG_BLE_CON_CENTRAL_WIFI_SHUTDOWN) || \
	defined(CONFIG_BLE_CON_PERIPHERAL_WIFI_SHUTDOWN)

	#define ENABLE_BLE_CONN_TEST

	K_THREAD_DEFINE(run_bt_connection,
		CONFIG_WIFI_THREAD_STACK_SIZE,
		run_bt_connection_test,
		NULL,
		NULL,
		NULL,
		CONFIG_WIFI_THREAD_PRIORITY,
		0,
		K_TICKS_FOREVER);
#endif

#if defined(CONFIG_WIFI_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_SCAN_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_SCAN_BLE_TP_PERIPH) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_TP_PERIPH)	|| \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_CENTRAL_WIFI_CON_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_SCAN_STABILITY) || \
	defined(CONFIG_BLE_CONN_PERIPHERAL_WIFI_CON_SCAN_STABILITY)

	#define ENABLE_WIFI_SCAN_TEST

	K_THREAD_DEFINE(run_wlan_scan,
		CONFIG_WIFI_THREAD_STACK_SIZE,
		run_wifi_scan_test,
		NULL,
		NULL,
		NULL,
		CONFIG_WIFI_THREAD_PRIORITY,
		0,
		K_TICKS_FOREVER);
#endif

#if defined(CONFIG_WIFI_CON_BLE_TP_CENTRAL) || defined(CONFIG_WIFI_CON_BLE_TP_PERIPH)

	#define ENABLE_WIFI_CONN_TEST

	K_THREAD_DEFINE(run_wlan_conn,
		CONFIG_WIFI_THREAD_STACK_SIZE,
		run_wifi_conn_test,
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
 * @brief Initailise Wi-Fi arguments in variables
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int __wifi_args_to_params(struct wifi_connect_req_params *params);
/**
 * @brief Request Wi-Fi connection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_connect(void);
/**
 * @brief Request Wi-Fi disconnection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wifi_disconnect(void);
/**
 * @brief parse Wi-Fi IPv4 address
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int parse_ipv4_addr(char *host, struct sockaddr_in *addr);
/**
 * @brief wait for next Wi-Fi event
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int wait_for_next_event(const char *event_name, int timeout);
/**
 * @brief Callback for UDP download results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void udp_download_results_cb(enum zperf_status status,
							struct zperf_results *result,
							void *user_data);
/**
 * @brief Callback for UDP upload results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void udp_upload_results_cb(enum zperf_status status,
							struct zperf_results *result,
							void *user_data);
/**
 * @brief Callback for TCP download results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void tcp_download_results_cb(enum zperf_status status,
							struct zperf_results *result,
							void *user_data);
/**
 * @brief Callback for TCP upload results
 *
 * @return Zero on success or (negative) error code otherwise.
 */
void tcp_upload_results_cb(enum zperf_status status,
							struct zperf_results *result,
							void *user_data);
/**
 * @brief Run Wi-Fi scan test
 */
void wifi_scan_test_run(void);

/**
 * @brief Run Wi-Fi connection test
 */
void wifi_connection_test_run(void);

/**
 * @brief Start BLE advertisement
 *
 * @return None
 */
void adv_start(void);

/**
 * @brief Start BLE scan
 *
 * @return None
 */
void scan_start(void);

/**
 * @brief Callback for Wi-Fi DHCP IP address
 *
 * @return None
 */
void print_dhcp_ip(struct net_mgmt_event_callback *cb);
/**
 * @brief Callback for Wi-Fi connection result
 *
 * @return No return value.
 */
void handle_wifi_connect_result(struct net_mgmt_event_callback *cb);

/**
 * @brief Callback for Wi-Fi disconnection result
 *
 * @return No return value.
 */
void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb);
/**
 * @brief Callback for Wi-Fi scan result
 *
 * @return No return value.
 */
void handle_wifi_scan_result(struct net_mgmt_event_callback *cb);

/**
 * @brief Callback for Wi-Fi scan done
 *
 * @return No return value.
 */
void handle_wifi_scan_done(struct net_mgmt_event_callback *cb);

/**
 * @brief Callback for Wi-Fi DHCP IP addreds assigned
 *
 * @return No return value.
 */
void print_dhcp_ip(struct net_mgmt_event_callback *cb);
/**
 * @brief Print common test parameters info
 *
 * @return None
 */
void print_common_test_params(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
	bool is_ble_central);
/**
 * @brief Print BLE connection test parameters info
 *
 * @return None
 */
void print_ble_connection_test_params(bool is_ble_central);

#ifdef CONFIG_TWT_ENABLE
	/**
	 * @brief Tear down TWT
	 *
	 * @return success/fail
	 */
	static int teardown_twt(void);

	/**
	 * @brief Wait for TWT response
	 *
	 * @return success/fail
	 */
	static int wait_for_twt_resp_received(void);

	/**
	 * @brief Setup TWT
	 *
	 * @return success/fail
	 */
	static int setup_twt(void);

	/**
	 * @brief Print TWT parameters
	 *
	 * @return None
	 */
	void print_twt_params(uint8_t dialog_token, uint8_t flow_id,
				     enum wifi_twt_negotiation_type negotiation_type,
				     bool responder, bool implicit, bool announce,
				     bool trigger, uint32_t twt_wake_interval,
				     uint64_t twt_interval);
#endif

#endif /* BT_COEX_TEST_FUNCTIONS_ */
