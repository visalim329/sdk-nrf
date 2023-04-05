/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief SR coexistence sample
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bluetooth/scan.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(coex, CONFIG_LOG_DEFAULT_LEVEL);

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

#include "bt_throughput_test.h"
#include "zephyr_fmac_main.h"

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | \
				NET_EVENT_WIFI_DISCONNECT_RESULT)

#define WIFI_SCAN_BLE_CON_CENTRAL
#define WIFI_SCAN_BLE_CON_PERIPH

#define WIFI_SCAN_BLE_TP_CENTRAL
#define WIFI_SCAN_BLE_TP_PERIPH

#define WIFI_CON_BLE_CON_CENTRAL
#define WIFI_CON_BLE_CON_PERIPH

#define WIFI_CON_BLE_TP_CENTRAL
#define WIFI_CON_BLE_TP_PERIPH

#define WIFI_TP_CLIENT_BLE_CON_CENTRAL
#define WIFI_TP_CLIENT_BLE_CON_PERIPH

#define WIFI_TP_CLIENT_BLE_TP_CENTRAL
#define WIFI_TP_CLIENT_BLE_TP_PERIPH

#define WIFI_TP_SERVER_BLE_CON_CENTRAL
#define WIFI_TP_SERVER_BLE_CON_PERIPH

#define WIFI_TP_SERVER_BLE_TP_CENTRAL
#define WIFI_TP_SERVER_BLE_TP_PERIPH

#define MAX_SSID_LEN 32
#define WIFI_CONNECTION_TIMEOUT 30

#define PRINT_WIFI_SCAN_RESULT
#define DEMARCATE_TEST_START

#define HIGHEST_CHANNUM_24G 14

extern bool ble_periph_connected;

uint32_t repeat_scan = 1;
static uint32_t wifi_scan_cnt_24g;
static uint32_t wifi_scan_cnt_5g;
static uint32_t wifi_scan_cmd_cnt;

static uint8_t wait_for_wifi_client_start;
uint8_t wait_for_ble_central_run = 0;
extern uint32_t ble_conn_success_cnt;
extern uint32_t ble_conn_fail_cnt;

static uint32_t wifi_conn_success_cnt;
static uint32_t wifi_conn_fail_cnt;

static struct sockaddr_in in4_addr_my = {
	.sin_family = AF_INET,
	.sin_port = htons(CONFIG_NET_CONFIG_PEER_IPV4_PORT),
};

static struct net_mgmt_event_callback wifi_sta_mgmt_cb;
static struct net_mgmt_event_callback net_addr_mgmt_cb;
static uint32_t scan_result_count;

static struct {
	uint8_t connected :1;
	uint8_t disconnect_requested: 1;
	uint8_t _unused : 6;
} context;

K_SEM_DEFINE(wait_for_next, 0, 1);
K_SEM_DEFINE(udp_callback, 0, 1);

static void run_bt_benchmark(void);
static int cmd_wifi_scan(void);

uint64_t time_stamp;

K_THREAD_DEFINE(run_bt_traffic,
		CONFIG_WIFI_THREAD_STACK_SIZE,
		run_bt_benchmark,
		NULL,
		NULL,
		NULL,
		CONFIG_WIFI_THREAD_PRIORITY,
		0,
		K_TICKS_FOREVER);

struct wifi_iface_status status = { 0 };

static int wifi_connection(bool test_wlan, bool wifi_coex_enable,
				bool antenna_mode);
static int config_pta(bool wifi_coex_enable, bool antenna_mode, bool ble_role,
				bool wlan_role);
static void print_test_params_info(bool test_wlan, bool test_ble,
				bool antenna_mode, bool ble_role, bool wifi_coex_enable,
				bool ble_coex_enable, bool coex_hardware_enable);
static int run_wifi_traffic(bool test_wlan);
static void start_ble_traffic(bool test_ble, bool ble_role);
static void check_wifi_traffic(bool test_wlan);
static void run_ble_traffic(bool test_ble, bool ble_role);
static void disconnect_wifi(bool test_wlan);
static void disconnect_ble(bool test_ble, bool ble_role);

static int wifi_scan_ble_conn_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable);

static int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable,
			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
			bool wlan_role, bool coex_hardware_enable);

static int wifi_scan_ble_tput_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable);

static int wifi_scan_ble_tput_peripheral(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable);

static int wifi_con_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable);

static int wifi_con_ble_con_peripheral(bool test_wlan, 
				bool wifi_coex_enable, bool test_ble, bool ble_role,
				bool wlan_role, bool antenna_mode, bool coex_hardware_enable);

static int wifi_con_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable);

static int wifi_con_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable);

static int wifi_tput_client_ble_con_central(bool test_wlan, 
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable);

static int wifi_tput_client_ble_con_peripheral(bool test_wlan,
				bool wifi_coex_enable, bool test_ble, bool ble_role,
				bool wlan_role, bool antenna_mode, bool coex_hardware_enable);

static int wifi_tput_client_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble, 
			bool ble_role, bool wlan_role, bool coex_hardware_enable);

static int wifi_tput_client_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable);

static int wifi_tput_server_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable);

static int wifi_tput_server_ble_con_peripheral(bool test_wlan,
			bool wifi_coex_enable,bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable);

static int wifi_tput_server_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable);

static int wifi_tput_server_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable);


static int cmd_wifi_status(void)
{
	struct net_if *iface = net_if_get_default();

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				sizeof(struct wifi_iface_status))) {
		LOG_INF("Status request failed\n");

		return -ENOEXEC;
	}

	LOG_INF("Status: successful");
	LOG_INF("==================");
	LOG_INF("State: %s\n", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		LOG_INF("Interface Mode: %s",
		       wifi_mode_txt(status.iface_mode));
		LOG_INF("Link Mode: %s",
		       wifi_link_mode_txt(status.link_mode));
		LOG_INF("SSID: %-32s", status.ssid);
		LOG_INF("BSSID: %s",
		       net_sprint_ll_addr_buf(
				status.bssid, WIFI_MAC_ADDR_LEN,
				mac_string_buf, sizeof(mac_string_buf)));
		LOG_INF("Band: %s", wifi_band_txt(status.band));
		LOG_INF("Channel: %d", status.channel);
		LOG_INF("Security: %s", wifi_security_txt(status.security));
		LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp));
		LOG_INF("RSSI: %d", status.rssi);
	}
	return 0;
}

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (status->status) {
		LOG_ERR("Connection request failed (%d)", status->status);
	} else {
		LOG_INF("Connected");
		context.connected = true;
	}

	cmd_wifi_status();
	k_sem_give(&wait_for_next);
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (context.disconnect_requested) {
		LOG_INF("Disconnection request %s (%d)",
			 status->status ? "failed" : "done",
					status->status);
		context.disconnect_requested = false;
	} else {
		LOG_INF("Disconnected");
		context.connected = false;
	}

	cmd_wifi_status();
}

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
		const struct wifi_scan_result *entry =
				(const struct wifi_scan_result *)cb->info;
		/* uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")]; */

		scan_result_count++;

	#ifdef PRINT_WIFI_SCAN_RESULT
		LOG_INF("%-4d | %-12s | %-4u  | %-4d",
			scan_result_count, entry->ssid, entry->channel, entry->rssi);
		/* To avoid message drops while printing Wi-Fi scan results */
		k_sleep(K_MSEC(1));  

		LOG_INF("Wi-Fi scan done for %d times\n", scan_result_count);
	#endif 

	if (entry->channel <= HIGHEST_CHANNUM_24G)
	{
		wifi_scan_cnt_24g++;
	} else {
		wifi_scan_cnt_5g++;
	}

	if (repeat_scan == 1) {
		cmd_wifi_scan();
		wifi_scan_cmd_cnt++;
	}
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
			(const struct wifi_status *)cb->info;

	#ifdef PRINT_WIFI_SCAN_RESULT
		if (status->status) {
				LOG_ERR("Scan request failed (%d)\n", status->status);
		} else {
				LOG_INF("Scan request done\n");
		}
	#endif
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
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

static void print_dhcp_ip(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	LOG_INF("IP address: %s", dhcp_info);
	k_sem_give(&wait_for_next);
}

static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
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

static int __wifi_args_to_params(struct wifi_connect_req_params *params)
{
	params->timeout = SYS_FOREVER_MS;

	/* SSID */
	params->ssid = CONFIG_STA_SSID;
	params->ssid_length = strlen(params->ssid);

	#if defined(CONFIG_STA_KEY_MGMT_WPA2)
		params->security = 1;
	#elif defined(CONFIG_STA_KEY_MGMT_WPA2_256)
		params->security = 2;
	#elif defined(CONFIG_STA_KEY_MGMT_WPA3)
		params->security = 3;
	#else
		params->security = 0;
	#endif

	#if !defined(CONFIG_STA_KEY_MGMT_NONE)
		params->psk = CONFIG_STA_PASSWORD;
		params->psk_length = strlen(params->psk);
	#endif

	params->channel = WIFI_CHANNEL_ANY;

	/* MFP (optional) */
	params->mfp = WIFI_MFP_OPTIONAL;

	return 0;
}
static int cmd_wifi_scan(void)
{
	struct net_if *iface = net_if_get_default();
	if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
		LOG_ERR("Scan request failed\n");
		return -ENOEXEC;
	}
	#ifdef PRINT_WIFI_SCAN_RESULT
		LOG_INF("Scan requested\n");
	#endif
	return 0;
}

static int wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;

	__wifi_args_to_params(&cnx_params);

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
		     &cnx_params, sizeof(struct wifi_connect_req_params))) {
		LOG_ERR("Connection request failed");
		return -ENOEXEC;
	}

	/* LOG_INF("Connection requested"); */
	return 0;
}

static int wifi_disconnect(void)
{
	struct net_if *iface = net_if_get_default();
	int status;

	context.disconnect_requested = true;

	status = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);

	if (status) {
		context.disconnect_requested = false;

		if (status == -EALREADY) {
			LOG_INF("Already disconnected");
		} else {
			LOG_ERR("Disconnect request failed");
			return -ENOEXEC;
		}
	} else {
		/* LOG_INF("Disconnect requested"); */
	}

	return 0;
}

static int parse_ipv4_addr(char *host, struct sockaddr_in *addr)
{
	int ret;

	if (!host) {
		return -EINVAL;
	}

	ret = net_addr_pton(AF_INET, host, &addr->sin_addr);
	if (ret < 0) {
		LOG_ERR("Invalid IPv4 address %s\n", host);
		return -EINVAL;
	}

	LOG_INF("IPv4 address %s", host);

	return 0;
}

int wait_for_next_event(const char *event_name, int timeout)
{
	int wait_result;

	if (event_name) {
		LOG_INF("Waiting for %s", event_name);
	}

	wait_result = k_sem_take(&wait_for_next, K_SECONDS(timeout));
	if (wait_result) {
		LOG_ERR("Timeout waiting for %s -> %d", event_name, wait_result);
		return -1;
	}

	LOG_INF("Got %s", event_name);
	k_sem_reset(&wait_for_next);

	return 0;
}

static void udp_download_results_cb(enum zperf_status status,
			   struct zperf_results *result,
			   void *user_data)
{
	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New session started.\n");
		wait_for_wifi_client_start = 1;
		break;

	case ZPERF_SESSION_FINISHED: {
		uint32_t rate_in_kbps;

		/* Compute baud rate */
		if (result->time_in_us != 0U) {
			rate_in_kbps = (uint32_t)
				(((uint64_t)result->total_len * 8ULL *
				  (uint64_t)USEC_PER_SEC) /
				 ((uint64_t)result->time_in_us * 1024ULL));
		} else {
			rate_in_kbps = 0U;
		}

		LOG_INF("End of session!\n");
		
		LOG_INF("Download results:");
		LOG_INF("%u bytes in %u ms",
				(result->nb_packets_rcvd * result->packet_size),
				(result->time_in_us / USEC_PER_MSEC));
				
		LOG_INF(" received packets:\t%u\n",
			      result->nb_packets_rcvd);
		LOG_INF(" nb packets lost:\t%u\n",
			      result->nb_packets_lost);
		LOG_INF(" nb packets outorder:\t%u\n",
			      result->nb_packets_outorder);


		LOG_INF(" rate: %u Kbps", rate_in_kbps);
		LOG_INF("\n");

		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("UDP session error.\n");
		break;
	}
}


static void udp_upload_results_cb(enum zperf_status status,
			struct zperf_results *result,
			void *user_data)
{
	unsigned int client_rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New UDP session started");
		wait_for_wifi_client_start = 1;
		break;
	case ZPERF_SESSION_FINISHED:
		LOG_INF("Wi-Fi benchmark: Upload completed!");
		if (result->client_time_in_us != 0U) {
			client_rate_in_kbps = (uint32_t)
				(((uint64_t)result->nb_packets_sent *
				  (uint64_t)result->packet_size * (uint64_t)8 *
				  (uint64_t)USEC_PER_SEC) /
				 ((uint64_t)result->client_time_in_us * 1024U));
		} else {
			client_rate_in_kbps = 0U;
		}
		/* print results */
		LOG_INF("Upload results:");
		LOG_INF("%u bytes in %u ms",
				(result->nb_packets_sent * result->packet_size),
				(result->client_time_in_us / USEC_PER_MSEC));
		LOG_INF("%u packets sent", result->nb_packets_sent);
		LOG_INF("%u packets lost", result->nb_packets_lost);
		LOG_INF("%u packets received", result->nb_packets_rcvd);
		k_sem_give(&udp_callback);
		break;
	case ZPERF_SESSION_ERROR:
		LOG_ERR("UDP session error");
		break;
	}
}

static void run_bt_benchmark(void)
{
	bt_throughput_test_run();
}

enum nrf_wifi_pta_wlan_op_band 
		wifi_mgmt_to_pta_band(enum wifi_frequency_bands band)
{
	switch (band) {
	case WIFI_FREQ_BAND_2_4_GHZ:
		return NRF_WIFI_PTA_WLAN_OP_BAND_2_4_GHZ;
	case WIFI_FREQ_BAND_5_GHZ:
		return NRF_WIFI_PTA_WLAN_OP_BAND_5_GHZ;
	default:
		return NRF_WIFI_PTA_WLAN_OP_BAND_NONE;
	}
}

static int run_wifi_traffic(bool test_wlan)
{
	int ret = 0;

	if (test_wlan) {
		if(IS_ENABLED(WIFI_ZPERF_SERVER)) {
			struct zperf_download_params params;
			params.port = CONFIG_NET_CONFIG_PEER_IPV4_PORT;
			
			ret = zperf_udp_download(&params, udp_download_results_cb, NULL);
			if (ret != 0) {
				LOG_ERR("Failed to start Wi-Fi benchmark: %d\n", ret);
				goto err;
			}		
			
		}
		else {
			struct zperf_upload_params params;

			/* Start Wi-Fi traffic */
			LOG_INF("Starting Wi-Fi benchmark: Zperf client");
			params.duration_ms = CONFIG_WIFI_TEST_DURATION;
			params.rate_kbps = CONFIG_WIFI_ZPERF_RATE;
			params.packet_size = CONFIG_WIFI_ZPERF_PKT_SIZE;
			parse_ipv4_addr(CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
				&in4_addr_my);
			net_sprint_ipv4_addr(&in4_addr_my.sin_addr);

			memcpy(&params.peer_addr, &in4_addr_my, sizeof(in4_addr_my));

			ret = zperf_udp_upload_async(&params, udp_upload_results_cb, NULL);
			if (ret != 0) {
				LOG_ERR("Failed to start Wi-Fi benchmark: %d\n", ret);
				goto err;
			}
		}
	}
	return 0;
err:
	return ret;
}

static void start_ble_traffic(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (ble_role) {
			/* Start BLE traffic */
			k_thread_start(run_bt_traffic);
		}
	}
}

static void check_wifi_traffic(bool test_wlan)
{
	if (test_wlan) {
		/* Run Wi-Fi traffic */
		if (k_sem_take(&udp_callback, K_FOREVER) != 0) {
			LOG_ERR("Results are not ready");
		} else {
			LOG_INF("UDP SESSION FINISHED");
		}
	}
}

static void run_ble_traffic(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (ble_role) {
			/* Run BLE traffic */
			k_thread_join(run_bt_traffic, K_FOREVER);
		}
	}
}

static void disconnect_wifi(bool test_wlan)
{
	if (test_wlan) {
		/* Wi-Fi disconnection */
		LOG_INF("Disconnecting Wi-Fi");
		wifi_disconnect();
	}
}


static void disconnect_ble(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/* BLE disconnection only  if role is central */
		if (ble_role) {
			LOG_INF("Disconnecting BLE");
			bt_throughput_test_exit();
		}
	}
}

static int wifi_connection(bool test_wlan, bool wifi_coex_enable, 
				bool antenna_mode)
{
	int ret = 0;

	if (test_wlan) {
		#if 0
		/* Controls if Wi-Fi posts requests to PTA */
		nrf_wifi_coex_enable(wifi_coex_enable);
		#endif 
		/* Wi-Fi connection */
		wifi_connect();

		if (wait_for_next_event("Wi-Fi Connection", WIFI_CONNECTION_TIMEOUT)) {
			goto err;
		}

		if (wait_for_next_event("Wi-Fi DHCP", 10)) {
			goto err;
		}
	}
	wifi_conn_success_cnt++;
	return 0;
err:
	wifi_conn_fail_cnt++;
	return ret;
}


//static int ble_connection(bool test_ble, bool ble_role)
//{
//	int ret = 0;
//
//	if (test_ble) {
//		/* BLE connection */
//		ret = bt_throughput_test_init(ble_role);
//		if (ret != 0) {
//			LOG_ERR("Failed to configure BLE connection test: %d\n", ret);
//			goto err;
//		}
//	}
//	return 0;
//err:
//	return ret;
//}

//static int ble_connection_central(bool test_ble, bool ble_role)
//{
//	int ret = 0;
//
//	if (test_ble) {
//		/* BLE connection */
//		ret = bt_scan_test(ble_role);
//		if (ret != 0) {
//			LOG_ERR("Failed to configure BLE connection test: %d\n", ret);
//			goto err;
//		}
//	}
//	return 0;
//err:
//	return ret;
//}

static int config_pta(bool wifi_coex_enable, bool antenna_mode, bool ble_role, 
			bool wlan_role)
{
	int ret = 0;
	enum nrf_wifi_pta_wlan_op_band wlan_band =
			wifi_mgmt_to_pta_band(status.band);

	if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_NONE) {
		LOG_ERR("Invalid Wi-Fi band: %d\n", wlan_band);
		goto err;
	}
	

	LOG_INF("Configuring PTA for %s", wifi_band_txt(status.band));
	ret = nrf_wifi_coex_config_pta(wlan_band, antenna_mode, ble_role,
			wlan_role);
	if (ret != 0) {
		LOG_ERR("Failed to configure PTA coex hardware: %d\n", ret);
		goto err;
	}
	return 0;
err:
	return ret;
}

static void print_test_params_info(bool test_wlan, bool test_ble,
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
		wait_for_wifi_client_start=0;
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
		LOG_INF("WLAN doesnt post requests to PTA");
	}

	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesnt post requests to PTA");
	}
	if (IS_ENABLED(CONFIG_RPU_ENABLE)) {
		LOG_INF("RPU enabled");
	} else {
		LOG_INF("RPU disabled");
	}
}

static int wifi_scan_ble_conn_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable)
{
	//int ble_connect_true = 0;
	//int ret = 0;
	//uint64_t ble_scanNconn_start_time = 0;
	//uint64_t ble_scanNconn_time = 0;
	uint64_t test_start_time = 0;
	//uint64_t test_run_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif

		cmd_wifi_scan();
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			scan_start();
			//bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
			k_sleep(K_SECONDS(1));

			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
			
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		#endif
		#ifdef CONNECTED_SCAN
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		#else
			cmd_wifi_scan();
		#endif
	}
	test_start_time = k_uptime_get_32();
	if(test_ble) {
		ble_connection(test_ble, ble_role);
		
		start_ble_traffic(test_ble, ble_role);

		run_ble_traffic(test_ble, ble_role);

		disconnect_ble(test_ble, ble_role);
	}
	
	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) > CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
			/* LOG_INF("waiting\n"); */
		}
		repeat_scan = 0;
	}

	if(test_ble) {
		LOG_INF("ble_conn_success_cnt = %u \n", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u \n", ble_conn_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u \n", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u \n", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u \n", wifi_scan_cmd_cnt);
	}
	LOG_INF("\n\n wifi_scan_ble_conn_central complete\n");
	return 0;
}

static int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable,
			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
			bool wlan_role, bool coex_hardware_enable)
{
	//int ble_connect_true = 0;
	//int ret = 0;
	//uint64_t ble_scanNconn_start_time = 0;
	//uint64_t ble_scanNconn_time = 0;
	uint64_t test_start_time = 0;
	//uint64_t test_run_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */
		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}
	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	if (test_wlan) {
		cmd_wifi_scan();
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		while (true) {
			if (!ble_periph_connected) {
				adv_start();
			}
			if ((k_uptime_get_32() - test_start_time) > 
				CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) > CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		repeat_scan = 0;
	}

	if(test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u \n", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u \n", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u \n", wifi_scan_cmd_cnt);
	}
	LOG_INF("\n\n wifi_scan_ble_conn_peripheral complete\n");

	return 0;
}

static int wifi_scan_ble_tput_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
		#ifdef CONNECTED_SCAN
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		#else
			cmd_wifi_scan();
		#endif
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();
	if(test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
		start_ble_traffic(test_ble, ble_role);
		run_ble_traffic(test_ble, ble_role);
		disconnect_ble(test_ble, ble_role);
	}
	if (test_wlan) {
		while (true) {
			if (k_uptime_get_32() - test_start_time > CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		repeat_scan = 0;
	}
	if(test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u \n", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u \n", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u \n", wifi_scan_cmd_cnt);
	}
	LOG_INF("\n\n wifi_scan_ble_tput_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_scan_ble_tput_peripheral(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;
	int ret = 0;

		cmd_wifi_scan();
	}
	
	test_start_time = k_uptime_get_32();
	
	if (test_ble) {
		if (!ble_role) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3)); 
		}
	}

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}

	#ifdef CONNECTED_SCAN
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		cmd_wifi_scan();
	#else
		if(test_wlan && test_ble) {
			while (true) {
				if (ble_periph_connected) {
					cmd_wifi_scan();
					break;
				}
			}
		} else if (test_wlan) {
			cmd_wifi_scan();
		}
	#endif
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();
	if (test_ble) {	
		//LOG_INF("BLE Throughput started\n");
		start_ble_traffic(test_ble, ble_role);

		run_ble_traffic(test_ble, ble_role);
	}
	while (true) {
		if (k_uptime_get_32() - test_start_time > CONFIG_WIFI_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}
	//temp fix to avoid ble disconn when peripheral
	//k_sleep(K_MSEC(CONFIG_BLE_TEST_DURATION));

	if (test_ble) {
		disconnect_ble(test_ble, ble_role);
	}

	//k_sleep(K_SECONDS(5));
	repeat_scan = 0;
	
	if(test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u \n", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u \n", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u \n", wifi_scan_cmd_cnt);
	}
	LOG_INF("\n\n wifi_scan_ble_tput_peripheral complete\n");
	return 0;
err:
	return ret;
}

static int wifi_con_ble_con_central(bool test_wlan, 
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}

	if (test_ble) {
		bt_connection_init(ble_role);
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			scan_start();
			k_sleep(K_SECONDS(1));
		}
		if (test_wlan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));
		}
		if (test_ble) {
			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
		}
		if (test_wlan) {
			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));
		}
		// note: CONFIG_BLE_TEST_DURATION = CONFIG_WIFI_TEST_DURATION
		if ((k_uptime_get_32() - test_start_time)
			> CONFIG_BLE_TEST_DURATION) { 
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if(test_ble) {
		LOG_INF("ble_conn_success_cnt = %u \n", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u \n", ble_conn_fail_cnt);
	}
	if(test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u \n", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u \n", wifi_conn_fail_cnt);
	}

	LOG_INF("\n\n wifi_con_ble_con_central complete\n");
	return 0;
}

static int wifi_con_ble_con_peripheral(bool test_wlan, 
				bool wifi_coex_enable, bool test_ble, bool ble_role,
				bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			if (!ble_periph_connected) {
				adv_start();
				//k_sleep(K_SECONDS(1));
			}
		}
		if (test_wlan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));
		}
		// Note: BLE disconnection is initiated by peer central.
		if (test_wlan) {
			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));
		}
		// note: CONFIG_BLE_TEST_DURATION = CONFIG_WIFI_TEST_DURATION
		if ((k_uptime_get_32() - test_start_time) > 
			CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if(test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u \n", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u \n", wifi_conn_fail_cnt);
	}

	LOG_INF("\n\n wifi_con_ble_con_peripheral complete\n");
	return 0;
}

static int wifi_con_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble, 
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		#if 0
			if (!coex_hardware_enable) {
				nrf_wifi_coex_hw_enable(coex_hardware_enable);
			}
		#endif
	}

	if (test_ble) {
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		//run_ble_traffic(test_ble, ble_role);
	}

	if (test_wlan) {
		while (true) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));

			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));

			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}

	if (test_ble) {
		run_ble_traffic(test_ble, ble_role);
		disconnect_ble(test_ble, ble_role);
	}

	if(test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u \n", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u \n", wifi_conn_fail_cnt);
	}

	LOG_INF("\n\n wifi_con_ble_tput_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_con_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if(test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		#if 0
			if (!coex_hardware_enable) {
				nrf_wifi_coex_hw_enable(coex_hardware_enable);
			}
		#endif
	}

	if (test_ble) {
		if (!ble_role) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3)); 
		}
	}

	if (test_ble) {
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		//run_ble_traffic(test_ble, ble_role);
	}

	if(test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
			LOG_INF("Run BLE central \n");
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_wlan) {
		while (true) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));

			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));

			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_ble) {
		run_ble_traffic(test_ble, ble_role);
		disconnect_ble(test_ble, ble_role);
	}

	if(test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u \n", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u \n", wifi_conn_fail_cnt);
	}

	LOG_INF("\n\n wifi_con_ble_tput_peripheral complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_client_ble_con_central(bool test_wlan, 
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if(test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif

		ret = run_wifi_traffic(test_wlan);
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			scan_start();
			k_sleep(K_SECONDS(1));

			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
			
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}

	if (test_wlan) {
		/* LOG_INF("check wifi traffic"); */
		check_wifi_traffic(test_wlan);
		disconnect_wifi(test_wlan);
	}

	if(test_ble) {
		LOG_INF("ble_conn_success_cnt = %u \n", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u \n", ble_conn_fail_cnt);
	}

	LOG_INF("\n\n wifi_tput_client_ble_con_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_client_ble_con_peripheral(bool test_wlan, 
				bool wifi_coex_enable, bool test_ble, bool ble_role,
				bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;


	if(test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	if (test_wlan) {
		ret = run_wifi_traffic(test_wlan);
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if (!ble_periph_connected) {
				adv_start();
			}
			if ((k_uptime_get_32() - test_start_time) > 
				CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_wlan) {
		/* LOG_INF("check wifi traffic"); */
		check_wifi_traffic(test_wlan);

		disconnect_wifi(test_wlan);
	}

	LOG_INF("\n\n wifi_tput_client_ble_con_peripheral complete\n");
	return 0;
err:
	return ret;
}


static int wifi_tput_client_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble, 
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;

	wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

#ifdef CONFIG_NRF700X_BT_COEX
	config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
#endif /* CONFIG_NRF700X_BT_COEX */

	/* Disable coexistence hardware module for coex disable test cases */
	#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
	#endif
	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	ret = run_wifi_traffic(test_wlan);
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}
	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF("\n\n wifi_tput_client_ble_tput_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_client_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;

	wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

#ifdef CONFIG_NRF700X_BT_COEX
	config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
#endif /* CONFIG_NRF700X_BT_COEX */

	
	if (!ble_role) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3)); 
	}

	/* Disable coexistence hardware module for coex disable test cases */
	#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
	#endif

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}

	start_ble_traffic(test_ble, ble_role);

	run_ble_traffic(test_ble, ble_role);

	if(test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
			LOG_INF("Run BLE central \n");
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	ret = run_wifi_traffic(test_wlan);
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	check_wifi_traffic(test_wlan);	

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);
	LOG_INF("\n\n wifi_tput_client_ble_tput_peripheral complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_server_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if(test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif

		ret = run_wifi_traffic(test_wlan);
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			scan_start();
			k_sleep(K_SECONDS(1));

			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
			
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}

	if (test_wlan) {
		/* LOG_INF("check wifi traffic"); */
		check_wifi_traffic(test_wlan);

		disconnect_wifi(test_wlan);
	}
	if(test_ble) {
		LOG_INF("ble_conn_success_cnt = %u \n", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u \n", ble_conn_fail_cnt);
	}
	LOG_INF("\n\n wifi_tput_server_ble_con_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_server_ble_con_peripheral(bool test_wlan,
			bool wifi_coex_enable,bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;


	if(test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif /* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		#if 0
		if (!coex_hardware_enable) {
			nrf_wifi_coex_hw_enable(coex_hardware_enable);
		}
		#endif
	}

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	if (test_wlan) {
		ret = run_wifi_traffic(test_wlan);
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if (!ble_periph_connected) {
				adv_start();
			}
			if ((k_uptime_get_32() - test_start_time) > 
				CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_wlan) {
		/* LOG_INF("check wifi traffic"); */
		check_wifi_traffic(test_wlan);

		disconnect_wifi(test_wlan);
	}
	LOG_INF("\n\n wifi_tput_server_ble_con_peripheral complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_server_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;

	wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

#ifdef CONFIG_NRF700X_BT_COEX
	config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
#endif /* CONFIG_NRF700X_BT_COEX */

	/* Disable coexistence hardware module for coex disable test cases */
	#if 0
	if (!coex_hardware_enable) {
		nrf_wifi_coex_hw_enable(coex_hardware_enable);
	}
	#endif

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}

	ret = run_wifi_traffic(test_wlan);
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	if(test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client\n");
			k_sleep(K_SECONDS(1));
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	wait_for_wifi_client_start=0;

	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF("\n\n wifi_tput_server_ble_tput_central complete\n");
	return 0;
err:
	return ret;
}

static int wifi_tput_server_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;

	wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

#ifdef CONFIG_NRF700X_BT_COEX
	config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
#endif /* CONFIG_NRF700X_BT_COEX */

	if (!ble_role) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	/* Disable coexistence hardware module for coex disable test cases */
	#if 0
	if (!coex_hardware_enable) {
		nrf_wifi_coex_hw_enable(coex_hardware_enable);
	}
	#endif

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d\n", ret);
			goto err;
		}
	}

	ret = run_wifi_traffic(test_wlan);
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}
	if(test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client\n");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start=0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("\n\n");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("\n\n");
	#endif 
	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF("\n\n wifi_tput_server_ble_tput_peripheral complete\n");
	return 0;
err:
	return ret;
}



int main(void)
{
	int ret = 0;
	bool wifi_coex_enable = IS_ENABLED(WIFI_COEX_ENABLE);
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);
	bool coex_hardware_enable = IS_ENABLED(CONFIG_COEX_HARDWARE_ENABLE);
	bool antenna_mode = IS_ENABLED(CONFIG_COEX_SEP_ANTENNAS);
	bool ble_role = IS_ENABLED(CONFIG_COEX_BT_CENTRAL);
	bool wlan_role = IS_ENABLED(WIFI_ZPERF_SERVER);

	bool test_wlan = IS_ENABLED(CONFIG_TEST_TYPE_WLAN);
	bool test_ble = IS_ENABLED(CONFIG_TEST_TYPE_BLE);

#if !defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
	&& !defined(CONFIG_COEX_SEP_ANTENNAS)
	BUILD_ASSERT("Shared antenna support is not available with nRF7002 shields");
#endif

	memset(&context, 0, sizeof(context));

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
		LOG_ERR("Unable to configure SR side switch: %d\n", ret);
		goto err;
	}

	/* Configure Coexistence Hardware non-PTA registers */
	LOG_INF("Configuring non-PTA registers.");
	ret = nrf_wifi_coex_config_non_pta(antenna_mode);
	if (ret != 0) {
		LOG_ERR("Configuring non-PTA registers of CoexHardware FAIL\n");
		goto err;
	}
#endif /* CONFIG_NRF700X_BT_COEX */

	#if 0
	if (!(IS_ENABLED(CONFIG_RPU_ENABLE))) {
		LOG_INF("RPU disabled");
		rpu_disable();
	}
	#endif

	#ifdef WIFI_SCAN_BLE_CON_CENTRAL
		LOG_INF("Test case: wifi_scan_ble_conn_central");
		ret = wifi_scan_ble_conn_central(wifi_coex_enable, antenna_mode,
				test_ble, test_wlan, ble_role, wlan_role, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_scan_ble_conn_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_SCAN_BLE_CON_PERIPH
		LOG_INF("Test case: wifi_scan_ble_conn_peripheral");
		ret = wifi_scan_ble_conn_peripheral(wifi_coex_enable, antenna_mode,
				test_ble, test_wlan, ble_role, wlan_role, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_scan_ble_conn_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_SCAN_BLE_TP_CENTRAL
		LOG_INF("Test case: wifi_scan_ble_tput_central");
		ret = wifi_scan_ble_tput_central(wifi_coex_enable, antenna_mode,
			test_ble, test_wlan, ble_role, wlan_role,coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_scan_ble_tput_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_SCAN_BLE_TP_PERIPH
		LOG_INF("Test case: wifi_scan_ble_tput_peripheral");
		ret = wifi_scan_ble_tput_peripheral(wifi_coex_enable, antenna_mode,
			test_ble, test_wlan, ble_role, wlan_role,coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_scan_ble_tput_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_CON_BLE_CON_CENTRAL
		LOG_INF("Test case: wifi_con_ble_con_central");
		ret = wifi_con_ble_con_central(test_wlan,
			wifi_coex_enable, test_ble, ble_role, wlan_role,
			antenna_mode, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_con_ble_con_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_CON_BLE_CON_PERIPH
		LOG_INF("Test case: wifi_con_ble_con_peripheral");
		ret = wifi_con_ble_con_peripheral(test_wlan, 
				wifi_coex_enable, test_ble, ble_role,
				wlan_role, antenna_mode, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_con_ble_con_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_CON_BLE_TP_CENTRAL
		LOG_INF("Test case: wifi_con_ble_tput_central");
		ret = wifi_con_ble_tput_central(test_wlan,
			wifi_coex_enable, antenna_mode, test_ble,
			ble_role, wlan_role, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_con_ble_tput_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_CON_BLE_TP_PERIPH
		LOG_INF("Test case: wifi_con_ble_tput_peripheral");
		ret = wifi_con_ble_tput_peripheral(test_wlan,
			wifi_coex_enable, antenna_mode, test_ble,
			ble_role, wlan_role, coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_con_ble_tput_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_TP_CLIENT_BLE_CON_CENTRAL
		LOG_INF("Test case: wifi_tput_client_ble_con_central");
		ret = wifi_tput_client_ble_con_central(test_wlan, wifi_coex_enable,
				test_ble, ble_role, wlan_role, antenna_mode,
				coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_tput_client_ble_con_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_TP_CLIENT_BLE_CON_PERIPH
		LOG_INF("Test case: wifi_tput_client_ble_con_peripheral");
		ret = wifi_tput_client_ble_con_peripheral(test_wlan, wifi_coex_enable,
				test_ble, ble_role, wlan_role, antenna_mode,
				coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_tput_client_ble_con_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_TP_SERVER_BLE_CON_CENTRAL
		LOG_INF("Test case: wifi_tput_server_ble_con_central");
		ret = wifi_tput_server_ble_con_central(test_wlan, wifi_coex_enable,
				test_ble, ble_role, wlan_role, antenna_mode,
				coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_tput_server_ble_con_central fail");
			goto err;
		}
	#endif

	#ifdef WIFI_TP_SERVER_BLE_CON_PERIPH
		LOG_INF("Test case: wifi_tput_server_ble_con_peripheral");
		ret = wifi_tput_server_ble_con_peripheral(test_wlan, wifi_coex_enable,
				test_ble, ble_role, wlan_role, antenna_mode,
				coex_hardware_enable);
		if (ret != 0) {
			LOG_ERR("Running wifi_tput_server_ble_con_peripheral fail");
			goto err;
		}
	#endif

	#ifdef WIFI_TP_CLIENT_BLE_TP_CENTRAL
		if (!IS_ENABLED(WIFI_ZPERF_SERVER)
			&& IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
			LOG_INF("\n Test case: wifi_tput_client_ble_tput_central \n");
			ret = wifi_tput_client_ble_tput_central(test_wlan,
			wifi_coex_enable, antenna_mode, test_ble, ble_role,
			wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_client_ble_tput_central fail");
				goto err;
			}
		}
	#endif

	#ifdef WIFI_TP_CLIENT_BLE_TP_PERIPH
		if (!IS_ENABLED(WIFI_ZPERF_SERVER)
			&& !IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
			LOG_INF("\n Test case: wifi_tput_client_ble_tput_peripheral \n");
			ret = wifi_tput_client_ble_tput_peripheral(test_wlan,
					wifi_coex_enable, antenna_mode, test_ble, ble_role,
					wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_client_ble_tput_peripheral fail");
				goto err;
			}
		}
	#endif

	#ifdef WIFI_TP_SERVER_BLE_TP_CENTRAL
		if (IS_ENABLED(WIFI_ZPERF_SERVER) 
			&& IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
			LOG_INF("\n Test case: wifi_tput_server_ble_tput_central \n");
			ret = wifi_tput_server_ble_tput_central(test_wlan, wifi_coex_enable,
					antenna_mode, test_ble, ble_role, wlan_role,
					coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_server_ble_tput_central fail");
				goto err;
			}
		}
	#endif

	#ifdef WIFI_TP_SERVER_BLE_TP_PERIPH
		if (IS_ENABLED(WIFI_ZPERF_SERVER)
			&& !IS_ENABLED(CONFIG_COEX_BT_CENTRAL)) {
			LOG_INF("\n Test case: wifi_tput_server_ble_tput_peripheral \n");
			ret = wifi_tput_server_ble_tput_peripheral(test_wlan,
					wifi_coex_enable, antenna_mode, test_ble, ble_role,
					wlan_role, coex_hardware_enable);
			if (ret != 0) {
				LOG_ERR("Running wifi_tput_server_ble_tput_peripheral fail");
				goto err;
			}
		}
	#endif

	return 0;

err:
	LOG_ERR("Returning with error");
	return ret;


}
