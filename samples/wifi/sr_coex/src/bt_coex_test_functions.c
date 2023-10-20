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

#include "bt_coex_test_functions.h"
#include <string.h>

#include <zephyr/kernel.h>
#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
#include <nrfx_clock.h>
#endif
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

void memset_context(void)
{
	memset(&context, 0, sizeof(context));
}

int cmd_wifi_status(void)
{
	struct net_if *iface = net_if_get_default();

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				sizeof(struct wifi_iface_status))) {
		LOG_INF("Status request failed");

		return -ENOEXEC;
	}

	LOG_INF("Status: successful\n");
	LOG_INF("==================\n");
	LOG_INF("State: %s\n", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		LOG_INF("Interface Mode: %s\n",
		       wifi_mode_txt(status.iface_mode));
		LOG_INF("Link Mode: %s\n",
		       wifi_link_mode_txt(status.link_mode));
		LOG_INF("SSID: %-32s\n", status.ssid);
		LOG_INF("BSSID: %s\n",
		       net_sprint_ll_addr_buf(
				status.bssid, WIFI_MAC_ADDR_LEN,
				mac_string_buf, sizeof(mac_string_buf)));
		LOG_INF("Band: %s\n", wifi_band_txt(status.band));
		LOG_INF("Channel: %d\n", status.channel);
		LOG_INF("Security: %s\n", wifi_security_txt(status.security));
		LOG_INF("MFP: %s\n", wifi_mfp_txt(status.mfp));
		LOG_INF("RSSI: %d\n", status.rssi);
	}

	return 0;
}

void print_dhcp_ip(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	LOG_INF("IP address: %s", dhcp_info);
	k_sem_give(&wait_for_next);
}

void wifi_net_mgmt_callback_functions(void)
{
	net_mgmt_init_event_callback(&wifi_sta_mgmt_cb, wifi_mgmt_event_handler,
		WIFI_MGMT_EVENTS);

	net_mgmt_add_event_callback(&wifi_sta_mgmt_cb);

	net_mgmt_init_event_callback(&net_addr_mgmt_cb, net_mgmt_event_handler,
		NET_EVENT_IPV4_DHCP_BOUND);

	net_mgmt_add_event_callback(&net_addr_mgmt_cb);

#ifdef CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT
	nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
#endif

	LOG_INF("Starting %s with CPU frequency: %d MHz", CONFIG_BOARD, SystemCoreClock/MHZ(1));

	k_sleep(K_SECONDS(1));
}

void net_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
		struct net_if *iface)
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
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	default:
		break;
	}
}

void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (status->status) {
		LOG_ERR("Wi-Fi Connection request failed (%d)", status->status);
	} else {
		LOG_INF("Connected");
		context.connected = true;
	}

	cmd_wifi_status();
	k_sem_give(&wait_for_next);
}

void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
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

int __wifi_args_to_params(struct wifi_connect_req_params *params)
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

int wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;

	__wifi_args_to_params(&cnx_params);

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
			&cnx_params, sizeof(struct wifi_connect_req_params))) {
		LOG_ERR("Wi-Fi Connection request failed");
		return -ENOEXEC;
	}
	return 0;
}

int wifi_disconnect(void)
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
		LOG_INF("Disconnect requested");
	}

	return 0;
}

int parse_ipv4_addr(char *host, struct sockaddr_in *addr)
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
	LOG_INF("Wi-Fi peer IPv4 address %s", host);

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
void udp_download_results_cb(enum zperf_status status, struct zperf_results *result,
		void *user_data)
{
	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New session started.");
		wait4_peer_wifi_client_to_start_tp_test = 1;
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

		LOG_INF("End of session!");

		LOG_INF("Download results:");
		LOG_INF("%u bytes in %u ms",
				(result->nb_packets_rcvd * result->packet_size),
				(result->time_in_us / USEC_PER_MSEC));
		/**
		 *LOG_INF(" received packets:\t%u",
		 *		  result->nb_packets_rcvd);
		 *LOG_INF(" nb packets lost:\t%u",
		 *		  result->nb_packets_lost);
		 *LOG_INF(" nb packets outorder:\t%u",
		 *		  result->nb_packets_outorder);
		 */
		LOG_INF("\nThroughput:%u kbps", rate_in_kbps);
		LOG_INF("");
		k_sem_give(&udp_callback);
		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("UDP session error.");
		break;
	}
}
void udp_upload_results_cb(enum zperf_status status,
			  struct zperf_results *result,
			  void *user_data)
{
	unsigned int client_rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New UDP session started");
		wait4_peer_wifi_client_to_start_tp_test = 1;
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


enum nrf_wifi_pta_wlan_op_band wifi_mgmt_to_pta_band(enum wifi_frequency_bands band)
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

int run_wifi_traffic_udp(void)
{
	int ret = 0;

#ifdef CONFIG_WIFI_ZPERF_SERVER
	struct zperf_download_params params = {0};

	params.port = CONFIG_NET_CONFIG_PEER_IPV4_PORT;

	ret = zperf_udp_download(&params, udp_download_results_cb, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to start UDP server session: %d", ret);
		return ret;
	}
#else
	struct zperf_upload_params params = {0};;

	/* Start Wi-Fi UDP traffic */
	LOG_INF("Starting Wi-Fi benchmark: Zperf UDP client");
	params.duration_ms = CONFIG_COEX_TEST_DURATION;
	params.rate_kbps = CONFIG_WIFI_ZPERF_RATE;
	params.packet_size = CONFIG_WIFI_ZPERF_PKT_SIZE;
	parse_ipv4_addr(CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &in4_addr_my);
	net_sprint_ipv4_addr(&in4_addr_my.sin_addr);

	memcpy(&params.peer_addr, &in4_addr_my, sizeof(in4_addr_my));
	ret = zperf_udp_upload_async(&params, udp_upload_results_cb, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi UDP benchmark: %d", ret);
		return ret;
	}
#endif

	return 0;
}

void check_wifi_traffic(void)
{
	/* Run Wi-Fi traffic */
	if (k_sem_take(&udp_callback, K_FOREVER) != 0) {
		LOG_ERR("Results are not ready");
	} else {
		LOG_INF("UDP SESSION FINISHED");
	}
}

int wifi_connection(void)
{
	/* Wi-Fi connection */
	wifi_connect();

	if (wait_for_next_event("Wi-Fi Connection", WIFI_CONNECTION_TIMEOUT)) {
		return -1;
	}

	if (wait_for_next_event("Wi-Fi DHCP", WIFI_DHCP_TIMEOUT)) {
		return -1;
	}

	return 0;
}

void wifi_disconnection(void)
{
	int ret = 0;
	/* Wi-Fi disconnection */
	LOG_INF("Disconnecting Wi-Fi");
	
	ret = wifi_disconnect();
	if (ret != 0) {
		LOG_INF("Disconnect failed");
	}
}

int config_pta(bool is_ant_mode_sep, bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	enum nrf_wifi_pta_wlan_op_band wlan_band = wifi_mgmt_to_pta_band(status.band);

	if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_NONE) {
		LOG_ERR("Invalid Wi-Fi band: %d", wlan_band);
		return -1;
	}

	/* Configure PTA registers of Coexistence Hardware */
	LOG_INF("Configuring PTA for %s", wifi_band_txt(status.band));
	ret = nrf_wifi_coex_config_pta(wlan_band, is_ant_mode_sep, is_ble_central,
			is_wlan_server);
	if (ret != 0) {
		LOG_ERR("Failed to configure PTA coex hardware: %d", ret);
		return -1;
	}
	return 0;
}

void run_bt_benchmark(void)
{
	bt_throughput_test_run();
}

void start_ble_activity(void)
{
	/* Start BLE traffic */
	k_thread_start(run_bt_traffic);

}

void run_ble_activity(void)
{
	k_thread_join(run_bt_traffic, K_FOREVER);
}

void print_common_test_params(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central)
{
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);
	bool is_wifi_band_2pt4g = IS_ENABLED(CONFIG_WIFI_BAND_2PT4G);

	LOG_INF("-------------------------------- Test parameters");

	if (test_wlan && test_ble) {
		LOG_INF("Running Wi-Fi and BLE tests");
	} else {
		if (test_wlan) {
			LOG_INF("Running Wi-Fi only test");
		} else {
			LOG_INF("Running BLE only test");
		}
	}
	LOG_INF("Test duration in milliseconds: %d", CONFIG_COEX_TEST_DURATION);
	if (is_wifi_band_2pt4g) {
		LOG_INF("Wi-Fi operates in 2.4G band");
	} else {
		LOG_INF("Wi-Fi operates in 5G band");
	}
	if (is_ant_mode_sep) {
		LOG_INF("Antenna mode : Separate antennas");
	} else {
		LOG_INF("Antenna mode : Shared antennas");
	}
	if (is_ble_central) {
		LOG_INF("BLE role : Central");
	} else {
		LOG_INF("BLE role : Peripheral");
	}
	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesn't post requests to PTA");
	}
	LOG_INF("--------------------------------");
}






void exit_bt_throughput_test(void)
{
	/** This is called if role is central. Disconnection in the
	 *case of peripheral is taken care by the peer central
	 */
	bt_throughput_test_exit();
}

int wifi_tput_ble_tput(bool test_wlan, bool is_ant_mode_sep,
	bool test_ble, bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;
	int64_t test_start_time = 0;
	LOG_INF(" Test case: wifi_tput_ble_tput");
	
	if (is_ble_central) {
		if (is_wlan_server) {				
			LOG_INF(" BLE central, Wi-Fi UDP server");
		} else {
			LOG_INF(" BLE central, Wi-Fi UDP client");
		}
	} else {
		if (is_wlan_server) {
			LOG_INF(" BLE peripheral, Wi-Fi UDP server");
		} else {
			LOG_INF(" BLE peripheral, Wi-Fi UDP client");
		}
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		#ifndef CHECK_WIFI_CONN_STATUS
			wifi_connection(); 
		#else
			ret = wifi_connection();
			k_sleep(K_SECONDS(3));
			if (ret != 0) {
				LOG_ERR("Wi-Fi connection failed. Running the test");
				LOG_ERR("further is not meaningful. So, exiting the test");
				return ret;
			}
		#endif
		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		if (!is_ble_central) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3));
		}
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			return ret;
		}
		
		if (!is_wlan_server) {
			if (is_ble_central) {
				/* nothing */
			} else {
				if (test_wlan && test_ble) {
					while (!wait4_peer_ble2_start_connection) {
						/* Peer BLE starts the the test. */
						LOG_INF("Run BLE central on peer");
						k_sleep(K_SECONDS(1));
					}
				wait4_peer_ble2_start_connection = 0;
				}
			}
		}
	}
	if (!is_wlan_server) {
#ifdef DEMARCATE_TEST_START
		LOG_INF("-------------------------start");
#endif
	}
	if (!is_wlan_server) {
		test_start_time = k_uptime_get_32();
	}
	if (test_wlan) {
		if (is_zperf_udp) {
			ret = run_wifi_traffic_udp();		
			if (ret != 0) {
				LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
				return ret;
			}
		}
			
		if (is_wlan_server) {
			while (!wait4_peer_wifi_client_to_start_tp_test) {
				LOG_INF("start WiFi client on peer");
				k_sleep(K_SECONDS(1));
			}
			wait4_peer_wifi_client_to_start_tp_test = 0;
			test_start_time = k_uptime_get_32();
		}
	}
	
	if (is_wlan_server) {
#ifdef DEMARCATE_TEST_START
		LOG_INF("-------------------------start");
#endif
	}

	if (test_ble) {
		if (is_ble_central) {
			start_ble_activity();
		} else {
			/* If DUT BLE is peripheral then the peer starts the activity. */
			while (true) {
				if (k_uptime_get_32() - test_start_time >
					CONFIG_COEX_TEST_DURATION) {
					break;
				}
				k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
			}
		}
	}

	if (test_wlan) {
		check_wifi_traffic();
	}

	if (test_ble) {
		if (is_ble_central) {
			/* run BLE activity and wait for the test duration */
			run_ble_activity();
		} else {
			/* Peer BLE that acts as central runs the traffic. */
		}
	}

	if (test_wlan) {
		wifi_disconnection();
	}

	if (test_ble) {
		if (is_ble_central) {
			exit_bt_throughput_test();
		}
	}

#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------end");
#endif

	return 0;
}
