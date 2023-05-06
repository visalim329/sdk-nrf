/**
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Wi-Fi Bluetooth LE coexistence functions
 */

#include "bt_coex_test_functions.h"

int8_t wifi_rssi = 127;

static int print_first_time;

void memset_context(void)
{
	memset(&context, 0, sizeof(context));
}

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)

int cmd_wifi_status(void)
{
	struct net_if *iface = net_if_get_default();

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				sizeof(struct wifi_iface_status))) {
		LOG_INF("Status request failed");

		return -ENOEXEC;
	}

	//LOG_INF("Status: successful");
	LOG_INF("==================");
	LOG_INF("State: %s", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		LOG_INF("Interface Mode: %s",
			   wifi_mode_txt(status.iface_mode));
		LOG_INF("Link Mode: %s",
			   wifi_link_mode_txt(status.link_mode));
		LOG_INF("SSID: %-32s", status.ssid);
		//LOG_INF("BSSID: %s",
		//	   net_sprint_ll_addr_buf(
		//		status.bssid, WIFI_MAC_ADDR_LEN,
		//		mac_string_buf, sizeof(mac_string_buf)));
		LOG_INF("Band: %s", wifi_band_txt(status.band));
		LOG_INF("Channel: %d", status.channel);
		LOG_INF("Security: %s", wifi_security_txt(status.security));
		//LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp));
		LOG_INF("WiFi RSSI: %d", status.rssi);
		wifi_rssi = status.rssi;			
	}
	
	return 0;
}

void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (status->status) {
		LOG_ERR("Connection request failed (%d)", status->status);
	} else {
		#ifdef PRINT_WIFI_CONN_RESULT
		LOG_INF("Connected");
		#endif
		context.connected = true;
	}
	if (print_first_time==0) {	
	//#ifdef PRINT_WIFI_CONN_RESULT
		cmd_wifi_status();
		print_first_time++;
	//#endif
	}
	wifi_conn_cnt_regr++;
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
	wifi_disconn_cnt_regr++;
#ifdef PRINT_WIFI_CONN_RESULT
	cmd_wifi_status();
#endif
}

void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
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

		LOG_INF("Wi-Fi scan done for %d times", scan_result_count);
	#endif

	if (entry->channel <= HIGHEST_CHANNUM_24G) {
		wifi_scan_cnt_24g++;
	} else {
		wifi_scan_cnt_5g++;
	}

	if (repeat_scan == 1) {
		cmd_wifi_scan();
		wifi_scan_cmd_cnt++;
	}
}

void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
			(const struct wifi_status *)cb->info;

	#ifdef PRINT_WIFI_SCAN_RESULT
		if (status->status) {
			LOG_ERR("Scan request failed (%d)", status->status);
		} else {
			LOG_INF("Scan request done");
		}
	#endif
}

/**
 *void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
 *				uint32_t mgmt_event, struct net_if *iface)
 *{
 *	const struct device *dev = iface->if_dev->dev;
 *	struct wifi_nrf_vif_ctx_zep *vif_ctx_zep = NULL;
 *	vif_ctx_zep = dev->data;
 *
 *	switch (mgmt_event) {
 *	case NET_EVENT_WIFI_CONNECT_RESULT:
 *		handle_wifi_connect_result(cb);
 *		break;
 *	case NET_EVENT_WIFI_DISCONNECT_RESULT:
 *		handle_wifi_disconnect_result(cb);
 *		break;
 *	case NET_EVENT_WIFI_SCAN_RESULT:
 *		vif_ctx_zep->scan_in_progress = 0;
 *		handle_wifi_scan_result(cb);
 *		break;
 *	case NET_EVENT_WIFI_SCAN_DONE:
 *		handle_wifi_scan_done(cb);
 *		break;
 *	default:
 *		break;
 *	}
 *}
 */


void print_dhcp_ip(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));
#ifdef PRINT_WIFI_SCAN_RESULT
	LOG_INF("IP address: %s", dhcp_info);
#endif
	k_sem_give(&wait_for_next);
}
/**
 *void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
 *				    uint32_t mgmt_event, struct net_if *iface)
 *{
 *	switch (mgmt_event) {
 *	case NET_EVENT_IPV4_DHCP_BOUND:
 *		print_dhcp_ip(cb);
 *		break;
 *	default:
 *		break;
 *	}
 *}
 */
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
int cmd_wifi_scan(void)
{
	struct net_if *iface = net_if_get_default();

	if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
		LOG_ERR("Scan request failed");
		return -ENOEXEC;
	}
	#ifdef PRINT_WIFI_SCAN_RESULT
		LOG_INF("Scan requested");
	#endif
	return 0;
}

int wifi_connect(void)
{
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params cnx_params;

	__wifi_args_to_params(&cnx_params);

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
			 &cnx_params, sizeof(struct wifi_connect_req_params))) {
		LOG_ERR("Connection request failed");
		return -ENOEXEC;
	}

	 LOG_INF("Connection requested"); 
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
		/* LOG_INF("Disconnect requested"); */
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
		LOG_ERR("Invalid IPv4 address %s", host);
		return -EINVAL;
	}

	LOG_INF("Peer IPv4 address %s", host);

	return 0;
}

int wait_for_next_event(const char *event_name, int timeout)
{
	int wait_result;
#ifdef PRINT_WIFI_CONN_RESULT
	if (event_name) {
		LOG_INF("Waiting for %s", event_name);
	}
#endif
	wait_result = k_sem_take(&wait_for_next, K_SECONDS(timeout));
	if (wait_result) {
		LOG_ERR("Timeout waiting for %s -> %d", event_name, wait_result);
		return -1;
	}
#ifdef PRINT_WIFI_CONN_RESULT
	LOG_INF("Got %s", event_name);
#endif
	k_sem_reset(&wait_for_next);

	return 0;
}

void tcp_upload_results_cb(enum zperf_status status,
			  struct zperf_results *result,
			  void *user_data)
{
	uint32_t client_rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New TCP session started.\n");
		wait_for_wifi_client_start = 1;
		break;

	case ZPERF_SESSION_FINISHED: {

		if (result->client_time_in_us != 0U) {
			client_rate_in_kbps = (uint32_t)
				(((uint64_t)result->nb_packets_sent *
				  (uint64_t)result->packet_size * (uint64_t)8 *
				  (uint64_t)USEC_PER_SEC) /
				 ((uint64_t)result->client_time_in_us * 1024U));
		} else {
			client_rate_in_kbps = 0U;
		}

		LOG_INF("Duration:\t%u", result->client_time_in_us);
		LOG_INF("Num packets:\t%u", result->nb_packets_sent);
		LOG_INF("Num errors:\t%u (retry or fail)\n",
						result->nb_packets_errors);
		LOG_INF("Rate in kbps:\t%u", client_rate_in_kbps);

		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("TCP upload failed\n");
		break;
	}
}

void tcp_download_results_cb(enum zperf_status status,
			   struct zperf_results *result,
			   void *user_data)
{
	uint32_t rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New TCP session started.\n");
		wait_for_wifi_client_start = 1;
		break;

	case ZPERF_SESSION_FINISHED: {

		/* Compute baud rate */
		if (result->time_in_us != 0U) {
			rate_in_kbps = (uint32_t)
				(((uint64_t)result->total_len * 8ULL *
				  (uint64_t)USEC_PER_SEC) /
				 ((uint64_t)result->time_in_us * 1024ULL));
		} else {
			rate_in_kbps = 0U;
		}

		LOG_INF("TCP session ended\n");
		LOG_INF("%u bytes in %u ms:", result->total_len,
						result->time_in_us/USEC_PER_MSEC);
		LOG_INF("\nrate in kbps:\t%u", rate_in_kbps);

		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("TCP session error.\n");
		break;
	}
}

void udp_download_results_cb(enum zperf_status status,
			   struct zperf_results *result,
			   void *user_data)
{
	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New session started.");
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

		LOG_INF("End of session!");

		LOG_INF("Download results:");
		LOG_INF("%u bytes in %u ms",
				(result->nb_packets_rcvd * result->packet_size),
				(result->time_in_us / USEC_PER_MSEC));

		LOG_INF(" received packets:\t%u",
				  result->nb_packets_rcvd);
		LOG_INF(" nb packets lost:\t%u",
				  result->nb_packets_lost);
		LOG_INF(" nb packets outorder:\t%u",
				  result->nb_packets_outorder);


		LOG_INF(" rate: %u Kbps", rate_in_kbps);
		LOG_INF("");

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

void run_bt_benchmark(void)
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

int run_wifi_traffic_tcp(bool test_wlan)
{
	int ret = 0;

	if (test_wlan) {

		if (IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER)) {
			struct zperf_download_params param;

			param.port = CONFIG_NET_CONFIG_PEER_IPV4_PORT;

			ret = zperf_tcp_download(&param, tcp_download_results_cb, NULL);
			if (ret != 0) {
				LOG_ERR("Failed to start TCP server session: %d", ret);
				goto err;
			}
			LOG_INF("TCP server started on port %u\n", param.port);

		} else {

			struct zperf_upload_params params;

			/* Start Wi-Fi TCP traffic */
			LOG_INF("Starting Wi-Fi benchmark: Zperf TCP client");
			params.duration_ms = CONFIG_WIFI_TEST_DURATION;
			params.rate_kbps = CONFIG_WIFI_ZPERF_RATE;
			params.packet_size = CONFIG_WIFI_ZPERF_PKT_SIZE;
			parse_ipv4_addr(CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
				&in4_addr_my);
			net_sprint_ipv4_addr(&in4_addr_my.sin_addr);

			memcpy(&params.peer_addr, &in4_addr_my, sizeof(in4_addr_my));

			ret = zperf_tcp_upload_async(&params, tcp_upload_results_cb,
						     NULL);
			if (ret != 0) {
				LOG_ERR("Failed to start TCP session: %d", ret);
				goto err;
			}
		}
	}
	return 0;
err:
	return ret;
}

int run_wifi_traffic(bool test_wlan)
{
	int ret = 0;

	if (test_wlan) {
		if (IS_ENABLED(CONFIG_WIFI_ZPERF_SERVER)) {
			struct zperf_download_params params;

			params.port = CONFIG_NET_CONFIG_PEER_IPV4_PORT;

			ret = zperf_udp_download(&params, udp_download_results_cb, NULL);
			if (ret != 0) {
				LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
				goto err;
			}
		} else {
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
				LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
				goto err;
			}
		}
	}
	return 0;
err:
	return ret;
}

void start_ble_traffic(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (ble_role) {
			/* Start BLE traffic */
			k_thread_start(run_bt_traffic);
		}
	}
}

void check_wifi_traffic(bool test_wlan)
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

void run_ble_traffic(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (ble_role) {
			/* Run BLE traffic */
			k_thread_join(run_bt_traffic, K_FOREVER);
		}
	}
}
void disconnect_wifi(bool test_wlan)
{
	if (test_wlan) {
		/* Wi-Fi disconnection */
	#ifdef PRINT_WIFI_CONN_RESULT
		LOG_INF("Disconnecting Wi-Fi");
	#endif
		wifi_disconnect();
	}
}

void disconnect_ble(bool test_ble, bool ble_role)
{
	if (test_ble) {
		/* BLE disconnection only  if role is central */
		if (ble_role) {
			LOG_INF("Disconnecting BLE");
			bt_throughput_test_exit();
		}
	}
}
int wifi_connection(bool test_wlan, bool wifi_coex_enable,
				bool antenna_mode)
{
	int ret = 0;

	if (test_wlan) {
		/* Controls if Wi-Fi posts requests to PTA */
		/**
		 *#if 0
		 *	nrf_wifi_coex_enable(wifi_coex_enable);
		 *#endif
		 */
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

/**
 *int ble_connection(bool test_ble, bool ble_role)
 *{
 *	int ret = 0;
 *
 *	if (test_ble) {
 *		ret = bt_throughput_test_init(ble_role);
 *		if (ret != 0) {
 *			LOG_ERR("Failed to configure BLE connection test: %d", ret);
 *			goto err;
 *		}
 *	}
 *	return 0;
 *err:
 *	return ret;
 *}
 *
 *int ble_connection_central(bool test_ble, bool ble_role)
 *{
 *	int ret = 0;
 *
 *	if (test_ble) {
 *		ret = bt_scan_test(ble_role);
 *		if (ret != 0) {
 *			LOG_ERR("Failed to configure BLE connection test: %d", ret);
 *			goto err;
 *		}
 *	}
 *	return 0;
 *err:
 *	return ret;
 *}
 */
int config_pta(bool wifi_coex_enable, bool antenna_mode, bool ble_role,
			bool wlan_role)
{
	int ret = 0;
	enum nrf_wifi_pta_wlan_op_band wlan_band =
			wifi_mgmt_to_pta_band(status.band);

	if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_NONE) {
		LOG_ERR("Invalid Wi-Fi band: %d", wlan_band);
		goto err;
	}

	//LOG_INF("Configuring PTA for %s", wifi_band_txt(status.band));
	ret = nrf_wifi_coex_config_pta(wlan_band, antenna_mode, ble_role,
			wlan_role);
	if (ret != 0) {
		LOG_ERR("Failed to configure PTA coex hardware: %d", ret);
		goto err;
	}
	return 0;
err:
	return ret;
}

int wifi_scan_ble_conn_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
		if (wifi_connected_scan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			scan_start();
			/* bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE); */
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
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
			/* LOG_INF("waiting"); */
		}
		repeat_scan = 0;
	}

	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
	LOG_INF(" wifi_scan_ble_conn_central complete");
	return 0;
}

int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable,
			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
			bool wlan_role, bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */
		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
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
		if (wifi_connected_scan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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
			if ((k_uptime_get_32() - test_start_time) >
					CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		repeat_scan = 0;
	}

	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
	LOG_INF(" wifi_scan_ble_conn_peripheral complete");

	return 0;
}

int wifi_scan_ble_tput_central(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */

		if (wifi_connected_scan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}

	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
		start_ble_traffic(test_ble, ble_role);
		run_ble_traffic(test_ble, ble_role);
		disconnect_ble(test_ble, ble_role);
	}

	if (test_wlan) {
		while (true) {
			if (k_uptime_get_32() - test_start_time >
				CONFIG_WIFI_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		repeat_scan = 0;
	}
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
	LOG_INF(" wifi_scan_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_scan_ble_tput_peripheral(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	if (test_ble) {
		if (!ble_role) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3));
		}
	}

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (wifi_connected_scan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		if (test_wlan && test_ble) {
			while (true) {
				if (ble_periph_connected) {
					cmd_wifi_scan();
					break;
				}
			}
		} else if (test_wlan) {
			cmd_wifi_scan();
		}
	} else {
		if (test_wlan && test_ble) {
			while (true) {
				if (ble_periph_connected) {
					cmd_wifi_scan();
					break;
				}
			}
		} else if (test_wlan) {
			cmd_wifi_scan();
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		/* LOG_INF("BLE Throughput started"); */
		start_ble_traffic(test_ble, ble_role);

		run_ble_traffic(test_ble, ble_role);
	}
	while (true) {
		if (k_uptime_get_32() - test_start_time > CONFIG_WIFI_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}
	/* temp fix to avoid ble disconn when peripheral */
	/*k_sleep(K_MSEC(CONFIG_BLE_TEST_DURATION)); */

	if (test_ble) {
		disconnect_ble(test_ble, ble_role);
	}

	/*k_sleep(K_SECONDS(5));*/
	repeat_scan = 0;

	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
	LOG_INF(" wifi_scan_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_con_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}

	if (test_ble) {
		bt_connection_init(ble_role);
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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
		/* note: CONFIG_BLE_TEST_DURATION = CONFIG_WIFI_TEST_DURATION */
		if ((k_uptime_get_32() - test_start_time)
			> CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}

	LOG_INF(" wifi_con_ble_con_central complete");
	return 0;
}

int wifi_con_ble_con_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role,
			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
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
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			if (!ble_periph_connected) {
				adv_start();
				/* k_sleep(K_SECONDS(1)); */
			}
		}
		if (test_wlan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));
		}
		/* Note: BLE disconnection is initiated by peer central.*/
		if (test_wlan) {
			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));
		}
		/* note: CONFIG_BLE_TEST_DURATION = CONFIG_WIFI_TEST_DURATION */
		if ((k_uptime_get_32() - test_start_time) >
			CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}

	LOG_INF(" wifi_con_ble_con_peripheral complete");
	return 0;
}

int wifi_con_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}

	if (test_ble) {
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		/* run_ble_traffic(test_ble, ble_role); */
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

	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}

	LOG_INF(" wifi_con_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_con_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
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
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		run_ble_traffic(test_ble, ble_role);
	}

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
			LOG_INF("Run BLE central");
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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
	if (test_ble && !test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
					CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_ble) {
		/* run_ble_traffic(test_ble, ble_role); */
		disconnect_ble(test_ble, ble_role);
	}

	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}

	LOG_INF(" wifi_con_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */

		if (zperf_udp_or_tcp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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

	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	}

	LOG_INF(" wifi_tput_client_ble_con_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_con_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role,
			bool wlan_role, bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp)
{
	int ret = 0;
	uint64_t test_start_time = 0;


	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
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
		if (zperf_udp_or_tcp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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

	LOG_INF(" wifi_tput_client_ble_con_peripheral complete");
	return 0;
err:
	return ret;
}


int wifi_tput_client_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable, bool zperf_udp_or_tcp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}
	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	if (zperf_udp_or_tcp) {
		ret = run_wifi_traffic(test_wlan);
	} else {
		ret = run_wifi_traffic_tcp(test_wlan);
	}

	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF(" wifi_tput_client_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable,
			bool zperf_udp_or_tcp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}
	if (!ble_role) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	start_ble_traffic(test_ble, ble_role);

	run_ble_traffic(test_ble, ble_role);

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
			LOG_INF("Run BLE central");
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	if (zperf_udp_or_tcp) {
		ret = run_wifi_traffic(test_wlan);
	} else {
		ret = run_wifi_traffic_tcp(test_wlan);
	}
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	check_wifi_traffic(test_wlan);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);
	LOG_INF(" wifi_tput_client_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_con_central(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable, bool zperf_udp_or_tcp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */

		if (zperf_udp_or_tcp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	}
	LOG_INF(" wifi_tput_server_ble_con_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_con_peripheral(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable, bool zperf_udp_or_tcp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
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
		if (zperf_udp_or_tcp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	if (test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
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
	LOG_INF(" wifi_tput_server_ble_con_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_tput_central(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable,
			bool zperf_udp_or_tcp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}
	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (zperf_udp_or_tcp) {
		ret = run_wifi_traffic(test_wlan);
	} else {
		ret = run_wifi_traffic_tcp(test_wlan);
	}
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	if (test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	wait_for_wifi_client_start = 0;

	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF(" wifi_tput_server_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_tput_peripheral(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable,
			bool zperf_udp_or_tcp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

		/* Disable coexistence hardware module for coex disable test cases */
		/**
		 *#if 0
		 *if (!coex_hardware_enable) {
		 *	nrf_wifi_coex_hw_enable(coex_hardware_enable);
		 *}
		 *#endif
		 */
	}

	if (!ble_role) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	if (test_ble) {
		/* ble_connection(test_ble, ble_role); */
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (zperf_udp_or_tcp) {
		ret = run_wifi_traffic(test_wlan);
	} else {
		ret = run_wifi_traffic_tcp(test_wlan);
	}
	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}
	if (test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	start_ble_traffic(test_ble, ble_role);

	check_wifi_traffic(test_wlan);

	run_ble_traffic(test_ble, ble_role);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, ble_role);

	LOG_INF(" wifi_tput_server_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

//int wifi_con_ble_con_central(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
//			bool antenna_mode, bool coex_hardware_enable)
int wifi_con_ble_con_central_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	/* Wi-Fi connection done only once */
	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		k_sleep(K_SECONDS(2));
	}

	if (test_ble) {
		bt_connection_init(ble_role);
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			scan_start();
			k_sleep(K_SECONDS(1));
		}

		if (test_ble) {
			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
		}
		if ((k_uptime_get_32() - test_start_time)
			> CONFIG_BLE_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (test_wlan) {
		disconnect_wifi(test_wlan);
		k_sleep(K_SECONDS(2));
		if(status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {			
			LOG_INF("Wi-Fi connection intact");
		}
	}
	if (test_wlan) {
		//disconnect_wifi(test_wlan);
		k_sleep(K_SECONDS(2));
	}
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
	LOG_INF(" wifi_con_ble_con_central_regr complete");
	return 0;
}
//int wifi_con_ble_con_peripheral(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role,
//			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
int wifi_con_ble_con_peripheral_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role,
			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		k_sleep(K_SECONDS(2));
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
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			if (!ble_periph_connected) {
				adv_start();
				/* k_sleep(K_SECONDS(1)); */
			}
		}
		/* Note: BLE disconnection is initiated by peer central.*/
		if ((k_uptime_get_32() - test_start_time) >
			CONFIG_BLE_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (test_wlan) {
		if(status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {			
			LOG_INF("Wi-Fi connection intact");
		}
	}

	if (test_wlan) {
		disconnect_wifi(test_wlan);
		k_sleep(K_SECONDS(2));
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
	LOG_INF(" wifi_con_ble_con_peripheral_regr complete");
	return 0;
}

//int wifi_con_ble_tput_central(bool test_wlan,
//			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
//			bool ble_role, bool wlan_role, bool coex_hardware_enable)
int wifi_con_ble_tput_central_regr(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		k_sleep(K_SECONDS(2));
	}

	if (test_ble) {
		ret = bt_throughput_test_init(ble_role);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	test_start_time = k_uptime_get_32();

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		/* run_ble_traffic(test_ble, ble_role); */
	}

	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_BLE_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}

	if (test_ble) {
		run_ble_traffic(test_ble, ble_role);
		disconnect_ble(test_ble, ble_role);
	}

	if (test_wlan) {
		if(status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {			
			LOG_INF("Wi-Fi connection intact");
		}
	}


	if (test_wlan) {
		disconnect_wifi(test_wlan);
		k_sleep(K_SECONDS(2));
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
	LOG_INF(" wifi_con_ble_tput_central_regr complete");
	return 0;
err:
	return ret;
}

//int wifi_con_ble_tput_peripheral(bool test_wlan,
//			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
//			bool ble_role, bool wlan_role, bool coex_hardware_enable)
int wifi_con_ble_tput_peripheral_regr(bool test_wlan,
			bool wifi_coex_enable, bool antenna_mode, bool test_ble,
			bool ble_role, bool wlan_role, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */

	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		k_sleep(K_SECONDS(2));
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
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (test_ble) {
		start_ble_traffic(test_ble, ble_role);
		run_ble_traffic(test_ble, ble_role);
	}

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
			LOG_INF("Run BLE central");
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	test_start_time = k_uptime_get_32();

	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_BLE_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_ble && !test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
					CONFIG_BLE_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_ble) {
		/* run_ble_traffic(test_ble, ble_role); */
		disconnect_ble(test_ble, ble_role);
	}

	if (test_wlan) {
		if(status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {			
			LOG_INF("Wi-Fi connection intact");
		}
	}

	if (test_wlan) {
		disconnect_wifi(test_wlan);
		k_sleep(K_SECONDS(2));
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
	LOG_INF(" wifi_con_ble_tput_peripheral_regr complete");
	return 0;
err:
	return ret;
}

//int wifi_scan_ble_conn_central(bool wifi_coex_enable, bool antenna_mode,
//	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
//	bool coex_hardware_enable, bool wifi_connected_scan)
int ble_conn_central_wifi_scan_regr(bool wifi_coex_enable, bool antenna_mode,
	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
	bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;
	
	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
	}
	
	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */
		if (wifi_connected_scan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}

	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
			/* LOG_INF("waiting"); */
		}
		repeat_scan = 0;
	}
	bt_disconnect_central();
	k_sleep(K_SECONDS(2));
	if (test_ble) {
		if (ble_central_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		bt_disconnect_central();
		k_sleep(K_SECONDS(2));
	}
	if (test_ble) {
		LOG_INF("ble_conn_cnt_regr = %u", ble_conn_cnt_regr);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}

	LOG_INF(" ble_conn_central_wifi_scan_regr complete");
	return 0;
}

//int wifi_con_ble_con_central(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
//			bool antenna_mode, bool coex_hardware_enable)
int ble_conn_central_wifi_con_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)

{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_wlan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));
		}
		if (test_wlan) {
			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));
		}
		if ((k_uptime_get_32() - test_start_time)
			> CONFIG_WIFI_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	if (test_ble) {
		if (ble_central_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		bt_disconnect_central();
		k_sleep(K_SECONDS(2));
	}

	LOG_INF(" ble_conn_central_wifi_con_regr complete");
	return 0;
}

//PENDING: ble_conn_central_wifi_ping_regr()


//int wifi_tput_client_ble_con_central(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
//			bool antenna_mode, bool coex_hardware_enable)
int ble_conn_central_wifi_tp_udp_client_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
	}

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		if (IS_ENABLED(CONFIG_WIFI_ZPERF_PROT_UDP)) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION_REGR) {
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

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	if (test_ble) {
		if (ble_central_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		bt_disconnect_central();
		k_sleep(K_SECONDS(2));
	}

	LOG_INF(" ble_conn_central_wifi_tp_udp_client_regr complete");
	return 0;
err:
	return ret;
}
//int wifi_tput_server_ble_con_central(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
//			bool antenna_mode, bool coex_hardware_enable)
int ble_conn_central_wifi_tp_udp_server_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role, bool wlan_role,
			bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
	}

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */

		if (IS_ENABLED(CONFIG_WIFI_ZPERF_PROT_UDP)) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION_REGR) {
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

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	if (test_ble) {
		if (ble_central_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		bt_disconnect_central();
		k_sleep(K_SECONDS(2));
	}

	LOG_INF(" ble_conn_central_wifi_tp_udp_server_regr complete");
	return 0;
err:
	return ret;
}


//int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable,
//			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
//			bool wlan_role, bool coex_hardware_enable, bool wifi_connected_scan)
int ble_conn_peripheral_wifi_scan_regr(bool wifi_coex_enable,
			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
			bool wlan_role, bool coex_hardware_enable, bool wifi_connected_scan)
{
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_wlan) {
		if (wifi_connected_scan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif

	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
	}
	if (test_wlan) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
					CONFIG_WIFI_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}
		repeat_scan = 0;
	}

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */

	// note - BLE disconnect is available for only central role.
	// may be try connection and decide if connection already exists.
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
	}
	
	LOG_INF(" ble_conn_peripheral_wifi_scan_regr complete");

	return 0;
}


//int wifi_con_ble_con_peripheral(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role,
//			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
int ble_conn_peripheral_wifi_con_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role,
			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_wlan) {
			wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
			k_sleep(K_SECONDS(2));
		}
		if (test_wlan) {
			disconnect_wifi(test_wlan);
			k_sleep(K_SECONDS(2));
		}

		if ((k_uptime_get_32() - test_start_time) >
			CONFIG_WIFI_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	// note - BLE disconnect is available for only central role.
	// may be try connection and decide if connection already exists.
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
	}

	LOG_INF(" ble_conn_peripheral_wifi_con_regr complete");
	return 0;
}

//PENDING: ble_conn_peripheral_wifi_ping_regr()

//int wifi_tput_client_ble_con_peripheral(bool test_wlan,
//			bool wifi_coex_enable, bool test_ble, bool ble_role,
//			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
int ble_conn_peripheral_wifi_tput_client_regr(bool test_wlan,
			bool wifi_coex_enable, bool test_ble, bool ble_role,
			bool wlan_role, bool antenna_mode, bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_wlan) {
		if (IS_ENABLED(CONFIG_WIFI_ZPERF_PROT_UDP)) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION_REGR) {
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
	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	// note - BLE disconnect is available for only central role.
	// may be try connection and decide if connection already exists.
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
	}

	LOG_INF(" ble_conn_peripheral_wifi_tp_udp_client_regr complete");
	return 0;
err:
	return ret;
}

//int wifi_tput_server_ble_con_peripheral(bool test_wlan, bool wifi_coex_enable,
//		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
//		bool coex_hardware_enable)
int ble_conn_peripheral_wifi_tput_server_regr(bool test_wlan, bool wifi_coex_enable,
		bool test_ble, bool ble_role, bool wlan_role, bool antenna_mode,
		bool coex_hardware_enable)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(ble_role);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
		wifi_connection(test_wlan, wifi_coex_enable, antenna_mode);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(wifi_coex_enable, antenna_mode, ble_role, wlan_role);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_wlan) {
		if (IS_ENABLED(CONFIG_WIFI_ZPERF_PROT_UDP)) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}
	if (test_wlan) {
		while (!wait_for_wifi_client_start) {
			LOG_INF("start WiFi client");
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION_REGR) {
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

	/* one time BT disconnection */
	/* PENDING - check if BT connection is intact */
	// note - BLE disconnect is available for only central role.
	// may be try connection and decide if connection already exists.
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
	}

	LOG_INF(" ble_conn_peripheral_wifi_tput_server_regr complete");
	return 0;
err:
	return ret;
}

//int wifi_scanble_conn_central(bool wifi_coex_enable, bool antenna_mode,
//	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
//	bool coex_hardware_enable, bool wifi_connected_scan)
int ble_con_central_wifi_shutdown(bool ble_role)
{
	uint64_t test_start_time = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();

	bt_connection_init(ble_role);
	k_sleep(K_SECONDS(1));

	bt_disconnect_central();
	k_sleep(K_SECONDS(2));
	
	while (true) {
		if ((k_uptime_get_32() - test_start_time)
			> CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}
	LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
	LOG_INF("ble_conn_fail_cnt = %u", ble_conn_fail_cnt);

	LOG_INF(" ble_con_central_wifi_shutdown complete");
	return 0;
}

//int wifi_scan_ble_conn_peripheral(bool wifi_coex_enable,
//			bool antenna_mode, bool test_ble, bool test_wlan, bool ble_role,
//			bool wlan_role, bool coex_hardware_enable, bool wifi_connected_scan)
int ble_con_peripheral_wifi_shutdown(bool ble_role)
{
	uint64_t test_start_time = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();

	bt_connection_init(ble_role);
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();
	while (true) {
		if ((k_uptime_get_32() - test_start_time) >
			CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	LOG_INF(" ble_con_peripheral_wifi_shutdown complete");

	return 0;
}

//int wifi_scan_ble_tput_central(bool wifi_coex_enable, bool antenna_mode,
//	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
//	bool coex_hardware_enable, bool wifi_connected_scan)
int ble_tp_central_wifi_shutdown(bool test_ble, bool ble_role)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();
	/* BLE connection */
	ret = bt_throughput_test_init(ble_role);
	if (ret != 0) {
		LOG_ERR("Failed to BT throughput init: %d", ret);
		goto err;
	}
	start_ble_traffic(test_ble, ble_role);
	run_ble_traffic(test_ble, ble_role);
	disconnect_ble(test_ble, ble_role);

	LOG_INF(" ble_tp_central_wifi_shutdown complete");
	return 0;
err:
	return ret;
}

//int wifi_scan_ble_tput_peripheral(bool wifi_coex_enable, bool antenna_mode,
//	bool test_ble, bool test_wlan, bool ble_role, bool wlan_role,
//	bool coex_hardware_enable, bool wifi_connected_scan)
int ble_tp_periph_wifi_shutdown(bool test_ble, bool ble_role)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	if (!ble_role) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	/* BLE connection */
	ret = bt_throughput_test_init(ble_role);
	if (ret != 0) {
		LOG_ERR("Failed to BT throughput init: %d", ret);
		goto err;
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("");
	LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
	LOG_INF("");
	#endif
	test_start_time = k_uptime_get_32();
	/* LOG_INF("BLE Throughput started"); */
	start_ble_traffic(test_ble, ble_role);

	run_ble_traffic(test_ble, ble_role);

	while (true) {
		if (k_uptime_get_32() - test_start_time > CONFIG_BLE_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	disconnect_ble(test_ble, ble_role);

	LOG_INF(" ble_tp_periph_wifi_shutdown complete");
	return 0;
err:
	return ret;
}
#endif
