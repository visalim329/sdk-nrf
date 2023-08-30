/**
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Wi-Fi and Bluetooth LE coexistence test functions
 */

#include "bt_coex_test_functions.h"

int8_t wifi_rssi = 127;
uint32_t print_wifi_scan_time;
uint64_t wifi_scan_start_time;
uint64_t wifi_scan_time;
uint32_t ble_le_datalen_failed;
uint32_t ble_phy_update_failed;
uint32_t ble_le_datalen_timeout;
uint32_t ble_phy_update_timeout;
uint32_t ble_conn_param_update_failed;
uint32_t ble_conn_param_update_timeout;

static int print_wifi_conn_status_once = 1;


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

	/* LOG_INF("Status: successful"); */
	LOG_INF("==================");
	LOG_INF("State: %s", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		if (print_wifi_conn_status_once == 1) {

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
			/* LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp)); */
			LOG_INF("WiFi RSSI: %d", status.rssi);

			print_wifi_conn_status_once++;
		}
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
		if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_INFO)) {
			LOG_INF("Connected");
		}
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
	wifi_disconn_cnt_stability++;
	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_INFO)) {
		cmd_wifi_status();
	}
}


void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_scan_result *entry =
			(const struct wifi_scan_result *)cb->info;
	/* uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")]; */

	scan_result_count++;

	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_INFO)) {
		LOG_INF("%-4d | %-12s | %-4u  | %-4d",
			scan_result_count, entry->ssid, entry->channel, entry->rssi);
		LOG_INF("Wi-Fi scan results for %d times", scan_result_count);
	}

	if (entry->channel <= HIGHEST_CHANNUM_24G) {
		wifi_scan_cnt_24g++;
	} else {
		wifi_scan_cnt_5g++;
	}
}

void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
	wifi_scan_time = k_uptime_get_32() - wifi_scan_start_time;
	/**if (print_wifi_scan_time) {
	 *	LOG_INF("wifi_scan_time=%d",wifi_scan_time);
	 *	}
	 */
	print_wifi_scan_time++;
	wifi_scan_start_time = k_uptime_get_32();

	k_sleep(K_MSEC(1));

	if (repeat_wifi_scan == 1) {
		wifi_scan_cmd_cnt++;
		cmd_wifi_scan();
	}

	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_INFO)) {
		const struct wifi_status *status =
			(const struct wifi_status *)cb->info;
		if (status->status) {
			LOG_ERR("Scan request failed (%d)", status->status);
		} else {
			LOG_INF("Scan request done");
		}
		k_sleep(K_MSEC(1)); /* in milliseconds, can't reduce further */
	}
}

void print_dhcp_ip(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_DHCP_INFO)) {
		LOG_INF("IP address: %s", dhcp_info);
	}
	k_sem_give(&wait_for_next);
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
int cmd_wifi_scan(void)
{
	struct net_if *iface = net_if_get_default();

	if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
		LOG_ERR("Scan request failed");
		return -ENOEXEC;
	}
	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_INFO)) {
		LOG_INF("Scan requested");
	}
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
	/* LOG_INF("Connection requested"); */
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
			/* LOG_ERR("Already disconnected"); */
			wifi_disconn_no_conn_cnt++;
		} else {
			/* LOG_ERR("Disconnect request failed"); */
			wifi_disconn_fail_cnt++;
			return -ENOEXEC;
		}
	} else {
		wifi_disconn_success_cnt++;
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
	LOG_INF("Wi-Fi peer IPv4 address %s", host);
	return 0;
}

int wait_for_next_event(const char *event_name, int timeout)
{
	int wait_result;

	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_INFO)) {
		if (event_name) {
			LOG_INF("Waiting for %s", event_name);
		}
	}
	wait_result = k_sem_take(&wait_for_next, K_SECONDS(timeout));
	if (wait_result) {
		LOG_ERR("Timeout waiting for %s -> %d", event_name, wait_result);
		return -1;
	}
	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_INFO)) {
		LOG_INF("Got %s", event_name);
	}
	k_sem_reset(&wait_for_next);

	return 0;
}

void tcp_upload_results_cb(enum zperf_status status, struct zperf_results *result,
		void *user_data)
{
	uint32_t client_rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New TCP session started.\n");
		wait4_peer_wifi_client_to_start_tp_test = 1;
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
		LOG_INF("\nrate in kbps:%u kbps", client_rate_in_kbps);
		k_sem_give(&udp_tcp_callback);
		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("TCP upload failed\n");
		break;
	}
}

void tcp_download_results_cb(enum zperf_status status, struct zperf_results *result,
		void *user_data)
{
	uint32_t rate_in_kbps;

	switch (status) {
	case ZPERF_SESSION_STARTED:
		LOG_INF("New TCP session started.\n");
		wait4_peer_wifi_client_to_start_tp_test = 1;
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
		LOG_INF("\nrate in kbps:%u kbps", rate_in_kbps);
		k_sem_give(&udp_tcp_callback);
		break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("TCP session error.\n");
		break;
	}
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
			LOG_INF("\nrate in kbps:%u kbps", rate_in_kbps);
			LOG_INF("");
			k_sem_give(&udp_tcp_callback);
			break;
	}

	case ZPERF_SESSION_ERROR:
		LOG_INF("UDP session error.");
		break;
	}
}


void udp_upload_results_cb(enum zperf_status status, struct zperf_results *result,
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
		k_sem_give(&udp_tcp_callback);
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

void run_bt_conn_benchmark(void)
{
	bt_conn_test_run();
}

void run_wifi_scan_benchmark(void)
{
	wifi_scan_test_run();
}

void run_wifi_conn_benchmark(void)
{
	wifi_connection_test_run();
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
			params.duration_ms = CONFIG_COEX_TEST_DURATION;
			params.rate_kbps = CONFIG_WIFI_ZPERF_RATE;
			params.packet_size = CONFIG_WIFI_ZPERF_PKT_SIZE;
			parse_ipv4_addr(CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
				&in4_addr_my);
			net_sprint_ipv4_addr(&in4_addr_my.sin_addr);

			memcpy(&params.peer_addr, &in4_addr_my, sizeof(in4_addr_my));

			ret = zperf_tcp_upload_async(&params, tcp_upload_results_cb, NULL);
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
			params.duration_ms = CONFIG_COEX_TEST_DURATION;
			params.rate_kbps = CONFIG_WIFI_ZPERF_RATE;
			params.packet_size = CONFIG_WIFI_ZPERF_PKT_SIZE;
			parse_ipv4_addr(CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &in4_addr_my);
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

void start_ble_activity(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/* In case BLE is peripheral, skip running BLE connection/traffic */
		#ifdef ENABLE_BLE_CONN_TEST
		if (is_ble_central) {
			k_thread_start(run_bt_connection);
		}
		#endif
		#ifdef ENABLE_BLE_TRAFFIC_TEST
		if (is_ble_central) {
			k_thread_start(run_bt_traffic);
		}
		#endif
	}
}

void start_wifi_activity(void)
{
	#ifdef ENABLE_WIFI_SCAN_TEST
		k_thread_start(run_wlan_scan);
	#endif
	#ifdef ENABLE_WIFI_CONN_TEST
		k_thread_start(run_wlan_conn);
	#endif
}

void check_wifi_traffic(bool test_wlan)
{
	if (test_wlan) {
		/* Run Wi-Fi traffic */
		if (k_sem_take(&udp_tcp_callback, K_FOREVER) != 0) {
			LOG_ERR("Results are not ready");
		} else {
			LOG_INF("UDP SESSION FINISHED");
		}
	}
}

void run_ble_activity(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/* In case BLE is peripheral, skip running BLE connection/traffic */
		#ifdef ENABLE_BLE_CONN_TEST
		if (is_ble_central) {
			k_thread_join(run_bt_connection, K_FOREVER);
		}
		#endif
		#ifdef ENABLE_BLE_TRAFFIC_TEST
		if (is_ble_central) {
			k_thread_join(run_bt_traffic, K_FOREVER);
		}
		#endif
	}
}

void run_wifi_activity(void)
{
	#ifdef ENABLE_WIFI_SCAN_TEST
		k_thread_join(run_wlan_scan, K_FOREVER);
	#endif
	#ifdef ENABLE_WIFI_CONN_TEST
		k_thread_join(run_wlan_conn, K_FOREVER);
	#endif
}

void exit_bt_throughput_test(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/** Disconnect BLE if role is central. Disconnection in the case of
		 * peripheral role is taken care by the peer central
		 */
		if (is_ble_central) {
			LOG_INF("Disconnecting BLE");
			bt_throughput_test_exit();
		}
	}
}
int wifi_connection(bool test_wlan)
{
	int ret = 0;

	if (test_wlan) {
		wifi_conn_attempt_cnt++;
		/* Wi-Fi connection */
		wifi_connect();

		if (wait_for_next_event("Wi-Fi Connection", WIFI_CONNECTION_TIMEOUT)) {
			wifi_conn_timeout_cnt++;
			ret = -1;
			goto err;
		}
		if (wait_for_next_event("Wi-Fi DHCP", WIFI_DHCP_TIMEOUT)) {
			wifi_dhcp_timeout_cnt++;
			ret = -1;
			goto err;
		}
	}
	wifi_conn_success_cnt++;
	return ret;
err:
	wifi_conn_fail_cnt++;
	return ret;
}
void wifi_disconnection(bool test_wlan)
{
	int ret = 0;
	if (test_wlan) {
		wifi_disconn_attempt_cnt++;
		/* Wi-Fi disconnection */
		if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_INFO)) {
			LOG_INF("Disconnecting Wi-Fi");
		}
		ret = wifi_disconnect();
		if (ret!=0) {
			LOG_INF("Disconnect failed");
		}
	}
}

int config_pta(bool is_ant_mode_sep, bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	enum nrf_wifi_pta_wlan_op_band wlan_band = wifi_mgmt_to_pta_band(status.band);

	if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_NONE) {
		LOG_ERR("Invalid Wi-Fi band: %d", wlan_band);
		goto err;
	}

	LOG_INF("Configuring PTA for %s", wifi_band_txt(status.band));
	ret = nrf_wifi_coex_config_pta(wlan_band, is_ant_mode_sep, is_ble_central,
			is_wlan_server);
	if (ret != 0) {
		LOG_ERR("Failed to configure PTA coex hardware: %d", ret);
		goto err;
	}
	return 0;
err:
	return ret;
}


void wifi_scan_test_run(void)
{

	int64_t test_start_time;

	test_start_time = k_uptime_get_32();

	wifi_scan_cmd_cnt++;
	cmd_wifi_scan();

	while (true) {
		if (k_uptime_get_32() - test_start_time > CONFIG_COEX_TEST_DURATION) {
			break;
		}
		k_sleep(K_MSEC(100)); /* in milliseconds. can be reduced to 1ms?? */
	}
}

void wifi_connection_test_run(void)
{
	uint64_t test_start_time = 0;
	int ret = 0;
	bool test_wlan = 1;
	while (true) {
		ret = wifi_connection(test_wlan);
		if (ret != 0) {
			LOG_INF("Wi-Fi connection failed");
		}
		k_sleep(KSLEEP_WIFI_CON_10MSEC);

		wifi_disconnection(test_wlan);
		k_sleep(KSLEEP_WIFI_DISCON_10MSEC);

		if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
			break;
		}
	}
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

// #ifdef ALLOC_BLE_PTI_WINDOW
	// uint32_t device_req_window = NRF_WIFI_SR_DEVICE;
	// uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
	// uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
	// uint32_t can_be_deferred = NRF_WIFI_NO;
	// uint32_t pti_window_duration = 700;
	// uint32_t gap_between_pti_windows = 50;
	// uint64_t test_start_time = 0;	

	// int wifi_scan_ble_connection(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		// bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
	// {
		// if (test_wlan) {
			// #if defined(CONFIG_NRF700X_BT_COEX)
				// config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
			// #endif/* CONFIG_NRF700X_BT_COEX */
		// }
		// if (test_ble) {
			// /* BT init and connection */
			// bt_connection_init(is_ble_central);
			// /* BLE disconnection */
			// bt_disconnect_central();
		// }
		
		// #ifdef DEMARCATE_TEST_START
		// LOG_INF("----------------------------------------------------------------");
		// #endif	

		// if (test_ble) {
			// start_ble_activity(test_ble, is_ble_central);
		// }



		// uint32_t loop_count=1;
		// for (loop_count=1; loop_count<=30; loop_count++) {
			
			// window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
			// nrf_wifi_coex_allocate_spw(device_req_window,
				// window_start_or_end, imp_of_request, can_be_deferred);
			
			// test_start_time = k_uptime_get_32();
			
			// while (true) {			
				// if ((k_uptime_get_32() - test_start_time) > pti_window_duration)
				// {
					// break;
				// }
				// k_sleep(K_MSEC(50));
			// }
			
			// window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
			// nrf_wifi_coex_allocate_spw(device_req_window,
				// window_start_or_end, imp_of_request, can_be_deferred);
				
			// k_sleep(K_MSEC(gap_between_pti_windows));
		// }

		// repeat_wifi_scan = 0;

		// if (test_wlan) {
			// LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
			// LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
			// LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
		// }

		// LOG_INF("ble_supervision_timeout = %u", ble_supervision_timeout);
	
		// return 0;
	// }

//#else
	int wifi_scan_ble_connection(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
	{
		uint64_t test_start_time = 0;
		int ret = 0;

		if (is_ble_central) {
			if (is_wifi_conn_scan) {
				LOG_INF("Test case: wifi_conn_scan_ble_conn_central");
			} else {
				LOG_INF("Test case: wifi_scan_ble_conn_central");
			}
		} else {
			if (is_wifi_conn_scan) {
				LOG_INF("Test case: wifi_conn_scan_ble_conn_peripheral");
			} else {
				LOG_INF("Test case: wifi_scan_ble_conn_peripheral");
			}
		}

		print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

		if (test_wlan) {
			if (is_wifi_conn_scan) {
				
				#ifndef CHECK_WIFI_CONN_STATUS
					wifi_connection(test_wlan);
				#else
					ret = wifi_connection(test_wlan); /* for connected scan */
					k_sleep(K_SECONDS(3));
					if (ret != 0) {
						LOG_ERR("Wi-Fi connection failed. Running the test\
								further is not meaningful. So, exiting the test");
						goto err;
					}
				#endif

			}
			#if defined(CONFIG_NRF700X_BT_COEX)
				config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
			#endif/* CONFIG_NRF700X_BT_COEX */
		}
		if (test_ble) {
			/* Initialize BLE by selecting role and connect it to peer device. */
			ble_connection_attempt_cnt++;
			bt_connection_init(is_ble_central);
			k_sleep(K_SECONDS(3));

			if (is_ble_central) {
				/**If BLE is central, disconnect the connection.
				 * Connection and disconnection happens in loop later.
				 */
				ble_disconnection_attempt_cnt++;
				bt_disconnect_central();
			} else {
				/**If BLE is peripheral, wait until peer BLE central
				 *  initiates the connection, DUT is connected to peer central
				 *  and update the PHY parameters.
				 */
				while (true) {
					if (ble_periph_connected) {
						break;
					}
					k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
				}
			}
		}

		#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		    /* peer central waits on this in automation */
			LOG_INF("Run BLE central");
			k_sleep(K_SECONDS(1));
		#endif

		if (!is_ble_central) {
			LOG_INF("DUT is in peripheral role, Check for BLE connection\
					counts on peer BLE side");
		}

		#ifdef DEMARCATE_TEST_START
			LOG_INF("-------------------------");
		#endif
		repeat_wifi_scan = 1;
		test_start_time = k_uptime_get_32();

		/* Begin BLE conections and disconnections for a period of BLE test duration */
		if (test_ble) {
			/* This is dummy function in case of peripehral */
			start_ble_activity(test_ble, is_ble_central);
		}

		/* Begin Wi-Fi scan and continue for test duration */
		if (test_wlan) {
			start_wifi_activity();
		}

		/* Wait for BLE activity completion i.e., for COEX_TEST_DURATION */
		if (test_ble) {
			/* This is dummy function in case of peripehral */
			run_ble_activity(test_ble, is_ble_central);
			/* If DUT BLE is in peripheral role wait for COEX_TEST_DURATION */
			if(!is_ble_central) {
				while(1) {
					if (k_uptime_get_32() - test_start_time > CONFIG_COEX_TEST_DURATION) {
						break;
					}
					k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
				}
			}
		}
		/* Wait for the completion of Wi-Fi scan and disconnect Wi-Fi if connected scan */
		if (test_wlan) {
			run_wifi_activity();
			/* Disconnect wifi if connected scan */
			if (is_wifi_conn_scan) {
				wifi_disconnection(test_wlan);
			}
		}
		/* Stop further Wi-Fi scan*/
		repeat_wifi_scan = 0;

	#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		if (test_ble) {
			if (is_ble_central) {
				LOG_INF("ble_connection_attempt_cnt = %u",
					ble_connection_attempt_cnt);
				LOG_INF("ble_connection_success_cnt = %u",
					ble_connection_success_cnt);


				LOG_INF("ble_disconnection_attempt_cnt = %u",
					ble_disconnection_attempt_cnt);
				LOG_INF("ble_disconnection_success_cnt = %u",
					ble_disconnection_success_cnt);
				LOG_INF("ble_disconnection_fail_cnt = %u",
					ble_disconnection_fail_cnt);
				LOG_INF("ble_discon_no_conn_cnt = %u",
					ble_discon_no_conn_cnt);
			} else {
				LOG_INF("check peer device for result counts");
				LOG_INF("Counts printed below are for information purpose\
				and not actual results.");

				LOG_INF("ble_le_datalen_failed = %u",
					ble_le_datalen_failed);
				LOG_INF("ble_phy_update_failed = %u",
					ble_phy_update_failed);
				LOG_INF("ble_le_datalen_timeout = %u",
					ble_le_datalen_timeout);

				LOG_INF("ble_phy_update_timeout = %u",
					ble_phy_update_timeout);
				LOG_INF("ble_conn_param_update_failed = %u",
					ble_conn_param_update_failed);
				LOG_INF("ble_conn_param_update_timeout = %u",
					ble_conn_param_update_timeout);
			}
		}
		if (test_wlan) {
			LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
			LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
			LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		}
	#endif
	err:
		return 0;
	}

//#endif
int wifi_scan_ble_tput(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	int ret = 0;
	int64_t test_start_time = 0;

	if (is_ble_central) {
		if (is_wifi_conn_scan) {
			LOG_INF("Test case: wifi_conn_scan_ble_tput_central");
		} else {
			LOG_INF("Test case: wifi_scan_ble_tput_central");
		}
	} else {
		if (is_wifi_conn_scan) {
			LOG_INF("Test case: wifi_conn_scan_ble_tput_peripheral");
		} else {
			LOG_INF("Test case: wifi_scan_ble_tput_peripheral");
		}
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		if (is_wifi_conn_scan) {
				#ifndef CHECK_WIFI_CONN_STATUS
					wifi_connection(test_wlan); /* for connected scan */
				#else
					ret = wifi_connection(test_wlan); /* for connected scan */
					k_sleep(K_SECONDS(3));
					if (ret != 0) {
						LOG_ERR("Wi-Fi connection failed. Running the test\
								further is not meaningful. So, exiting the test");
						goto err;
					}
				#endif
		}
		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		/* Initialize throughput test. This does connection parameters configurations. */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BLE throughput init");
			/* no meaning in running the coex test and checking the results */
			goto bt_init_error;
		}
	}

	repeat_wifi_scan = 1;

	if (test_ble) {
		/** In case of central, start BLE traffic for BLE_TEST_DURATION.
		 * In case of peripheral, peer device begins the traffic.
		 */
		if (is_ble_central) {
			#ifdef DEMARCATE_TEST_START
			LOG_INF("-------------------------");
			#endif
			test_start_time = k_uptime_get_32();
			start_ble_activity(test_ble, is_ble_central);
		} else {
			while (!wait4_peer_ble2_start_connection) {
				/* Peer BLE starts the the test. */
				#ifdef CONFIG_PRINTS_FOR_AUTOMATION
				LOG_INF("Run BLE central");
				#endif
				k_sleep(K_SECONDS(1));
			}
			wait4_peer_ble2_start_connection = 0;
			#ifdef DEMARCATE_TEST_START
				LOG_INF("-------------------------");
			#endif
			test_start_time = k_uptime_get_32();
		}
	}

	/* Begin Wi-Fi scan and repeat it for Test Duration period. */
	if (test_wlan) {
		start_wifi_activity();
	}

	if (test_ble) {
		if (is_ble_central) {
			/* Wait for BLE traffic completion in case of central and exit */
			run_ble_activity(test_ble, is_ble_central);
			exit_bt_throughput_test(test_ble, is_ble_central);
		} else {
			/** If BLE is peripheral, above thread doesnot wait for
			 *test completion, need to wait for test duration.
			 */
			while (1) {
				if (k_uptime_get_32() - test_start_time >
					CONFIG_COEX_TEST_DURATION) {
					break;
				}
				k_sleep(K_MSEC(100)); 
			}
		}
	}
	/* Wait for the completion of Wi-Fi activity i.e., for WLAN_TEST_DURATION */
	if (test_wlan) {
		run_wifi_activity();
		/* Disconnect wifi if connected scan */
		if (is_wifi_conn_scan) {
			wifi_disconnection(test_wlan);
		}
	}
	/* Stop further Wi-Fi scan*/
	repeat_wifi_scan = 0;

		#ifdef CONFIG_PRINTS_FOR_AUTOMATION
			if (test_wlan) {
				LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
				LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
				LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
			}
		#endif
		return 0;

bt_init_error:

		LOG_ERR("Failed to BT throughput init: %d", ret);
		return 0;
}

int wifi_con_ble_tput(bool test_wlan, bool is_ant_mode_sep,
	bool test_ble, bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	int64_t test_start_time = 0;

	if (is_ble_central) {
		LOG_INF("Test case: wifi_con_ble_tput_central");
	} else {
		LOG_INF("Test case: wifi_con_ble_tput_peripheral");
	}

	LOG_INF("test_wlan = %d", test_wlan);
	LOG_INF("test_ble = %d", test_ble);

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		#if defined(CONFIG_NRF700X_BT_COEX)
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif

	test_start_time = k_uptime_get_32();

	/* start Wi-Fi connection and disconnection sequence for test duration period */
	if (test_wlan) {
		start_wifi_activity();
	}

	if (test_ble) {
		/** Start BLE traffic for BLE_TEST_DURATION. In case of peripheral,
		 *peer device begins traffic, this is a dummy function
		 */
		start_ble_activity(test_ble, is_ble_central);
		if (is_ble_central) {
			/* Wait for BLE traffic completion in case of central and exit */
			run_ble_activity(test_ble, is_ble_central);
			exit_bt_throughput_test(test_ble, is_ble_central);
		} else {
			/* If BLE is peripheral then wait for BLE test to complete */
			while (1) {
				if (k_uptime_get_32() - test_start_time >
					CONFIG_COEX_TEST_DURATION) {
					break;
				}
				k_sleep(K_MSEC(100));
			}
		}
	}
	if (test_wlan) {
		/* Wait for the completion of Wi-Fi activity */
		run_wifi_activity();
	}

	#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		if (test_wlan) {
			LOG_INF("wifi_conn_attempt_cnt = %u", wifi_conn_attempt_cnt);
			LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
			LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
			LOG_INF("wifi_conn_timeout_cnt = %u", wifi_conn_timeout_cnt);
			LOG_INF("wifi_dhcp_timeout_cnt = %u", wifi_dhcp_timeout_cnt);
			LOG_INF("wifi_disconn_attempt_cnt = %u", wifi_disconn_attempt_cnt);
			LOG_INF("wifi_disconn_success_cnt = %u", wifi_disconn_success_cnt);
			LOG_INF("wifi_disconn_fail_cnt = %u", wifi_disconn_fail_cnt);
			LOG_INF("wifi_disconn_no_conn_cnt = %u", wifi_disconn_no_conn_cnt);
		}
	#endif
	return 0;
err:
	return ret;
}

int wifi_tput_ble_con(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (is_ble_central) {
		if (!is_wlan_server) {
			if (is_zperf_udp) {
				LOG_INF("Test case: wifi_tput_udp_client_ble_con_central");
			} else {
				LOG_INF("Test case: wifi_tp_tcp_client_ble_con_central");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF("Test case: wifi_tput_udp_server_ble_con_central");
			} else {
				LOG_INF("Test case: wifi_tp_tcp_server_ble_con_central");
			}
		}
	} else {
		if (!is_wlan_server) {
			if (is_zperf_udp) {
				LOG_INF("Test case: wifi_tput_udp_client_ble_con_peripheral");
			} else {
				LOG_INF("Test case: wifi_tp_tcp_client_ble_con_peripheral");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF("Test case: wifi_tput_udp_server_ble_con_peripheral");
			} else {
				LOG_INF("Test case: wifi_tput_tcp_server_ble_con_peripheral");
			}
		}
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		wifi_connection(test_wlan);
		if (ret != 0) {
			LOG_ERR("Wi-Fi connection failed. Running the test\
					further is not meaningful.So, exiting the test");
			goto err;
		}
		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(3)); /* B4 start. not in loop. no need to reduce */
		if (is_ble_central) {
			/** If BLE is central, disconnect the connection.
			 *Connection and disconnection happens in loop later.
			 */
			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates the connection, DUT is connected to peer central
			 *and update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}

	if (test_wlan) {
		if (is_zperf_udp) {
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
		if (is_wlan_server) {
			while (!wait4_peer_wifi_client_to_start_tp_test) {
				#ifdef CONFIG_PRINTS_FOR_AUTOMATION
				LOG_INF("start WiFi client");
				#endif
				k_sleep(K_SECONDS(1)); /* Before test start. in a loop. can be reducd to 100ms*/
			}
			wait4_peer_wifi_client_to_start_tp_test = 0;
		}
	}


	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if (is_ble_central) {
				ble_connection_attempt_cnt++;
				scan_start();
				k_sleep(KSLEEP_SCAN_START_1SEC);

				ble_disconnection_attempt_cnt++;
				bt_disconnect_central();
				k_sleep(KSLEEP_WHILE_DISCON_CENTRAL_2SEC);
			} else {
				if (!ble_periph_connected) {
					/* ble_connection_attempt_cnt and ble_disconnection_attempt_cnt to be on peer BLE */
					adv_start();
					k_sleep(KSLEEP_ADV_START_1SEC);
				}
				/** disconnection initiation in peripheral case is
				 * taken care by peer BLE in central role
				 */
			}
			if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1)); /* can be removed as other ksleeps  inside loop. */
		}
	}

	if (test_wlan) {
		check_wifi_traffic(test_wlan);
		wifi_disconnection(test_wlan);
	}

	#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		if (test_ble) {
			if (is_ble_central) {
				LOG_INF("ble_connection_attempt_cnt = %u",
					ble_connection_attempt_cnt);
				LOG_INF("ble_connection_success_cnt = %u",
					ble_connection_success_cnt-1);

				LOG_INF("ble_disconnection_attempt_cnt = %u",
					ble_disconnection_attempt_cnt);
				LOG_INF("ble_disconnection_success_cnt = %u",
					ble_disconnection_success_cnt-1);
				LOG_INF("ble_disconnection_fail_cnt = %u",
					ble_disconnection_fail_cnt);
				LOG_INF("ble_discon_no_conn_cnt = %u", ble_discon_no_conn_cnt);
			} else {
				/* counts for peripheral case are printed on peer BLE */
			}
		}
	#endif

	return 0;
err:
	return ret;
}


int wifi_tput_ble_tput(bool test_wlan, bool is_ant_mode_sep,
	bool test_ble, bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;

	if (is_ble_central) {
		if (!is_wlan_server) {
			if (is_zperf_udp) {
				LOG_INF(" Test case: wifi_tput_udp_client_ble_tput_central");
			} else {
				LOG_INF(" Test case: wifi_tput_tcp_client_ble_tput_central");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF(" Test case: wifi_tput_udp_server_ble_tput_central");
			} else {
				LOG_INF(" Test case: wifi_tput_tcp_server_ble_tput_central");
			}
		}

	} else {
		if (!is_wlan_server) {
			if (is_zperf_udp) {
				LOG_INF(" Test case: wifi_tput_udp_client_ble_tput_peripheral");
			} else {
				LOG_INF(" Test case: wifi_tput_tcp_client_ble_tput_peripheral");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF(" Test case: wifi_tput_udp_server_ble_tput_peripheral");
			} else {
				LOG_INF(" Test case: wifi_tput_tcp_server_ble_tput_peripheral");
			}
		}
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		wifi_connection(test_wlan);
		if (ret != 0) {
			LOG_ERR("Wi-Fi connection failed. Running the test\
					further is not meaningful. So, exiting the test");
			goto err;
		}
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
			goto err;
		}
		if (!is_wlan_server) {
			if (is_ble_central) {
				/* nothing */
			} else {
				if (test_wlan && test_ble) {
					while (!wait4_peer_ble2_start_connection) {
						#ifdef CONFIG_PRINTS_FOR_AUTOMATION
						LOG_INF("Run BLE central");
						#endif
						k_sleep(K_SECONDS(1)); /* Before test start. in a loop. no need to reduce */
					}
					wait4_peer_ble2_start_connection = 0;
				}
			}
		}
	}
	if (!is_wlan_server) {
		#ifdef DEMARCATE_TEST_START
		LOG_INF("-------------------------");
		#endif
	}
	if (test_wlan) {
		if (is_zperf_udp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}

		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
				goto err;
		}
		if (is_wlan_server) {
			while (!wait4_peer_wifi_client_to_start_tp_test) {
				#ifdef CONFIG_PRINTS_FOR_AUTOMATION
				LOG_INF("start WiFi client");
				#endif
				k_sleep(K_SECONDS(1)); /* Before test start. in a loop. can be reducd to 100ms*/
			}
			wait4_peer_wifi_client_to_start_tp_test = 0;
		}
	}
	if (is_wlan_server) {
		#ifdef DEMARCATE_TEST_START
		LOG_INF("-------------------------");
		#endif
	}

	/** If BLE role is peripheral, then the following .
	 * function doesnot start BLE traffic.Peer BLE that acts
	 *as central starts the traffic.
	 */

	start_ble_activity(test_ble, is_ble_central);

	check_wifi_traffic(test_wlan);

	/** If BLE role is peripheral, then the following function doesn't
	 * run BLE traffic. Peer BLE that acts as central starts the traffic.
	 */
	run_ble_activity(test_ble, is_ble_central);

	wifi_disconnection(test_wlan);

	exit_bt_throughput_test(test_ble, is_ble_central);

	return 0;
err:
	return ret;
}

int wifi_con_stability_ble_con_interference(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;
	//uint64_t test_start_time = 0;
	int ret = 0;

	if (is_ble_central) {
		LOG_INF("Test case: wifi_con_ble_con_central_stability");
	} else {
		LOG_INF("Test case: wifi_con_ble_con_peripheral_stability");
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	/* Wi-Fi connection done only once */
	if (test_wlan) {
		ret=wifi_connection(test_wlan);
		if (ret != 0) {
			LOG_ERR("Wi-Fi connection failed. Running the test\
				further is not meaningful. So, exiting the test");
			goto err;
		}
		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(2)); /* B4 start. not in loop. no need to reduce */
		if (is_ble_central) {
			/** If BLE is central, disconnect the connection.
			 *Connection and disconnection happens in loop later.
			 */
			bt_disconnect_central();
			k_sleep(K_SECONDS(2)); /* B4 start. not in loop. no need to reduce */
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates the connection, DUT is connected to peer central
			 * and update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif

	test_start_time = k_uptime_get_32();

	/** BLE connection and disconnection iteratively until test duration is complete */
	while (true) {
		if (test_ble) { /* For BLE only and Wi-Fi + BLE cases */
			if (is_ble_central) {
				if (!ble_central_connected) {
					scan_start();
					k_sleep(KSLEEP_SCAN_START_1SEC);
				}
				bt_disconnect_central();
				k_sleep(KSLEEP_WHILE_DISCON_CENTRAL_2SEC);
			} else {
				if (!ble_periph_connected) {
					adv_start();
					k_sleep(KSLEEP_ADV_START_1SEC);
				}
				/**If DUT acts as peripheral then BLE disconnection
				 *is initiated by peer central.
				 */
			}
		}

		if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));  /* can be removed as other ksleeps  inside loop. */
	}
	/* check Wi-Fi connection status. Disconnect if not disconnected already */
	if (test_wlan) {
		if (status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {
			LOG_INF("Wi-Fi connection intact");
			wifi_disconnection(test_wlan);
			k_sleep(KSLEEP_WIFI_DISCON_2SEC);
		}
	}

#ifdef CONFIG_PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_stability = %u", wifi_conn_cnt_stability);
		LOG_INF("wifi_disconn_cnt_stability = %u", wifi_disconn_cnt_stability);
	}
#endif
	/* Additional information related to BLE.*/
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);

		LOG_INF("BLE connection results may be ignored.\
			That provides information on whether BLE acted as interference");
	}
	return 0;
err:
	return ret;
}


int wifi_con_stability_ble_tput_interference(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	uint64_t test_start_time = 0;
	uint64_t wifi_con_intact_cnt = 0;

	if (is_ble_central) {
		LOG_INF("Test case: wifi_con_ble_tput_central_stability");
	} else {
		LOG_INF("Test case: wifi_con_ble_tput_peripheral_stability");
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		/* Wi-Fi connection done only once */
		#ifndef CHECK_WIFI_CONN_STATUS
			wifi_connection(test_wlan);
		#else
			ret = wifi_connection(test_wlan);
			k_sleep(K_SECONDS(3));
			if (ret != 0) {
				LOG_ERR("Wi-Fi connection failed. Running the test\
						further is not meaningful. So, exiting the test");
				goto err;
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
	}

	if (test_ble) {
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	/* BLE only and Wi-Fi+BLE tests : Start BLE interference */
	if (test_ble) {
		if (is_ble_central) {
			#ifdef DEMARCATE_TEST_START
			LOG_INF("-------------------------");
			#endif
			test_start_time = k_uptime_get_32();

			start_ble_activity(test_ble, is_ble_central);
		} else {
			while (!wait4_peer_ble2_start_connection) {
				/* Peer BLE starts the the test. */
				#ifdef CONFIG_PRINTS_FOR_AUTOMATION
				LOG_INF("Run BLE central");
				#endif
				k_sleep(K_SECONDS(1));
			}
			wait4_peer_ble2_start_connection = 0;
			#ifdef DEMARCATE_TEST_START
			LOG_INF("-------------------------");
			#endif
			test_start_time = k_uptime_get_32();
		}
	}

	/* Wait for test duration to complete if WLAN only case */
	if (test_wlan && !test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
		}
	}

	/* BLE only and Wi-Fi+BLE tests  */
	if (test_ble) {
		if (is_ble_central) {
			/* Wait for BLE traffic completion in case of central and exit */
			run_ble_activity(test_ble, is_ble_central);
			exit_bt_throughput_test(test_ble, is_ble_central);
		} else {
			while (true) {
				if ((k_uptime_get_32() - test_start_time) >
					CONFIG_COEX_TEST_DURATION) {
					break;
				}
				k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
			}
			/* Peer BLE does the disconnects in the case of peripheral */
		}
	}

	/* check Wi-Fi connection status. Disconnect if not disconnected already */
	if (test_wlan) {
		if (status.state < WIFI_STATE_ASSOCIATED) {
			LOG_INF("Wi-Fi disconnected");
		} else {
			LOG_INF("Wi-Fi connection intact");
			wifi_con_intact_cnt++;
			wifi_disconnection(test_wlan);
			k_sleep(KSLEEP_WIFI_DISCON_2SEC);
		}
	}

#ifdef CONFIG_PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_attempt_cnt = %u", wifi_conn_attempt_cnt);
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_con_intact_cnt = %u", wifi_con_intact_cnt);

		LOG_INF("BLE throughput results may be ignored.\
				That provides information on whether BLE acted as interference");
	}
#endif

	return 0;
err:
	return ret;
}

int ble_con_stability_wifi_scan_interference(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;
	uint64_t ble_con_intact_cnt = 0;
	int ret = 0;

	if (is_ble_central) {
		if (!is_wifi_conn_scan) {
			LOG_ERR("Test case: ble_conn_central_wifi_scan_stability");
		} else {
			LOG_ERR("Test case: ble_conn_central_wifi_conn_scan_stability");
		}
	} else {
		if (!is_wifi_conn_scan) {
			LOG_INF("Test case: ble_conn_peripheral_wifi_scan_stability");
		} else {
			LOG_INF("Test case: ble_conn_peripheral_wifi_conn_scan_stability");
		}
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	/* one time BT connection */
	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		ble_connection_attempt_cnt++;
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(2)); 
		if (is_ble_central) {
			/* nothing */
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates the connection, DUT is connected to peer central
			 * and update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}
	if (test_wlan) {
		if (is_wifi_conn_scan) {
			ret=wifi_connection(test_wlan);
			if (ret != 0) {
				LOG_ERR("Wi-Fi connection failed. Running the test\
						further is not meaningful. So, exiting the test");
				return ret;
			}
		}
		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	/* start Wi-Fi scan and continue for test duration */
	if (test_wlan) {
		cmd_wifi_scan();
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	/* wait for the Wi-Fi interferecne to cover for test duration */
	while (true) {
		if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
			break;
		}
		k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
	}

	/* stop the Wi-Fi iterative scan interference happening further */
	if (test_wlan) {
		repeat_wifi_scan = 0;
	}

	/* check BLE connection status */
	if (test_ble) {
		if (is_ble_central) {
			if (ble_central_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
			} else {
				LOG_INF("BLE disconnected");
			}
			bt_disconnect_central();
		} else {
			if (ble_periph_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
			} else {
				LOG_INF("BLE disconnected");
			}
		}
		k_sleep(K_SECONDS(2));
	}
#ifdef CONFIG_PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);		
		LOG_INF("ble_con_intact_cnt = %u", ble_con_intact_cnt);		
	}
#endif
	if (test_wlan) {
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);

		LOG_INF("Wi-Fi scan results may be ignored. That provides information\
				on whether Wi-Fi acted as interference");
	}

	return 0;
}

#if 0
int ble_con_stability_wifi_conn_interference(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;
	uint64_t ble_con_intact_cnt = 0;

	if (is_ble_central) {
		LOG_ERR("Test case: ble_conn_central_wifi_con_stability");
	} else {
		LOG_ERR("Test case: ble_conn_peripheral_wifi_con_stability");
	}

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (test_wlan) {
		#if defined(CONFIG_NRF700X_BT_COEX)
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	/* one time BT connection */
	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		ble_connection_attempt_cnt++;
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(2)); /* B4 start. not in loop. no need to reduce */
		if (is_ble_central) {
			/* nothing */
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates the connection, DUT is connected to peer central
			 * and update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	/* Wait for test duration to complete if BLE only case */
	if (!test_wlan && test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
		}
	}

	while (true) {
		/* provide Wi-Fi connect->disconnect interference */
		if (test_wlan) { /* For Wi-Fi only and Wi-Fi + BLE cases */
			wifi_connection(test_wlan);
			k_sleep(KSLEEP_WIFI_CON_2SEC);

			wifi_disconnection(test_wlan);
			k_sleep(KSLEEP_WIFI_DISCON_2SEC);
		}
		/* Wait for test duration to complete in BLE only , Wi-Fi and BLE cases */
		if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1)); /* can be removed as other ksleeps  inside loop. */
	}

	/* check BLE connection status */
	if (test_ble) {
		if (is_ble_central) {
			if (ble_central_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
				bt_disconnect_central();
				k_sleep(K_SECONDS(2));
			} else {
				LOG_INF("BLE disconnected");
			}
		} else {
			if (ble_periph_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
			} else {
				LOG_INF("BLE disconnected");
			}
		}
	}
#ifdef CONFIG_PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);		
		LOG_INF("ble_con_intact_cnt = %u", ble_con_intact_cnt);
	}
#endif
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);

		LOG_INF("Wi-Fi connection results may be ignored. That provides information\
				on whether Wi-Fi acted as interference");
	}
	return 0;
}
#endif

int ble_con_stability_wifi_tput_interference(bool test_wlan, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;
	uint64_t ble_con_intact_cnt = 0;

	print_common_test_params(is_ant_mode_sep, test_ble, test_wlan, is_ble_central);

	if (!is_wlan_server) {
		if (is_ble_central) {
			if (is_zperf_udp) {
				LOG_INF("Test case:\
					ble_conn_central_wifi_tput_udp_client_stability");
			} else {
				LOG_INF("Test case:\
					ble_conn_central_wifi_tput_tcp_client_stability");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF("Test case:\
					ble_conn_peripheral_wifi_tput_udp_client_stability");
			} else {
				LOG_INF("Test case:\
					ble_conn_peripheral_wifi_tput_tcp_client_stability");
			}
		}
	} else {

		if (is_ble_central) {
			if (is_zperf_udp) {
				LOG_INF("Test case:\
					ble_conn_central_wifi_tput_udp_server_stability");
			} else {
				LOG_INF("Test case:\
					ble_conn_central_wifi_tput_tcp_server_stability");
			}
		} else {
			if (is_zperf_udp) {
				LOG_INF("Test case:\
					ble_conn_peripheral_wifi_tput_udp_server_stability");
			} else {
				LOG_INF("Test case:\
					ble_conn_peripheral_wifi_tput_tcp_server_stability");
			}
		}
	}

	/* one time BT connection */
	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		ble_connection_attempt_cnt++;
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(2)); /* B4 start. not in loop. no need to reduce */
		if (is_ble_central) {
			/* nothing */
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates theconnection, DUT is connected to peer central and
			 * update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}

	if (test_wlan) {
		#ifndef CHECK_WIFI_CONN_STATUS
			wifi_connection(test_wlan);
		#else
			ret = wifi_connection(test_wlan);
			k_sleep(K_SECONDS(3));
			if (ret != 0) {
				LOG_ERR("Wi-Fi connection failed. Running the test\
						further is not meaningful. So, exiting the test");
				goto err;
			}
		#endif

		#if defined(CONFIG_NRF700X_BT_COEX)
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	/* start Wi-Fi throughput interference */
	if (test_wlan) {
		if (is_zperf_udp) {
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
		if (is_wlan_server) {
			while (!wait4_peer_wifi_client_to_start_tp_test) {
				#ifdef CONFIG_PRINTS_FOR_AUTOMATION
				LOG_INF("start peer WiFi client");
				#endif
				k_sleep(K_SECONDS(1)); /* Before test start. in a loop. can be reducd to 100ms*/
			}
			wait4_peer_wifi_client_to_start_tp_test = 0;
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	/* BLE only case    : continue until test duration is complete */
	/* BLE + Wi-Fi cases: continue Wi-Fi interference until test duration is complete */
	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
		}
	}

	if (test_wlan) {
		check_wifi_traffic(test_wlan);
		wifi_disconnection(test_wlan);
	}

	/* check BLE connection status */
	if (test_ble) {
		if (is_ble_central) {
			if (ble_central_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
			} else {
				LOG_INF("BLE disconnected");
			}
			bt_disconnect_central();
			k_sleep(K_SECONDS(2));
		} else {
			if (ble_periph_connected) {
				LOG_INF("BLE Conn Intact");
				ble_con_intact_cnt++;
			} else {
				LOG_INF("BLE disconnected");
			}
		}
	}
	#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		if (test_ble) {
			LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
			LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);		
			LOG_INF("ble_con_intact_cnt = %u", ble_con_intact_cnt);
		}
	#endif

	LOG_INF("Wi-Fi throughput results may be ignored.\
			That provides information on whether Wi-Fi acted as interference");

	return 0;
err:
	return ret;
}


#if 0
int ble_con_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);

	if (is_ble_central) {
		LOG_INF("Test case: ble_con_central_wifi_shutdown");
	} else {
		LOG_INF("Test case: ble_con_peripheral_wifi_shutdown");
	}

	LOG_INF("Test duration in milliseconds: %d", CONFIG_COEX_TEST_DURATION);
	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesn't post requests to PTA");
	}

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();

	/* BLE onetime connection */
	if (test_ble) {
		/* Initialize BLE by selecting role and connect it to peer device. */
		bt_connection_init(is_ble_central);
		k_sleep(K_SECONDS(2));
		if (is_ble_central) {
			/** If BLE is central, disconnect the connection.
			 *Connection and disconnection happens in loop later.
			 */
			bt_disconnect_central();
			k_sleep(K_SECONDS(3));
		} else {
			/**If BLE is peripheral, wait until peer BLE central
			 * initiates the connection, DUT is connected to peer central
			 * and update the PHY parameters.
			 */
			while (true) {
				if (ble_periph_connected) {
					break;
				}
				k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
			}
		}
	}


	 #ifdef DEMARCATE_TEST_START
	LOG_INF("-------------------------");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		ble_connection_attempt_cnt++;
		scan_start();
		while (true) {
			if (is_ble_central) {				
				if (ble_central_connected) {
					ble_disconnection_attempt_cnt++;
					bt_disconnect_central();
				}
			} 
			if ((k_uptime_get_32() - test_start_time) > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(3)); // this can be commented 
		}
	} 

#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		if (is_ble_central) {
			LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
			LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt-1);


			LOG_INF("ble_disconnection_attempt_cnt = %u",
					ble_disconnection_attempt_cnt);
			LOG_INF("ble_disconnection_success_cnt = %u",
					ble_disconnection_success_cnt-1);
			LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
			LOG_INF("ble_discon_no_conn_cnt = %u", ble_discon_no_conn_cnt);
		} else {
			LOG_INF("BLE result counts for peripheral case are printed on peer BLE");
		}
#endif

	return 0;
}
#endif 

int ble_con_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);

	if (is_ble_central) {
		LOG_INF("Test case: ble_con_central_wifi_shutdown");
	} else {
		LOG_INF("Test case: ble_con_peripheral_wifi_shutdown");
	}

	LOG_INF("Test duration in milliseconds: %d", CONFIG_COEX_TEST_DURATION);
	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesn't post requests to PTA");
	}

	/* disable RPU i.e. Wi-Fi shutdown */
	//rpu_disable();

	/* Initialize BLE by selecting role and connect it to peer device. */
	ble_connection_attempt_cnt++;
	bt_connection_init(is_ble_central);

	if (is_ble_central) {
		/**If BLE is central, disconnect the connection.
		 * Connection and disconnection happens in loop later.
		 */
		ble_disconnection_attempt_cnt++;
			bt_disconnect_central();
	} else {
		/**If BLE is peripheral, wait until peer BLE central
		 *  initiates the connection, DUT is connected to peer central
		 *  and update the PHY parameters.
		 */
		while (true) {
			if (ble_periph_connected) {
				break;
			}
			k_sleep(KSLEEP_WHILE_PERIP_CONN_CHECK_1SEC);
		}
	}


	#ifdef CONFIG_PRINTS_FOR_AUTOMATION
		/* peer central waits on this in automation */
		LOG_INF("Run BLE central");
	#endif

	if (!is_ble_central) {
		LOG_INF("DUT is in peripheral role, Check for BLE connection\
				counts on peer BLE side");
	}

	#ifdef DEMARCATE_TEST_START
		LOG_INF("-------------------------");
	#endif
	
	test_start_time = k_uptime_get_32();

	/* Begin BLE conections and disconnections for a period of BLE test duration */

	/* This is dummy function in case of peripehral */
	start_ble_activity(test_ble, is_ble_central);

	/* Wait for BLE activity completion i.e., for COEX_TEST_DURATION */

	/* This is dummy function in case of peripehral */
	run_ble_activity(test_ble, is_ble_central);
	/* If DUT BLE is in peripheral role wait for COEX_TEST_DURATION */
	if(!is_ble_central) {
		while(1) {
			if (k_uptime_get_32() - test_start_time > CONFIG_COEX_TEST_DURATION) {
				break;
			}
			k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
		}
	}

	
	#ifdef CONFIG_PRINTS_FOR_AUTOMATION

	if (is_ble_central) {
		LOG_INF("ble_connection_attempt_cnt = %u",
			ble_connection_attempt_cnt);
		LOG_INF("ble_connection_success_cnt = %u",
			ble_connection_success_cnt);


		LOG_INF("ble_disconnection_attempt_cnt = %u",
			ble_disconnection_attempt_cnt);
		LOG_INF("ble_disconnection_success_cnt = %u",
			ble_disconnection_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u",
			ble_disconnection_fail_cnt);
		LOG_INF("ble_discon_no_conn_cnt = %u",
			ble_discon_no_conn_cnt);
	} else {
		LOG_INF("check peer device for result counts");
		LOG_INF("Counts printed below are for information purpose\
		and not actual results.");

		LOG_INF("ble_le_datalen_failed = %u",
			ble_le_datalen_failed);
		LOG_INF("ble_phy_update_failed = %u",
			ble_phy_update_failed);
		LOG_INF("ble_le_datalen_timeout = %u",
			ble_le_datalen_timeout);

		LOG_INF("ble_phy_update_timeout = %u",
			ble_phy_update_timeout);
		LOG_INF("ble_conn_param_update_failed = %u",
			ble_conn_param_update_failed);
		LOG_INF("ble_conn_param_update_timeout = %u",
			ble_conn_param_update_timeout);
	}
	#endif
	err:
		return 0;
}

int ble_tput_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;
	int ret = 0;
	bool ble_coex_enable = IS_ENABLED(CONFIG_MPSL_CX);

	if (is_ble_central) {
		LOG_INF("Test case: ble_tput_central_wifi_shutdown");
	} else {
		LOG_INF("Test case: ble_tput_periph_wifi_shutdown");
	}

	LOG_INF("Test duration in milliseconds: %d", CONFIG_COEX_TEST_DURATION);

	if (ble_coex_enable) {
		LOG_INF("BLE posts requests to PTA");
	} else {
		LOG_INF("BLE doesn't post requests to PTA");
	}

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();

	if (test_ble) {

		if (!is_ble_central) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3));
		}
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}

		#ifdef DEMARCATE_TEST_START
		LOG_INF("");
		LOG_INF("-------------------------");
		LOG_INF("");
		#endif
		test_start_time = k_uptime_get_32();

		/* start BLE throughput */
		start_ble_activity(test_ble, is_ble_central);
		/* Waits for the test duration in the case of BLE central */
		run_ble_activity(test_ble, is_ble_central);

		/* Wait for test duration in the case of BLE peripheral */
		if (!is_ble_central) {
			while (true) {
				if (k_uptime_get_32() - test_start_time >
					CONFIG_COEX_TEST_DURATION) {
					break;
				}
				k_sleep(KSLEEP_WHILE_ONLY_TEST_DUR_CHECK_1SEC);
			}
		}
		exit_bt_throughput_test(test_ble, is_ble_central);
	}
	return 0;
err:
	return ret;
}
