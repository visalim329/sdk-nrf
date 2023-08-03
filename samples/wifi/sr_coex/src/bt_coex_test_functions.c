/**
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Wi-Fi and Bluetooth LE coexistence test functions
 */

#include "bt_coex_test_functions.h"

int8_t wifi_rssi = 127;


static uint32_t wifi_scan_done = 0;

static uint32_t wait_to_start_next_scan = 0;
static uint32_t start_wlan_scan = 0;
static uint32_t wfi_scan_result_cnt = 0;

static uint32_t max_loop_cnt = 2;
static uint32_t num_scan_req_done = 0;

static int print_first_time;

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
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
			LOG_INF("Connected");
		}
		context.connected = true;
	}
	if (print_first_time==0) {	
	//	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
		cmd_wifi_status();
		print_first_time++;
	//}
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
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
		cmd_wifi_status();
	}
}

uint64_t scan_start_time = 0;
uint64_t scan_time = 0;
		
void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{

//scan_time = k_uptime_get_32() - scan_start_time;
//LOG_INF("scan_time=%d", scan_time);

	//LOG_INF("in handle_wifi_scan_result()");
	k_sleep(K_MSEC(100));
		
	wfi_scan_result_cnt++;
	
	const struct wifi_scan_result *entry =
			(const struct wifi_scan_result *)cb->info;
	/* uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")]; */

	scan_result_count++;

	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_RESULT)) {
		LOG_INF("%-4d | %-12s | %-4u  | %-4d",
			scan_result_count, entry->ssid, entry->channel, entry->rssi);
		/* To avoid message drops while printing Wi-Fi scan results */
		k_sleep(K_MSEC(1));

		LOG_INF("Wi-Fi scan done for %d times", scan_result_count);
	}

	if (entry->channel <= HIGHEST_CHANNUM_24G) {
		wifi_scan_cnt_24g++;
	} else {
		wifi_scan_cnt_5g++;
	}
#if 0 // this is done as part of scan done
	if((wfi_scan_result_cnt%2)==0) {
		if (repeat_scan == 1) {			
				//LOG_INF("starting wifi scan");
				//k_sleep(K_SECONDS(1)); // to be removed
				wifi_scan_cmd_cnt++;
	
				//scan_start_time = k_uptime_get_32();			
				cmd_wifi_scan();
		}
	}		
#endif
							//	if((wfi_scan_result_cnt%2)==0) {
							//		if (repeat_scan == 1) {			
							//			if(wait_to_start_next_scan==1){
							//				while(1) {
							//					if(start_wlan_scan){
							//						start_wlan_scan =0;
							//						break;
							//					}				
							//				}
							//			}
							//			//if(num_scan_req_done<=max_loop_cnt){
							//				LOG_INF("starting wifi scan");
							//				k_sleep(K_SECONDS(3));
							//				wifi_scan_cmd_cnt++;
							//				cmd_wifi_scan();
							//				wifi_scan_cmd_cnt++;
							//			//}
							//			
							//		} else	{
							//			/* cmd_wifi_scan(); */ /* should not call Wi-Fi scan */
							//			wifi_scan_cmd_cnt++;
							//		}
							//	}		
}

void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
	/* scan_time = k_uptime_get_32() - scan_start_time;
	LOG_INF("scan_time=%d",scan_time);
	scan_start_time = k_uptime_get_32(); */
	wifi_scan_cmd_cnt++;
	cmd_wifi_scan();
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_RESULT)) {
		const struct wifi_status *status =
			(const struct wifi_status *)cb->info;

		if (status->status) {
			LOG_ERR("Scan request failed (%d)", status->status);
		} else {
			LOG_INF("Scan request done");
		}
		k_sleep(K_MSEC(1));
	}

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
	
	if (IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_RESULT)) {
	LOG_INF("IP address: %s", dhcp_info);
	}
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
	
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_SCAN_RESULT)) {
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

	LOG_INF("Wi-Fi peer IPv4 address %s", host);

	return 0;
}

int wait_for_next_event(const char *event_name, int timeout)
{
	int wait_result;
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
		if (event_name) {
			LOG_INF("Waiting for %s", event_name);
		}
	}
	wait_result = k_sem_take(&wait_for_next, K_SECONDS(timeout));
	if (wait_result) {
		LOG_ERR("Timeout waiting for %s -> %d", event_name, wait_result);
		return -1;
	}
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
		LOG_INF("Got %s", event_name);
	}
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
		LOG_INF("\nrate in kbps:%u kbps", client_rate_in_kbps);
		k_sem_give(&udp_tcp_callback);
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
		LOG_INF("\nrate in kbps:%u kbps", rate_in_kbps);
		k_sem_give(&udp_tcp_callback);
		
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
#if 1
void run_bt_conn_benchmark(void)
{
	//bool is_ble_central=ble_mode;
	bt_conn_test_run();
}
#endif
#if 1
void run_wifi_activity_benchmark(void)
{
	wifi_scan_test_run();
}
#endif
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
#if 1
void start_ble_activity(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
	/*  In case BLE is peripheral, skip running BLE traffic */
		#ifdef CONFIG_WIFI_SCAN_BLE_CON_CENTRAL || CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL ||\
		CONFIG_WIFI_CON_BLE_CON_CENTRAL || CONFIG_WIFI_SCAN_BLE_CON_PERIPH || CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH
			/* Start BLE connection */
			LOG_INF("starting BT Connection");
			k_thread_start(run_bt_connection);
		#endif 
		#ifdef CONFIG_WIFI_SCAN_BLE_TP_CENTRAL || CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL ||\
		CONFIG_WIFI_CON_BLE_TP_CENTRAL 
			/* Start BLE traffic */
			LOG_INF("starting BT traffic");
			k_thread_start(run_bt_traffic);
		#endif 

	}
}
#endif 

#if 0
void start_ble_activity(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (is_ble_central) {
			/* Start BLE connection */
			k_thread_start(run_bt_connection);
		}
	}
}
#endif

#if 1
void start_wifi_activity()
{
	k_thread_start(run_wlan_scan);
}
#endif

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
			
		#ifdef CONFIG_WIFI_SCAN_BLE_CON_CENTRAL || CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL ||\
		CONFIG_WIFI_CON_BLE_CON_CENTRAL || CONFIG_WIFI_SCAN_BLE_CON_PERIPH || CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH
		/* Run BLE connection */
		k_thread_join(run_bt_connection, K_FOREVER);
		#endif 
		#ifdef CONFIG_WIFI_SCAN_BLE_TP_CENTRAL || CONFIG_WIFI_CON_SCAN_BLE_TP_CENTRAL ||\
		CONFIG_WIFI_CON_BLE_TP_CENTRAL
		/* Run BLE traffic */
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (is_ble_central) {
			k_thread_join(run_bt_traffic, K_FOREVER);
		}
		#endif

	}
}
#if 0
void run_ble_activity(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/*  In case BLE is peripheral, skip running BLE traffic */
		if (is_ble_central) {
			/* Run BLE traffic */
			k_thread_join(run_bt_connection, K_FOREVER);
		}
	}
}
#endif

#if 1
void run_wifi_activity()
{

	k_thread_join(run_wlan_scan, K_FOREVER);

}
#endif

void disconnect_wifi(bool test_wlan)
{
	if (test_wlan) {
		/* Wi-Fi disconnection */
	if(IS_ENABLED(CONFIG_DEBUG_PRINT_WIFI_CONN_RESULT)) {
		LOG_INF("Disconnecting Wi-Fi");
	}
		wifi_disconnect();
	}
}

void disconnect_ble(bool test_ble, bool is_ble_central)
{
	if (test_ble) {
		/* BLE disconnection only  if role is central */
		if (is_ble_central) {
			LOG_INF("Disconnecting BLE");
			bt_throughput_test_exit();
		}
	}
}
int wifi_connection(bool test_wlan,	bool is_ant_mode_sep)
{
	int ret = 0;

	if (test_wlan) {
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
 *int ble_connection(bool test_ble, bool is_ble_central)
 *{
 *	int ret = 0;
 *
 *	if (test_ble) {
 *		ret = bt_throughput_test_init(is_ble_central);
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
 *int ble_connection_central(bool test_ble, bool is_ble_central)
 *{
 *	int ret = 0;
 *
 *	if (test_ble) {
 *		ret = bt_scan_test(is_ble_central);
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
int config_pta(bool is_ant_mode_sep, bool is_ble_central,
			bool is_wlan_server)
{
	int ret = 0;
	enum nrf_wifi_pta_wlan_op_band wlan_band =
			wifi_mgmt_to_pta_band(status.band);

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



/* Inside loop: BLE connection starts first, WLAN PTI window next. */
//int wifi_scan_ble_conn_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
//		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
//{
//	uint64_t test_start_time = 0;
//
//	uint32_t device_req_window = NRF_WIFI_WIFI_DEVICE;
//	uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
//	uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
//	uint32_t can_be_deferred = NRF_WIFI_NO;
//	uint32_t wlan_pti_window_duration = 2000;
//
//	uint32_t loop_cnt = 1;
//	
//	#ifdef CONFIG_NRF700X_BT_COEX
//		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
//	#endif/* CONFIG_NRF700X_BT_COEX */
//
//	
//	#ifdef DEMARCATE_TEST_START
//	LOG_INF("----------------------------------------------------------------");
//	#endif
//	
//	if (test_ble) {
//		/* BLE connection */
//		bt_connection_init(is_ble_central);
//		
//		/* BLE disconnection */
//		bt_disconnect_central();
//		k_sleep(K_SECONDS(2));
//			
//		for(loop_cnt=1; loop_cnt<=4; loop_cnt++) {
//			LOG_INF("----------------------------------");
//			LOG_INF("");
//			
//			window_start_or_end = NRF_WIFI_START_REQ_WINDOW; 
//			
//			/* Try BLE connection */
//			scan_start();
//			
//			k_sleep(K_MSEC(1000)); // for stable BLE connection
//			
//			/* start a single pti window for WLAN */
//			nrf_wifi_coex_allocate_spw(device_req_window,
//				window_start_or_end, imp_of_request, can_be_deferred);
//			
//			k_sleep(K_MSEC(wlan_pti_window_duration));
//								
//			
//			/* check  BLE connection status */
//			if (ble_central_connected) {
//				LOG_INF("BLE Conn Intact");
//			} else {
//				LOG_INF("BLE disconnected");
//			}
//
//			/* end request window */
//			window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
//			nrf_wifi_coex_allocate_spw(device_req_window,
//				window_start_or_end, imp_of_request, can_be_deferred);
//			
//			/* BLE disconnection */
//			bt_disconnect_central();
//			//k_sleep(K_SECONDS(2)); // commented as part of debug	
//		}		
//	}
//	#ifdef PRINTS_FOR_AUTOMATION
//		 if (test_ble) {
//			LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
//			LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
//		} 
//	#endif
//	return 0;
//}


//	/* In side loop: WLAN PTI window first, BLE connection next. */
//	int wifi_scan_ble_conn_central( bool is_ant_mode_sep, bool test_ble, bool test_wlan,
//	    bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
//	{
//		uint64_t test_start_time = 0;
//
//		uint32_t device_req_window = NRF_WIFI_WIFI_DEVICE;
//		uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
//		uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
//		uint32_t can_be_deferred = NRF_WIFI_NO;
//		uint32_t wlan_pti_window_duration = 2000;
//		uint32_t coex_hw_conf_time = 100;
//			
//
//		uint32_t loop_cnt = 1;
//		
//		uint32_t ble_supervision_timeout = 1000;
//		
//		#ifdef CONFIG_NRF700X_BT_COEX
//			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
//		#endif/* CONFIG_NRF700X_BT_COEX */
//
//		
//		#ifdef DEMARCATE_TEST_START
//		LOG_INF("");
//		LOG_INF("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
//		LOG_INF("");
//		#endif
//		
//		if (test_ble) {
//			/* BLE connection */
//			bt_connection_init(is_ble_central);
//			
//			/* BLE disconnection */
//			bt_disconnect_central();
//			k_sleep(K_SECONDS(2));
//			
//			
//			for(loop_cnt=1; loop_cnt<=8; loop_cnt++) {
//				LOG_INF("----------------------------------");				
//
//				window_start_or_end = NRF_WIFI_START_REQ_WINDOW; 
//					
//				/* start a single pti window for WLAN */
//				#if 1
//					nrf_wifi_coex_allocate_spw(device_req_window,
//						window_start_or_end, imp_of_request, can_be_deferred);
//					k_sleep(K_MSEC(coex_hw_conf_time)); // for coex HW config to take place 
//				#else
//					LOG_INF("30sec wait time to configure priority window from codescape");		
//					k_sleep(K_SECONDS(30));
//				#endif
//				
//				
//				// PTI window ready for WLAN
//				
//				/* Try BLE connection */
//				scan_start();						
//				
//				k_sleep(K_MSEC(wlan_pti_window_duration)); //scan duration
//									
//			
//				/* end request window */
//				window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
//				nrf_wifi_coex_allocate_spw(device_req_window,
//					window_start_or_end, imp_of_request, can_be_deferred);
//				
//				if (ble_supervision_timeout>=wlan_pti_window_duration) {
//				
//					k_sleep(K_MSEC(ble_supervision_timeout-wlan_pti_window_duration)); // for BLE to try connection in the absence of WLAN pti  window
//				}
//				
//				/* BLE disconnection */
//				bt_disconnect_central();
//				//k_sleep(K_SECONDS(2)); // commented as part of debug	
//			}		
//		}
//		#ifdef PRINTS_FOR_AUTOMATION
//			 if (test_ble) {
//				LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt-1);
//				LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
//			} 
//		#endif
//		return 0;
//	}


//	/* In side loop: WLAN PTI window first, BLE connection next. */
//	int wifi_scan_ble_conn_central( bool is_ant_mode_sep, bool test_ble, bool test_wlan,
//		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
//	{
//		uint64_t test_start_time = 0;
//
//		uint32_t device_req_window = NRF_WIFI_WIFI_DEVICE;
//		uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
//		uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
//		uint32_t can_be_deferred = NRF_WIFI_NO;
//		uint32_t wlan_pti_window_duration = 1500;
//		uint32_t coex_hw_conf_time = 100;
//			
//
//		uint32_t loop_cnt = 1;
//		
//		uint32_t ble_supervision_timeout = 1000;
//		
//		#ifdef CONFIG_NRF700X_BT_COEX
//			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
//		#endif/* CONFIG_NRF700X_BT_COEX */
//
//		
//		#ifdef DEMARCATE_TEST_START
//		LOG_INF("----------------------------------------------------------------");
//		#endif
//		
//		if (test_ble) {
//			/* BLE connection */
//			bt_connection_init(is_ble_central);
//			
//			/* BLE disconnection */
//			bt_disconnect_central();
//			k_sleep(K_SECONDS(2));
//				
//			for(loop_cnt=1; loop_cnt<=4; loop_cnt++) {
//				LOG_INF("----------------------------------");				
//				
//				window_start_or_end = NRF_WIFI_START_REQ_WINDOW; 
//				
//				/* start a single pti window for WLAN */
//				nrf_wifi_coex_allocate_spw(device_req_window,
//					window_start_or_end, imp_of_request, can_be_deferred);
//				k_sleep(K_MSEC(coex_hw_conf_time)); // for coex HW config to take place 
//				
//				// PTI window ready for WLAN
//				
//				/* Try BLE connection */
//				scan_start();						
//				
//				k_sleep(K_MSEC(wlan_pti_window_duration));
//									
//			
//				/* end request window */
//				window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
//				nrf_wifi_coex_allocate_spw(device_req_window,
//					window_start_or_end, imp_of_request, can_be_deferred);
//				
//				if (ble_supervision_timeout>=wlan_pti_window_duration) {
//				
//					k_sleep(K_MSEC(ble_supervision_timeout-wlan_pti_window_duration)); // for BLE to try connection in the absence of WLAN pti  window
//				}
//				
//				/* BLE disconnection */
//				bt_disconnect_central();
//				//k_sleep(K_SECONDS(2)); // commented as part of debug	
//			}		
//		}
//		#ifdef PRINTS_FOR_AUTOMATION
//			 if (test_ble) {
//				LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
//				LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
//			} 
//		#endif
//		return 0;
//	}



//	int wifi_scan_ble_conn_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
//			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
//	{
//		uint64_t test_start_time = 0;	
//
//
//		uint32_t device_req_window = NRF_WIFI_WIFI_DEVICE;
//		uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
//		uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
//		uint32_t can_be_deferred = NRF_WIFI_NO;
//		
//		uint32_t end_wifi_pti_window = 0;
//		uint32_t loop_cnt = 1;
//		wait_to_start_next_scan=1; /* used in outside this function */
//
//
//		if (test_wlan) {
//			#ifdef CONFIG_NRF700X_BT_COEX
//				config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
//			#endif/* CONFIG_NRF700X_BT_COEX */
//
//
//			if (!test_ble) {		
//				repeat_scan = 0;   // To avoid Wi-Fi scan happening outside this loop
//				if (is_wifi_conn_scan) {
//					wifi_connection(test_wlan, is_ant_mode_sep);
//				}
//				for(loop_cnt=1; loop_cnt<=max_loop_cnt; loop_cnt++) {
//						cmd_wifi_scan();
//						/* to make sure Wi-Fi scan is complere when repeat_scan = 0 */
//						k_sleep(K_SECONDS(5));
//				}
//			} else {
//				/* nothing. Both WLAN and BLE are taken care below */
//			}
//		}
//		#ifdef DEMARCATE_TEST_START
//		LOG_INF("----------------------------------------------------------------");
//		k_sleep(K_SECONDS(3));
//		#endif
//		
//
//		if (test_ble) {
//			
//			/* BT init and connection */
//			bt_connection_init(is_ble_central);
//			/* BLE disconnection */
//			bt_disconnect_central();
//			//LOG_INF("BLE onetime init, connection and disconnection done");
//			//k_sleep(K_SECONDS(5));
//			if (test_wlan) {
//				if (is_wifi_conn_scan) {
//					wifi_connection(test_wlan, is_ant_mode_sep);
//				}
//			}
//			
//	//==========================
//			repeat_scan = 1; 
//	//==========================
//			test_start_time = k_uptime_get_32();
//						
//			if (test_wlan) {
//				#if 1
//					LOG_INF("Calling Wi-Fi scan");
//					k_sleep(K_SECONDS(3));
//					cmd_wifi_scan();
//					
//					
//
//					//LOG_INF("Waiting on test time completion");
//					//k_sleep(K_SECONDS(3));	
//					//while (true) { // 
//					//	LOG_INF("1111");
//					//	if ((k_uptime_get_32() - test_start_time) >
//					//		CONFIG_WIFI_TEST_DURATION) {
//					//		break;
//					//	}
//					//	k_sleep(K_SECONDS(1));	
//					//}
//
//
//					end_wifi_pti_window = 0;				
//				#else
//					wifi_scan_done=1; /* to avoid Wi-Fi dependency in the case Wi-Fi scan is not called */
//					/* start request window */
//					window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
//					nrf_wifi_coex_allocate_spw(device_req_window,
//					window_start_or_end, imp_of_request, can_be_deferred);			
//					end_wifi_pti_window = 1;
//				#endif
//			} else {
//				// bug fix
//				wifi_scan_done=1; /* to avoid Wi-Fi dependency in the case of BLE only test */
//			}	
//			
//			for (loop_cnt = 1; loop_cnt <= max_loop_cnt; loop_cnt++) {
//
//				/* BLE connection */
//				LOG_INF("Calling BLE connection for %d time",loop_cnt);
//				k_sleep(K_SECONDS(2));
//				scan_start();
//	 
//				while (1) {				
//					LOG_INF("check for wifi scan done");
//					k_sleep(K_SECONDS(2));
//					//if (wifi_scan_done) {
//						/* this check is to make sure wifi_scan_done set to 1 is not 
//						disturbed in case of BLE only */
//						if (test_wlan) { //bug fix for BLE only
//							#if 1 // working version with wifi scan
//								LOG_INF("wifi_scan_done is 1");
//								k_sleep(K_SECONDS(5));
//							#else
//								/* For BLE connection to happen in case of BLE only. */
//								//bug fix for BLE only
//								k_sleep(K_SECONDS(2)); // without this, BLE connection is 1 for loop 2
//							#endif 
//							
//							wifi_scan_done=0;
//						}
//
//						/* end request window */
//						if (end_wifi_pti_window) {
//							window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
//							nrf_wifi_coex_allocate_spw(device_req_window,
//								window_start_or_end, imp_of_request, can_be_deferred);
//						}
//						if (ble_central_connected) { // check if BLE connected
//							bt_disconnect_central();
//						}
//						if (test_wlan) { 
//							LOG_INF("sleep of 3sec before Wi-Fi next scan and BLE next connection");
//							k_sleep(K_SECONDS(3));
//							start_wlan_scan =1;
//						} else {
//							LOG_INF("sleep of 3sec before BLE next connection");
//							k_sleep(K_SECONDS(3));
//						}
//						
//						break;			
//						
//					//}
//				}
//				
//			}
//				
//		}
//
//		if (test_wlan) {
//			while (true) {
//				if ((k_uptime_get_32() - test_start_time) >
//					(CONFIG_WIFI_TEST_DURATION+10000)) {
//					break;
//				}
//				k_sleep(K_SECONDS(1));
//				/* LOG_INF("waiting"); */
//			}
//			repeat_scan = 0;
//		}
//		#ifdef PRINTS_FOR_AUTOMATION
//			if (test_ble) {
//				LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt-1); // minus one considering BLE conn + disconnect outside loop 
//				
//				LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
//			}
//			if (test_wlan) {
//				LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
//				LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
//				LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
//			}
//		#endif
//		LOG_INF(" wifi_scan_ble_conn_central complete");
//		return 0;
//	}


#ifdef ALLOC_WLAN_PTI_WINDOW
uint32_t device_req_window = NRF_WIFI_WIFI_DEVICE;
uint32_t window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
uint32_t imp_of_request = NRF_WIFI_HIGHEST_IMPORTANCE;
uint32_t can_be_deferred = NRF_WIFI_NO;
uint32_t wlan_pti_window_duration = 2000;
uint64_t test_start_time = 0;	
#endif 
#if 0
int wifi_scan_ble_conn_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_ble) {
		/* BT init and connection */
		bt_connection_init(is_ble_central);
		/* BLE disconnection */
		bt_disconnect_central();
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;	
	
	start_wifi_activity();
	start_ble_activity(test_ble, is_ble_central);
	run_wifi_activity();
	#ifndef ALLOC_WLAN_PTI_WINDOW
	run_ble_activity(test_ble, is_ble_central);
	#else
		uint32_t loop_count=1;
		for (loop_count=1; loop_count<=30; loop_count++) {
			
			window_start_or_end = NRF_WIFI_START_REQ_WINDOW;
			nrf_wifi_coex_allocate_spw(device_req_window,
				window_start_or_end, imp_of_request, can_be_deferred);
			
			test_start_time = k_uptime_get_32();
			
			while (true) {			
				if ((k_uptime_get_32() - test_start_time) > 1200)
				{
					break;
				}
				k_sleep(K_MSEC(50));
			}
			
			window_start_or_end = NRF_WIFI_END_REQ_WINDOW;
			nrf_wifi_coex_allocate_spw(device_req_window,
				window_start_or_end, imp_of_request, can_be_deferred);
			k_sleep(K_MSEC(100));
		}
	
		run_ble_activity(test_ble, is_ble_central);
	#endif	

	repeat_scan = 0;

	#ifdef PRINTS_FOR_AUTOMATION
		 if (test_ble) {
			LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
			LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt-1);			
			LOG_INF("ble_connection_fail_cnt = %u", ble_connection_fail_cnt);
			
			LOG_INF("ble_disconnection_attempt_cnt = %u", ble_disconnection_attempt_cnt);
			LOG_INF("ble_disconnection_success_cnt = %u", ble_disconnection_success_cnt-1);		
			LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
			LOG_INF("ble_discon_no_conn_cnt = %u", ble_discon_no_conn_cnt);			
		} 
		if (test_wlan) {
			LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
			LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
			LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
		}

	#endif
	
	LOG_INF("ble_supervision_timeout = %u", ble_supervision_timeout);
	
	
	return 0;
}
#endif

int wifi_scan_ble_conn_test(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
	bool is_ble_central,	bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;
	LOG_INF("In wifi_scan_ble_conn_test");

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_ble) {
		/* BT init and connection */
		bt_connection_init(is_ble_central);
		if(is_ble_central) {
			/* BLE disconnection */
			bt_disconnect_central();
		} else {
			while (true) {
				if (ble_periph_connected) {
					break;
				}
			}
		}
	}
	
	#ifdef DEMARCATE_TEST_START
		LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;	
	uint32_t stamp= k_uptime_get_32();
	
	if (test_wlan) {
		start_wifi_activity();
	}
	if (test_ble) {
		start_ble_activity(test_ble, is_ble_central);
	}
	if (test_wlan) {
		run_wifi_activity();
	}
	if (test_ble) {
		run_ble_activity(test_ble, is_ble_central);
		if(!is_ble_central) {
			while(1) {
				if (k_uptime_get_32() - stamp > CONFIG_BLE_TEST_DURATION) {
					break;
				}
				k_sleep(K_SECONDS(1));
			}
		}
	}
	if (test_wlan) {
		if (is_wifi_conn_scan) {
			disconnect_wifi(test_wlan);
		}
	}

	repeat_scan = 0;
	
#ifdef PRINTS_FOR_AUTOMATION
		 if (test_ble) {
			LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
			LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt-1);			
			LOG_INF("ble_connection_fail_cnt = %u", ble_connection_fail_cnt);
			
			LOG_INF("ble_disconnection_attempt_cnt = %u", ble_disconnection_attempt_cnt);
			LOG_INF("ble_disconnection_success_cnt = %u", ble_disconnection_success_cnt-1);		
			LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
			LOG_INF("ble_discon_no_conn_cnt = %u", ble_discon_no_conn_cnt);			
		} 
		if (test_wlan) {
			LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
			LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
			LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
		}

#endif
	LOG_INF(" wifi_scan_ble_conn_peripheral complete");

	return 0;
}

#if 0
int wifi_scan_ble_conn_peripheral(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
	bool is_ble_central,	bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	
		#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;	
	
	start_wifi_activity();
	start_ble_activity(test_ble, is_ble_central);
	run_wifi_activity();
	run_ble_activity(test_ble, is_ble_central);

	repeat_scan = 0;
	
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
#endif
	LOG_INF(" wifi_scan_ble_conn_peripheral complete");

	return 0;
}
#endif

int wifi_scan_ble_tput_test(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	int ret=0;
	LOG_INF("In wifi_scan_ble_tput_test");
	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;
	uint32_t stamp= k_uptime_get_32();
	
	if (test_wlan) {
		if (is_wifi_conn_scan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
			start_wifi_activity();
		} else {
			start_wifi_activity();
		}
	}
	
	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			return ret;
		}
		start_ble_activity(test_ble, is_ble_central);
		if (is_ble_central) {
			run_ble_activity(test_ble, is_ble_central);
		} else {
			while(1) {
				if (k_uptime_get_32() - stamp > CONFIG_BLE_TEST_DURATION) {
					break;
				}
				k_sleep(K_MSEC(100));
			}
		}
		disconnect_ble(test_ble, is_ble_central);
	}

	if (test_wlan) {
		run_wifi_activity();
		if (is_wifi_conn_scan) {
			disconnect_wifi(test_wlan);
		}
	}

	repeat_scan = 0;
	
	#ifdef PRINTS_FOR_AUTOMATION
		if (test_wlan) {
			LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
			LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
			LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
		}

	#endif
	
	return 0;
}

#if 0
int wifi_scan_ble_tput_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
		if (is_wifi_conn_scan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}

	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
		start_ble_activity(test_ble, is_ble_central);
		run_ble_activity(test_ble, is_ble_central);
		disconnect_ble(test_ble, is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
#endif
	LOG_INF(" wifi_scan_ble_tput_central complete");
	return 0;
err:
	return ret;
}
#endif
#if 0
int wifi_scan_ble_tput_peripheral(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	if (test_ble) {
		if (!is_ble_central) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3));
		}
	}

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (is_wifi_conn_scan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
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
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		/* LOG_INF("BLE Throughput started"); */
		start_ble_activity(test_ble, is_ble_central);

		run_ble_activity(test_ble, is_ble_central);
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
		disconnect_ble(test_ble, is_ble_central);
	}

	/*k_sleep(K_SECONDS(5));*/
	repeat_scan = 0;
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
#endif
	LOG_INF(" wifi_scan_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}
#endif

int wifi_con_ble_con_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		/* BT init and connection */
		bt_connection_init(is_ble_central);
		/* BLE disconnection */
		bt_disconnect_central();
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;	
	
	start_wifi_activity();
	start_ble_activity(test_ble, is_ble_central);
	run_wifi_activity();
	run_ble_activity(test_ble, is_ble_central);

	repeat_scan = 0;

	#ifdef PRINTS_FOR_AUTOMATION
		 if (test_ble) {
			LOG_INF("ble_connection_attempt_cnt = %u", ble_connection_attempt_cnt);
			LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt-1);			
			LOG_INF("ble_connection_fail_cnt = %u", ble_connection_fail_cnt);
			
			LOG_INF("ble_disconnection_attempt_cnt = %u", ble_disconnection_attempt_cnt);
			LOG_INF("ble_disconnection_success_cnt = %u", ble_disconnection_success_cnt-1);		
			LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
			LOG_INF("ble_discon_no_conn_cnt = %u", ble_discon_no_conn_cnt);			
		} 
		if (test_wlan) {
			LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
			LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}

	#endif
	
	return 0;
}

#if 0
int wifi_con_ble_con_central(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			scan_start();
			k_sleep(K_SECONDS(1));
		}
		if (test_wlan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" wifi_con_ble_con_central complete");
	return 0;
}
#endif

int wifi_con_ble_con_peripheral(bool test_wlan,	bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
			wifi_connection(test_wlan, is_ant_mode_sep);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" wifi_con_ble_con_peripheral complete");
	return 0;
}

int wifi_con_ble_tput_central(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
			bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	int  ret=0;
	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif	
	
	repeat_scan = 1;	
	
	if (test_wlan) {
		if (is_wifi_conn_scan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
			start_wifi_activity();
		} else {
			start_wifi_activity();
		}
	}
	
	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
		start_ble_activity(test_ble, is_ble_central);
		run_ble_activity(test_ble, is_ble_central);
		disconnect_ble(test_ble, is_ble_central);
	}

	run_wifi_activity();

	repeat_scan = 0;

	#ifdef PRINTS_FOR_AUTOMATION
		if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
	#endif
	LOG_INF(" wifi_con_ble_tput_central complete");
	return 0;
err:
	return ret;

}

#if 0
int wifi_con_ble_tput_central(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
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
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		start_ble_activity(test_ble, is_ble_central);
		/* run_ble_activity(test_ble, is_ble_central); */
	}

	if (test_wlan) {
		while (true) {
			wifi_connection(test_wlan, is_ant_mode_sep);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		run_ble_activity(test_ble, is_ble_central);
		disconnect_ble(test_ble, is_ble_central);
	}

	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" wifi_con_ble_tput_central complete");
	return 0;
err:
	return ret;
}
#endif

int wifi_con_ble_tput_peripheral(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
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

	if (test_ble) {
		start_ble_activity(test_ble, is_ble_central);
		run_ble_activity(test_ble, is_ble_central);
	}

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("Run BLE central");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_wlan) {
		while (true) {
			wifi_connection(test_wlan, is_ant_mode_sep);
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
		/* run_ble_activity(test_ble, is_ble_central); */
		disconnect_ble(test_ble, is_ble_central);
	}
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" wifi_con_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_con_central(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
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
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
	}
#endif
	LOG_INF(" wifi_tput_client_ble_con_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_con_peripheral(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;


	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
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
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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


int wifi_tput_client_ble_tput_central(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */

	}
	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif

	if (is_zperf_udp) {
		ret = run_wifi_traffic(test_wlan);
	} else {
		ret = run_wifi_traffic_tcp(test_wlan);
	}

	if (ret != 0) {
		LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
	}

	start_ble_activity(test_ble, is_ble_central);

	check_wifi_traffic(test_wlan);

	run_ble_activity(test_ble, is_ble_central);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, is_ble_central);

	LOG_INF(" wifi_tput_client_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_client_ble_tput_peripheral(bool test_wlan, bool is_ant_mode_sep,
		bool test_ble, bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (!is_ble_central) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	start_ble_activity(test_ble, is_ble_central);

	run_ble_activity(test_ble, is_ble_central);

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("Run BLE central");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	if (is_zperf_udp) {
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

	disconnect_ble(test_ble, is_ble_central);
	LOG_INF(" wifi_tput_client_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_con_central(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */

		if (is_zperf_udp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
		while (!wait_for_wifi_client_start) {
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		bt_connection_init(is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
	}
#endif
	LOG_INF(" wifi_tput_server_ble_con_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_con_peripheral(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
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
		while (!wait_for_wifi_client_start) {
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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

int wifi_tput_server_ble_tput_central(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (is_zperf_udp) {
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
		#ifdef PRINTS_FOR_AUTOMATION
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	wait_for_wifi_client_start = 0;

	start_ble_activity(test_ble, is_ble_central);

	check_wifi_traffic(test_wlan);

	run_ble_activity(test_ble, is_ble_central);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, is_ble_central);

	LOG_INF(" wifi_tput_server_ble_tput_central complete");
	return 0;
err:
	return ret;
}

int wifi_tput_server_ble_tput_peripheral(bool test_wlan, bool is_ant_mode_sep,
		bool test_ble, bool is_ble_central, bool is_wlan_server, bool is_zperf_udp)
{
	int ret = 0;

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	if (!is_ble_central) {
		LOG_INF("Make sure peer BLE role is central");
		k_sleep(K_SECONDS(3));
	}

	if (test_ble) {
		/* ble_connection(test_ble, is_ble_central); */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}

	if (is_zperf_udp) {
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
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	start_ble_activity(test_ble, is_ble_central);

	check_wifi_traffic(test_wlan);

	run_ble_activity(test_ble, is_ble_central);

	disconnect_wifi(test_wlan);

	disconnect_ble(test_ble, is_ble_central);

	LOG_INF(" wifi_tput_server_ble_tput_peripheral complete");
	return 0;
err:
	return ret;
}

int wifi_con_ble_con_central_regr(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	/* Wi-Fi connection done only once */
	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		k_sleep(K_SECONDS(2));
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
#endif
	LOG_INF(" wifi_con_ble_con_central_regr complete");
	return 0;
}

int wifi_con_ble_con_peripheral_regr(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		k_sleep(K_SECONDS(2));
	}

	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif

	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_ble) {
			if (!ble_periph_connected) {
				adv_start();
				 k_sleep(K_SECONDS(1)); 
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
#endif
	LOG_INF(" wifi_con_ble_con_peripheral_regr complete");
	return 0;
}

int wifi_con_ble_tput_central_regr(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */

	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		k_sleep(K_SECONDS(2));
	}

	if (test_ble) {
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif

	test_start_time = k_uptime_get_32();

	if (test_ble) {
		start_ble_activity(test_ble, is_ble_central);
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
		run_ble_activity(test_ble, is_ble_central);
		disconnect_ble(test_ble, is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
#endif
	LOG_INF(" wifi_con_ble_tput_central_regr complete");
	return 0;
err:
	return ret;
}

int wifi_con_ble_tput_peripheral_regr(bool test_wlan, bool is_ant_mode_sep, bool test_ble,
		bool is_ble_central, bool is_wlan_server)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */

	}
	/* Wi-Fi connection done only once */	
	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		k_sleep(K_SECONDS(2));
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

	if (test_ble) {
		start_ble_activity(test_ble, is_ble_central);
	}

	if (test_wlan && test_ble) {
		while (!wait_for_ble_central_run) {
		#ifdef PRINTS_FOR_AUTOMATION 
			LOG_INF("Run BLE central");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_ble_central_run = 0;
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
		run_ble_activity(test_ble, is_ble_central); 
		disconnect_ble(test_ble, is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_conn_cnt_regr = %u", wifi_conn_cnt_regr);
		LOG_INF("wifi_disconn_cnt_regr = %u", wifi_disconn_cnt_regr);
	}
#endif
	LOG_INF(" wifi_con_ble_tput_peripheral_regr complete");
	return 0;
err:
	return ret;
}

int ble_conn_central_wifi_scan_regr(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central, bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;
	
	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
	}
	
	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
		if (is_wifi_conn_scan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if ((k_uptime_get_32() - test_start_time) >
			CONFIG_WIFI_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
		/* LOG_INF("waiting"); */
	}
	
	if (test_wlan) {
		repeat_scan = 0;
	}

	if (test_ble) {
		if (ble_central_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		bt_disconnect_central();
		k_sleep(K_SECONDS(2));
	}
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
#endif
	LOG_INF(" ble_conn_central_wifi_scan_regr complete");
	return 0;
}

int ble_conn_central_wifi_con_regr(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_wlan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}

	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" ble_conn_central_wifi_con_regr complete");
	return 0;
}

//PENDING: ble_conn_central_wifi_ping_regr()

int ble_conn_central_wifi_tput_client_regr(bool test_wlan, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
	}

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */

		if (is_zperf_udp) {
			LOG_INF("UDP traffic");
			ret = run_wifi_traffic(test_wlan);
		} else {
			LOG_INF("TCP traffic");
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	if (test_ble) {
		while (true) {
			if ((k_uptime_get_32() - test_start_time)
				> CONFIG_WIFI_TEST_DURATION_REGR) {
				break;
			}
			k_sleep(K_SECONDS(2));
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
#endif
	LOG_INF(" ble_conn_central_wifi_tp_udp_client_regr complete");
	return 0;
err:
	return ret;
}

int ble_conn_central_wifi_tput_server_regr(bool test_wlan, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
	}

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);

		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */

		if (is_zperf_udp) {
			ret = run_wifi_traffic(test_wlan);
		} else {
			ret = run_wifi_traffic_tcp(test_wlan);
		}
		if (ret != 0) {
			LOG_ERR("Failed to start Wi-Fi benchmark: %d", ret);
			goto err;
		}
		while (!wait_for_wifi_client_start) {
		#ifdef PRINTS_FOR_AUTOMATION
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
#endif
	LOG_INF(" ble_conn_central_wifi_tp_udp_server_regr complete");
	return 0;
err:
	return ret;
}

int ble_conn_peripheral_wifi_scan_regr(bool is_ant_mode_sep, bool test_ble, bool test_wlan,
		bool is_ble_central,	bool is_wlan_server, bool is_wifi_conn_scan)
{
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}
	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}
	if (test_wlan) {
		if (is_wifi_conn_scan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
			cmd_wifi_scan();
		} else {
			cmd_wifi_scan();
		}
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif

	test_start_time = k_uptime_get_32();

	while (true) {
		if ((k_uptime_get_32() - test_start_time) >
				CONFIG_WIFI_TEST_DURATION_REGR) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}
	
	if (test_wlan) {
		repeat_scan = 0;
	}


	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		k_sleep(K_SECONDS(2));
	}
#ifdef PRINTS_FOR_AUTOMATION
	if (test_wlan) {
		LOG_INF("wifi_scan_cnt_24g = %u", wifi_scan_cnt_24g);
		LOG_INF("wifi_scan_cnt_5g = %u", wifi_scan_cnt_5g);
		LOG_INF("wifi_scan_cmd_cnt = %u", wifi_scan_cmd_cnt);
	}
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
#endif	
	LOG_INF(" ble_conn_peripheral_wifi_scan_regr complete");

	return 0;
}


int ble_conn_peripheral_wifi_con_regr(bool test_wlan, bool test_ble, bool is_ble_central,
		bool is_wlan_server, bool is_ant_mode_sep)
{
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
	#ifdef CONFIG_NRF700X_BT_COEX
		config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
	#endif/* CONFIG_NRF700X_BT_COEX */
	}

	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();

	while (true) {
		if (test_wlan) {
			wifi_connection(test_wlan, is_ant_mode_sep);
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

	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
	}
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
	if (test_wlan) {
		LOG_INF("wifi_conn_success_cnt = %u", wifi_conn_success_cnt);
		LOG_INF("wifi_conn_fail_cnt = %u", wifi_conn_fail_cnt);
	}
#endif
	LOG_INF(" ble_conn_peripheral_wifi_con_regr complete");
	return 0;
}

//PENDING: ble_conn_peripheral_wifi_ping_regr()

int ble_conn_peripheral_wifi_tput_client_regr(bool test_wlan, bool test_ble,
		bool is_ble_central,	bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
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
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
#endif
	LOG_INF(" ble_conn_peripheral_wifi_tp_udp_client_regr complete");
	return 0;
err:
	return ret;
}


int ble_conn_peripheral_wifi_tput_server_regr(bool test_wlan, bool test_ble,
		bool is_ble_central, bool is_wlan_server, bool is_ant_mode_sep, bool is_zperf_udp)
{
	int ret = 0;
	uint64_t test_start_time = 0;

	/* one time BT connection */
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
	}

	if (test_wlan) {
		wifi_connection(test_wlan, is_ant_mode_sep);
		#ifdef CONFIG_NRF700X_BT_COEX
			config_pta(is_ant_mode_sep, is_ble_central, is_wlan_server);
		#endif/* CONFIG_NRF700X_BT_COEX */
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
		while (!wait_for_wifi_client_start) {
		#ifdef PRINTS_FOR_AUTOMATION
			LOG_INF("start WiFi client");
		#endif
			k_sleep(K_SECONDS(1));
		}
		wait_for_wifi_client_start = 0;
	}
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
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
#ifdef PRINTS_FOR_AUTOMATION
	if (test_ble) {
		if (ble_periph_connected) {
			LOG_INF("BLE Conn Intact");
		} else {
			LOG_INF("BLE disconnected");
		}
		LOG_INF("ble_connection_success_cnt = %u", ble_connection_success_cnt);
		LOG_INF("ble_disconn_cnt_regr = %u", ble_disconn_cnt_regr);
	}
#endif
	LOG_INF(" ble_conn_peripheral_wifi_tput_server_regr complete");
	return 0;
err:
	return ret;
}

int ble_con_central_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	test_start_time = k_uptime_get_32();
	if (test_ble) {
		bt_connection_init(is_ble_central);
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
#ifdef PRINTS_FOR_AUTOMATION		
		LOG_INF("ble_conn_success_cnt = %u", ble_conn_success_cnt);
		LOG_INF("ble_disconnection_fail_cnt = %u", ble_disconnection_fail_cnt);
#endif
	}
	LOG_INF(" ble_con_central_wifi_shutdown complete");
	return 0;
}

int ble_con_peripheral_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	if (test_ble) {
		bt_connection_init(is_ble_central);
		while (true) {
			if (ble_periph_connected) {
				break;
			}
		}
		
		#ifdef DEMARCATE_TEST_START
		LOG_INF("----------------------------------------------------------------");
		#endif
		test_start_time = k_uptime_get_32();
		
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

	LOG_INF(" ble_con_peripheral_wifi_shutdown complete");

	return 0;
}

int ble_tput_central_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	#ifdef DEMARCATE_TEST_START
	LOG_INF("----------------------------------------------------------------");
	#endif
	if (test_ble) {
		test_start_time = k_uptime_get_32();
		/* BLE connection */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}
		start_ble_activity(test_ble, is_ble_central);
		run_ble_activity(test_ble, is_ble_central);
		disconnect_ble(test_ble, is_ble_central);
	}

	LOG_INF(" ble_tput_central_wifi_shutdown complete");
	return 0;
err:
	return ret;
}

int ble_tput_periph_wifi_shutdown(bool test_ble, bool is_ble_central)
{
	uint64_t test_start_time = 0;
	int ret = 0;

	/* disable RPU i.e. Wi-Fi shutdown */
	rpu_disable();
	
	if (test_ble) {
		if (!is_ble_central) {
			LOG_INF("Make sure peer BLE role is central");
			k_sleep(K_SECONDS(3));
		}

		/* BLE connection */
		ret = bt_throughput_test_init(is_ble_central);
		if (ret != 0) {
			LOG_ERR("Failed to BT throughput init: %d", ret);
			goto err;
		}

		#ifdef DEMARCATE_TEST_START
		LOG_INF("");
		LOG_INF("----------------------------------------------------------------");
		LOG_INF("");
		#endif
		test_start_time = k_uptime_get_32();
		/* LOG_INF("BLE Throughput started"); */
		start_ble_activity(test_ble, is_ble_central);

		run_ble_activity(test_ble, is_ble_central);

		while (true) {
			if (k_uptime_get_32() - test_start_time > CONFIG_BLE_TEST_DURATION) {
				break;
			}
			k_sleep(K_SECONDS(1));
		}

		disconnect_ble(test_ble, is_ble_central);
	}

	LOG_INF(" ble_tput_periph_wifi_shutdown complete");
	return 0;
err:
	return ret;
}

