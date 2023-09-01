/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <stdlib.h>


#include "bt_utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_utils, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(CONFIG_WIFI_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_CON_SCAN_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_CLIENT_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_UDP_SERVER_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_CLIENT_BLE_CON_PERIPH) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_CENTRAL) || \
	defined(CONFIG_WIFI_TP_TCP_SERVER_BLE_CON_PERIPH) || \
	defined(CONFIG_BLE_CON_CENTRAL_WIFI_SHUTDOWN) || \
	defined(CONFIG_BLE_CON_PERIPHERAL_WIFI_SHUTDOWN)

	/**nothing . These are the tests in which the BLE connection
	 *is done multiple times in a loop
	 */
#else
	#define BLE_TX_PWR_CTRL_RSSI
#endif

uint32_t ble_connection_success_cnt;
uint32_t ble_connection_attempt_cnt;


uint32_t ble_disconnection_attempt_cnt;
uint32_t ble_disconnection_success_cnt;
uint32_t ble_disconnection_fail_cnt;
uint32_t ble_discon_no_conn_cnt;
uint32_t ble_discon_no_conn;

uint32_t wifi_scan_cmd_cnt;

uint32_t ble_supervision_timeout;
extern uint32_t ble_le_datalen_failed;
extern uint32_t ble_phy_update_failed;
extern uint32_t ble_le_datalen_timeout;
extern uint32_t ble_phy_update_timeout;
extern uint32_t ble_conn_param_update_failed;
extern uint32_t ble_conn_param_update_timeout;

int8_t ble_txpower = 127;
int8_t ble_rssi = 127;
static int print_ble_conn_status_once = 1;

#ifdef BLE_TX_PWR_CTRL_RSSI
	/* to get/set BLE Tx power and read BLE RSSI for coex sample */
	#include <stddef.h>
	#include <zephyr/sys/printk.h>
	#include <zephyr/sys/util.h>
	#include <zephyr/sys/byteorder.h>
	#include <zephyr/bluetooth/hci_vs.h>

	#include <zephyr/bluetooth/services/hrs.h>

	static uint16_t default_conn_handle;
#endif


#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
/*#include <zephyr/sys/LOG_INF.h>*/

#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/throughput.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <zephyr/shell/shell_uart.h>

#include <dk_buttons_and_leds.h>

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define THROUGHPUT_CONFIG_TIMEOUT 20
#define SCAN_CONFIG_TIMEOUT 20


/* #define PRINT_BLE_UPDATES */

static K_SEM_DEFINE(throughput_sem, 0, 1);

extern uint8_t wait4_peer_ble2_start_connection;
bool ble_periph_connected;
bool ble_central_connected;

uint64_t ble_scan2conn_start_time;
int64_t ble_scan2conn_time;
uint32_t ble_disconn_cnt_stability;

static volatile bool data_length_req;
static volatile bool test_ready;
static struct bt_conn *default_conn;
static struct bt_throughput throughput;
static struct bt_uuid *uuid128 = BT_UUID_THROUGHPUT;
static struct bt_gatt_exchange_params exchange_params;

static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(CONFIG_BLE_INTERVAL_MIN, CONFIG_BLE_INTERVAL_MAX, 0, 400);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		0xBB, 0x4A, 0xFF, 0x4F, 0xAD, 0x03, 0x41, 0x5D,
		0xA9, 0x6C, 0x9D, 0x6C, 0xDD, 0xDA, 0x83, 0x04),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

void button_handler_cb(uint32_t button_state, uint32_t has_changed);
#ifdef PRINT_BLE_UPDATES
static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}
#endif

void instruction_print(void)
{
	/**
	 * LOG_INF("Type 'config' to change the configuration parameters.");
	 * LOG_INF("You can use the Tab key to autocomplete your input.");
	 * LOG_INF("Type 'run' when you are ready to run the test.");
	 */
}


void scan_filter_match(struct bt_scan_device_info *device_info,
		       struct bt_scan_filter_match *filter_match,
		       bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

#ifdef PRINT_BLE_UPDATES
	LOG_INF("Filters matched. Address: %s connectable: %d",
		addr, connectable);
#endif
}

void scan_filter_no_match(struct bt_scan_device_info *device_info,
			  bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
/* #ifdef PRINT_BLE_UPDATES */
/**#if 0
 *	LOG_INF("Filter not match. Address: %s connectable: %d",
 *				addr, connectable);
 *#endif
 */
}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_ERR("Connecting failed");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
		scan_connecting_error, NULL);

void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	struct bt_conn_info info = {0};
	int err;
#ifdef PRINT_BLE_UPDATES
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
#endif

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return;
	}

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		instruction_print();
		test_ready = true;
	}
}

void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	int err;
	struct bt_throughput *throughput = context;
#ifdef PRINT_BLE_UPDATES
	LOG_INF("Service discovery completed");
#endif
	bt_gatt_dm_data_print(dm);
	bt_throughput_handles_assign(dm, throughput);
	bt_gatt_dm_data_release(dm);

	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(default_conn, &exchange_params);

	if (err) {
		#ifdef PRINT_BLE_UPDATES
		LOG_ERR("MTU exchange failed (err %d)", err);
		#endif
	} else {
		#ifdef PRINT_BLE_UPDATES
		LOG_INF("MTU exchange pending");
		#endif
	}
}

void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_INF("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

void connected(struct bt_conn *conn, uint8_t hci_err)
{
	struct bt_conn_info info = {0};
	int err;

	if (hci_err) {
		if (hci_err == BT_HCI_ERR_UNKNOWN_CONN_ID) {
			/* Canceled creating connection */
			return;
		}
		LOG_ERR("Connection failed (err 0x%02x)", hci_err);
		return;
	}

	if (default_conn) {
		LOG_INF("Connection exists, disconnect second connection");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	default_conn = bt_conn_ref(conn);

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);

		return;
	}
	ble_connection_success_cnt++;
	ble_central_connected = true;
#ifdef CONFIG_PRINTS_FOR_AUTOMATION
	if (print_ble_conn_status_once) {
		LOG_INF("Connected as %s", info.role ==
			BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
		LOG_INF("Conn. interval is %u units", info.le.interval);
		print_ble_conn_status_once = 0;
	}
#endif
	if (info.role == BT_CONN_ROLE_CENTRAL) {
		err = bt_gatt_dm_start(default_conn,
				BT_UUID_THROUGHPUT,
				&discovery_cb,
				&throughput);

		if (err) {
			LOG_ERR("Discover failed (err %d)", err);
		}
	} else {
		/**this is moved to connection_configuration_set()
		 * so that PHY update is  cleanly done before starting
		 * the coex test when DUT is in peripheral role. If this is
		 *If this is not done then coex test proceeds immediately
		 * after connection is done (before completion of PHY update
		 * params) and due to interference PHY update may fail
		 */

		/* ble_periph_connected = true; */
	}

	#ifdef BLE_TX_PWR_CTRL_RSSI
		char addr[BT_ADDR_LE_STR_LEN];
		int8_t get_txp = 0;
		int8_t set_txp = 0;
		int ret;
		int8_t rssi = 0xFF;
		int8_t txp_adaptive;

		printk("BLE Target Tx power %d\n", set_txp);
		default_conn = bt_conn_ref(conn);
		ret = bt_hci_get_conn_handle(default_conn,
						 &default_conn_handle);
		if (ret) {
			printk("No connection handle (err %d)\n", ret);
		} else {
			read_conn_rssi(default_conn_handle, &rssi);
			/* printk("Connected (%d) - RSSI = %d\n", */
			/*	   default_conn_handle, rssi); */

			/* Send first at the default selected power */
			bt_addr_le_to_str(bt_conn_get_dst(conn),
							  addr, sizeof(addr));
			/* printk("Connected via connection (%d) at %s\n", */
			/*	   default_conn_handle, addr); */
			get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
					 default_conn_handle, &get_txp);
			/* printk("Connection (%d) - Initial Tx Power = %d\n", */
			/*    default_conn_handle, get_txp); */
			/* sets Tx power to RADIO_TXP_DEFAULT */
			/* printk("Changing Tx power to = %d\n", set_txp); */

			set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
					 default_conn_handle,
					 set_txp);
			get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
					 default_conn_handle, &get_txp);
			printk("BLE Connection (%d)\n", default_conn_handle);
			printk("coex sample -->connected(): BLE Tx Power: %d\n", get_txp);
			read_conn_rssi(default_conn_handle, &rssi);
			printk("coex sample -->connected(): BLE RSSI: %d\n", rssi);
			ble_txpower = get_txp;
			ble_rssi = rssi;
		}
	#endif
}

void scan_init(void)
{
	int err;
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = CONFIG_BT_LE_SCAN_INTERVAL,
		.window = CONFIG_BT_LE_SCAN_WINDOW,
	};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
		.scan_param = &scan_param,
		.conn_param = conn_param
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, uuid128);
	if (err) {
		LOG_INF("Scanning filters cannot be set");

		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_INF("Filters cannot be turned on");
	}
}

void scan_start(void)
{
	int err;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
	#ifdef PRINT_BLE_UPDATES
		LOG_ERR("Starting scanning failed (err %d)", err);
	#endif
		return;
	}
}

void adv_start(void)
{
	struct bt_le_adv_param *adv_param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE |
				BT_LE_ADV_OPT_ONE_TIME,
				CONFIG_BT_GAP_ADV_FAST_INT_MIN_2,
				CONFIG_BT_GAP_ADV_FAST_INT_MAX_2,
				NULL);
	int err;

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to start advertiser (%d)", err);
		return;
	}
}
void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info = {0};
	int err;
#ifdef PRINT_BLE_UPDATES
	LOG_INF("Disconnected (reason 0x%02x)", reason);
#endif

	test_ready = false;
	ble_periph_connected = false;
	ble_central_connected = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
	/* Disconnection count for central is available in bt_disconnect_central() */
	if (!IS_ENABLED(CONFIG_BT_ROLE_CENTRAL)) {
		ble_disconnection_success_cnt++;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (%d)\n", err);
		return;
	}
	/* Re-connect using same roles */
	if (info.role == BT_CONN_ROLE_CENTRAL) {
		ble_connection_attempt_cnt++;
		scan_start(); 		
	} else {
		adv_start();
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.");
	LOG_INF("Minimum interval: %d, Maximum interval: %d",
	       param->interval_min, param->interval_max);
	LOG_INF("Latency: %d, Timeout: %d", param->latency, param->timeout);

	return true;
}

void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("Connection parameters updated."
	       " interval: %d, latency: %d, timeout: %d",
	       interval, latency, timeout);

	k_sem_give(&throughput_sem);
}

void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
#ifdef PRINT_BLE_UPDATES
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));
#endif
	k_sem_give(&throughput_sem);
}

void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	if (!data_length_req) {
		return;
	}
#ifdef PRINT_BLE_UPDATES
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
#endif
	data_length_req = false;
	k_sem_give(&throughput_sem);
}


static uint8_t throughput_read(const struct bt_throughput_metrics *met)
{
	LOG_INF("[peer] received %u bytes (%u KB)"
	       " in %u GATT writes at %u bps",
	       met->write_len, met->write_len / 1024, met->write_count,
	       met->write_rate);

	k_sem_give(&throughput_sem);

	return BT_GATT_ITER_STOP;
}

void throughput_received(const struct bt_throughput_metrics *met)
{
	static uint32_t kb;

	if (met->write_len == 0) {
		kb = 0;
		wait4_peer_ble2_start_connection = 1;
		LOG_INF("");

		return;
	}

	if ((met->write_len / 1024) != kb) {
		kb = (met->write_len / 1024);
		/* LOG_INF("="); */
	}
}
void throughput_send(const struct bt_throughput_metrics *met)
{
	LOG_INF("[local] received %u bytes (%u KB)"
		" in %u GATT writes at %u bps",
		met->write_len, met->write_len / 1024,
		met->write_count, met->write_rate);
}

static struct button_handler button = {
	.cb = button_handler_cb,
};

void select_role(bool is_central)
{
	int err;
	static bool role_selected;

	if (role_selected) {
		LOG_INF("Cannot change role after it was selected once.");
		return;
	} else if (is_central) {
		scan_start();
	} else {
		adv_start();
	}

	role_selected = true;

	/* The role has been selected, button are not needed any more. */
	err = dk_button_handler_remove(&button);
	if (err) {
		LOG_INF("Button disable error: %d", err);
	}
}

void button_handler_cb(uint32_t button_state, uint32_t has_changed)
{
	ARG_UNUSED(has_changed);

	if (button_state & DK_BTN1_MSK) {
		select_role(true);
	} else if (button_state & DK_BTN2_MSK) {
		select_role(false);
	}
}

void buttons_init(void)
{
	int err = 0;

	err = dk_buttons_init(NULL);
	if (err) {
		LOG_ERR("Buttons initialization failed.");
		return;
	}

	/**
	 *Add dynamic buttons handler. Buttons should be activated only when
	 * during the board role choosing.
	 */
	dk_button_handler_add(&button);
}

int connection_configuration_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len)
{
	int err=0;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		goto end_of_function;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		LOG_INF("'run' command shall be executed only on the central board");
	}

	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		ble_phy_update_failed++;
		LOG_ERR("PHY update failed: %d", err);
		goto end_of_function;
	}
#ifdef PRINT_BLE_UPDATES
	LOG_INF("PHY update pending");
#endif
	err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
	if (err) {
		ble_phy_update_timeout++;
		LOG_INF("PHY update timeout");
		goto end_of_function;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			ble_le_datalen_failed++;
			LOG_ERR("LE data length update failed: %d",
				    err);
			goto end_of_function;
		}
#ifdef PRINT_BLE_UPDATES
		LOG_INF("LE Data length update pending");
#endif
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			ble_le_datalen_timeout++;
			LOG_INF("LE Data Length update timeout");
			goto end_of_function;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			ble_conn_param_update_failed++;
			LOG_ERR("Connection parameters update failed: %d",
				    err);

			goto end_of_function;
		}

		LOG_INF("Connection parameters update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			ble_conn_param_update_timeout++;
			LOG_INF("Connection parameters update timeout");
			goto end_of_function;
		}
	}
end_of_function:
	/* LOG_INF("supervision timeout %d", conn_param->timeout); */
	ble_supervision_timeout = conn_param->timeout;

	ble_periph_connected = true;
	return err;
}

int bt_throughput_test_run(void)
{
	int err;
	int64_t stamp;
	int64_t delta;
	uint32_t data = 0;

	/* a dummy data buffer */
	static char dummy[495];

	if (!default_conn) {
		LOG_INF("Device is disconnected %s",
			    "Connect to the peer device before running test");
		return -EFAULT;
	}

	if (!test_ready) {
		LOG_INF("Device is not ready."
			"Please wait for the service discovery and MTU exchange end");
		return 0;
	}

	/* LOG_INF("==== Starting throughput test ===="); */

	/* reset peer metrics */
	err = bt_throughput_write(&throughput, dummy, 1);
	if (err) {
		LOG_ERR("Reset peer metrics failed.");
		return err;
	}

	/* get cycle stamp */
	stamp = k_uptime_get_32();

	delta = 0;
	while (true) {
		err = bt_throughput_write(&throughput, dummy, 495);
		if (err) {
			LOG_ERR("GATT write failed (err %d)", err);
			break;
		}
		data += 495;
		if (k_uptime_get_32() - stamp > CONFIG_COEX_TEST_DURATION) {
			break;
		}
	}

	delta = k_uptime_delta(&stamp);

	LOG_INF("Done");
	LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps",
	       data, data / 1024, delta, ((uint64_t)data * 8 / delta));

	/* read back char from peer */
	err = bt_throughput_read(&throughput);
	if (err) {
		LOG_ERR("GATT read failed (err %d)", err);
		return err;
	}

	k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));

	instruction_print();
	/* to stop scan after the test duration is complete */
	scan_init();
	return 0;
}

void bt_conn_test_run(void)
{

	int64_t stamp;
	int64_t delta;

	/* get cycle stamp */
	stamp = k_uptime_get_32();

	delta = 0;
	/*After the disconnection in loop, scan_start() in disconnected() will take of
	repeting scan starts until the end of test duration.*/
	ble_connection_attempt_cnt++;
	scan_start(); 
	while (true) {
		if (IS_ENABLED(CONFIG_BT_ROLE_CENTRAL)) {
			/* if (ble_central_connected) { */
				ble_disconnection_attempt_cnt++;
				bt_disconnect_central();
			/* } */
		}		
		/* start scan to attempt a new connection, if BLE is not connected */
		if (ble_discon_no_conn != 0) {
			ble_discon_no_conn = 0;
			ble_connection_attempt_cnt++;
			scan_start();			
		}
		
		if (k_uptime_get_32() - stamp > CONFIG_COEX_TEST_DURATION) {
			break;
		}
		/* sleep time of less than 2 seconds throws coredump errors.*/
		k_sleep(K_SECONDS(2));
	}
	/* to stop scan after the test duration is complete */
	scan_init();
}

static const struct bt_throughput_cb throughput_cb = {
	.data_read = throughput_read,
	.data_received = throughput_received,
	.data_send = throughput_send
};

int bt_throughput_test_init(bool is_ble_central)
{
	int err;
	int64_t stamp;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	/* LOG_INF("Bluetooth initialized"); */
	scan_init();

	err = bt_throughput_init(&throughput, &throughput_cb);
	if (err) {
		LOG_ERR("Throughput service initialization failed.");
		return err;
	}

	buttons_init();

	select_role(is_ble_central);

	/**
	 *LOG_INF("Waiting for connection.");
	 *ble_scan2conn_start_time = k_uptime_get_32();
	 *ble_scan2conn_time = 0;
	 */

	stamp = k_uptime_get_32();
	while (k_uptime_delta(&stamp) / MSEC_PER_SEC < THROUGHPUT_CONFIG_TIMEOUT) {
		if (default_conn) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (!default_conn) {
		LOG_INF("Cannot set up connection.");
		return -ENOTCONN;
	}

	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for scan %lld ms", ble_scan2conn_time);
	 *ble_scan2conn_start_time = k_uptime_get_32();
	 *ble_scan2conn_time = 0;
	 */

	uint32_t conn_cfg_status = connection_configuration_set(
			BT_LE_CONN_PARAM(CONFIG_BLE_INTERVAL_MIN,
			CONFIG_BLE_INTERVAL_MAX,
			CONFIG_BT_CONN_LATENCY, CONFIG_BT_SUPERVISION_TIMEOUT),
			BT_CONN_LE_PHY_PARAM_2M,
			BT_LE_DATA_LEN_PARAM_MAX);
	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for connecion %lld ms", ble_scan2conn_time);
	 */

	return conn_cfg_status;
}



int bt_connection_init(bool is_ble_central)
{
	int err;
	int64_t stamp;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	/* LOG_INF("Bluetooth initialized"); */
	scan_init();

	/**
	 *err = bt_throughput_init(&throughput, &throughput_cb);
	 *if (err) {
	 *	LOG_ERR("Throughput service initialization failed.");
	 *	return err;
	 *}
	 */
	buttons_init();

	select_role(is_ble_central);

	/**
	 *LOG_INF("Waiting for connection.");
	 *ble_scan2conn_start_time = k_uptime_get_32();
	 *ble_scan2conn_time = 0;
	 */

	stamp = k_uptime_get_32();
	while (k_uptime_delta(&stamp) / MSEC_PER_SEC < SCAN_CONFIG_TIMEOUT) {
		if (default_conn) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	if (!default_conn) {
		LOG_INF("Cannot set up connection.");
		return -ENOTCONN;
	}

	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for scan %lld ms", ble_scan2conn_time);
	 *ble_scan2conn_start_time = k_uptime_get_32();
	 *ble_scan2conn_time = 0;
	 */

	uint32_t conn_cfg_status = connection_configuration_set(
			BT_LE_CONN_PARAM(CONFIG_BLE_INTERVAL_MIN,
			CONFIG_BLE_INTERVAL_MAX,
			CONFIG_BT_CONN_LATENCY, CONFIG_BT_SUPERVISION_TIMEOUT),
			BT_CONN_LE_PHY_PARAM_2M,
			BT_LE_DATA_LEN_PARAM_MAX);
	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for connecion %lld ms", ble_scan2conn_time);
	 */
	return conn_cfg_status;
}


int bt_disconnect_central(void)
{
	int err;

	if (!default_conn) {
		/* LOG_INF("Not connected!"); */
		ble_discon_no_conn_cnt++;
		ble_discon_no_conn++;
		return -ENOTCONN;
	}

	err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err) {
		/* LOG_INF("Cannot disconnect!"); */
		ble_disconnection_fail_cnt++;
		return err;
	}
	
	ble_disconnection_success_cnt++;
	return 0;
}
int bt_throughput_test_exit(void)
{
	int err;

	if (!default_conn) {
		LOG_INF("Not connected!");
		return -ENOTCONN;
	}
	err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err) {
		LOG_INF("Cannot disconnect!");
		return err;
	}
	return 0;
}




#ifdef BLE_TX_PWR_CTRL_RSSI

void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
	struct bt_hci_cp_vs_read_tx_power_level *cp;
	struct bt_hci_rp_vs_read_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	*tx_pwr_lvl = 0xFF;
	buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("coex sample, get tx pow,  Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_read_tx_power_level *)
			  rsp->data)->status : 0;
		printk("coex sample, get tx pow, Read Tx power err: %d reason 0x%02x\n",
			err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*tx_pwr_lvl = rp->tx_power_level;

	net_buf_unref(rsp);
}

void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("coex sample, read conn rssi, Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		printk("coex sample, read conn rssi, Read RSSI err: %d reason 0x%02x\n",
			err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}


void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("coex sample, set tx pow, Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_write_tx_power_level *)
			  rsp->data)->status : 0;
		printk("coex sample, set tx pow,  Set Tx power err: %d reason 0x%02x\n",
			err, reason);
		return;
	}

	rp = (void *)rsp->data;
	/* printk("Actual Tx Power: %d\n", rp->selected_tx_power); */

	net_buf_unref(rsp);
}
#endif


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated
};
