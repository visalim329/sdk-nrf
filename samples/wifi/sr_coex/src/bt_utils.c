/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <stdlib.h>

#include "common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_utils, CONFIG_LOG_DEFAULT_LEVEL);


#include "bt_utils.h"

#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)
	#include <zephyr/kernel.h>
	#include <zephyr/console/console.h>
	#include <string.h>
	#include <stdlib.h>
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

	#include "main.h"

	#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
	#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
	#define INTERVAL_MIN	80	/* 80 units, 100 ms */
	#define INTERVAL_MAX	80	/* 80 units, 100 ms */


	#define THROUGHPUT_CONFIG_TIMEOUT (20)
	#define SCAN_CONFIG_TIMEOUT 20
	#define BLE_CONN_CENTRAL_TEST_DURATION 20000 /* milliseconds*/

	#define CONN_LATENCY 0
	#define SUPERVISION_TIMEOUT 1000

	static uint32_t ble_conn_success_cnt;
	static uint32_t ble_conn_fail_cnt;

	static K_SEM_DEFINE(throughput_sem, 0, 1);

	static volatile bool data_length_req;
	static volatile bool test_ready;
	static struct bt_conn *default_conn;
	static struct bt_throughput throughput;
	static struct bt_uuid *uuid128 = BT_UUID_THROUGHPUT;
	static struct bt_gatt_exchange_params exchange_params;
	static struct bt_le_conn_param *conn_param =
		BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

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

	#define PRINT_BLE_UPDATES
#endif

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
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

	#define CONN_LATENCY 0
	#define SUPERVISION_TIMEOUT 1000

	#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
	#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

	#define THROUGHPUT_CONFIG_TIMEOUT 20
	#define SCAN_CONFIG_TIMEOUT 20


	/*#define PRINT_BLE_UPDATES*/

	static K_SEM_DEFINE(throughput_sem, 0, 1);

	extern uint8_t wait_for_ble_central_run;
	uint32_t ble_conn_success_cnt;
	uint32_t ble_conn_fail_cnt;
	bool ble_periph_connected;

	uint64_t ble_scan2conn_start_time;
	int64_t ble_scan2conn_time;

	static volatile bool data_length_req;
	static volatile bool test_ready;
	static struct bt_conn *default_conn;
	static struct bt_throughput throughput;
	static struct bt_uuid *uuid128 = BT_UUID_THROUGHPUT;
	static struct bt_gatt_exchange_params exchange_params;

	static struct bt_le_conn_param *conn_param =
		BT_LE_CONN_PARAM(CONFIG_INTERVAL_MIN, CONFIG_INTERVAL_MAX, 0, 400);

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


#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)

void instruction_print(void)
{
	LOG_INF("Type 'config' to change the configuration parameters.");
	LOG_INF("You can use the Tab key to autocomplete your input.");
	LOG_INF("Type 'run' when you are ready to run the test.");
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info = {0};
	int err;

	LOG_INF("Disconnected (reason 0x%02x)", reason);

	test_ready = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info (%d)", err);
		return;
	}

	/* Re-connect using same roles */
	if (info.role == BT_CONN_ROLE_CENTRAL) {
		scan_start();
	} else {
		adv_start();
	}
}

void throughput_received(const struct bt_throughput_metrics *met)
{
	static uint32_t kb;

	if (met->write_len == 0) {
		kb = 0;
		LOG_INF("");

		return;
	}

	if ((met->write_len / 1024) != kb) {
		kb = (met->write_len / 1024);
		printk("=");
	}
}

int connection_configuration_set_new(const struct shell *shell,
			const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len)
{
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		shell_error(shell, "Failed to get connection info %d", err);
		return err;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		shell_error(shell,
		"'run' command shall be executed only on the central board");
	}

	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		shell_error(shell, "PHY update failed: %d", err);
		return err;
	}

	shell_print(shell, "PHY update pending");
	err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
	if (err) {
		shell_error(shell, "PHY update timeout");
		return err;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			shell_error(shell, "LE data length update failed: %d",
					err);
			return err;
		}

		shell_print(shell, "LE Data length update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			shell_error(shell, "LE Data Length update timeout");
			return err;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			shell_error(shell,
					"Connection parameters update failed: %d",
					err);
			return err;
		}

		shell_print(shell, "Connection parameters update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			shell_error(shell,
					"Connection parameters update timeout");
			return err;
		}
	}
	return 0;
}

int test_run(const struct shell *shell,
		 const struct bt_le_conn_param *conn_param,
		 const struct bt_conn_le_phy_param *phy,
		 const struct bt_conn_le_data_len_param *data_len)
{
	#ifdef BLE_PEER_CONN_CENTRAL_TEST
		ble_iterative_conn_central();
		return 0;
	#endif

	#ifdef BLE_PEER_THROUGHPUT_TEST
		int err;
		uint64_t stamp;
		int64_t delta;
		uint32_t data = 0;

		/* a dummy data buffer */
		static char dummy[495];

		if (!default_conn) {
			shell_error(shell, "Device is disconnected %s",
					"Connect to the peer device before running test");
			return -EFAULT;
		}

		if (!test_ready) {
			shell_error(shell, "Device is not ready."
				"Please wait for the service discovery and MTU exchange end");
			return 0;
		}

		shell_print(shell, "==== Starting throughput test ====");

		err = connection_configuration_set_new(shell, conn_param, phy, data_len);
		if (err) {
			return err;
		}

		/* Make sure that all BLE procedures are finished. */
		k_sleep(K_MSEC(500));

		/* reset peer metrics */
		err = bt_throughput_write(&throughput, dummy, 1);
		if (err) {
			shell_error(shell, "Reset peer metrics failed.");
			return err;
		}

		/* get cycle stamp */
		stamp = k_uptime_get_32();

		delta = 0;
		while (true) {
			err = bt_throughput_write(&throughput, dummy, 495);
			if (err) {
				shell_error(shell, "GATT write failed (err %d)", err);
				break;
			}
			data += 495;
			if (k_uptime_get_32() - stamp > CONFIG_BLE_TEST_DURATION) {
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
			shell_error(shell, "GATT read failed (err %d)", err);
			return err;
		}

		k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));

		/* instruction_print(); */

		return 0;
	#endif
}

int connection_config_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len)
{
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return err;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		LOG_INF("'run' command shall be executed only on the central board");
	}

	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		LOG_ERR("PHY update failed: %d", err);
		return err;
	}

	LOG_INF("PHY update pending");
	err = k_sem_take(&throughput_sem, K_SECONDS(SCAN_CONFIG_TIMEOUT));
	if (err) {
		LOG_ERR("PHY update timeout");
		return err;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			LOG_ERR("LE data length update failed: %d",
					err);
			return err;
		}

		LOG_INF("LE Data length update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(SCAN_CONFIG_TIMEOUT));
		if (err) {
			LOG_ERR("LE Data Length update timeout");
			return err;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			LOG_ERR("Connection parameters update failed: %d",
					err);
			return err;
		}

		LOG_INF("Connection parameters update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(SCAN_CONFIG_TIMEOUT));
		if (err) {
			LOG_ERR("Connection parameters update timeout");
			return err;
		}
	}

	return 0;
}

int bt_connection_init(bool ble_role)
{
	(void) ble_role;
	int64_t stamp;

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

	uint32_t conn_cfg_status = connection_config_set(
			BT_LE_CONN_PARAM(INTERVAL_MIN,
			INTERVAL_MAX,
			CONN_LATENCY, SUPERVISION_TIMEOUT),
			BT_CONN_LE_PHY_PARAM_2M,
			BT_LE_DATA_LEN_PARAM_MAX);

	return conn_cfg_status;
}

void ble_iterative_conn_central(void)
{
	uint64_t test_start_time = 0;
	int err;

	test_start_time = k_uptime_get_32();

	while (true) {
		bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		/* Should wait for more time for conn establishment. So used 3 sec */
		k_sleep(K_SECONDS(3));

		/* LOG_INF("************ Disconect BLE **********"); */
		err = bt_disconnect_central();
		if (err) {
			LOG_ERR("Cannot disconnect!");
		}
		k_sleep(K_SECONDS(2));

		if (k_uptime_get_32() - test_start_time > BLE_CONN_CENTRAL_TEST_DURATION) {
			break;
		}
		k_sleep(K_SECONDS(1));
	}
	LOG_INF(" ble_conn_success_cnt = %u", ble_conn_success_cnt);
	LOG_INF(" ble_conn_fail_cnt = %u", ble_conn_fail_cnt);
	/* to stop scan after the results are printed */
	scan_init();
}

#endif


#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
void instruction_print(void)
{
	/**
	 * LOG_INF("Type 'config' to change the configuration parameters.");
	 * LOG_INF("You can use the Tab key to autocomplete your input.");
	 * LOG_INF("Type 'run' when you are ready to run the test.");
	 */
}
#endif

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
#ifdef PRINT_BLE_UPDATES
	LOG_INF("Filter not match. Address: %s connectable: %d",
				addr, connectable);
#endif
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
	LOG_ERR("MTU exchange %s", att_err == 0 ? "successful" : "failed");
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

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)

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
#ifdef PRINT_BLE_UPDATES
	LOG_INF("Connected as %s",
	      info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
	LOG_INF("Conn. interval is %u units", info.le.interval);
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
		/*#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)*/
		ble_periph_connected = true;
		/*#endif*/
	}
}
#endif

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
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
void disconnected(struct bt_conn *conn, uint8_t reason)
{
#ifdef PRINT_BLE_UPDATES
	LOG_INF("Disconnected (reason 0x%02x)", reason);
#endif

	test_ready = false;
	ble_periph_connected = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}
#endif
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
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
void throughput_received(const struct bt_throughput_metrics *met)
{
	static uint32_t kb;

	if (met->write_len == 0) {
		kb = 0;
		wait_for_ble_central_run = 1;
		LOG_INF("");

		return;
	}

	if ((met->write_len / 1024) != kb) {
		kb = (met->write_len / 1024);
		/* LOG_INF("="); */
	}
}
#endif
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
		LOG_INF("Central. Starting scanning");
		scan_start();
	} else {
		LOG_INF("Peripheral. Starting advertising");
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
	int err;

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
#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
int connection_configuration_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len)
{
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return err;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		LOG_INF("'run' command shall be executed only on the central board");
	}

	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		LOG_ERR("PHY update failed: %d", err);
		return err;
	}
#ifdef PRINT_BLE_UPDATES
	LOG_INF("PHY update pending");
#endif
	err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
	if (err) {
		LOG_INF("PHY update timeout");
		return err;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			LOG_ERR("LE data length update failed: %d",
				    err);
			return err;
		}
#ifdef PRINT_BLE_UPDATES
		LOG_INF("LE Data length update pending");
#endif
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			LOG_INF("LE Data Length update timeout");
			return err;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			LOG_ERR("Connection parameters update failed: %d",
				    err);
			return err;
		}

		LOG_INF("Connection parameters update pending");
		err = k_sem_take(&throughput_sem, K_SECONDS(THROUGHPUT_CONFIG_TIMEOUT));
		if (err) {
			LOG_INF("Connection parameters update timeout");
			return err;
		}
	}

	return 0;
}
#endif

int bt_throughput_test_run(void)
{
	int err;
	uint64_t stamp;
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
		if (k_uptime_get_32() - stamp > CONFIG_BLE_TEST_DURATION) {
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

	return 0;
}


/*#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)*/
static const struct bt_throughput_cb throughput_cb = {
	.data_read = throughput_read,
	.data_received = throughput_received,
	.data_send = throughput_send
};
/*#endif*/

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
int bt_throughput_test_init(bool ble_role)
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

	select_role(ble_role);

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
			BT_LE_CONN_PARAM(CONFIG_INTERVAL_MIN,
			CONFIG_INTERVAL_MAX,
			CONN_LATENCY, SUPERVISION_TIMEOUT),
			BT_CONN_LE_PHY_PARAM_2M,
			BT_LE_DATA_LEN_PARAM_MAX);
	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for connecion %lld ms", ble_scan2conn_time);
	 */

	return conn_cfg_status;
}
#endif

#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)
int bt_connection_init(bool ble_role)
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

	select_role(ble_role);

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
			BT_LE_CONN_PARAM(CONFIG_INTERVAL_MIN,
			CONFIG_INTERVAL_MAX,
			CONN_LATENCY, SUPERVISION_TIMEOUT),
			BT_CONN_LE_PHY_PARAM_2M,
			BT_LE_DATA_LEN_PARAM_MAX);
	/**
	 *ble_scan2conn_time = k_uptime_delta(&ble_scan2conn_start_time);
	 *LOG_INF("Time taken for connecion %lld ms", ble_scan2conn_time);
	 */

	return conn_cfg_status;
}
#endif

int bt_disconnect_central(void)
{
	int err;

	if (!default_conn) {
		LOG_INF("Not connected!");
		ble_conn_fail_cnt++;
		return -ENOTCONN;
	}
	ble_conn_success_cnt++;

	err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err) {
		LOG_INF("Cannot disconnect!");
		return err;
	}
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



#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)
void ble_peer_tp_conn(void) /* void main(void) */
{
	#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)
		int err;
		#ifdef BLE_PEER_THROUGHPUT_TEST
		LOG_INF("Starting Bluetooth Throughput example");
		#endif
		#ifdef BLE_PEER_CONN_CENTRAL_TEST
		LOG_INF("Starting test that runs BLE connection in central role");
		#endif

		err = bt_enable(NULL);
		if (err) {
			LOG_ERR("Bluetooth init failed (err %d)", err);
			return;
		}

		LOG_INF("Bluetooth initialized");

		scan_init();

		#ifdef BLE_PEER_THROUGHPUT_TEST
		err = bt_throughput_init(&throughput, &throughput_cb);
		if (err) {
			LOG_ERR("Throughput service initialization failed.");
			return;
		}
		#endif

		LOG_INF("");
		LOG_INF("Press button 1 or type \"central\" on the central board.");
		LOG_INF("Press button 2 or type \"peripheral\" on the peripheral board.");

		buttons_init();

		#ifdef BLE_PEER_CONN_CENTRAL_TEST
		bool ble_role = true;

		bt_connection_init(ble_role);

		#endif

	#endif
}

static void connected(struct bt_conn *conn, uint8_t hci_err)
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

	LOG_INF("Connected as %s",
		   info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
	LOG_INF("Conn. interval is %u units", info.le.interval);

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		err = bt_gatt_dm_start(default_conn,
					   BT_UUID_THROUGHPUT,
					   &discovery_cb,
					   &throughput);

		if (err) {
			LOG_ERR("Discover failed (err %d)", err);
		}
	}
}

#endif

/*#if !defined(BLE_PEER_THROUGHPUT_TEST) && !defined(BLE_PEER_CONN_CENTRAL_TEST)*/
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated
};
/*#endif*/
