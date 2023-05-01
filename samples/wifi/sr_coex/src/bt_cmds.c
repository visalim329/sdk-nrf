/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>

#include <zephyr/bluetooth/conn.h>

#include <errno.h>
#include <zephyr/shell/shell.h>
#include <zephyr/types.h>

#include "main.h"

#define INTERVAL_MIN 80 /* 80 units, 100 ms */
#define INTERVAL_MAX 80 /* 80 units, 100 ms */
#define CONN_LATENCY 0

#define MIN_CONN_INTERVAL   6
#define MAX_CONN_INTERVAL   3200
#define SUPERVISION_TIMEOUT 1000
#if 0

	//------------------------------
	#include <stddef.h>
	#include <zephyr/sys/printk.h>
	#include <zephyr/sys/util.h>
	#include <zephyr/sys/byteorder.h>

	//#include <zephyr/bluetooth/bluetooth.h>
	//#include <zephyr/bluetooth/hci.h>
	#include <zephyr/bluetooth/hci_vs.h>

	//#include <zephyr/bluetooth/conn.h>
	//#include <zephyr/bluetooth/uuid.h>
	//#include <zephyr/bluetooth/gatt.h>
	#include <zephyr/bluetooth/services/hrs.h>

	static struct bt_conn *default_conn;
	static uint16_t default_conn_handle;

	//static const struct bt_data ad[] = {
	//	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	//	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
	//};

	//#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
	//#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
	#define DEVICE_BEACON_TXPOWER_NUM  8

	static struct k_thread pwr_thread_data;
	static K_THREAD_STACK_DEFINE(pwr_thread_stack, 512);

	static const int8_t txp[DEVICE_BEACON_TXPOWER_NUM] = {4, 0, -3, -8,
								-15, -18, -23, -30};
	static const struct bt_le_adv_param *param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
				0x0020, 0x0020, NULL);
	//---------------------
#endif
#if defined(BLE_PEER_THROUGHPUT_TEST) || defined(BLE_PEER_CONN_CENTRAL_TEST)
static struct test_params {
	struct bt_le_conn_param *conn_param;
	struct bt_conn_le_phy_param *phy;
	struct bt_conn_le_data_len_param *data_len;
} test_params = {
	.conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, CONN_LATENCY,
				       SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

static const char *phy_str(const struct bt_conn_le_phy_param *phy)
{
	static const char *const str[] = {
		"1 Mbps",
		"2 Mbps",
		"Coded S2",
		"Coded S8",
		"Unknown"
	};

	switch (phy->pref_tx_phy) {
	case BT_GAP_LE_PHY_1M:
		return str[0];

	case BT_GAP_LE_PHY_2M:
		return str[1];

	case BT_GAP_LE_PHY_CODED:
		if (phy->options == BT_CONN_LE_PHY_OPT_CODED_S2) {
			return str[2];
		} else if (phy->options == BT_CONN_LE_PHY_OPT_CODED_S8) {
			return str[3];
		}

	default:
		return str[4];
	}
}

int default_cmd(const struct shell *shell, size_t argc,
		       char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	if (argc > 2) {
		shell_error(shell, "%s: bad parameters count.", argv[0]);
		return -EINVAL;
	}

	if (argc == 2) {
		shell_error(shell, "Uknown argument: %s", argv[1]);
		return -EINVAL;
	}

	return 0;
}

int cmd_phy_1m(const struct shell *shell, size_t argc,
		      char **argv)
{
	test_params.phy->options = BT_CONN_LE_PHY_OPT_NONE;
	test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_1M;
	test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_1M;

	shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

	return 0;
}

int cmd_phy_2m(const struct shell *shell, size_t argc,
		      char **argv)
{
	test_params.phy->options = BT_CONN_LE_PHY_OPT_NONE;
	test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_2M;
	test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_2M;

	shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

	return 0;
}

#if defined(RADIO_MODE_MODE_Ble_LR500Kbit) || defined(NRF5340_XXAA_APPLICATION)
int cmd_phy_coded_s2(const struct shell *shell, size_t argc,
			    char **argv)
{
	test_params.phy->options = BT_CONN_LE_PHY_OPT_CODED_S2;
	test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_CODED;
	test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_CODED;

	shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

	return 0;
}
#endif /* defined(RADIO_MODE_MODE_Ble_LR1500Kbit) ||
	* defined(NRF5340_XXAA_APPLICATION)
	*/

#if defined(RADIO_MODE_MODE_Ble_LR125Kbit) || defined(NRF5340_XXAA_APPLICATION)
int cmd_phy_coded_s8(const struct shell *shell, size_t argc,
			    char **argv)
{
	test_params.phy->options = BT_CONN_LE_PHY_OPT_CODED_S8;
	test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_CODED;
	test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_CODED;

	shell_print(shell, "PHY set to: %s",
		    phy_str(test_params.phy));

	return 0;
}
#endif /* defined(RADIO_MODE_MODE_Ble_LR125Kbit) ||
	* defined(NRF5340_XXAA_APPLICATION)
	*/

int data_len_cmd(const struct shell *shell, size_t argc,
			char **argv)
{
	uint16_t data_len;

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	if (argc > 2) {
		shell_error(shell, "%s: bad parameters count", argv[0]);
		return -EINVAL;
	}

	data_len = strtol(argv[1], NULL, 10);

	if ((data_len < BT_GAP_DATA_LEN_DEFAULT) ||
	    (data_len > BT_GAP_DATA_LEN_MAX)) {
		shell_error(shell, "%s: Invalid setting: %d", argv[0],
			    data_len);
		shell_error(shell,
			    "LE Data Packet Length must be between: %d and %d",
			    BT_GAP_DATA_LEN_DEFAULT, BT_GAP_DATA_LEN_MAX);
		return -EINVAL;
	}

	test_params.data_len->tx_max_len = data_len;
	test_params.data_len->tx_max_time = BT_GAP_DATA_TIME_MAX;

	shell_print(shell, "LE Data Packet Length set to: %d", data_len);

	return 0;
}

int conn_interval_cmd(const struct shell *shell, size_t argc,
			     char **argv)
{
	uint16_t interval;

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	if (argc > 2) {
		shell_error(shell, "%s: bad parameters count", argv[0]);
		return -EINVAL;
	}

	interval = strtol(argv[1], NULL, 10);

	if ((interval < MIN_CONN_INTERVAL) ||
	    (interval > MAX_CONN_INTERVAL)) {
		shell_error(shell, "%s: Invalid setting: %d", argv[0],
			    interval);
		shell_error(shell,
			    "Connection interval must be between: %d and %d",
			    MIN_CONN_INTERVAL, MAX_CONN_INTERVAL);
		return -EINVAL;
	}

	test_params.conn_param->interval_max = interval;
	test_params.conn_param->interval_min = interval;
	test_params.conn_param->latency = 0;
	test_params.conn_param->timeout = SUPERVISION_TIMEOUT;

	shell_print(shell, "Connection interval set to: %d",
		    interval);

	return 0;
}

int print_cmd(const struct shell *shell, size_t argc,
		     char **argv)
{
	shell_print(shell, "==== Current test configuration ====\n");
	shell_print(shell, "Data length:\t\t%d\n"
		    "Connection interval:\t%d units\n"
		    "Preferred PHY:\t\t%s\n",
		    test_params.data_len->tx_max_len,
		    test_params.conn_param->interval_min,
		    phy_str(test_params.phy));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(phy_sub,
	SHELL_CMD(1M, NULL, "Set preferred PHY to 1Mbps", cmd_phy_1m),
	SHELL_CMD(2M, NULL, "Set preferred PHY to 2Mbps", cmd_phy_2m),
#if defined(RADIO_MODE_MODE_Ble_LR500Kbit) || defined(NRF5340_XXAA_APPLICATION)
	SHELL_CMD(coded_s2, NULL, "Set preferred PHY to Coded S2",
		  cmd_phy_coded_s2),
#endif
#if defined(RADIO_MODE_MODE_Ble_LR125Kbit) || defined(NRF5340_XXAA_APPLICATION)
	SHELL_CMD(coded_s8, NULL, "Set preferred PHY to Coded S8",
		  cmd_phy_coded_s8),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
	SHELL_CMD(data_length, NULL, "Configure data length", data_len_cmd),
	SHELL_CMD(conn_interval, NULL,
		  "Configure connection interval <1.25ms units>",
		  conn_interval_cmd),
	SHELL_CMD(phy, &phy_sub, "Configure connection interval", default_cmd),
	SHELL_CMD(print, NULL, "Print current configuration", print_cmd),
	SHELL_SUBCMD_SET_END
);


int test_run_cmd(const struct shell *shell, size_t argc,
			char **argv)
{
	return test_run(shell, test_params.conn_param, test_params.phy,
			test_params.data_len);
}

int test_central_cmd(const struct shell *shell, size_t argc,
			    char **argv)
{
	select_role(true);
	return 0;
}
#if 0
static void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		printk("Read RSSI err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}

void modulate_tx_power(void *p1, void *p2, void *p3)
{
	int8_t txp_get = 0;
	uint8_t idx = 0;

	while (1) {
		if (!default_conn) {
			printk("Set Tx power level to %d\n", txp[idx]);
			set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
				     0, txp[idx]);

			k_sleep(K_SECONDS(5));

			printk("Get Tx power level -> ");
			get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
				     0, &txp_get);
			printk("TXP = %d\n", txp_get);

			idx = (idx+1) % DEVICE_BEACON_TXPOWER_NUM;
		} else {
			int8_t rssi = 0xFF;
			int8_t txp_adaptive;

			idx = 0;

			read_conn_rssi(default_conn_handle, &rssi);
			printk("Connected (%d) - RSSI = %d\n",
			       default_conn_handle, rssi);
			if (rssi > -70) {
				txp_adaptive = -20;
			} else if (rssi > -90) {
				txp_adaptive = -12;
			} else {
				txp_adaptive = -4;
			}
			printk("Adaptive Tx power selected = %d\n",
			       txp_adaptive);
			set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
				     default_conn_handle, txp_adaptive);
			get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
				     default_conn_handle, &txp_get);
			printk("Connection (%d) TXP = %d\n",
			       default_conn_handle, txp_get);

			k_sleep(K_SECONDS(1));
		}
	}
}

#endif

int test_peripheral_cmd(const struct shell *shell, size_t argc,
			       char **argv)
{
	select_role(false);
#if 0	
	////---------------------------------------------------------------------------
	//int8_t txp_get = 0xFF;
	//int err;
	//printk("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ 1\n");
	//default_conn = NULL;
	//printk("Starting Dynamic Tx Power Beacon Demo\n");
	//
	//printk("Get Tx power level ->");
	//get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, &txp_get);
	//printk("-> default TXP = %d\n", txp_get);
	//printk("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ 2\n");
	///* Wait for 5 seconds to give a chance users/testers
	// * to check that default Tx power is indeed the one
	// * selected in Kconfig.
	// */
	//k_sleep(K_SECONDS(5));
	//
	//k_thread_create(&pwr_thread_data, pwr_thread_stack,
	//		K_THREAD_STACK_SIZEOF(pwr_thread_stack),
	//		modulate_tx_power, NULL, NULL, NULL,
	//		K_PRIO_COOP(10),
	//		0, K_NO_WAIT);
	//k_thread_name_set(&pwr_thread_data, "DYN TX");
	//printk("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ 3\n");
	//---------------------------------------------------------------------------
#endif	
	return 0;
}

SHELL_CMD_REGISTER(config, &sub_config, "Configure the example", default_cmd);
SHELL_CMD_REGISTER(run, NULL, "Run the test", test_run_cmd);
SHELL_CMD_REGISTER(central, NULL, "Select central role", test_central_cmd);
SHELL_CMD_REGISTER(peripheral, NULL, "Select peripheral role", test_peripheral_cmd);
#endif
