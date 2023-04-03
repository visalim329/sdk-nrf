/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef THROUGHPUT_MAIN_H_
#define THROUGHPUT_MAIN_H_

/**
 * @brief Run the test
 *
 * @param shell       Shell instance where output will be printed.
 * @param conn_param  Connection parameters.
 * @param phy         Phy parameters.
 * @param data_len    Maximum transmission payload.
 */

#define WIFI_SCAN_BLE_CON_PERIPH
//#define DEFAULT_BT_TPUT_TEST

int test_run(const struct shell *shell,
	     const struct bt_le_conn_param *conn_param,
	     const struct bt_conn_le_phy_param *phy,
	     const struct bt_conn_le_data_len_param *data_len);

/**
 * @brief Set the board into a specific role.
 *
 * @param is_central true for central role, false for peripheral role.
 */
void select_role(bool is_central);

int connection_config_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len);
			
int bt_connection_init(void);
int wifi_scan_ble_conn_central(void);
int bt_disconnect_central(void);

#endif /* THROUGHPUT_MAIN_H_ */
