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

/**
 * @brief Bluetooth connection configuration.
 *
 * @param Connection parameters
 */
int connection_config_set(const struct bt_le_conn_param *conn_param,
			const struct bt_conn_le_phy_param *phy,
			const struct bt_conn_le_data_len_param *data_len);

/**
 * @brief Bluetooth one time configuration and connection.
 *
 * @param None
 */
int bt_connection_init(void);

/**
 * @brief Iterative Bluetooth connection.
 *
 * @param None
 */
void ble_iterative_conn_central(void);

/**
 * @brief Disconnect the connection if role is central.
 *
 * @param None
 */
int bt_disconnect_central(void);
#endif /* THROUGHPUT_MAIN_H_ */
