/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_UTILS_H_
#define BT_UTILS_H_


#define K_SLEEP_100MSEC 100
#define K_SLEEP_1SEC K_SECONDS(1)
#define K_SLEEP_2SEC K_SECONDS(2)

/**
 * Initialize BLE throughput test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_throughput_test_init(bool is_ble_central);

/**
 * @brief Run BLE throughput test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_throughput_test_run(void);

/**
 * @brief Run BLE connection test
 */
void bt_conn_test_run(void);

/**
 * @brief Exit BLE throughput test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_throughput_test_exit(void);
/**
 * Initialize BLE scan --> connection test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_connection_init(bool is_ble_central);

/**
 * @brief Initialization for BT connection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_disconnect_central(void);

#endif /* BT_UTILS_H_ */
