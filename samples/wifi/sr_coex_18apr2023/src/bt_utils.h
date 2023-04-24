/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_UTILS_H_
#define BT_UTILS_H_

/**
 * Initialize BLE throughput test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_throughput_test_init(bool ble_role);

/**
 * @brief Run BLE throughput test
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_throughput_test_run(void);

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
int bt_connection_init(bool ble_role);

/**
 * @brief Initialization for BT connection
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_disconnect_central(void);

#endif /* BT_UTILS_H_ */
