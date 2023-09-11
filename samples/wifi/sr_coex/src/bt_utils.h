/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_UTILS_H_
#define BT_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

#define RSSI_INIT_VALUE 127

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

/**
 * @brief Start BLE advertisement
 *
 * @return None
 */
void adv_start(void);

/**
 * @brief Start BLE scan
 *
 * @return None
 */
void scan_start(void);

/**
 * @brief Read BLE RSSI
 *
 * @return None
 */
void read_conn_rssi(uint16_t handle, int8_t *rssi);

/**
 * @brief Set BLE Tx power
 *
 * @return None
 */
void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl);

/**
 * @brief Get BLE Tx power
 *
 * @return None
 */
void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl);

#endif /* BT_UTILS_H_ */
