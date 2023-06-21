/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @brief Header containing Coexistence APIs.
 */

#ifndef __ZEPHYR_FMAC_COEX_H__
#define __ZEPHYR_FMAC_COEX_H__

#include <stdbool.h>

/* Indicates WLAN frequency band of operation */
enum nrf_wifi_pta_wlan_op_band {
	NRF_WIFI_PTA_WLAN_OP_BAND_2_4_GHZ = 0,
	NRF_WIFI_PTA_WLAN_OP_BAND_5_GHZ,

	NRF_WIFI_PTA_WLAN_OP_BAND_NONE = 0xFF
};

/**
 * @function   nrf_wifi_coex_config_pta(enum nrf_wifi_pta_wlan_op_band wlan_band,
 *             bool antenna_mode, bool ble_role, bool wlan_role)
 *
 * @brief      Function used to configure PTA tables of coexistence hardware.
 *
 * @param[in]  enum nrf_wifi_pta_wlan_op_band wlan_band
 * @param[in]  antenna_mode
 *             Indicates whether separate antenans are used or not.
 * @param[in]  ble_role
 *             Indicates whether BLE role is central or not.
 @param[in]    wlan_role
 *             Indicates whether WLAN role is server or not.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_config_pta(enum nrf_wifi_pta_wlan_op_band wlan_band, bool antenna_mode,
		bool ble_role, bool wlan_role);

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
/**
 * @function   nrf_wifi_config_sr_switch(bool antenna_mode, bool bt_external_antenna)
 *
 * @brief      Function used to configure SR side switch (nRF5340 side switch in nRF7002 DK).
 *
 * @param[in]  antenna_mode
 *               Indicates whether separate antenans are used or not.
  * @param[in] bt_external_antenna
 *               Indicates whether the external antenna to be used or not.
 *
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_config_sr_switch(bool antenna_mode, bool bt_external_antenna);
#endif

/**
 * @function   nrf_wifi_coex_config_non_pta(bool antenna_mode)
 *
 * @brief      Function used to configure non-PTA registers of coexistence hardware.
 *
 * @param[in]  antenna_mode
 *             Indicates whether separate antenans are used or not.
 *
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_config_non_pta(bool antenna_mode);

/**
 * @function   nrf_wifi_coex_hw_reset(void)
 *
 * @brief      Function used to reset coexistence hardware.
 *
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_hw_reset(void);

/**
 * @function   nrf_wifi_coex_hw_enable(bool coex_hw_enable)
 *
 * @brief      Function used to enable or disable the Coexistence Hardware(CH) module.
 *
 * @param[in]  coex_hw_enable
 *             Indicates whether CH is enabled or disabled.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_hw_enable(bool coex_hw_enable);

/**
 * @function   nrf_wifi_coex_enable(bool wifi_coex_enable)
 *
 * @brief      Function used to enable or disable Wi-Fi coexistence.
 *             Enabling Wi-Fi coexistence means, Wi-Fi posts requests
 *             to PTA and considers grant from PTA to continue/abort transactions.
 *
 * @param[in]  wifi_coex_enable
 *             To enable, set this to COEX_ENABLE.
 *             To disable, set this to COEX_DISABLE.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_enable(bool wifi_coex_enable);

/**
 * @function   nrf_wifi_coex_allocate_spw(uint32_t device_req_window, uint32_t window_start_or_end,
 *                 uint32_t imp_of_request,uint32_t can_be_deferred
 *
 * @brief      Function used to request coexistence hardware for a priority window.
 * @param[in]  device_req_window
 *             Indicates device requesting a priority window.
 *             Set this to WIFI_DEVICE if window is for Wi-Fi device.
 *             Set this to SR_DEVICE if window is for SR device.
 * @param[in]  window_start_or_end
 *             Indicates if request is posted to START or END a priority window .
 *             Set to START_REQ_WINDOW to request a priority window before the
 *             start of the activity.
 *             Set to END_REQ_WINDOW to request to stop allocation of priority
 *             window after the end of the activity.
 * @param[in]  can_be_deferred
 *             Indicates if the activity for which the window is requested can bedeferred or not.
 *             Set to YES or NO depending on the type of activity that Wi-Fi/SR needs to protect.
 * @param[in]  imp_of_request
 *             Indicates importance of activity for which priority window is requested.
 *             Set to LESS_IMPORTANC or MEDIUM_IMPORTANCE or HIGH_IMPORTANCE or
 *             HIGHEST_IMPORTANCE depending on the activity that Wi-Fi/SR device
 *             needs to protect.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_allocate_spw(uint32_t device_req_window,
	uint32_t window_start_or_end,
	uint32_t imp_of_request,
	uint32_t can_be_deferred);

/**
 * @function   nrf_wifi_coex_allocate_ppw(int32_t start_or_stop, uint32_t first_pti_window,
 *                  uint32_t wifi_window_duration, uint32_t sr_window_duration)
 *
 * @brief      Function used to allocate Periodic Priority Windows (PPWs) to Wi-Fi and SR devices.
 *
 * @param[in]  start_or_stop
 *             Indicates whether to start or stop allocation of PPWs.
 *             Set to START_ALLOC_WINDOWS to start allocation of PPWs.
 *             Set to STOP_ALLOC_WINDOWS to stop allocation of PPWs.
 * @param[in]  first_pti_window
 *             Indicates the first-priority window in the series of PPWs.
 *             Set to WIFI_WINDOW to allocate first priority window to Wi-Fi.
 *             Set to SR_WINDOW to allocate first priority window to SR.
 * @param[in]  wifi_window_duration
 *             Indicates duration of Wi-Fi priority window in multiple of 100us.
 *             Maximum value allowed is 4095.
 * @param[in]  sr_window_duration
 *             Indicates duration of SR priority window in multiple of 100us.
 *             Maximum value allowed is 4095.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_allocate_ppw(uint32_t start_or_stop,
	uint32_t first_pti_window,
	uint32_t wifi_window_duration,
	uint32_t sr_window_duration);

/**
 * @function   nrf_wifi_coex_start_collect_sr_traffic_info(int32_t num_sets_requested)
 *
 * @brief      Function used to collect SR traffic information.
 *
 * @param[in]  num_sets_requested
 *             Number of sets of duration and periodicity of SR traffic to be collected.
 * @return     None.
 */
int nrf_wifi_coex_start_collect_sr_traffic_info(int32_t num_sets_requested);

/**
 * @function   nrf_wifi_coex_allocate_vpw(uint32_t start_or_stop, uint32_t first_pti_window,
 *                  uint32_t wifi_window_duratuint32_t wifi_window_duration)
 *
 * @brief      Function used to allocate Virtual Priority Windows (VPWs) to Wi-Fi.
 *
 * @param[in]  start_or_stop
 *             Indicates whether to start or stop allocation of PPWs.
 *             Set to START_ALLOC_WINDOWS to start allocation of PPWs.
 *             Set to STOP_ALLOC_WINDOWS to stop allocation of PPWs.
 * @param[in]  wifi_window_duration
 *             Indicates duration of Wi-Fi priority window in multiple of 100us.
 *             Maximum value allowed is 4095.
 * @return     Returns status of configuration.
 *             Returns zero upon successful configuration.
 *             Returns non-zero upon unsuccessful configuration.
 */
int nrf_wifi_coex_allocate_vpw(int32_t start_or_stop,
	uint32_t wifi_window_duration);

#endif /* __ZEPHYR_FMAC_COEX_H__ */
