/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Coexistence functions
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "zephyr_coex.h"
#include "zephyr_coex_struct.h"
#include "zephyr_fmac_main.h"
#include "fmac_api.h"

//LOG_MODULE_DECLARE(nrf_wifi, CONFIG_WIFI_LOG_LEVEL);

extern struct nrf_wifi_drv_priv_zep rpu_drv_priv_zep;
static struct nrf_wifi_ctx_zep *rpu_ctx = &rpu_drv_priv_zep.rpu_ctx_zep;

#define CH_BASE_ADDRESS ABS_EXT_SYS_WLANSYSCOEX_CH_CONTROL
#define WLAN_MAC_CTRL_BASE_ADDRESS ABS_PMB_WLAN_MAC_CTRL_PULSED_SOFTWARE_RESET
#define COEX_CONFIG_FIXED_PARAMS 4
#define COEX_MAX_WINDOW_MUL_100US 4095
#define COEX_WLAN_MAC_CTRL_CLK_FREQ 0x50
#define COEX_REG_CFG_OFFSET_SHIFT 24


/* copied from uccp530_77_registers_ext_sys_bus.h of UCCP toolkit */
#define EXT_SYS_WLANSYSCOEX_CH_CONTROL	0x0000
#define ABS_EXT_SYS_WLANSYSCOEX_CH_CONTROL	0xA401BC00UL
#define EXT_SYS_WLANSYSCOEX_ENABLE_SHIFT	1
#define EXT_SYS_WLANSYSCOEX_CH_TIME_REFERENCE	0x0004
#define EXT_SYS_WLANSYSCOEX_CH_SR_INFO_STATUS	0x0040
#define EXT_SYS_WLANSYSCOEX_CH_NO_WINDOW_LOOKUP_0	0x008C
#define EXT_SYS_WLANSYSCOEX_CH_NO_WINDOW_LOOKUP_44	0x013C
#define EXT_SYS_WLANSYSCOEX_CH_SR_WINDOW_LOOKUP_0	0x0078
#define EXT_SYS_WLANSYSCOEX_CH_SR_WINDOW_LOOKUP_4	0x0088
#define EXT_SYS_WLANSYSCOEX_CH_WLAN_WINDOW_LOOKUP_0	0x0064
#define EXT_SYS_WLANSYSCOEX_CH_WLAN_WINDOW_LOOKUP_4	0x0074
#define EXT_SYS_WLANSYSCOEX_RESET_SHIFT	0

/* copied from uccp530_77_registers.h of UCCP toolkit */
#define PMB_WLAN_MAC_CTRL_CLK_FREQ_MHZ_SHIFT	3
#define PMB_WLAN_MAC_CTRL_PTA_CONTROL_REG_MASK	0x00000007UL
#define PMB_WLAN_MAC_CTRL_PTA_CONTROL_REG_SHIFT	0
#define ABS_PMB_WLAN_MAC_CTRL_PTA_CONTROL	0xA5009A34UL
#define ABS_PMB_WLAN_MAC_CTRL_PULSED_SOFTWARE_RESET	0xA5009A00UL

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
#define NRF_RADIO_COEX_NODE DT_NODELABEL(nrf_radio_coex)
static const struct gpio_dt_spec btrf_switch_spec =
	GPIO_DT_SPEC_GET(NRF_RADIO_COEX_NODE, btrf_switch_gpios);
#endif

/* PTA registers configuration of Coexistence Hardware */
/* Separate antenna configuration tables */

/* Both WLAN and SR operates in 2.4GHz. WLAN and SR frequency of operation overlaps */
const uint16_t config_buffer_client_SEA1[] = {
	0x0019, 0x00F6, 0x0008, 0x0062, 0x00F5,
	0x00F5, 0x0019, 0x0019, 0x0074, 0x0074,
	0x0008, 0x01E2, 0x00D5, 0x00D5, 0x01F6,
	0x01F6, 0x0061, 0x0061, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x0019, 0x0019, 0x0008,
	0x0008, 0x00F5, 0x00F5, 0x00D5, 0x00D5,
	0x0008, 0x01E2, 0x0051, 0x0051, 0x0074,
	0x0074, 0x00F6, 0x0019, 0x0062, 0x0019,
	0x00F6, 0x0008, 0x0062, 0x0008, 0x001A
};

/* Both WLAN and SR operates in 2.4GHz. WLAN and SR frequency of operation overlaps */
const uint16_t config_buffer_server_SEA1[] = {
	0x0019, 0x00F6, 0x0008, 0x0062, 0x00F5,
	0x00F5, 0x00E2, 0x00E2, 0x01F6, 0x01F6,
	0x01E2, 0x01E2, 0x00D5, 0x00D5, 0x01F6,
	0x01F6, 0x00E1, 0x0061, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x00E1, 0x00E1, 0x0008,
	0x0008, 0x00F5, 0x00F5, 0x00D5, 0x00D5,
	0x01E2, 0x01E2, 0x00D1, 0x0051, 0x01F6,
	0x0074, 0x00F6, 0x0019, 0x0062, 0x0019,
	0x00F6, 0x0008, 0x0062, 0x0008, 0x001A
};




/* Both WLAN and SR operates in 2.4GHz. WLAN and SR frequency of operation doesn't overlap */
const uint16_t config_buffer_SEA2[] = {
	0x0019, 0x00F6, 0x0008, 0x0062, 0x00F5,
	0x00F5, 0x0019, 0x0019, 0x0074, 0x01F6,
	0x0060, 0x0060, 0x00D5, 0x00D5, 0x01F6,
	0x01F6, 0x0061, 0x0019, 0x0060, 0x0060,
	0x0004, 0x0004, 0x0019, 0x0019, 0x0060,
	0x0060, 0x00F5, 0x00F5, 0x00D5, 0x00D5,
	0x0060, 0x0060, 0x0061, 0x0019, 0x0074,
	0x01F6, 0x00F6, 0x0019, 0x00E2, 0x0019,
	0x00F6, 0x0008, 0x00E2, 0x0008, 0x001A
};

/* SR window */
const uint16_t config_buffer_SEA3[] = {
	0x01F8, 0x01E0, 0x01F4, 0x01E0, 0x0004
};

/* WLAN window */
const uint16_t config_buffer_SEA4[] = {
	0x0012, 0x0000, 0x0011, 0x0000, 0x0011
};
/* Shared antenna configuration tables */
/* SR request posted before the transaction */
const uint16_t config_buffer_server_central_SHA1[] = {
	0x0019, 0x00F6, 0x0008, 0x00E2, 0x00F5,
	0x00F5, 0x00E2, 0x00E2, 0x01F6, 0x01F6,
	0x01E2, 0x01E2, 0x00F5, 0x00F5, 0x01F6,
	0x01F6, 0x00E1, 0x00E1, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x00E2, 0x00E2, 0x0008,
	0x0008, 0x00F5, 0x00F5, 0x00F5, 0x00F5,
	0x01E2, 0x01E2, 0x00E1, 0x00E1, 0x01F6,
	0x01F6, 0x00F6, 0x0019, 0x00E2, 0x0019,
	0x00F6, 0x0008, 0x00E2, 0x0008, 0x001A
};

/* SR request posted during the transaction */
const uint16_t config_buffer_server_peripheral_SHA2[] = {
	0x0019, 0x0016, 0x0008, 0x00E2, 0x00F5,
	0x00F5, 0x0019, 0x0019, 0x01F6, 0x01F6,
	0x0008, 0x01E2, 0x00F5, 0x00F5, 0x01F6,
	0x01F6, 0x00E1, 0x00E1, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x00E1, 0x00E1, 0x0008,
	0x0008, 0x00F6, 0x00F6, 0x00F5, 0x00F5,
	0x0008, 0x01E2, 0x00E1, 0x00E1, 0x0004,
	0x0004, 0x00F6, 0x0019, 0x00E2, 0x0019,
	0x00F6, 0x0008, 0x00E2, 0x0008, 0x001A
};

/* SR request posted before the transaction */
const uint16_t config_buffer_client_central_SHA1[] = {
	0x0019, 0x00F6, 0x0008, 0x00E2, 0x0015,
	0x00F5, 0x0019, 0x0019, 0x0004, 0x01F6,
	0x0008, 0x01E2, 0x00F5, 0x00F5, 0x01F6,
	0x01F6, 0x00E1, 0x00E1, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x0019, 0x0019, 0x0008,
	0x0008, 0x0015, 0x00F5, 0x00F5, 0x00F5,
	0x0008, 0x01E2, 0x00E1, 0x00E1, 0x0004,
	0x01F6, 0x00F6, 0x0019, 0x00E2, 0x0019,
	0x00F6, 0x0008, 0x00E2, 0x0008, 0x001A
};

/* SR request posted during the transaction */
const uint16_t config_buffer_client_peripheral_SHA2[] = {
	0x0019, 0x0016, 0x0008, 0x00E2, 0x0015,
	0x0015, 0x0019, 0x0019, 0x0004, 0x0004,
	0x0008, 0x01E2, 0x00F5, 0x00F5, 0x01F6,
	0x01F6, 0x00E1, 0x00E1, 0x01E2, 0x0008,
	0x0004, 0x0004, 0x0019, 0x0019, 0x0008,
	0x0008, 0x0015, 0x0015, 0x00F5, 0x00F5,
	0x0008, 0x01E2, 0x00E1, 0x00E1, 0x0004,
	0x0004, 0x00F6, 0x0019, 0x00E2, 0x0019,
	0x00F6, 0x0008, 0x00E2, 0x0008, 0x001A
};

/* SR window */
const uint16_t config_buffer_SHA3[] = {
	0x01F8, 0x01E0, 0x01F4, 0x01E0, 0x0004
};

/* WLAN window */
uint16_t config_buffer_SHA4[] = {
	0x0012, 0x0000, 0x0011, 0x0000, 0x0011
};
/* Shared/separate antennas, WLAN in 5GHz */
const uint16_t config_buffer_5G[] = {
	0x0039, 0x0076, 0x0028, 0x0062, 0x0075,
	0x0075, 0x0061, 0x0061, 0x0074, 0x0074,
	0x0060, 0x0060, 0x0075, 0x0075, 0x0064,
	0x0064, 0x0071, 0x0071, 0x0060, 0x0060,
	0x0064, 0x0064, 0x0061, 0x0061, 0x0060,
	0x0060, 0x0075, 0x0075, 0x0075, 0x0075,
	0x0060, 0x0060, 0x0071, 0x0071, 0x0074,
	0x0074, 0x0076, 0x0039, 0x0062, 0x0039,
	0x0076, 0x0028, 0x0062, 0x0028, 0x003A
};

/* non-PTA register configuration of coexistence hardware */
/* Shared antenna */
const uint32_t ch_config_sha[] = {
	0x00000028, 0x00000000, 0x001E1023, 0x00000000, 0x00000000,
	0x00000000, 0x00000021, 0x000002ca, 0x00000050, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000
};

/* Separate antennas */
const uint32_t ch_config_sep[] = {
	0x00000028, 0x00000000, 0x001E1023, 0x00000000, 0x00000000,
	0x00000000, 0x00000021, 0x000002ca, 0x00000055, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000
};

int nrf_wifi_coex_config_non_pta(bool antenna_mode)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_ch_configuration params  = { 0 };
	const uint32_t *config_buffer_ptr = NULL;
	uint32_t start_offset = 0;
	uint32_t num_reg_to_config = 1;
	uint32_t cmd_len, index;

	/* Offset from the base address of CH */
	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_TIME_REFERENCE -
	EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	/* Number of registers to be configured */
	num_reg_to_config = ((EXT_SYS_WLANSYSCOEX_CH_SR_INFO_STATUS -
	EXT_SYS_WLANSYSCOEX_CH_TIME_REFERENCE) >> 2) + 1;

	if (antenna_mode) {
		config_buffer_ptr = ch_config_sep;
	} else {
		config_buffer_ptr = ch_config_sha;
	}

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	for (index = 0; index < num_reg_to_config; index++) {
		params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
			(*(config_buffer_ptr + index));
		start_offset++;
	}

	cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config) * sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
					   (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		return -1;
	}

	return 0;
}

int nrf_wifi_coex_config_pta(enum nrf_wifi_pta_wlan_op_band wlan_band,
							bool antenna_mode,
							bool ble_role, bool wlan_role)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_ch_configuration params  = { 0 };
	const uint16_t *config_buffer_ptr = NULL;
	uint32_t start_offset = 0;
	uint32_t num_reg_to_config = 1;
	uint32_t cmd_len, index;

	/* common for both SHA and SEA */
	/* Indicates offset from the base address of CH */
	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_NO_WINDOW_LOOKUP_0 -
	EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	/* Number of contiguous registers to be configured starting from base+offset */
	num_reg_to_config = ((EXT_SYS_WLANSYSCOEX_CH_NO_WINDOW_LOOKUP_44 -
	EXT_SYS_WLANSYSCOEX_CH_NO_WINDOW_LOOKUP_0) >> 2) + 1;

	if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_2_4_GHZ) {
		/* WLAN operating in 2.4GHz */
		if (antenna_mode) {
			/* separate antennas configuration */
			if (wlan_role) {
				/* WLAN role server */
				config_buffer_ptr = config_buffer_server_SEA1;
			} else {
				/* WLAN role client */
				config_buffer_ptr = config_buffer_client_SEA1;
			}
		} else {
			/* Shared antenna configuration */
			if (wlan_role) {
				/* WLAN role server */
				if (ble_role) {
					/* BLE role central */
					config_buffer_ptr = config_buffer_server_central_SHA1;
				} else {
					/* BLE role peripheral */
					config_buffer_ptr = config_buffer_server_peripheral_SHA2;
				}
			} else {
				/* WLAN role client */
				if (ble_role) {
					/* BLE role central */
					config_buffer_ptr = config_buffer_client_central_SHA1;
				} else {
					/* BLE role peripheral */
					config_buffer_ptr = config_buffer_client_peripheral_SHA2;
				}
			}
		}
	} else if (wlan_band == NRF_WIFI_PTA_WLAN_OP_BAND_5_GHZ) {
		/* WLAN operating in 5GHz */
		config_buffer_ptr = config_buffer_5G;
	} else {
		return -EINVAL;
	}

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	for (index = 0; index < num_reg_to_config; index++) {
		params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
			(*(config_buffer_ptr+index));
		start_offset++;
	}

	cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config) * sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
					   (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		return -1;
	}
	/* SR window */
	if (antenna_mode) {
		/* separate antennas configuration */
		config_buffer_ptr = config_buffer_SEA3;
	} else {
		/* Shared antenna configuration */
		config_buffer_ptr = config_buffer_SHA3;
	}

	/* Indicates offset from the base address of CH */
	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_SR_WINDOW_LOOKUP_0 -
	EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	/* Number of contiguous registers to be configured starting from base+offset */
	num_reg_to_config = ((EXT_SYS_WLANSYSCOEX_CH_SR_WINDOW_LOOKUP_4 -
	EXT_SYS_WLANSYSCOEX_CH_SR_WINDOW_LOOKUP_0) >> 2) + 1;

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	for (index = 0; index < num_reg_to_config; index++) {
		params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
			(*(config_buffer_ptr+index));
		start_offset++;
	}

	cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config) * sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
					   (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		return -1;
	}

	/* WLAN window */
	if (antenna_mode) {
		/* separate antennas configuration */
		config_buffer_ptr = config_buffer_SEA4;
	} else {
		/* Shared antenna configuration */
		config_buffer_ptr = config_buffer_SHA4;
	}

	/* common for both SHA and SEA */
	/* Indicates offset from the base address of CH */
	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_WLAN_WINDOW_LOOKUP_0 -
	EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	/* Number of contiguous registers to be configured starting from base+offset */
	num_reg_to_config = ((EXT_SYS_WLANSYSCOEX_CH_WLAN_WINDOW_LOOKUP_4 -
	EXT_SYS_WLANSYSCOEX_CH_WLAN_WINDOW_LOOKUP_0) >> 2) + 1;

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	for (index = 0; index < num_reg_to_config; index++) {
		params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
			(*(config_buffer_ptr+index));
		start_offset++;
	}

	cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config) * sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
					   (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		return -1;
	}

	return 0;
}

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
int nrf_wifi_config_sr_switch(bool antenna_mode, bool bt_external_antenna)
{
	int ret;

	if (!device_is_ready(btrf_switch_spec.port)) {
		//LOG_ERR("Unable to open GPIO device\n");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&btrf_switch_spec, GPIO_OUTPUT);
	if (ret < 0) {
		//LOG_ERR("Unable to configure GPIO device\n");
		return -1;
	}

	
	if (bt_external_antenna) {
		gpio_pin_set_dt(&btrf_switch_spec, 0x1);
		////LOG_INF("Antenna used: External.\n");
		////LOG_INF("Antenna mode: Separate.\n");
		////LOG_INF("GPIO P1.10 set to 1\n");
	} else {
		if (antenna_mode) {
			gpio_pin_set_dt(&btrf_switch_spec, 0x0);
			////LOG_INF("Antenna used: Chip. Separate antennas.\n");
			////LOG_INF("GPIO P1.10 set to 0\n");
		} else {
			gpio_pin_set_dt(&btrf_switch_spec, 0x1);
			////LOG_INF("Antenna used: Chip. Shared antenna.\n");
			////LOG_INF("GPIO P1.10 set to 1\n");
		}
	}
	////LOG_INF("Successfully configured GPIO P1.10\n");

	return 0;
}
#endif

int nrf_wifi_coex_hw_reset(void)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_ch_configuration params  = { 0 };
	uint32_t num_reg_to_config = 1;
	uint32_t start_offset = 0;
	uint32_t index = 0;
	uint32_t coex_hw_reset = 1;
	uint32_t cmd_len;

	/* reset CH */
	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_CONTROL - EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
	(coex_hw_reset << EXT_SYS_WLANSYSCOEX_RESET_SHIFT);

	cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config) * sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
				(void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("CH reset configuration failed\n");
		return -1;
	}

	return 0;
}

int nrf_wifi_coex_hw_enable(bool coex_hw_enable)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_ch_configuration params  = { 0 };
	uint32_t num_reg_to_config = 1;
	uint32_t start_offset = 0;
	uint32_t index = 0;

	if ((uint32_t)coex_hw_enable > 1) {
		//LOG_ERR("Coex: Invalid coex enable config (%d)\n", coex_hw_enable);
		return -ENOEXEC;
	}

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_COEX_HARDWARE;
	params.hw_block_base_addr = CH_BASE_ADDRESS;

	start_offset = ((EXT_SYS_WLANSYSCOEX_CH_CONTROL - EXT_SYS_WLANSYSCOEX_CH_CONTROL) >> 2);
	params.configbuf[index] = (start_offset << COEX_REG_CFG_OFFSET_SHIFT) |
	(coex_hw_enable << EXT_SYS_WLANSYSCOEX_ENABLE_SHIFT);

	uint32_t cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config)*sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx, (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: CH enable configuration failed\n");
		return -ENOEXEC;
	}

	return 0;
}

int nrf_wifi_coex_enable(bool wifi_coex_enable)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_ch_configuration params  = { 0 };
	uint32_t num_reg_to_config = 1;

	/* ABS_PMB_WLAN_MAC_CTRL_PTA_CONTROL */
	/* parameters for coex enable as default */
	uint32_t register_value = 0;
	uint32_t phy_rx_req_enable = 1;
	uint32_t mac_rx_req_enable = 1;
	uint32_t mac_tx_req_enable = 1;

	uint32_t pta_if_ctrl = 0;
	uint32_t config_value = 0;

	if ((uint32_t)wifi_coex_enable > 1) {
		//LOG_ERR("Invalid WLAN coex enable config (%d)\n", wifi_coex_enable);
		return -ENOEXEC;
	}

	if (wifi_coex_enable == 0) {
		phy_rx_req_enable = 0;
		mac_rx_req_enable = 0;
		mac_tx_req_enable = 0;
	}

	num_reg_to_config = 1;

	register_value = COEX_WLAN_MAC_CTRL_CLK_FREQ << PMB_WLAN_MAC_CTRL_CLK_FREQ_MHZ_SHIFT;
	config_value = register_value & (~PMB_WLAN_MAC_CTRL_PTA_CONTROL_REG_MASK);
	pta_if_ctrl = (phy_rx_req_enable << 2)  | (mac_rx_req_enable << 1) |
				  (mac_tx_req_enable << 0);
	pta_if_ctrl = (pta_if_ctrl << PMB_WLAN_MAC_CTRL_PTA_CONTROL_REG_SHIFT);
	pta_if_ctrl = config_value | pta_if_ctrl;

	uint32_t pta_control_offset = (ABS_PMB_WLAN_MAC_CTRL_PTA_CONTROL -
	WLAN_MAC_CTRL_BASE_ADDRESS) >> 2;

	params.message_id = NRF_WIFI_HW_CONFIGURATION;
	params.num_reg_to_config = num_reg_to_config;
	params.hw_to_config = NRF_WIFI_MAC_CTRL;
	params.hw_block_base_addr = WLAN_MAC_CTRL_BASE_ADDRESS;

	params.configbuf[0] = (pta_control_offset<<COEX_REG_CFG_OFFSET_SHIFT) | (pta_if_ctrl);

	uint32_t cmd_len = (COEX_CONFIG_FIXED_PARAMS + num_reg_to_config)*sizeof(uint32_t);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx, (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: WLAN coex enable configuration failed\n");
		return -ENOEXEC;
	}

	return 0;
}

/*  External single priority window */
int nrf_wifi_coex_allocate_spw(uint32_t device_req_window,
	uint32_t window_start_or_end,
	uint32_t imp_of_request,
	uint32_t can_be_deferred)
{
	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_allocate_pti_window params  = { 0 };

	params.message_id = NRF_WIFI_ALLOCATE_PTI_WINDOW;
	params.device_req_window = device_req_window;
	params.window_start_or_end = window_start_or_end;
	params.imp_of_request = imp_of_request;
	params.can_be_deferred = can_be_deferred;

	if (params.device_req_window > NRF_WIFI_WIFI_DEVICE) {
		//LOG_ERR("Invalid device requesting window (%d)\n",
		//		  params.device_req_window);
		return -ENOEXEC;
	}
	if (params.window_start_or_end > NRF_WIFI_START_REQ_WINDOW) {
		//LOG_ERR("Invalid window start/end (%d)\n",
		//		  params.window_start_or_end);
		return -ENOEXEC;
	}

	if (params.imp_of_request > NRF_WIFI_HIGHEST_IMPORTANCE) {
		//LOG_ERR("Invalid importance level of window (%d)\n",
		//		  params.imp_of_request);
		return -ENOEXEC;
	}

	if (params.can_be_deferred > NRF_WIFI_YES) {
		//LOG_ERR("Invalid window deferred option (%d)\n",
		//		  params.can_be_deferred);
		return -ENOEXEC;
	}

	uint32_t cmd_len = sizeof(struct nrf_wifi_coex_allocate_pti_window);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx,
					   (void *)(&params), cmd_len);


	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: configuration for SPW allocation failed\n");
		return -ENOEXEC;
	}

	return 0;
}

int nrf_wifi_coex_allocate_ppw(uint32_t start_or_stop,
	uint32_t first_pti_window,
	uint32_t wifi_window_duration,
	uint32_t sr_window_duration)
{

	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_allocate_ppw params  = { 0 };

	params.message_id = NRF_WIFI_ALLOCATE_PPW;
	params.start_or_stop = start_or_stop;
	params.first_pti_window = first_pti_window;
	params.wifi_window_duration = wifi_window_duration;
	params.sr_window_duration = sr_window_duration;

	if (params.start_or_stop > NRF_WIFI_START_ALLOC_WINDOWS) {
		//LOG_ERR("Invalid start/stop PPW windows (%d)\n",
		//		  params.start_or_stop);
		return -ENOEXEC;
	}
	if (params.first_pti_window > NRF_WIFI_SR_WINDOW) {
		//LOG_ERR("Invalid first priority window (%d)\n",
		//		  params.first_pti_window);
		return -ENOEXEC;
	}

	if (params.wifi_window_duration > COEX_MAX_WINDOW_MUL_100US) {
		//LOG_ERR("Invalid WLAN window duration (%d)\n",
		//		  params.wifi_window_duration);
		return -ENOEXEC;
	}

	if (params.sr_window_duration > COEX_MAX_WINDOW_MUL_100US) {
		//LOG_ERR("Invalid SR window duration (%d)\n",
		//		  params.sr_window_duration);

		return -ENOEXEC;
	}

	uint32_t cmd_len = sizeof(struct nrf_wifi_coex_allocate_ppw);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx, (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: configuration for PPW allocation failed\n");
		return -ENOEXEC;
	}

	return 0;
}

int nrf_wifi_coex_start_collect_sr_traffic_info(int32_t num_sets_requested)
{

	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_collect_sr_traffic_info params  = { 0 };

	params.message_id = NRF_WIFI_COLLECT_SR_TRAFFIC_INFO;
	params.num_sets_requested = num_sets_requested;


	uint32_t cmd_len = sizeof(struct nrf_wifi_coex_collect_sr_traffic_info);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx, (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: configuration for PPW allocation failed\n");
		return -ENOEXEC;
	}

	return 0;
}


int nrf_wifi_coex_allocate_vpw(int32_t start_or_stop,
	uint32_t wifi_window_duration)
{

	enum nrf_wifi_status status = NRF_WIFI_STATUS_FAIL;
	struct nrf_wifi_coex_allocate_vpw params  = { 0 };

	params.message_id = NRF_WIFI_ALLOCATE_VPW;
	params.start_or_stop = start_or_stop;
	params.wifi_window_duration = wifi_window_duration;

	if (params.start_or_stop > NRF_WIFI_START_ALLOC_WINDOWS) {
		//LOG_ERR("Invalid start/stop VPW windows (%d)\n",
		//		  params.start_or_stop);
		return -ENOEXEC;
	}

	if (params.wifi_window_duration > COEX_MAX_WINDOW_MUL_100US) {
		//LOG_ERR("Invalid WLAN window duration (%d)\n",
		//		  params.wifi_window_duration);
		return -ENOEXEC;
	}

	uint32_t cmd_len = sizeof(struct nrf_wifi_coex_allocate_vpw);

	status = nrf_wifi_fmac_conf_btcoex(rpu_ctx->rpu_ctx, (void *)(&params), cmd_len);

	if (status != NRF_WIFI_STATUS_SUCCESS) {
		//LOG_ERR("Coex: configuration for VPW allocation failed\n");
		return -ENOEXEC;
	}

	return 0;
}
