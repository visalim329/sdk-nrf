cp ../../prj_5g.conf ../../nrf/samples/wifi/sr_coex/prj.conf
cd ../../nrf/samples/wifi/sr_coex/
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_COEX_SEP_ANTENNAS=n  -DCONFIG_TEST_TYPE_WLAN_ONLY=y -DCONFIG_NRF_WIFI_LOW_POWER=y
mkdir wlan_only
cp -rf build/zephyr/merged.hex wlan_only/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_only/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_COEX_SEP_ANTENNAS=n -DCONFIG_TEST_TYPE_BLE_ONLY=y -DCONFIG_NRF_WIFI_LOW_POWER=y
mkdir ble_only
cp -rf build/zephyr/merged.hex ble_only/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex ble_only/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_COEX_SEP_ANTENNAS=n -DCONFIG_TEST_TYPE_WLAN_BLE=y -DCONFIG_NRF_WIFI_LOW_POWER=y
mkdir wlan_ble_cd
cp -rf build/zephyr/merged.hex wlan_ble_cd/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_ble_cd/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=y -Dhci_rpmsg_CONFIG_MPSL_CX=y -DCONFIG_COEX_SEP_ANTENNAS=n -DCONFIG_TEST_TYPE_WLAN_BLE=y -DCONFIG_NRF_WIFI_LOW_POWER=y
mkdir wlan_ble_ce
cp -rf build/zephyr/merged.hex wlan_ble_ce/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_ble_ce/merged_CPUNET.hex
mkdir hexes_sha_5g
mv wlan_only ble_only wlan_ble_cd wlan_ble_ce hexes_sha_5g/
mv hexes_sha_5g ../../../../

