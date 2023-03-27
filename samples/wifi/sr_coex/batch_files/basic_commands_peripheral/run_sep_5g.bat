cp ../../prj_5g.conf ../../nrf/samples/wifi/sr_coex/prj.conf
cd ../../nrf/samples/wifi/sr_coex/
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_WIFI_ZPERF_SERVER=n -DCONFIG_COEX_BT_CENTRAL=n -DCONFIG_NET_PKT_RX_COUNT=2 -DCONFIG_NET_BUF_RX_COUNT=2 -DCONFIG_NET_PKT_TX_COUNT=24 -DCONFIG_NET_BUF_TX_COUNT=48 -DCONFIG_COEX_SEP_ANTENNAS=y -DCONFIG_TEST_TYPE_WLAN_ONLY=y 
mkdir wlan_only
cp -rf build/zephyr/merged.hex wlan_only/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_only/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_WIFI_ZPERF_SERVER=n -DCONFIG_COEX_BT_CENTRAL=n -DCONFIG_NET_PKT_RX_COUNT=2 -DCONFIG_NET_BUF_RX_COUNT=2 -DCONFIG_NET_PKT_TX_COUNT=24 -DCONFIG_NET_BUF_TX_COUNT=48 -DCONFIG_COEX_SEP_ANTENNAS=y -DCONFIG_TEST_TYPE_BLE_ONLY=y
mkdir ble_only
cp -rf build/zephyr/merged.hex ble_only/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex ble_only/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=n -Dhci_rpmsg_CONFIG_MPSL_CX=n -DCONFIG_WIFI_ZPERF_SERVER=n -DCONFIG_COEX_BT_CENTRAL=n -DCONFIG_NET_PKT_RX_COUNT=2 -DCONFIG_NET_BUF_RX_COUNT=2 -DCONFIG_NET_PKT_TX_COUNT=24 -DCONFIG_NET_BUF_TX_COUNT=48 -DCONFIG_COEX_SEP_ANTENNAS=y -DCONFIG_TEST_TYPE_WLAN_BLE=y
mkdir wlan_ble_cd
cp -rf build/zephyr/merged.hex wlan_ble_cd/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_ble_cd/merged_CPUNET.hex
west build -p -b nrf7002dk_nrf5340_cpuapp -- -DCONFIG_MPSL_CX=y -Dhci_rpmsg_CONFIG_MPSL_CX=y -DCONFIG_WIFI_ZPERF_SERVER=n -DCONFIG_COEX_BT_CENTRAL=n -DCONFIG_NET_PKT_RX_COUNT=2 -DCONFIG_NET_BUF_RX_COUNT=2 -DCONFIG_NET_PKT_TX_COUNT=24 -DCONFIG_NET_BUF_TX_COUNT=48 -DCONFIG_COEX_SEP_ANTENNAS=y -DCONFIG_TEST_TYPE_WLAN_BLE=y
mkdir wlan_ble_ce
cp -rf build/zephyr/merged.hex wlan_ble_ce/merged.hex
cp -rf build/hci_rpmsg/zephyr/merged_CPUNET.hex wlan_ble_ce/merged_CPUNET.hex
mkdir hexes_sep_5g
mv wlan_only ble_only wlan_ble_cd wlan_ble_ce hexes_sep_5g/
mv hexes_sep_5g ../../../../


