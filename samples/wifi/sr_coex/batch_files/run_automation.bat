cd  basic_commands
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g_client_central.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_24g_client_central.log

cd ../basic_commands_peripheral
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g_client_peripheral.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_24g_client_peripheral.log

cd ../basic_commands_server_central
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g_server_central.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_24g_server_central.log

cd ../basic_commands_server_peripheral
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g_server_peripheral.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_24g_server_peripheral.log

