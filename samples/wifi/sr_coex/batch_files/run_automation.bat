cd basic_commands
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_24g.log
cd ../powersave_enable
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_powersave_en_24g.log
cd ../powersave_disable
./run.bat
./copy_hex.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_24g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_powersave_dis_24g.log
cd ../basic_commands
./copy_hex_5g.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_5g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_5g.log
cd ../powersave_enable
./copy_hex_5g.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_5g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_powersave_en_5g.log
cd ../powersave_disable
./copy_hex_5g.bat
cd /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/
./bt_open_5g.sh
cd -
cp /home/murali/visali/coex_automation_test/stqa/automation/ctf/test-sdk-wifi/outcomes/logs/wifi.log wifi_ble_powersave_dis_5g.log
