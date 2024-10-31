idf_component_register(SRCS "cmd_ble.c" "gatts_table_creat_demo.c" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES led-display-manager nvs_flash bt serial_console cmd_nvs cmd_system AsyncUDP ESPAsyncE131)
                    # also want to add ESPAsyncE131, but it uses Arduino library and only Wifi interface