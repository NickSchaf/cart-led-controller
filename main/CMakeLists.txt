idf_component_register(SRCS "cmd_ble.c" "gatts_table_creat_demo.c" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES led-controller nvs_flash bt serial_console cmd_nvs cmd_system)