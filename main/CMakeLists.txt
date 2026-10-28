idf_component_register(
    SRCS
        "main.cpp"
        "cmd_sniffer.cpp"
        "cmd_tx.cpp"
        "config.cpp"
        "events.cpp"
        "fw_update.cpp"
        "led.cpp"
        "ota.cpp"
        "spi.cpp"
        "bt_stream.cpp"
        "usb_stream.cpp"
        "obu_comm.cpp"
        "obu_wifi.cpp"
    PRIV_REQUIRES
        app_trace
        app_update
        bt
        console
        esp_coex
        esp_driver_usb_serial_jtag
        esp_eth
        esp_https_ota
        esp_netif
        esp_wifi
        fatfs
        nvs_flash
    INCLUDE_DIRS
        "."
)


# C++20 zero-fills omitted designated-initializer fields; the warning only adds
# noise for the partially initialized ESP-IDF C structs used throughout.
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-missing-field-initializers>)
