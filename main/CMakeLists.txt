set(req_components esp_event esp_netif lwip nvs_flash usb mdns)

if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND req_components esp_eth)
elseif(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND req_components esp_wifi esp_driver_gpio)
endif()

idf_component_register(
    SRCS
        "main.c"
        "discovery_service.c"
        "network_init.c"
        "usb_backend.c"
        "usbip_server.c"
    INCLUDE_DIRS "."
    REQUIRES ${req_components}
)
