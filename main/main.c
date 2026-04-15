#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "discovery_service.h"
#include "log_server.h"
#include "network_init.h"
#include "usb_backend.h"
#include "usbip_server.h"
#if CONFIG_USBIP_VIRTUAL_LOGIC_ANALYZER
#include "virtual_perfetto_logic.h"
#endif

static const char *TAG = "app";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(network_init_start());
    ESP_ERROR_CHECK(log_server_start());
    ESP_ERROR_CHECK(usb_backend_start());
#if CONFIG_USBIP_VIRTUAL_LOGIC_ANALYZER
    ESP_ERROR_CHECK(virtual_perfetto_logic_start());
#endif
    ESP_ERROR_CHECK(discovery_service_start());
    ESP_ERROR_CHECK(usbip_server_start());

    ESP_LOGI(TAG, "USB/IP bridge started");
}
