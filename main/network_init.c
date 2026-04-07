#include "network_init.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "sdkconfig.h"

#if CONFIG_USBIP_TRANSPORT_ETHERNET
#include "esp_eth.h"
#endif

#if CONFIG_USBIP_TRANSPORT_WIFI
#include "esp_wifi.h"
#endif

static const char *TAG = "network";

#if CONFIG_USBIP_TRANSPORT_ETHERNET
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    if (event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet link up");
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "Ethernet link down");
    } else if (event_id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "Ethernet started");
    } else if (event_id == ETHERNET_EVENT_STOP) {
        ESP_LOGW(TAG, "Ethernet stopped");
    }
}

static void got_eth_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;

    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Ethernet IPv4: " IPSTR, IP2STR(&event->ip_info.ip));
}

static esp_err_t network_init_ethernet(void)
{
    esp_err_t err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_eth_ip_event_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
    if (eth_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_USBIP_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_USBIP_ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = CONFIG_USBIP_ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = CONFIG_USBIP_ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == NULL) {
        return ESP_FAIL;
    }

    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (phy == NULL) {
        return ESP_FAIL;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    err = esp_eth_driver_install(&eth_config, &eth_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
    if (err != ESP_OK) {
        return err;
    }

    err = esp_eth_start(eth_handle);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG,
             "Ethernet initialized (MDC=%d MDIO=%d PHY_ADDR=%d RST=%d)",
             CONFIG_USBIP_ETH_MDC_GPIO,
             CONFIG_USBIP_ETH_MDIO_GPIO,
             CONFIG_USBIP_ETH_PHY_ADDR,
             CONFIG_USBIP_ETH_PHY_RST_GPIO);
    return ESP_OK;
}
#endif

#if CONFIG_USBIP_TRANSPORT_WIFI
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static int s_wifi_retry_count;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        (void)esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_count < CONFIG_USBIP_WIFI_MAX_RETRY) {
            (void)esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "Wi-Fi retry %d/%d", s_wifi_retry_count, CONFIG_USBIP_WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi IPv4: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static wifi_auth_mode_t wifi_authmode_from_config(void)
{
    switch (CONFIG_USBIP_WIFI_AUTHMODE) {
    case 0:
        return WIFI_AUTH_OPEN;
    case 1:
        return WIFI_AUTH_WPA_PSK;
    case 2:
        return WIFI_AUTH_WPA2_PSK;
    case 3:
    default:
        return WIFI_AUTH_WPA2_WPA3_PSK;
    }
}

static esp_err_t network_init_wifi(void)
{
    if (strlen(CONFIG_USBIP_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "Wi-Fi SSID is empty. Set USBIP_WIFI_SSID in menuconfig or sdkconfig defaults.");
        return ESP_ERR_INVALID_ARG;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = wifi_authmode_from_config(),
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_USBIP_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_USBIP_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
#if CONFIG_USBIP_WIFI_DISABLE_MODEM_SLEEP
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi power save disabled (WIFI_PS_NONE)");
#endif

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected to SSID \"%s\"", CONFIG_USBIP_WIFI_SSID);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Wi-Fi failed to connect to SSID \"%s\"", CONFIG_USBIP_WIFI_SSID);
    return ESP_FAIL;
}
#endif

esp_err_t network_init_start(void)
{
#if CONFIG_USBIP_TRANSPORT_ETHERNET
    return network_init_ethernet();
#elif CONFIG_USBIP_TRANSPORT_WIFI
    return network_init_wifi();
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
