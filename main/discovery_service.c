#include "discovery_service.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "mdns.h"
#include "sdkconfig.h"

#include "usb_backend.h"
#include "usbip_protocol.h"

#define USBIP_MDNS_SERVICE_TYPE "_usbip"
#define USBIP_MDNS_SERVICE_PROTO "_tcp"
#define USBIP_DISCOVERY_TASK_STACK 4096
#define USBIP_DISCOVERY_TASK_PRIORITY 5
#define USBIP_DISCOVERY_REFRESH_MS 1000

static const char *TAG = "discovery";

typedef struct {
    uint32_t count;
    char primary_busid[sizeof(((usbip_backend_device_t *)0)->busid)];
    uint16_t primary_vid;
    uint16_t primary_pid;
} discovery_snapshot_t;

static discovery_snapshot_t s_last_snapshot;

static bool snapshot_changed(const discovery_snapshot_t *a, const discovery_snapshot_t *b)
{
    return (a->count != b->count) ||
           (a->primary_vid != b->primary_vid) ||
           (a->primary_pid != b->primary_pid) ||
           (strcmp(a->primary_busid, b->primary_busid) != 0);
}

static esp_err_t discovery_publish_txt_from_snapshot(const discovery_snapshot_t *snapshot)
{
    char state[12];
    char busid[sizeof(snapshot->primary_busid)];
    char vid[8];
    char pid[8];
    char count[12];

    snprintf(count, sizeof(count), "%" PRIu32, snapshot->count);

    if (snapshot->count == 0) {
        strlcpy(state, "idle", sizeof(state));
        strlcpy(busid, "-", sizeof(busid));
        strlcpy(vid, "-", sizeof(vid));
        strlcpy(pid, "-", sizeof(pid));
    } else if (snapshot->count == 1) {
        strlcpy(state, "exporting", sizeof(state));
        strlcpy(busid, snapshot->primary_busid, sizeof(busid));
        snprintf(vid, sizeof(vid), "%04x", snapshot->primary_vid);
        snprintf(pid, sizeof(pid), "%04x", snapshot->primary_pid);
    } else {
        strlcpy(state, "exporting", sizeof(state));
        strlcpy(busid, "multi", sizeof(busid));
        strlcpy(vid, "-", sizeof(vid));
        strlcpy(pid, "-", sizeof(pid));
    }

    mdns_txt_item_t txt[] = {
        {.key = "transport", .value = "usbip"},
        {.key = "state", .value = state},
        {.key = "count", .value = count},
        {.key = "busid", .value = busid},
        {.key = "vid", .value = vid},
        {.key = "pid", .value = pid},
        {.key = "target", .value = CONFIG_IDF_TARGET},
    };

    return mdns_service_txt_set(USBIP_MDNS_SERVICE_TYPE, USBIP_MDNS_SERVICE_PROTO, txt, sizeof(txt) / sizeof(txt[0]));
}

static void discovery_task(void *arg)
{
    (void)arg;

    while (true) {
        usbip_backend_device_t devices[CONFIG_USBIP_MAX_DEVICES];
        const size_t count = usb_backend_get_devices(devices, CONFIG_USBIP_MAX_DEVICES);

        discovery_snapshot_t current = {
            .count = (uint32_t)count,
            .primary_vid = 0,
            .primary_pid = 0,
        };

        strlcpy(current.primary_busid, "-", sizeof(current.primary_busid));

        if (count > 0) {
            strlcpy(current.primary_busid, devices[0].busid, sizeof(current.primary_busid));
            current.primary_vid = devices[0].id_vendor;
            current.primary_pid = devices[0].id_product;
        }

        if (snapshot_changed(&current, &s_last_snapshot)) {
            esp_err_t err = discovery_publish_txt_from_snapshot(&current);
            if (err == ESP_OK) {
                s_last_snapshot = current;
                ESP_LOGI(TAG,
                         "Updated DNS-SD TXT: state=%s count=%" PRIu32 " busid=%s",
                         current.count ? "exporting" : "idle",
                         current.count,
                         current.count > 1 ? "multi" : current.primary_busid);
            } else {
                ESP_LOGW(TAG, "mdns_service_txt_set failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(USBIP_DISCOVERY_REFRESH_MS));
    }
}

esp_err_t discovery_service_start(void)
{
    uint8_t mac[6] = {0};
    esp_err_t err = esp_efuse_mac_get_default(mac);
    if (err != ESP_OK) {
        return err;
    }

    char hostname[32];
    snprintf(hostname, sizeof(hostname), "usbip-%02x%02x%02x", mac[3], mac[4], mac[5]);

    char instance[48];
    snprintf(instance, sizeof(instance), "ESP USB/IP Bridge (%s)", CONFIG_IDF_TARGET);

    err = mdns_init();
    if (err != ESP_OK) {
        return err;
    }

    err = mdns_hostname_set(hostname);
    if (err != ESP_OK) {
        return err;
    }

    err = mdns_instance_name_set(instance);
    if (err != ESP_OK) {
        return err;
    }

    discovery_snapshot_t initial = {
        .count = 0,
        .primary_vid = 0,
        .primary_pid = 0,
    };
    strlcpy(initial.primary_busid, "-", sizeof(initial.primary_busid));

    mdns_txt_item_t initial_txt[] = {
        {.key = "transport", .value = "usbip"},
        {.key = "state", .value = "idle"},
        {.key = "count", .value = "0"},
        {.key = "busid", .value = "-"},
        {.key = "vid", .value = "-"},
        {.key = "pid", .value = "-"},
        {.key = "target", .value = CONFIG_IDF_TARGET},
    };

    err = mdns_service_add(NULL,
                           USBIP_MDNS_SERVICE_TYPE,
                           USBIP_MDNS_SERVICE_PROTO,
                           USBIP_TCP_PORT,
                           initial_txt,
                           sizeof(initial_txt) / sizeof(initial_txt[0]));
    if (err != ESP_OK) {
        return err;
    }

    s_last_snapshot = initial;

    if (xTaskCreate(discovery_task,
                    "usbip_discovery",
                    USBIP_DISCOVERY_TASK_STACK,
                    NULL,
                    USBIP_DISCOVERY_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DNS-SD service advertised: %s.%s port=%d host=%s.local", USBIP_MDNS_SERVICE_TYPE, USBIP_MDNS_SERVICE_PROTO, USBIP_TCP_PORT, hostname);
    return ESP_OK;
}
