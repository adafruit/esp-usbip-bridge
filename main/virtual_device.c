#include "virtual_device.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "virtual_dev";

static virtual_device_t *s_devices[VIRTUAL_DEVICE_MAX];
static size_t s_device_count = 0;

esp_err_t virtual_device_register(virtual_device_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_device_count >= VIRTUAL_DEVICE_MAX) {
        ESP_LOGE(TAG, "No free virtual device slots (max=%d)", VIRTUAL_DEVICE_MAX);
        return ESP_ERR_NO_MEM;
    }

    /* Assign busid and path on the virtual bus. */
    uint32_t devnum = s_device_count + 1;
    dev->desc.present = true;
    dev->desc.busnum = VIRTUAL_DEVICE_BUSNUM;
    dev->desc.devnum = devnum;
    snprintf(dev->desc.busid, sizeof(dev->desc.busid), "%u-%"PRIu32,
             VIRTUAL_DEVICE_BUSNUM, devnum);
    snprintf(dev->desc.path, sizeof(dev->desc.path), "/esp-virtual/%u-%"PRIu32,
             VIRTUAL_DEVICE_BUSNUM, devnum);

    s_devices[s_device_count] = dev;
    s_device_count++;

    ESP_LOGI(TAG, "Registered virtual device busid=%s vid=%04x pid=%04x",
             dev->desc.busid, dev->desc.id_vendor, dev->desc.id_product);

    return ESP_OK;
}

virtual_device_t *virtual_device_find_by_busid(const char busid[32])
{
    if (busid == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < s_device_count; i++) {
        /* Compare the full 32-byte busid field. */
        char expected[32] = {0};
        strlcpy(expected, s_devices[i]->desc.busid, sizeof(expected));
        if (memcmp(busid, expected, sizeof(expected)) == 0) {
            return s_devices[i];
        }
    }

    return NULL;
}

size_t virtual_device_get_all(usbip_backend_device_t *out_devices, size_t max_devices)
{
    if (out_devices == NULL || max_devices == 0) {
        return 0;
    }

    size_t copied = 0;
    for (size_t i = 0; i < s_device_count && copied < max_devices; i++) {
        out_devices[copied++] = s_devices[i]->desc;
    }

    return copied;
}
