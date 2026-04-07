#include "usb_backend.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "sdkconfig.h"
#include "usb/usb_host.h"
#include "usb/usb_types_stack.h"

#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_USBIP_S3_USB_OTG_DEVKIT_POWER
#include "driver/gpio.h"
#endif

#define USB_BACKEND_QUEUE_LEN 8
#define USB_BACKEND_EVENT_QUEUE_LEN 16
#define USB_BACKEND_TASK_STACK 8192
#define USB_BACKEND_TASK_PRIORITY 9
#define USB_BACKEND_DAEMON_TASK_STACK 4096
#define USB_BACKEND_DAEMON_TASK_PRIORITY 10

#ifndef USB_CLASS_HUB
#define USB_CLASS_HUB 0x09
#endif

typedef enum {
    USB_BACKEND_EVENT_NEW_DEV = 1,
    USB_BACKEND_EVENT_DEV_GONE = 2,
} usb_backend_event_type_t;

typedef struct {
    usb_backend_event_type_t type;
    union {
        uint8_t address;
        usb_device_handle_t dev_hdl;
    } u;
} usb_backend_event_t;

typedef struct {
    char busid[32];
    usb_setup_packet_t setup;
    const uint8_t *out_data;
    size_t out_len;
    uint8_t *in_data;
    size_t in_capacity;
    size_t *in_len_out;
    int *status_out;
    SemaphoreHandle_t done_sem;
} usb_backend_ctrl_req_t;

typedef struct {
    char busid[32];
    uint8_t endpoint_addr;
    const uint8_t *out_data;
    size_t out_len;
    uint8_t *in_data;
    size_t in_capacity;
    size_t *in_len_out;
    int *status_out;
    SemaphoreHandle_t done_sem;
} usb_backend_bulk_req_t;

typedef struct {
    bool done;
} transfer_done_ctx_t;

typedef struct {
    bool in_use;
    bool interfaces_claimed;
    usb_device_handle_t dev_hdl;
    usbip_backend_device_t device;
} usb_backend_device_slot_t;

typedef struct {
    SemaphoreHandle_t state_mutex;
    QueueHandle_t ctrl_req_queue;
    QueueHandle_t bulk_req_queue;
    QueueHandle_t event_queue;

    usb_host_client_handle_t client_hdl;
    usb_backend_device_slot_t devices[CONFIG_USBIP_MAX_DEVICES];
} usb_backend_state_t;

static const char *TAG = "usb_backend";
static usb_backend_state_t s_state;

#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_USBIP_S3_USB_OTG_DEVKIT_POWER
static esp_err_t usb_backend_s3_usb_otg_devkit_power_init(void)
{
    const uint64_t pin_mask = (1ULL << CONFIG_USBIP_S3_USB_BOOST_EN_GPIO) |
                              (1ULL << CONFIG_USBIP_S3_USB_DEV_VBUS_EN_GPIO) |
                              (1ULL << CONFIG_USBIP_S3_USB_LIMIT_EN_GPIO) |
                              (1ULL << CONFIG_USBIP_S3_USB_SEL_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(CONFIG_USBIP_S3_USB_BOOST_EN_GPIO, 0);
    gpio_set_level(CONFIG_USBIP_S3_USB_DEV_VBUS_EN_GPIO, 1);
    gpio_set_level(CONFIG_USBIP_S3_USB_LIMIT_EN_GPIO, 1);
    gpio_set_level(CONFIG_USBIP_S3_USB_SEL_GPIO, 1);

    ESP_LOGI(TAG,
             "Configured ESP32-S3-USB-OTG host power GPIOs (%d,%d,%d,%d)",
             CONFIG_USBIP_S3_USB_BOOST_EN_GPIO,
             CONFIG_USBIP_S3_USB_DEV_VBUS_EN_GPIO,
             CONFIG_USBIP_S3_USB_LIMIT_EN_GPIO,
             CONFIG_USBIP_S3_USB_SEL_GPIO);
    return ESP_OK;
}
#endif

static uint32_t usb_speed_to_usbip(usb_speed_t speed)
{
    switch (speed) {
    case USB_SPEED_LOW:
        return 1;
    case USB_SPEED_FULL:
        return 2;
    case USB_SPEED_HIGH:
        return 3;
    default:
        return 0;
    }
}

static void parse_interfaces(const usb_config_desc_t *config_desc, usbip_backend_device_t *device)
{
    if (config_desc == NULL || device == NULL) {
        return;
    }

    const uint8_t *raw = (const uint8_t *)config_desc;
    const size_t total_len = config_desc->wTotalLength;

    uint8_t count = 0;
    for (size_t offset = 0; offset + 2 <= total_len;) {
        const uint8_t desc_len = raw[offset];
        const uint8_t desc_type = raw[offset + 1];

        if (desc_len < 2 || (offset + desc_len) > total_len) {
            break;
        }

        if (desc_type == USB_B_DESCRIPTOR_TYPE_INTERFACE && desc_len >= 9 && count < USBIP_MAX_INTERFACES) {
            device->interfaces[count].interface_class = raw[offset + 5];
            device->interfaces[count].interface_subclass = raw[offset + 6];
            device->interfaces[count].interface_protocol = raw[offset + 7];
            count++;
        }

        offset += desc_len;
    }

    device->num_interfaces = count;
}

static bool is_hub_device(const usb_device_desc_t *dev_desc, const usbip_backend_device_t *device)
{
    if (dev_desc != NULL && dev_desc->bDeviceClass == USB_CLASS_HUB) {
        return true;
    }

    for (uint8_t i = 0; i < device->num_interfaces; i++) {
        if (device->interfaces[i].interface_class == USB_CLASS_HUB) {
            return true;
        }
    }

    return false;
}

static bool busid_matches_key(const char key[32], const char stored_busid[32])
{
    char expected[32] = {0};
    strlcpy(expected, stored_busid, sizeof(expected));
    return memcmp(key, expected, sizeof(expected)) == 0;
}

static int find_slot_by_busid_locked(const char busid[32])
{
    for (int i = 0; i < CONFIG_USBIP_MAX_DEVICES; i++) {
        if (!s_state.devices[i].in_use) {
            continue;
        }
        if (busid_matches_key(busid, s_state.devices[i].device.busid)) {
            return i;
        }
    }
    return -1;
}

static int find_slot_by_handle_locked(usb_device_handle_t dev_hdl)
{
    for (int i = 0; i < CONFIG_USBIP_MAX_DEVICES; i++) {
        if (!s_state.devices[i].in_use) {
            continue;
        }
        if (s_state.devices[i].dev_hdl == dev_hdl) {
            return i;
        }
    }
    return -1;
}

static int find_slot_by_devnum_locked(uint32_t devnum)
{
    for (int i = 0; i < CONFIG_USBIP_MAX_DEVICES; i++) {
        if (!s_state.devices[i].in_use) {
            continue;
        }
        if (s_state.devices[i].device.devnum == devnum) {
            return i;
        }
    }
    return -1;
}

static int find_free_slot_locked(void)
{
    for (int i = 0; i < CONFIG_USBIP_MAX_DEVICES; i++) {
        if (!s_state.devices[i].in_use) {
            return i;
        }
    }
    return -1;
}

static void clear_slot_locked(int slot)
{
    if (slot < 0 || slot >= CONFIG_USBIP_MAX_DEVICES) {
        return;
    }

    memset(&s_state.devices[slot], 0, sizeof(s_state.devices[slot]));
}

static void release_interfaces_locked(int slot)
{
    if (slot < 0 || slot >= CONFIG_USBIP_MAX_DEVICES || !s_state.devices[slot].in_use) {
        return;
    }
    if (!s_state.devices[slot].interfaces_claimed) {
        return;
    }

    usb_device_handle_t dev_hdl = s_state.devices[slot].dev_hdl;
    const usbip_backend_device_t *device = &s_state.devices[slot].device;

    for (uint8_t i = 0; i < device->num_interfaces; i++) {
        esp_err_t err = usb_host_interface_release(s_state.client_hdl, dev_hdl, i);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_interface_release(%u) failed: %s", i, esp_err_to_name(err));
        }
    }

    s_state.devices[slot].interfaces_claimed = false;
}

static void close_slot_locked(int slot)
{
    if (slot < 0 || slot >= CONFIG_USBIP_MAX_DEVICES || !s_state.devices[slot].in_use) {
        return;
    }

    release_interfaces_locked(slot);

    usb_device_handle_t dev_hdl = s_state.devices[slot].dev_hdl;
    if (dev_hdl != NULL) {
        esp_err_t err = usb_host_device_close(s_state.client_hdl, dev_hdl);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_device_close failed: %s", esp_err_to_name(err));
        }
    }

    clear_slot_locked(slot);
}

static void usb_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    usb_backend_state_t *state = (usb_backend_state_t *)arg;

    usb_backend_event_t evt;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        evt.type = USB_BACKEND_EVENT_NEW_DEV;
        evt.u.address = event_msg->new_dev.address;
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        evt.type = USB_BACKEND_EVENT_DEV_GONE;
        evt.u.dev_hdl = event_msg->dev_gone.dev_hdl;
    } else {
        return;
    }

    if (xQueueSend(state->event_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Dropping USB event type=%d due to full queue", (int)evt.type);
    }
}

static void export_new_device(uint8_t address)
{
    usb_device_handle_t dev_hdl = NULL;
    esp_err_t err = usb_host_device_open(s_state.client_hdl, address, &dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_device_open(%u) failed: %s", address, esp_err_to_name(err));
        return;
    }

    const usb_device_desc_t *dev_desc = NULL;
    err = usb_host_get_device_descriptor(dev_hdl, &dev_desc);
    if (err != ESP_OK || dev_desc == NULL) {
        ESP_LOGW(TAG, "usb_host_get_device_descriptor failed: %s", esp_err_to_name(err));
        usb_host_device_close(s_state.client_hdl, dev_hdl);
        return;
    }

    usb_device_info_t dev_info;
    err = usb_host_device_info(dev_hdl, &dev_info);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_device_info failed: %s", esp_err_to_name(err));
        usb_host_device_close(s_state.client_hdl, dev_hdl);
        return;
    }

    usbip_backend_device_t device;
    memset(&device, 0, sizeof(device));

    device.present = true;
    device.busnum = 1;
    device.devnum = dev_info.dev_addr;
    device.speed = usb_speed_to_usbip(dev_info.speed);

    device.id_vendor = dev_desc->idVendor;
    device.id_product = dev_desc->idProduct;
    device.bcd_device = dev_desc->bcdDevice;
    device.device_class = dev_desc->bDeviceClass;
    device.device_subclass = dev_desc->bDeviceSubClass;
    device.device_protocol = dev_desc->bDeviceProtocol;
    device.num_configurations = dev_desc->bNumConfigurations;
    device.configuration_value = dev_info.bConfigurationValue;

    const usb_config_desc_t *config_desc = NULL;
    err = usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
    if (err == ESP_OK && config_desc != NULL) {
        parse_interfaces(config_desc, &device);
    }

    if (is_hub_device(dev_desc, &device)) {
        ESP_LOGI(TAG, "Hub detected at address=%u, managing locally (not exported)", address);
        usb_host_device_close(s_state.client_hdl, dev_hdl);
        return;
    }

    snprintf(device.busid, sizeof(device.busid), "1-%u", dev_info.dev_addr);
    snprintf(device.path, sizeof(device.path), "/esp-usb-host/1-%u", dev_info.dev_addr);

    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);

    const int existing_slot = find_slot_by_devnum_locked(device.devnum);
    if (existing_slot >= 0) {
        close_slot_locked(existing_slot);
    }

    const int free_slot = find_free_slot_locked();
    if (free_slot < 0) {
        xSemaphoreGive(s_state.state_mutex);
        ESP_LOGW(TAG, "No free export slots left (max=%d)", CONFIG_USBIP_MAX_DEVICES);
        usb_host_device_close(s_state.client_hdl, dev_hdl);
        return;
    }

    s_state.devices[free_slot].in_use = true;
    s_state.devices[free_slot].dev_hdl = dev_hdl;
    s_state.devices[free_slot].device = device;
    s_state.devices[free_slot].interfaces_claimed = false;

    /* Claim all interfaces so bulk/interrupt endpoints are usable. */
    for (uint8_t i = 0; i < device.num_interfaces; i++) {
        esp_err_t claim_err = usb_host_interface_claim(s_state.client_hdl, dev_hdl, i, 0);
        if (claim_err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_interface_claim(%u) failed: %s", i, esp_err_to_name(claim_err));
        }
    }
    s_state.devices[free_slot].interfaces_claimed = true;

    xSemaphoreGive(s_state.state_mutex);

    ESP_LOGI(TAG,
             "Exporting USB device busid=%s vid=%04x pid=%04x",
             device.busid,
             device.id_vendor,
             device.id_product);
}

static void remove_gone_device(usb_device_handle_t dev_hdl)
{
    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);

    const int slot = find_slot_by_handle_locked(dev_hdl);
    if (slot >= 0) {
        char busid[32] = {0};
        strlcpy(busid, s_state.devices[slot].device.busid, sizeof(busid));
        close_slot_locked(slot);
        xSemaphoreGive(s_state.state_mutex);
        ESP_LOGI(TAG, "USB device disconnected: %s", busid);
        return;
    }

    xSemaphoreGive(s_state.state_mutex);
}

static void process_backend_events(void)
{
    usb_backend_event_t evt;
    while (xQueueReceive(s_state.event_queue, &evt, 0) == pdTRUE) {
        if (evt.type == USB_BACKEND_EVENT_NEW_DEV) {
            export_new_device(evt.u.address);
        } else if (evt.type == USB_BACKEND_EVENT_DEV_GONE) {
            remove_gone_device(evt.u.dev_hdl);
        }
    }
}

static void control_transfer_done_cb(usb_transfer_t *transfer)
{
    transfer_done_ctx_t *ctx = (transfer_done_ctx_t *)transfer->context;
    ctx->done = true;
}

static int map_transfer_status_to_errno(usb_transfer_status_t status)
{
    switch (status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        return 0;
    case USB_TRANSFER_STATUS_TIMED_OUT:
        return -ETIMEDOUT;
    case USB_TRANSFER_STATUS_CANCELED:
        return -ECONNRESET;
    case USB_TRANSFER_STATUS_STALL:
        return -EPIPE;
    case USB_TRANSFER_STATUS_NO_DEVICE:
        return -ENODEV;
    default:
        return -EIO;
    }
}

static int process_control_request(const usb_backend_ctrl_req_t *req)
{
    if (req == NULL) {
        return -EINVAL;
    }

    if (req->in_len_out != NULL) {
        *req->in_len_out = 0;
    }

    usb_device_handle_t dev_hdl = NULL;
    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int slot = find_slot_by_busid_locked(req->busid);
    if (slot >= 0) {
        dev_hdl = s_state.devices[slot].dev_hdl;
    }
    xSemaphoreGive(s_state.state_mutex);

    if (dev_hdl == NULL) {
        return -ENODEV;
    }

    const bool is_in = (req->setup.bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;
    const size_t payload_len = is_in ? req->in_capacity : req->out_len;
    if (payload_len > CONFIG_USBIP_MAX_TRANSFER) {
        return -EMSGSIZE;
    }

    usb_transfer_t *transfer = NULL;
    const size_t transfer_size = USB_SETUP_PACKET_SIZE + payload_len;
    esp_err_t err = usb_host_transfer_alloc(transfer_size, 0, &transfer);
    if (err != ESP_OK || transfer == NULL) {
        return -ENOMEM;
    }

    memcpy(transfer->data_buffer, &req->setup, USB_SETUP_PACKET_SIZE);
    if (!is_in && req->out_len > 0 && req->out_data != NULL) {
        memcpy(transfer->data_buffer + USB_SETUP_PACKET_SIZE, req->out_data, req->out_len);
    }

    transfer_done_ctx_t done_ctx = {
        .done = false,
    };

    transfer->callback = control_transfer_done_cb;
    transfer->context = &done_ctx;
    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = 0;
    transfer->num_bytes = transfer_size;
    transfer->timeout_ms = 5000;

    err = usb_host_transfer_submit_control(s_state.client_hdl, transfer);
    if (err != ESP_OK) {
        usb_host_transfer_free(transfer);
        if (err == ESP_ERR_INVALID_STATE) {
            return -ENODEV;
        }
        return -EIO;
    }

    while (!done_ctx.done) {
        err = usb_host_client_handle_events(s_state.client_hdl, pdMS_TO_TICKS(10));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            break;
        }
        process_backend_events();
    }

    int status = map_transfer_status_to_errno(transfer->status);
    if (status == 0 && is_in && req->in_data != NULL && req->in_capacity > 0) {
        const size_t bytes = (transfer->actual_num_bytes > USB_SETUP_PACKET_SIZE)
                                 ? (transfer->actual_num_bytes - USB_SETUP_PACKET_SIZE)
                                 : 0;
        const size_t copy_len = (bytes > req->in_capacity) ? req->in_capacity : bytes;
        memcpy(req->in_data, transfer->data_buffer + USB_SETUP_PACKET_SIZE, copy_len);
        if (req->in_len_out != NULL) {
            *req->in_len_out = copy_len;
        }
    }

    usb_host_transfer_free(transfer);
    return status;
}

static int process_bulk_request(const usb_backend_bulk_req_t *req)
{
    if (req == NULL) {
        return -EINVAL;
    }

    if (req->in_len_out != NULL) {
        *req->in_len_out = 0;
    }

    usb_device_handle_t dev_hdl = NULL;
    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int slot = find_slot_by_busid_locked(req->busid);
    if (slot >= 0) {
        dev_hdl = s_state.devices[slot].dev_hdl;
    }
    xSemaphoreGive(s_state.state_mutex);

    if (dev_hdl == NULL) {
        return -ENODEV;
    }

    const bool is_in = (req->endpoint_addr & 0x80) != 0;
    const size_t payload_len = is_in ? req->in_capacity : req->out_len;
    if (payload_len > CONFIG_USBIP_MAX_TRANSFER) {
        return -EMSGSIZE;
    }

    usb_transfer_t *transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(payload_len, 0, &transfer);
    if (err != ESP_OK || transfer == NULL) {
        return -ENOMEM;
    }

    if (!is_in && req->out_len > 0 && req->out_data != NULL) {
        memcpy(transfer->data_buffer, req->out_data, req->out_len);
    }

    transfer_done_ctx_t done_ctx = {
        .done = false,
    };

    transfer->callback = control_transfer_done_cb;
    transfer->context = &done_ctx;
    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = req->endpoint_addr;
    transfer->num_bytes = payload_len;
    transfer->timeout_ms = 5000;

    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        usb_host_transfer_free(transfer);
        if (err == ESP_ERR_INVALID_STATE) {
            return -ENODEV;
        }
        return -EIO;
    }

    while (!done_ctx.done) {
        err = usb_host_client_handle_events(s_state.client_hdl, pdMS_TO_TICKS(10));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            break;
        }
        process_backend_events();
    }

    int status = map_transfer_status_to_errno(transfer->status);
    if (status == 0 && is_in && req->in_data != NULL && req->in_capacity > 0) {
        const size_t bytes = transfer->actual_num_bytes;
        const size_t copy_len = (bytes > req->in_capacity) ? req->in_capacity : bytes;
        memcpy(req->in_data, transfer->data_buffer, copy_len);
        if (req->in_len_out != NULL) {
            *req->in_len_out = copy_len;
        }
    }

    usb_host_transfer_free(transfer);
    return status;
}

static void usb_backend_daemon_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t event_flags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events failed: %s", esp_err_to_name(err));
            continue;
        }

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void usb_backend_task(void *arg)
{
    (void)arg;

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = USB_BACKEND_EVENT_QUEUE_LEN,
        .async = {
            .client_event_callback = usb_client_event_cb,
            .callback_arg = &s_state,
        },
    };

    esp_err_t err = usb_host_client_register(&client_config, &s_state.client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_client_register failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB backend task running");

    while (true) {
        err = usb_host_client_handle_events(s_state.client_hdl, pdMS_TO_TICKS(10));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "usb_host_client_handle_events failed: %s", esp_err_to_name(err));
        }

        process_backend_events();

        usb_backend_ctrl_req_t ctrl_req;
        while (xQueueReceive(s_state.ctrl_req_queue, &ctrl_req, 0) == pdTRUE) {
            int status = process_control_request(&ctrl_req);
            if (ctrl_req.status_out != NULL) {
                *ctrl_req.status_out = status;
            }
            xSemaphoreGive(ctrl_req.done_sem);
        }

        usb_backend_bulk_req_t bulk_req;
        while (xQueueReceive(s_state.bulk_req_queue, &bulk_req, 0) == pdTRUE) {
            int status = process_bulk_request(&bulk_req);
            if (bulk_req.status_out != NULL) {
                *bulk_req.status_out = status;
            }
            xSemaphoreGive(bulk_req.done_sem);
        }
    }
}

esp_err_t usb_backend_start(void)
{
    memset(&s_state, 0, sizeof(s_state));

#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_USBIP_S3_USB_OTG_DEVKIT_POWER
    esp_err_t board_power_err = usb_backend_s3_usb_otg_devkit_power_init();
    if (board_power_err != ESP_OK) {
        return board_power_err;
    }
#endif

    s_state.state_mutex = xSemaphoreCreateMutex();
    if (s_state.state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_state.ctrl_req_queue = xQueueCreate(USB_BACKEND_QUEUE_LEN, sizeof(usb_backend_ctrl_req_t));
    if (s_state.ctrl_req_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_state.bulk_req_queue = xQueueCreate(USB_BACKEND_QUEUE_LEN, sizeof(usb_backend_bulk_req_t));
    if (s_state.bulk_req_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_state.event_queue = xQueueCreate(USB_BACKEND_EVENT_QUEUE_LEN, sizeof(usb_backend_event_t));
    if (s_state.event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = 0,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreate(usb_backend_daemon_task,
                    "usb_host_daemon",
                    USB_BACKEND_DAEMON_TASK_STACK,
                    NULL,
                    USB_BACKEND_DAEMON_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_FAIL;
    }

    if (xTaskCreate(usb_backend_task,
                    "usb_backend",
                    USB_BACKEND_TASK_STACK,
                    NULL,
                    USB_BACKEND_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

size_t usb_backend_get_devices(usbip_backend_device_t *out_devices, size_t max_devices)
{
    if (out_devices == NULL || max_devices == 0) {
        return 0;
    }

    size_t copied = 0;

    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    for (int i = 0; i < CONFIG_USBIP_MAX_DEVICES && copied < max_devices; i++) {
        if (!s_state.devices[i].in_use) {
            continue;
        }

        out_devices[copied++] = s_state.devices[i].device;
    }
    xSemaphoreGive(s_state.state_mutex);

    return copied;
}

bool usb_backend_get_device_by_busid(const char busid[32], usbip_backend_device_t *out_device)
{
    if (busid == NULL || out_device == NULL) {
        return false;
    }

    bool found = false;

    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int slot = find_slot_by_busid_locked(busid);
    if (slot >= 0) {
        *out_device = s_state.devices[slot].device;
        found = true;
    }
    xSemaphoreGive(s_state.state_mutex);

    return found;
}

int usb_backend_control_transfer(const char busid[32],
                                 const usb_setup_packet_t *setup,
                                 const uint8_t *out_data,
                                 size_t out_len,
                                 uint8_t *in_data,
                                 size_t in_capacity,
                                 size_t *in_len)
{
    if (busid == NULL || setup == NULL || in_len == NULL) {
        return -EINVAL;
    }

    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL) {
        return -ENOMEM;
    }

    int status = -EIO;
    *in_len = 0;

    usb_backend_ctrl_req_t req;
    memset(&req, 0, sizeof(req));

    memcpy(req.busid, busid, sizeof(req.busid));
    req.setup = *setup;
    req.out_data = out_data;
    req.out_len = out_len;
    req.in_data = in_data;
    req.in_capacity = in_capacity;
    req.in_len_out = in_len;
    req.status_out = &status;
    req.done_sem = done_sem;

    if (xQueueSend(s_state.ctrl_req_queue, &req, pdMS_TO_TICKS(1000)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return -EAGAIN;
    }

    if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(7000)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return -ETIMEDOUT;
    }

    vSemaphoreDelete(done_sem);
    return status;
}

int usb_backend_bulk_transfer(const char busid[32],
                              uint8_t endpoint_addr,
                              const uint8_t *out_data,
                              size_t out_len,
                              uint8_t *in_data,
                              size_t in_capacity,
                              size_t *in_len)
{
    if (busid == NULL || in_len == NULL) {
        return -EINVAL;
    }

    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL) {
        return -ENOMEM;
    }

    int status = -EIO;
    *in_len = 0;

    usb_backend_bulk_req_t req;
    memset(&req, 0, sizeof(req));

    memcpy(req.busid, busid, sizeof(req.busid));
    req.endpoint_addr = endpoint_addr;
    req.out_data = out_data;
    req.out_len = out_len;
    req.in_data = in_data;
    req.in_capacity = in_capacity;
    req.in_len_out = in_len;
    req.status_out = &status;
    req.done_sem = done_sem;

    if (xQueueSend(s_state.bulk_req_queue, &req, pdMS_TO_TICKS(1000)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return -EAGAIN;
    }

    if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(7000)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return -ETIMEDOUT;
    }

    vSemaphoreDelete(done_sem);
    return status;
}
