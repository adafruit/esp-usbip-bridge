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

#include "virtual_device.h"

#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_USBIP_S3_USB_OTG_DEVKIT_POWER
#include "driver/gpio.h"
#endif

#define USB_BACKEND_EVENT_QUEUE_LEN 16
#define USB_BACKEND_TASK_STACK 8192
#define USB_BACKEND_TASK_PRIORITY 9
#define USB_BACKEND_DAEMON_TASK_STACK 4096
#define USB_BACKEND_DAEMON_TASK_PRIORITY 10

#define USB_BACKEND_NUM_PIPES CONFIG_USBIP_NUM_PIPES

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

/* Unified per-pipe request slot.  One slot per host pipe, shared across
   all devices and endpoints.  The caller fills a free slot, sets
   active = true, notifies the backend task, then waits on done_sem. */
typedef struct {
    bool assigned;                  /* true = reserved for a device+endpoint */
    volatile bool active;           /* true = transfer pending */
    char busid[32];
    uint8_t endpoint_addr;          /* 0 for control, 0x8N for IN, 0x0N for OUT */
    usb_setup_packet_t setup;       /* only used when endpoint_addr == 0 */
    const uint8_t *out_data;
    size_t out_len;
    uint8_t *in_data;
    size_t in_capacity;
    size_t *in_len_out;
    int *status_out;
    volatile bool *cancel;
    SemaphoreHandle_t done_sem;     /* pre-allocated, not per-call */
} usb_backend_pipe_req_t;

typedef struct {
    bool done;
    volatile bool *cancel;
} transfer_done_ctx_t;

typedef struct {
    bool in_use;
    bool interfaces_claimed;
    usb_device_handle_t dev_hdl;
    usbip_backend_device_t device;
    int ep0_pipe;                                   /* pipe slot for EP0, -1 = none */
    int ep_pipes[USBIP_MAX_ENDPOINTS];              /* pipe slot per endpoint, -1 = none */
} usb_backend_device_slot_t;

typedef struct {
    SemaphoreHandle_t state_mutex;
    QueueHandle_t event_queue;
    TaskHandle_t task_hdl;

    usb_backend_pipe_req_t pipes[USB_BACKEND_NUM_PIPES];

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

static void parse_descriptors(const usb_config_desc_t *config_desc, usbip_backend_device_t *device)
{
    if (config_desc == NULL || device == NULL) {
        return;
    }

    const uint8_t *raw = (const uint8_t *)config_desc;
    const size_t total_len = config_desc->wTotalLength;

    uint8_t intf_count = 0;
    uint8_t ep_count = 0;
    for (size_t offset = 0; offset + 2 <= total_len;) {
        const uint8_t desc_len = raw[offset];
        const uint8_t desc_type = raw[offset + 1];

        if (desc_len < 2 || (offset + desc_len) > total_len) {
            break;
        }

        if (desc_type == USB_B_DESCRIPTOR_TYPE_INTERFACE && desc_len >= 9 && intf_count < USBIP_MAX_INTERFACES) {
            device->interfaces[intf_count].interface_class = raw[offset + 5];
            device->interfaces[intf_count].interface_subclass = raw[offset + 6];
            device->interfaces[intf_count].interface_protocol = raw[offset + 7];
            intf_count++;
        } else if (desc_type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && desc_len >= 7 && ep_count < USBIP_MAX_ENDPOINTS) {
            device->endpoints[ep_count].address = raw[offset + 2];
            device->endpoints[ep_count].attributes = raw[offset + 3];
            device->endpoints[ep_count].max_packet_size = raw[offset + 4] | (raw[offset + 5] << 8);
            device->endpoints[ep_count].interval = raw[offset + 6];
            ep_count++;
        }

        offset += desc_len;
    }

    device->num_interfaces = intf_count;
    device->num_endpoints = ep_count;
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

static int alloc_pipe_locked(void)
{
    for (int i = 0; i < USB_BACKEND_NUM_PIPES; i++) {
        if (!s_state.pipes[i].assigned) {
            s_state.pipes[i].assigned = true;
            s_state.pipes[i].active = false;
            return i;
        }
    }
    return -1;
}

static void free_pipe_locked(int pipe)
{
    if (pipe >= 0 && pipe < USB_BACKEND_NUM_PIPES) {
        s_state.pipes[pipe].assigned = false;
        s_state.pipes[pipe].active = false;
    }
}

static void clear_slot_locked(int slot)
{
    if (slot < 0 || slot >= CONFIG_USBIP_MAX_DEVICES) {
        return;
    }

    /* Free any assigned pipe slots. */
    free_pipe_locked(s_state.devices[slot].ep0_pipe);
    for (int i = 0; i < USBIP_MAX_ENDPOINTS; i++) {
        free_pipe_locked(s_state.devices[slot].ep_pipes[i]);
    }

    memset(&s_state.devices[slot], 0, sizeof(s_state.devices[slot]));
    s_state.devices[slot].ep0_pipe = -1;
    for (int i = 0; i < USBIP_MAX_ENDPOINTS; i++) {
        s_state.devices[slot].ep_pipes[i] = -1;
    }
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

    /* Free endpoint pipe slots. */
    for (int i = 0; i < USBIP_MAX_ENDPOINTS; i++) {
        free_pipe_locked(s_state.devices[slot].ep_pipes[i]);
        s_state.devices[slot].ep_pipes[i] = -1;
    }

    s_state.devices[slot].interfaces_claimed = false;
}

static esp_err_t ensure_interfaces_claimed_locked(int slot)
{
    if (slot < 0 || slot >= CONFIG_USBIP_MAX_DEVICES || !s_state.devices[slot].in_use) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state.devices[slot].interfaces_claimed) {
        return ESP_OK;
    }

    usb_device_handle_t dev_hdl = s_state.devices[slot].dev_hdl;
    const usbip_backend_device_t *device = &s_state.devices[slot].device;

    ESP_LOGI(TAG, "Claiming %u interfaces for %s (lazy)", device->num_interfaces, device->busid);
    for (uint8_t i = 0; i < device->num_interfaces; i++) {
        esp_err_t err = usb_host_interface_claim(s_state.client_hdl, dev_hdl, i, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_interface_claim(%u) failed: %s", i, esp_err_to_name(err));
            return err;
        }
    }

    /* Assign a pipe slot for each endpoint. */
    for (uint8_t i = 0; i < device->num_endpoints; i++) {
        int pipe = alloc_pipe_locked();
        if (pipe < 0) {
            ESP_LOGE(TAG, "No free pipe slots for endpoint 0x%02x on %s",
                     device->endpoints[i].address, device->busid);
            return ESP_ERR_NO_MEM;
        }
        s_state.devices[slot].ep_pipes[i] = pipe;
    }

    s_state.devices[slot].interfaces_claimed = true;
    return ESP_OK;
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
        ESP_LOGI(TAG, "USB device attached at address %u", event_msg->new_dev.address);
        evt.type = USB_BACKEND_EVENT_NEW_DEV;
        evt.u.address = event_msg->new_dev.address;
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGW(TAG, "USB device detached");
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
        parse_descriptors(config_desc, &device);
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

    /* Initialize pipe slot mappings. */
    s_state.devices[free_slot].ep0_pipe = alloc_pipe_locked();
    if (s_state.devices[free_slot].ep0_pipe < 0) {
        ESP_LOGW(TAG, "No free pipe slot for EP0 on %s", device.busid);
    }
    for (int i = 0; i < USBIP_MAX_ENDPOINTS; i++) {
        s_state.devices[free_slot].ep_pipes[i] = -1;
    }

    /* Non-zero endpoint pipes are allocated lazily when interfaces are
       claimed, to conserve HCD channels. */

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

static uint16_t get_endpoint_mps_locked(int dev_slot, uint8_t endpoint_addr)
{
    uint16_t mps = 64;  /* safe default */
    usb_device_handle_t dev = s_state.devices[dev_slot].dev_hdl;
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK || cfg == NULL) {
        return mps;
    }
    for (int intf_num = 0; intf_num < s_state.devices[dev_slot].device.num_interfaces; intf_num++) {
        int offset = 0;
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, intf_num, 0, &offset);
        if (intf == NULL) {
            continue;
        }
        for (int i = 0; i < intf->bNumEndpoints; i++) {
            const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &offset);
            if (ep != NULL && ep->bEndpointAddress == endpoint_addr) {
                return ep->wMaxPacketSize;
            }
        }
    }
    return mps;
}

static int process_pipe_request(usb_backend_pipe_req_t *req)
{
    if (req->in_len_out != NULL) {
        *req->in_len_out = 0;
    }

    const bool is_control = (req->endpoint_addr == 0);
    const bool is_in = is_control
        ? (req->setup.bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0
        : (req->endpoint_addr & 0x80) != 0;

    usb_device_handle_t dev_hdl = NULL;
    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int dev_slot = find_slot_by_busid_locked(req->busid);
    if (dev_slot >= 0) {
        if (!is_control) {
            esp_err_t claim_err = ensure_interfaces_claimed_locked(dev_slot);
            if (claim_err != ESP_OK) {
                xSemaphoreGive(s_state.state_mutex);
                return -EIO;
            }
        }
        dev_hdl = s_state.devices[dev_slot].dev_hdl;
    }

    const size_t payload_len = is_in ? req->in_capacity : req->out_len;

    /* For IN bulk/interrupt, round up to MPS while we hold the mutex. */
    size_t xfer_len = is_control ? (USB_SETUP_PACKET_SIZE + payload_len) : payload_len;
    if (!is_control && is_in && payload_len > 0 && dev_slot >= 0) {
        uint16_t mps = get_endpoint_mps_locked(dev_slot, req->endpoint_addr);
        if (mps > 0 && (xfer_len % mps) != 0) {
            xfer_len = ((xfer_len + mps - 1) / mps) * mps;
        }
    }
    xSemaphoreGive(s_state.state_mutex);

    if (dev_hdl == NULL) {
        return -ENODEV;
    }
    if (payload_len > CONFIG_USBIP_MAX_TRANSFER) {
        return -EMSGSIZE;
    }

    usb_transfer_t *transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(xfer_len, 0, &transfer);
    if (err != ESP_OK || transfer == NULL) {
        return -ENOMEM;
    }

    if (is_control) {
        memcpy(transfer->data_buffer, &req->setup, USB_SETUP_PACKET_SIZE);
        if (!is_in && req->out_len > 0 && req->out_data != NULL) {
            memcpy(transfer->data_buffer + USB_SETUP_PACKET_SIZE, req->out_data, req->out_len);
        }
    } else if (!is_in && req->out_len > 0 && req->out_data != NULL) {
        memcpy(transfer->data_buffer, req->out_data, req->out_len);
    }

    transfer_done_ctx_t done_ctx = {
        .done = false,
        .cancel = req->cancel,
    };

    transfer->callback = control_transfer_done_cb;
    transfer->context = &done_ctx;
    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = req->endpoint_addr;
    transfer->num_bytes = xfer_len;
    transfer->timeout_ms = 5000;

    if (is_control) {
        err = usb_host_transfer_submit_control(s_state.client_hdl, transfer);
    } else {
        err = usb_host_transfer_submit(transfer);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "submit failed: %s (ep=0x%02x)", esp_err_to_name(err), req->endpoint_addr);
        usb_host_transfer_free(transfer);
        return (err == ESP_ERR_INVALID_STATE) ? -ENODEV : -EIO;
    }

    bool cancelled = false;
    while (!done_ctx.done) {
        if (!cancelled && req->cancel != NULL && *req->cancel) {
            cancelled = true;
        }
        err = usb_host_client_handle_events(s_state.client_hdl, pdMS_TO_TICKS(10));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "client_handle_events error: %s, done=%d (ep=0x%02x)",
                     esp_err_to_name(err), done_ctx.done, req->endpoint_addr);
            break;
        }
    }

    if (cancelled) {
        usb_host_transfer_free(transfer);
        return -ECONNRESET;
    }

    int status = map_transfer_status_to_errno(transfer->status);
    if (status != 0) {
        ESP_LOGW(TAG, "transfer failed: usb_status=%d errno=%d done=%d (ep=0x%02x)",
                 transfer->status, status, done_ctx.done, req->endpoint_addr);
    }
    if (status == 0 && is_in && req->in_data != NULL && req->in_capacity > 0) {
        size_t bytes;
        if (is_control) {
            bytes = (transfer->actual_num_bytes > USB_SETUP_PACKET_SIZE)
                        ? (transfer->actual_num_bytes - USB_SETUP_PACKET_SIZE)
                        : 0;
        } else {
            bytes = transfer->actual_num_bytes;
            if (bytes > payload_len) {
                bytes = payload_len;
            }
        }
        const size_t copy_len = (bytes > req->in_capacity) ? req->in_capacity : bytes;
        const uint8_t *src = is_control
            ? (transfer->data_buffer + USB_SETUP_PACKET_SIZE)
            : transfer->data_buffer;
        memcpy(req->in_data, src, copy_len);
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

    ESP_LOGI(TAG, "USB backend task running (%d pipe slots)", USB_BACKEND_NUM_PIPES);

    while (true) {
        /* Wait for a notification from a caller or a short timeout so we
           still pump USB events regularly. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        err = usb_host_client_handle_events(s_state.client_hdl, 0);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "usb_host_client_handle_events failed: %s", esp_err_to_name(err));
        }

        process_backend_events();

        /* Process all active pipe request slots. */
        for (int i = 0; i < USB_BACKEND_NUM_PIPES; i++) {
            usb_backend_pipe_req_t *pipe = &s_state.pipes[i];
            if (!pipe->active) {
                continue;
            }

            int status = process_pipe_request(pipe);
            if (pipe->status_out != NULL) {
                *pipe->status_out = status;
            }
            pipe->active = false;
            xSemaphoreGive(pipe->done_sem);
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

    /* Pre-allocate a binary semaphore for each pipe slot. */
    for (int i = 0; i < USB_BACKEND_NUM_PIPES; i++) {
        s_state.pipes[i].done_sem = xSemaphoreCreateBinary();
        if (s_state.pipes[i].done_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
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
                    &s_state.task_hdl) != pdPASS) {
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

    /* Append virtual devices. */
    if (copied < max_devices) {
        copied += virtual_device_get_all(out_devices + copied, max_devices - copied);
    }

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

    if (!found) {
        virtual_device_t *vdev = virtual_device_find_by_busid(busid);
        if (vdev != NULL) {
            *out_device = vdev->desc;
            found = true;
        }
    }

    return found;
}

/* Look up the pre-assigned pipe slot for this device+endpoint,
   fill it, wake the backend task, and wait. */
static int submit_pipe_request(const char busid[32],
                               uint8_t endpoint_addr,
                               const usb_setup_packet_t *setup,
                               const uint8_t *out_data,
                               size_t out_len,
                               uint8_t *in_data,
                               size_t in_capacity,
                               size_t *in_len,
                               volatile bool *cancel)
{
    if (busid == NULL || in_len == NULL) {
        return -EINVAL;
    }

    *in_len = 0;

    /* Look up the pre-assigned pipe slot.  For non-zero endpoints that
       haven't had interfaces claimed yet, use the EP0 pipe — the backend
       task will claim interfaces (and assign proper pipe slots) before
       submitting the transfer. */
    int pipe_idx = -1;
    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int dev_slot = find_slot_by_busid_locked(busid);
    if (dev_slot >= 0) {
        if (endpoint_addr == 0) {
            pipe_idx = s_state.devices[dev_slot].ep0_pipe;
        } else {
            const usbip_backend_device_t *device = &s_state.devices[dev_slot].device;
            for (uint8_t i = 0; i < device->num_endpoints; i++) {
                if (device->endpoints[i].address == endpoint_addr) {
                    pipe_idx = s_state.devices[dev_slot].ep_pipes[i];
                    break;
                }
            }
            /* If interfaces aren't claimed yet, use the EP0 pipe slot
               temporarily.  process_pipe_request will claim interfaces
               before submitting the USB transfer. */
            if (pipe_idx < 0) {
                pipe_idx = s_state.devices[dev_slot].ep0_pipe;
            }
        }
    }
    xSemaphoreGive(s_state.state_mutex);

    if (dev_slot < 0) {
        return -ENODEV;
    }
    if (pipe_idx < 0) {
        ESP_LOGW(TAG, "No pipe slot for ep=0x%02x on %s", endpoint_addr, busid);
        return -ENODEV;
    }
    usb_backend_pipe_req_t *pipe = &s_state.pipes[pipe_idx];

    int status = -EIO;

    memcpy(pipe->busid, busid, sizeof(pipe->busid));
    pipe->endpoint_addr = endpoint_addr;
    if (setup != NULL) {
        pipe->setup = *setup;
    }
    pipe->out_data = out_data;
    pipe->out_len = out_len;
    pipe->in_data = in_data;
    pipe->in_capacity = in_capacity;
    pipe->in_len_out = in_len;
    pipe->status_out = &status;
    pipe->cancel = cancel;

    /* Mark active and wake the backend task. */
    pipe->active = true;
    xTaskNotifyGive(s_state.task_hdl);

    /* Wait for the backend task to process this slot.  The semaphore is
       pre-allocated so it can never be freed out from under us. */
    xSemaphoreTake(pipe->done_sem, portMAX_DELAY);

    return status;
}

int usb_backend_control_transfer(const char busid[32],
                                 const usb_setup_packet_t *setup,
                                 const uint8_t *out_data,
                                 size_t out_len,
                                 uint8_t *in_data,
                                 size_t in_capacity,
                                 size_t *in_len,
                                 volatile bool *cancel)
{
    if (setup == NULL) {
        return -EINVAL;
    }
    return submit_pipe_request(busid, 0, setup,
                               out_data, out_len,
                               in_data, in_capacity, in_len,
                               cancel);
}

int usb_backend_bulk_transfer(const char busid[32],
                              uint8_t endpoint_addr,
                              const uint8_t *out_data,
                              size_t out_len,
                              uint8_t *in_data,
                              size_t in_capacity,
                              size_t *in_len,
                              volatile bool *cancel)
{
    return submit_pipe_request(busid, endpoint_addr, NULL,
                               out_data, out_len,
                               in_data, in_capacity, in_len,
                               cancel);
}

int usb_backend_interrupt_transfer(const char busid[32],
                                   uint8_t endpoint_addr,
                                   const uint8_t *out_data,
                                   size_t out_len,
                                   uint8_t *in_data,
                                   size_t in_capacity,
                                   size_t *in_len,
                                   volatile bool *cancel)
{
    return submit_pipe_request(busid, endpoint_addr, NULL,
                               out_data, out_len,
                               in_data, in_capacity, in_len,
                               cancel);
}

bool usb_backend_is_interrupt_endpoint(const char busid[32], uint8_t ep_num, uint8_t direction)
{
    if (busid == NULL) {
        return false;
    }

    const uint8_t ep_addr = ep_num | (direction ? 0x80 : 0x00);

    xSemaphoreTake(s_state.state_mutex, portMAX_DELAY);
    const int slot = find_slot_by_busid_locked(busid);
    if (slot < 0) {
        xSemaphoreGive(s_state.state_mutex);
        return false;
    }

    const usbip_backend_device_t *device = &s_state.devices[slot].device;
    for (uint8_t i = 0; i < device->num_endpoints; i++) {
        if (device->endpoints[i].address == ep_addr) {
            bool is_intr = (device->endpoints[i].attributes & 0x03) == 0x03;
            xSemaphoreGive(s_state.state_mutex);
            return is_intr;
        }
    }

    xSemaphoreGive(s_state.state_mutex);
    return false;
}
