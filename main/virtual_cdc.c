#include "virtual_cdc.h"

#include <errno.h>
#include <string.h>
#include "esp_log.h"
#include "usb/usb_types_ch9.h"

static const char *TAG = "virtual_cdc";

/* USB descriptor type constants. */
#define DESC_TYPE_STRING 0x03

/* CDC class request codes. */
#define CDC_SET_LINE_CODING        0x20
#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22

/* Build a USB string descriptor from a C string.
   Returns number of bytes written to buf, or 0 on error. */
static size_t make_string_desc(const char *str, uint8_t *buf, size_t buf_size)
{
    if (str == NULL) {
        /* Empty string descriptor. */
        if (buf_size < 2) {
            return 0;
        }
        buf[0] = 2;
        buf[1] = DESC_TYPE_STRING;
        return 2;
    }

    size_t slen = strlen(str);
    size_t desc_len = 2 + slen * 2;
    if (desc_len > 255 || desc_len > buf_size) {
        desc_len = (buf_size < 255) ? buf_size : 254;
        desc_len &= ~1u; /* keep even */
        slen = (desc_len - 2) / 2;
    }
    buf[0] = (uint8_t)desc_len;
    buf[1] = DESC_TYPE_STRING;
    for (size_t i = 0; i < slen; i++) {
        buf[2 + i * 2] = (uint8_t)str[i];
        buf[2 + i * 2 + 1] = 0;
    }
    return desc_len;
}

/* String descriptor 0: language ID (US English). */
static const uint8_t s_lang_desc[] = { 4, DESC_TYPE_STRING, 0x09, 0x04 };

static int handle_get_descriptor(virtual_cdc_ctx_t *ctx,
                                 const usb_setup_packet_t *setup,
                                 uint8_t *in_data, size_t in_capacity, size_t *in_len)
{
    const uint8_t desc_type = setup->wValue >> 8;
    const uint8_t desc_index = setup->wValue & 0xFF;
    size_t len = 0;

    switch (desc_type) {
    case USB_B_DESCRIPTOR_TYPE_DEVICE:
        len = 18;
        if (len > in_capacity) {
            len = in_capacity;
        }
        memcpy(in_data, ctx->device_desc, len);
        *in_len = len;
        return 0;

    case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
        len = ctx->config_desc_len;
        if (len > in_capacity) {
            len = in_capacity;
        }
        memcpy(in_data, ctx->config_desc, len);
        *in_len = len;
        return 0;

    case DESC_TYPE_STRING: {
        uint8_t tmp[128];
        size_t desc_len = 0;

        if (desc_index == 0) {
            desc_len = sizeof(s_lang_desc);
            memcpy(tmp, s_lang_desc, desc_len);
        } else if (desc_index == 1) {
            desc_len = make_string_desc(ctx->str_manufacturer, tmp, sizeof(tmp));
        } else if (desc_index == 2) {
            desc_len = make_string_desc(ctx->str_product, tmp, sizeof(tmp));
        } else if (desc_index == 3) {
            desc_len = make_string_desc(ctx->str_serial, tmp, sizeof(tmp));
        } else {
            return -EPIPE; /* STALL — unknown string index */
        }

        if (desc_len == 0) {
            return -EPIPE;
        }
        len = desc_len;
        if (len > in_capacity) {
            len = in_capacity;
        }
        memcpy(in_data, tmp, len);
        *in_len = len;
        return 0;
    }

    case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
        /* Full-speed only device — STALL device qualifier. */
        return -EPIPE;

    default:
        ESP_LOGW(TAG, "Unknown descriptor type 0x%02x", desc_type);
        return -EPIPE;
    }
}

static int cdc_control_transfer(virtual_device_t *dev,
                                const usb_setup_packet_t *setup,
                                const uint8_t *out_data, size_t out_len,
                                uint8_t *in_data, size_t in_capacity, size_t *in_len)
{
    virtual_cdc_ctx_t *ctx = (virtual_cdc_ctx_t *)dev->ctx;
    *in_len = 0;

    const uint8_t req_type = setup->bmRequestType;
    const uint8_t req = setup->bRequest;

    /* Standard device requests. */
    if ((req_type & USB_BM_REQUEST_TYPE_TYPE_MASK) == USB_BM_REQUEST_TYPE_TYPE_STANDARD) {
        switch (req) {
        case USB_B_REQUEST_GET_DESCRIPTOR:
            return handle_get_descriptor(ctx, setup, in_data, in_capacity, in_len);

        case USB_B_REQUEST_SET_CONFIGURATION:
            ctx->current_config = setup->wValue & 0xFF;
            ESP_LOGI(TAG, "SET_CONFIGURATION %u", ctx->current_config);
            return 0;

        case USB_B_REQUEST_GET_CONFIGURATION:
            if (in_capacity >= 1) {
                in_data[0] = ctx->current_config;
                *in_len = 1;
            }
            return 0;

        case USB_B_REQUEST_SET_INTERFACE:
            ESP_LOGI(TAG, "SET_INTERFACE %u alt %u", setup->wIndex, setup->wValue);
            return 0;

        case USB_B_REQUEST_GET_STATUS:
            if (in_capacity >= 2) {
                in_data[0] = 0;
                in_data[1] = 0;
                *in_len = 2;
            }
            return 0;

        case USB_B_REQUEST_CLEAR_FEATURE:
        case USB_B_REQUEST_SET_FEATURE:
            return 0;

        default:
            ESP_LOGW(TAG, "Unhandled std request 0x%02x", req);
            return -EPIPE;
        }
    }

    /* CDC class requests. */
    if ((req_type & USB_BM_REQUEST_TYPE_TYPE_MASK) == USB_BM_REQUEST_TYPE_TYPE_CLASS) {
        switch (req) {
        case CDC_SET_LINE_CODING:
            if (out_data != NULL && out_len >= sizeof(ctx->line_coding)) {
                memcpy(ctx->line_coding, out_data, sizeof(ctx->line_coding));
            }
            ESP_LOGD(TAG, "SET_LINE_CODING");
            return 0;

        case CDC_GET_LINE_CODING:
            if (in_capacity >= sizeof(ctx->line_coding)) {
                memcpy(in_data, ctx->line_coding, sizeof(ctx->line_coding));
                *in_len = sizeof(ctx->line_coding);
            }
            return 0;

        case CDC_SET_CONTROL_LINE_STATE:
            ESP_LOGD(TAG, "SET_CONTROL_LINE_STATE 0x%04x", setup->wValue);
            return 0;

        default:
            ESP_LOGW(TAG, "Unhandled CDC class request 0x%02x", req);
            return -EPIPE;
        }
    }

    ESP_LOGW(TAG, "Unhandled request type=0x%02x req=0x%02x", req_type, req);
    return -EPIPE;
}

static int cdc_data_transfer(virtual_device_t *dev,
                             uint8_t ep_addr,
                             const uint8_t *out_data, size_t out_len,
                             uint8_t *in_data, size_t in_capacity, size_t *in_len)
{
    virtual_cdc_ctx_t *ctx = (virtual_cdc_ctx_t *)dev->ctx;
    *in_len = 0;

    if (ep_addr == ctx->bulk_out_ep) {
        /* Host -> device: push into rx stream buffer. */
        if (out_data != NULL && out_len > 0) {
            xStreamBufferSend(ctx->rx, out_data, out_len, pdMS_TO_TICKS(1000));
        }
        return 0;
    }

    if (ep_addr == ctx->bulk_in_ep) {
        /* Device -> host: read from tx stream buffer.
           Non-blocking — the URB stream is sequential, so blocking here
           would prevent processing of bulk OUT URBs that carry commands.
           The host kernel resubmits bulk IN URBs continuously. */
        size_t n = xStreamBufferReceive(ctx->tx, in_data, in_capacity, 0);
        *in_len = n;
        return 0;
    }

    if (ep_addr == ctx->notify_ep) {
        /* CDC notification endpoint — no pending notifications.
           Return success with zero bytes; the kernel will resubmit
           after the endpoint's bInterval. */
        return 0;
    }

    ESP_LOGW(TAG, "Unknown endpoint 0x%02x", ep_addr);
    return -EPIPE;
}

const virtual_device_ops_t virtual_cdc_ops = {
    .control_transfer = cdc_control_transfer,
    .data_transfer = cdc_data_transfer,
};
