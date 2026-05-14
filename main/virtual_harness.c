#include "virtual_harness.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

#include "virtual_device.h"
#include "virtual_cdc.h"
#include "harness_io.h"
#include "harness_scpi.h"
#include "harness_dut.h"

static const char *TAG = "vdev_harness";

#define IO_RX_SIZE  256
#define IO_TX_SIZE  (8 * 1024)

/* ---- USB descriptors for a CDC-ACM device ---- */

static const uint8_t s_device_desc[] = {
    18,                         /* bLength */
    0x01,                       /* bDescriptorType = DEVICE */
    0x00, 0x02,                 /* bcdUSB = 2.00 */
    0x02,                       /* bDeviceClass = CDC */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    64,                         /* bMaxPacketSize0 */
    0xFE, 0xCA,                 /* idVendor  = 0xCAFE (test) */
    0x02, 0x40,                 /* idProduct = 0x4002 */
    0x00, 0x01,                 /* bcdDevice = 1.00 */
    1,                          /* iManufacturer */
    2,                          /* iProduct */
    3,                          /* iSerialNumber */
    1,                          /* bNumConfigurations */
};

/* Configuration descriptor: CDC control interface + CDC data interface.
   Total length = 9 (config) + 9 (intf0) + 5+5+4+5 (CDC func) + 7 (ep notify)
                + 9 (intf1) + 7 (ep bulk out) + 7 (ep bulk in) = 67 bytes */
static const uint8_t s_config_desc[] = {
    /* Configuration descriptor */
    9, 0x02,
    67, 0x00,                   /* wTotalLength */
    2,                          /* bNumInterfaces */
    1,                          /* bConfigurationValue */
    0,                          /* iConfiguration */
    0x80,                       /* bmAttributes = bus-powered */
    50,                         /* bMaxPower = 100 mA */

    /* Interface 0: CDC Communication */
    9, 0x04,
    0,                          /* bInterfaceNumber */
    0,                          /* bAlternateSetting */
    1,                          /* bNumEndpoints */
    0x02,                       /* bInterfaceClass = CDC */
    0x02,                       /* bInterfaceSubClass = ACM */
    0x00,                       /* bInterfaceProtocol */
    0,                          /* iInterface */

    /* CDC Header Functional Descriptor */
    5, 0x24, 0x00, 0x20, 0x01,

    /* CDC Call Management Functional Descriptor */
    5, 0x24, 0x01, 0x00, 0x01,

    /* CDC Abstract Control Management Functional Descriptor */
    4, 0x24, 0x02, 0x02,

    /* CDC Union Functional Descriptor */
    5, 0x24, 0x06, 0x00, 0x01,

    /* Endpoint 3 IN — Notification (interrupt) */
    7, 0x05,
    0x83,                       /* bEndpointAddress = EP3 IN */
    0x03,                       /* bmAttributes = interrupt */
    8, 0x00,                    /* wMaxPacketSize = 8 */
    255,                        /* bInterval */

    /* Interface 1: CDC Data */
    9, 0x04,
    1,                          /* bInterfaceNumber */
    0,                          /* bAlternateSetting */
    2,                          /* bNumEndpoints */
    0x0A,                       /* bInterfaceClass = CDC Data */
    0x00,                       /* bInterfaceSubClass */
    0x00,                       /* bInterfaceProtocol */
    0,                          /* iInterface */

    /* Endpoint 2 OUT — Bulk */
    7, 0x05,
    0x02,                       /* bEndpointAddress = EP2 OUT */
    0x02,                       /* bmAttributes = bulk */
    64, 0x00,                   /* wMaxPacketSize = 64 */
    0,                          /* bInterval */

    /* Endpoint 1 IN — Bulk */
    7, 0x05,
    0x81,                       /* bEndpointAddress = EP1 IN */
    0x02,                       /* bmAttributes = bulk */
    64, 0x00,                   /* wMaxPacketSize = 64 */
    0,                          /* bInterval */
};

_Static_assert(sizeof(s_config_desc) == 67, "config descriptor size mismatch");

/* ---- Static instances ---- */

static harness_io_t s_io;
static virtual_cdc_ctx_t s_cdc_ctx;
static virtual_device_t s_vdev;

esp_err_t virtual_harness_start(void)
{
    /* DUT metadata is read by harness_gpio/harness_bus pin resolvers. */
    esp_err_t err = harness_dut_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "harness_dut_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Create stream-buffer I/O for the SCPI parser. */
    err = harness_io_create(IO_RX_SIZE, IO_TX_SIZE, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "harness_io_create failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Set up CDC context. */
    memset(&s_cdc_ctx, 0, sizeof(s_cdc_ctx));
    s_cdc_ctx.rx = s_io.rx;
    s_cdc_ctx.tx = s_io.tx;
    s_cdc_ctx.device_desc = s_device_desc;
    s_cdc_ctx.config_desc = s_config_desc;
    s_cdc_ctx.config_desc_len = sizeof(s_config_desc);
    s_cdc_ctx.str_manufacturer = "Adafruit";
    s_cdc_ctx.str_product = "ESP Test Harness";
    s_cdc_ctx.str_serial = "VIRTUAL-002";
    s_cdc_ctx.bulk_in_ep = 0x81;
    s_cdc_ctx.bulk_out_ep = 0x02;
    s_cdc_ctx.notify_ep = 0x83;

    /* Default line coding: 115200 8N1. */
    uint32_t baud = 115200;
    memcpy(s_cdc_ctx.line_coding, &baud, 4);
    s_cdc_ctx.line_coding[4] = 0;  /* 1 stop bit */
    s_cdc_ctx.line_coding[5] = 0;  /* no parity */
    s_cdc_ctx.line_coding[6] = 8;  /* 8 data bits */

    /* Set up virtual device. */
    memset(&s_vdev, 0, sizeof(s_vdev));
    s_vdev.ops = &virtual_cdc_ops;
    s_vdev.ctx = &s_cdc_ctx;

    /* Fill device descriptor fields for USB/IP export. */
    s_vdev.desc.speed = 2; /* full-speed */
    s_vdev.desc.id_vendor = 0xCAFE;
    s_vdev.desc.id_product = 0x4002;
    s_vdev.desc.bcd_device = 0x0100;
    s_vdev.desc.device_class = 0x02;    /* CDC */
    s_vdev.desc.device_subclass = 0x00;
    s_vdev.desc.device_protocol = 0x00;
    s_vdev.desc.num_configurations = 1;
    s_vdev.desc.configuration_value = 1;
    s_vdev.desc.num_interfaces = 2;

    /* Interface descriptors for USB/IP device list. */
    s_vdev.desc.interfaces[0].interface_class = 0x02;     /* CDC */
    s_vdev.desc.interfaces[0].interface_subclass = 0x02;  /* ACM */
    s_vdev.desc.interfaces[0].interface_protocol = 0x00;
    s_vdev.desc.interfaces[1].interface_class = 0x0A;     /* CDC Data */
    s_vdev.desc.interfaces[1].interface_subclass = 0x00;
    s_vdev.desc.interfaces[1].interface_protocol = 0x00;

    /* Endpoint descriptors for USB/IP. */
    s_vdev.desc.num_endpoints = 3;
    s_vdev.desc.endpoints[0].address = 0x83;
    s_vdev.desc.endpoints[0].attributes = 0x03;  /* interrupt */
    s_vdev.desc.endpoints[0].max_packet_size = 8;
    s_vdev.desc.endpoints[0].interval = 255;
    s_vdev.desc.endpoints[1].address = 0x02;
    s_vdev.desc.endpoints[1].attributes = 0x02;  /* bulk */
    s_vdev.desc.endpoints[1].max_packet_size = 64;
    s_vdev.desc.endpoints[1].interval = 0;
    s_vdev.desc.endpoints[2].address = 0x81;
    s_vdev.desc.endpoints[2].attributes = 0x02;  /* bulk */
    s_vdev.desc.endpoints[2].max_packet_size = 64;
    s_vdev.desc.endpoints[2].interval = 0;

    /* Register the virtual device. */
    err = virtual_device_register(&s_vdev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "virtual_device_register failed: %s", esp_err_to_name(err));
        harness_io_destroy(&s_io);
        return err;
    }

    /* Start the SCPI parser task. */
    harness_scpi_config_t scpi_cfg = {
        .io = &s_io,
        .task_stack_size = 8192,
        .task_priority = 5,
    };
    err = harness_scpi_init(&scpi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "harness_scpi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Virtual test harness device started on %s", s_vdev.desc.busid);
    return ESP_OK;
}
