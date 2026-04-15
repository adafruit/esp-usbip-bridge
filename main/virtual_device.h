#ifndef VIRTUAL_DEVICE_H
#define VIRTUAL_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "usb_backend.h"

#define VIRTUAL_DEVICE_MAX 4
#define VIRTUAL_DEVICE_BUSNUM 2

typedef struct virtual_device virtual_device_t;

typedef struct {
    /* Handle a control transfer (EP0).  Return 0 on success, negative errno on error. */
    int (*control_transfer)(virtual_device_t *dev,
                            const usb_setup_packet_t *setup,
                            const uint8_t *out_data, size_t out_len,
                            uint8_t *in_data, size_t in_capacity, size_t *in_len);

    /* Handle a bulk or interrupt transfer on a non-zero endpoint.
       ep_addr has direction bit set (0x8N for IN, 0x0N for OUT).
       Return 0 on success, negative errno on error. */
    int (*data_transfer)(virtual_device_t *dev,
                         uint8_t ep_addr,
                         const uint8_t *out_data, size_t out_len,
                         uint8_t *in_data, size_t in_capacity, size_t *in_len);
} virtual_device_ops_t;

struct virtual_device {
    const virtual_device_ops_t *ops;
    usbip_backend_device_t desc;
    void *ctx;
};

/* Register a virtual device.  Assigns a busid on bus VIRTUAL_DEVICE_BUSNUM.
   The device will appear in usb_backend_get_devices() and be addressable
   by its busid for import and transfers.  Returns ESP_OK on success. */
esp_err_t virtual_device_register(virtual_device_t *dev);

/* Look up a virtual device by busid.  Returns NULL if not found. */
virtual_device_t *virtual_device_find_by_busid(const char busid[32]);

/* Get all registered virtual devices.  Returns the number copied. */
size_t virtual_device_get_all(usbip_backend_device_t *out_devices, size_t max_devices);

#endif
