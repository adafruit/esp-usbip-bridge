#ifndef USB_BACKEND_H
#define USB_BACKEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "usb/usb_types_ch9.h"

#include "usbip_protocol.h"

typedef struct {
    uint8_t interface_class;
    uint8_t interface_subclass;
    uint8_t interface_protocol;
} usbip_backend_interface_t;

#define USBIP_MAX_ENDPOINTS 16

typedef struct {
    uint8_t address;
    uint8_t attributes;
    uint16_t max_packet_size;
    uint8_t interval;
} usbip_backend_endpoint_t;

typedef struct {
    bool present;
    char path[256];
    char busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t id_vendor;
    uint16_t id_product;
    uint16_t bcd_device;
    uint8_t device_class;
    uint8_t device_subclass;
    uint8_t device_protocol;
    uint8_t configuration_value;
    uint8_t num_configurations;
    uint8_t num_interfaces;
    usbip_backend_interface_t interfaces[USBIP_MAX_INTERFACES];
    uint8_t num_endpoints;
    usbip_backend_endpoint_t endpoints[USBIP_MAX_ENDPOINTS];
} usbip_backend_device_t;

esp_err_t usb_backend_start(void);
size_t usb_backend_get_devices(usbip_backend_device_t *out_devices, size_t max_devices);
bool usb_backend_get_device_by_busid(const char busid[32], usbip_backend_device_t *out_device);
int usb_backend_control_transfer(const char busid[32],
                                 const usb_setup_packet_t *setup,
                                 const uint8_t *out_data,
                                 size_t out_len,
                                 uint8_t *in_data,
                                 size_t in_capacity,
                                 size_t *in_len,
                                 volatile bool *cancel);
int usb_backend_bulk_transfer(const char busid[32],
                              uint8_t endpoint_addr,
                              const uint8_t *out_data,
                              size_t out_len,
                              uint8_t *in_data,
                              size_t in_capacity,
                              size_t *in_len,
                              volatile bool *cancel);
int usb_backend_interrupt_transfer(const char busid[32],
                                   uint8_t endpoint_addr,
                                   const uint8_t *out_data,
                                   size_t out_len,
                                   uint8_t *in_data,
                                   size_t in_capacity,
                                   size_t *in_len,
                                   volatile bool *cancel);
bool usb_backend_is_interrupt_endpoint(const char busid[32], uint8_t ep_num, uint8_t direction);

#endif
