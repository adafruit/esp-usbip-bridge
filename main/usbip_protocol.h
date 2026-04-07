#ifndef USBIP_PROTOCOL_H
#define USBIP_PROTOCOL_H

#include <stdint.h>

#define USBIP_TCP_PORT 3240
#define USBIP_VERSION 0x0111

#define USBIP_OP_REQ_IMPORT 0x8003
#define USBIP_OP_REP_IMPORT 0x0003
#define USBIP_OP_REQ_DEVLIST 0x8005
#define USBIP_OP_REP_DEVLIST 0x0005

#define USBIP_CMD_SUBMIT 0x00000001
#define USBIP_CMD_UNLINK 0x00000002
#define USBIP_RET_SUBMIT 0x00000003
#define USBIP_RET_UNLINK 0x00000004

#define USBIP_DIR_OUT 0
#define USBIP_DIR_IN 1

#define USBIP_NON_ISO_PACKETS 0xFFFFFFFFu

#define USBIP_MAX_INTERFACES 8

typedef struct __attribute__((packed)) {
    uint16_t version;
    uint16_t code;
    uint32_t status;
} usbip_op_common_t;

typedef struct __attribute__((packed)) {
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
} usbip_header_basic_t;

typedef struct __attribute__((packed)) {
    uint32_t transfer_flags;
    int32_t transfer_buffer_length;
    int32_t start_frame;
    int32_t number_of_packets;
    int32_t interval;
    uint8_t setup[8];
} usbip_cmd_submit_t;

typedef struct __attribute__((packed)) {
    int32_t status;
    uint32_t actual_length;
    int32_t start_frame;
    int32_t number_of_packets;
    int32_t error_count;
    uint64_t padding;
} usbip_ret_submit_t;

typedef struct __attribute__((packed)) {
    uint32_t unlink_seqnum;
    uint32_t padding[6];
} usbip_cmd_unlink_t;

typedef struct __attribute__((packed)) {
    int32_t status;
    uint8_t padding[24];
} usbip_ret_unlink_t;

typedef struct __attribute__((packed)) {
    usbip_header_basic_t base;
    union {
        usbip_cmd_submit_t cmd_submit;
        usbip_ret_submit_t ret_submit;
        usbip_cmd_unlink_t cmd_unlink;
        usbip_ret_unlink_t ret_unlink;
    } u;
} usbip_header_t;

typedef struct __attribute__((packed)) {
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
} usbip_device_desc_t;

typedef struct __attribute__((packed)) {
    uint8_t interface_class;
    uint8_t interface_subclass;
    uint8_t interface_protocol;
    uint8_t padding;
} usbip_interface_desc_t;

_Static_assert(sizeof(usbip_header_t) == 48, "USB/IP header must be 48 bytes");

#endif
