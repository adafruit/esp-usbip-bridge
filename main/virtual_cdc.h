#ifndef VIRTUAL_CDC_H
#define VIRTUAL_CDC_H

#include <stdint.h>
#include "virtual_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

/* CDC-ACM virtual device context.  Handles Chapter 9 standard requests
   and CDC class requests.  Bulk IN/OUT map to stream buffers. */
typedef struct {
    /* Stream buffers for data exchange (not owned — caller provides). */
    StreamBufferHandle_t rx;   /* host -> device (bulk OUT) */
    StreamBufferHandle_t tx;   /* device -> host (bulk IN)  */

    /* USB descriptors (caller provides, must remain valid). */
    const uint8_t *device_desc;        /* 18 bytes */
    const uint8_t *config_desc;        /* full configuration descriptor blob */
    size_t config_desc_len;
    const char *str_manufacturer;
    const char *str_product;
    const char *str_serial;

    /* Endpoint addresses from the config descriptor. */
    uint8_t bulk_in_ep;       /* e.g. 0x81 */
    uint8_t bulk_out_ep;      /* e.g. 0x02 */
    uint8_t notify_ep;        /* e.g. 0x83 (interrupt IN, optional) */

    /* Internal state. */
    uint8_t current_config;
    uint8_t line_coding[7];   /* baud(4) + stop(1) + parity(1) + data_bits(1) */
} virtual_cdc_ctx_t;

/* Virtual device ops for CDC-ACM.  Use with virtual_device_t.ops. */
extern const virtual_device_ops_t virtual_cdc_ops;

#endif
