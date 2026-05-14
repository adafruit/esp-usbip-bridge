#ifndef VIRTUAL_HARNESS_H
#define VIRTUAL_HARNESS_H

#include "esp_err.h"

/* Register a virtual CDC-ACM device that runs the esp-harness SCPI parser
   internally.  The device appears as a USB serial port to the USB/IP host.
   Call after usb_backend_start() and after nvs_flash_init(). */
esp_err_t virtual_harness_start(void);

#endif
