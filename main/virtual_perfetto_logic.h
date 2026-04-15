#ifndef VIRTUAL_PERFETTO_LOGIC_H
#define VIRTUAL_PERFETTO_LOGIC_H

#include "esp_err.h"

/* Register a virtual CDC-ACM device that runs the SUMP/Perfetto logic
   analyzer internally.  The device appears as a USB serial port to the
   USB/IP host.  Call after usb_backend_start(). */
esp_err_t virtual_perfetto_logic_start(void);

#endif
