cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Force Full-Speed only mode on the DWC OTG USB controller.
# Split transactions (TT) required for FS devices behind HS hubs are not
# supported in the DWC OTG's Scatter/Gather DMA (DDMA) mode. By forcing
# FS-only, hubs negotiate at FS and all downstream FS devices work natively.
idf_build_set_property(COMPILE_DEFINITIONS "USB_DWC_FSLS_ONLY" APPEND)

project(esp32p4_usbip_bridge)
