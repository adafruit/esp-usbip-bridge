cmake_minimum_required(VERSION 3.16)

# Make esp-harness components available to the IDF build.  Whether they're
# compiled is gated by main's REQUIRES list, which checks
# CONFIG_USBIP_VIRTUAL_HARNESS.
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/esp-harness/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Force Full-Speed only mode on the DWC OTG USB controller.
# Split transactions (TT) required for FS devices behind HS hubs are not
# supported in the DWC OTG's Scatter/Gather DMA (DDMA) mode. By forcing
# FS-only, hubs negotiate at FS and all downstream FS devices work natively.
idf_build_set_property(COMPILE_DEFINITIONS "USB_DWC_FSLS_ONLY" APPEND)

project(esp32p4_usbip_bridge)
