#include "usbip_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "usb_backend.h"
#include "usbip_protocol.h"

static const char *TAG = "usbip";

static bool read_exact(int fd, void *buf, size_t len)
{
    uint8_t *ptr = (uint8_t *)buf;
    size_t remaining = len;

    while (remaining > 0) {
        const ssize_t n = recv(fd, ptr, remaining, 0);
        if (n <= 0) {
            return false;
        }
        ptr += n;
        remaining -= (size_t)n;
    }

    return true;
}

static bool write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *ptr = (const uint8_t *)buf;
    size_t remaining = len;

    while (remaining > 0) {
        const ssize_t n = send(fd, ptr, remaining, 0);
        if (n <= 0) {
            return false;
        }
        ptr += n;
        remaining -= (size_t)n;
    }

    return true;
}

static bool discard_exact(int fd, size_t len)
{
    uint8_t scratch[128];
    size_t remaining = len;

    while (remaining > 0) {
        size_t chunk = remaining;
        if (chunk > sizeof(scratch)) {
            chunk = sizeof(scratch);
        }

        if (!read_exact(fd, scratch, chunk)) {
            return false;
        }

        remaining -= chunk;
    }

    return true;
}

static uint32_t make_devid(const usbip_backend_device_t *device)
{
    return (device->busnum << 16) | (device->devnum & 0xFFFFu);
}

static void fill_wire_device_desc(const usbip_backend_device_t *src, usbip_device_desc_t *dst)
{
    memset(dst, 0, sizeof(*dst));

    memcpy(dst->path, src->path, sizeof(dst->path));
    memcpy(dst->busid, src->busid, sizeof(dst->busid));

    dst->busnum = htonl(src->busnum);
    dst->devnum = htonl(src->devnum);
    dst->speed = htonl(src->speed);
    dst->id_vendor = htons(src->id_vendor);
    dst->id_product = htons(src->id_product);
    dst->bcd_device = htons(src->bcd_device);
    dst->device_class = src->device_class;
    dst->device_subclass = src->device_subclass;
    dst->device_protocol = src->device_protocol;
    dst->configuration_value = src->configuration_value;
    dst->num_configurations = src->num_configurations;
    dst->num_interfaces = src->num_interfaces;
}

static bool send_device_with_interfaces(int fd, const usbip_backend_device_t *device)
{
    usbip_device_desc_t wire_device;
    fill_wire_device_desc(device, &wire_device);

    if (!write_all(fd, &wire_device, sizeof(wire_device))) {
        return false;
    }

    for (uint8_t i = 0; i < device->num_interfaces; i++) {
        usbip_interface_desc_t intf = {
            .interface_class = device->interfaces[i].interface_class,
            .interface_subclass = device->interfaces[i].interface_subclass,
            .interface_protocol = device->interfaces[i].interface_protocol,
            .padding = 0,
        };
        if (!write_all(fd, &intf, sizeof(intf))) {
            return false;
        }
    }

    return true;
}

static bool send_op_common(int fd, uint16_t code, uint32_t status)
{
    usbip_op_common_t reply = {
        .version = htons(USBIP_VERSION),
        .code = htons(code),
        .status = htonl(status),
    };

    return write_all(fd, &reply, sizeof(reply));
}

static bool send_ret_submit(int fd,
                            const usbip_header_t *request,
                            int32_t status,
                            const uint8_t *payload,
                            uint32_t payload_len)
{
    usbip_header_t reply;
    memset(&reply, 0, sizeof(reply));

    reply.base.command = htonl(USBIP_RET_SUBMIT);
    reply.base.seqnum = request->base.seqnum;
    reply.base.devid = request->base.devid;
    reply.base.direction = request->base.direction;
    reply.base.ep = request->base.ep;

    reply.u.ret_submit.status = htonl((uint32_t)status);
    reply.u.ret_submit.actual_length = htonl(payload_len);
    reply.u.ret_submit.start_frame = htonl(0);
    reply.u.ret_submit.number_of_packets = htonl(USBIP_NON_ISO_PACKETS);
    reply.u.ret_submit.error_count = htonl(0);
    reply.u.ret_submit.padding = 0;

    if (!write_all(fd, &reply, sizeof(reply))) {
        return false;
    }

    if (payload_len > 0 && payload != NULL) {
        if (!write_all(fd, payload, payload_len)) {
            return false;
        }
    }

    return true;
}

static bool send_ret_unlink(int fd, const usbip_header_t *request, int32_t status)
{
    usbip_header_t reply;
    memset(&reply, 0, sizeof(reply));

    reply.base.command = htonl(USBIP_RET_UNLINK);
    reply.base.seqnum = request->base.seqnum;
    reply.base.devid = request->base.devid;
    reply.base.direction = request->base.direction;
    reply.base.ep = request->base.ep;

    reply.u.ret_unlink.status = htonl((uint32_t)status);

    return write_all(fd, &reply, sizeof(reply));
}

static bool handle_submit(int fd,
                          const usbip_header_t *request,
                          const char imported_busid[32],
                          uint32_t expected_devid)
{
    const uint32_t devid = ntohl(request->base.devid);
    const uint32_t direction = ntohl(request->base.direction);
    const uint32_t endpoint = ntohl(request->base.ep);
    const int32_t req_len = (int32_t)ntohl((uint32_t)request->u.cmd_submit.transfer_buffer_length);
    const uint32_t packet_count = (uint32_t)ntohl((uint32_t)request->u.cmd_submit.number_of_packets);


    if (req_len < 0) {
        return send_ret_submit(fd, request, -EINVAL, NULL, 0);
    }
    if (req_len > CONFIG_USBIP_MAX_TRANSFER) {
        if (direction == USBIP_DIR_OUT && req_len > 0) {
            if (!discard_exact(fd, (size_t)req_len)) {
                return false;
            }
        }
        return send_ret_submit(fd, request, -EMSGSIZE, NULL, 0);
    }
    if (direction != USBIP_DIR_OUT && direction != USBIP_DIR_IN) {
        return false;
    }

    if (devid != expected_devid) {
        if (direction == USBIP_DIR_OUT && req_len > 0) {
            if (!discard_exact(fd, (size_t)req_len)) {
                return false;
            }
        }
        return send_ret_submit(fd, request, -ENODEV, NULL, 0);
    }

    uint8_t *out_buf = NULL;
    if (direction == USBIP_DIR_OUT && req_len > 0) {
        out_buf = malloc((size_t)req_len);
        if (out_buf == NULL) {
            if (!discard_exact(fd, (size_t)req_len)) {
                return false;
            }
            return send_ret_submit(fd, request, -ENOMEM, NULL, 0);
        }

        if (!read_exact(fd, out_buf, (size_t)req_len)) {
            free(out_buf);
            return false;
        }
    }

    /* The kernel sends number_of_packets=0 for non-ISO transfers while
       some clients send 0xFFFFFFFF (-1).  Reject only positive values,
       which indicate actual isochronous packet counts (unsupported). */
    if (packet_count != USBIP_NON_ISO_PACKETS && packet_count != 0) {
        free(out_buf);
        return send_ret_submit(fd, request, -EOPNOTSUPP, NULL, 0);
    }

    uint8_t *in_buf = NULL;
    const size_t in_cap = (direction == USBIP_DIR_IN) ? (size_t)req_len : 0;
    if (in_cap > 0) {
        in_buf = malloc(in_cap);
        if (in_buf == NULL) {
            free(out_buf);
            return send_ret_submit(fd, request, -ENOMEM, NULL, 0);
        }
    }

    size_t in_len = 0;
    int status;

    if (endpoint == 0) {
        /* Control transfer on EP0. */
        usb_setup_packet_t setup;
        memcpy(&setup, request->u.cmd_submit.setup, sizeof(setup));
        const bool setup_in = (setup.bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;
        if (((direction == USBIP_DIR_IN) ? true : false) != setup_in) {
            free(out_buf);
            free(in_buf);
            return send_ret_submit(fd, request, -EINVAL, NULL, 0);
        }

        status = usb_backend_control_transfer(imported_busid,
                                              &setup,
                                              out_buf,
                                              (direction == USBIP_DIR_OUT) ? (size_t)req_len : 0,
                                              in_buf,
                                              in_cap,
                                              &in_len);
    } else {
        /* Bulk or interrupt transfer on a non-zero endpoint. */
        const uint8_t ep_addr = (uint8_t)(endpoint |
                                          (direction == USBIP_DIR_IN ? 0x80 : 0x00));

        if (usb_backend_is_interrupt_endpoint(imported_busid, endpoint, direction == USBIP_DIR_IN)) {
            status = usb_backend_interrupt_transfer(imported_busid,
                                                    ep_addr,
                                                    out_buf,
                                                    (direction == USBIP_DIR_OUT) ? (size_t)req_len : 0,
                                                    in_buf,
                                                    in_cap,
                                                    &in_len);
        } else {
            status = usb_backend_bulk_transfer(imported_busid,
                                               ep_addr,
                                               out_buf,
                                               (direction == USBIP_DIR_OUT) ? (size_t)req_len : 0,
                                               in_buf,
                                               in_cap,
                                               &in_len);
        }
    }

    const bool ok = send_ret_submit(fd,
                                    request,
                                    status,
                                    (status == 0 && direction == USBIP_DIR_IN) ? in_buf : NULL,
                                    (status == 0 && direction == USBIP_DIR_IN) ? (uint32_t)in_len : 0);

    free(out_buf);
    free(in_buf);

    return ok;
}

static bool handle_urb_stream(int fd, const char imported_busid[32], uint32_t expected_devid)
{
    while (true) {
        usbip_header_t request;
        if (!read_exact(fd, &request, sizeof(request))) {
            return false;
        }

        const uint32_t command = ntohl(request.base.command);
        if (command == USBIP_CMD_SUBMIT) {
            if (!handle_submit(fd, &request, imported_busid, expected_devid)) {
                return false;
            }
        } else if (command == USBIP_CMD_UNLINK) {
            if (!send_ret_unlink(fd, &request, 0)) {
                return false;
            }
        } else {
            ESP_LOGW(TAG, "Unsupported USB/IP command: 0x%08" PRIx32, command);
            return false;
        }
    }
}

static bool handle_devlist_request(int fd)
{
    usbip_backend_device_t *devices = malloc(CONFIG_USBIP_MAX_DEVICES * sizeof(usbip_backend_device_t));
    if (devices == NULL) {
        return false;
    }

    const size_t device_count = usb_backend_get_devices(devices, CONFIG_USBIP_MAX_DEVICES);

    if (!send_op_common(fd, USBIP_OP_REP_DEVLIST, 0)) {
        free(devices);
        return false;
    }

    const uint32_t count = htonl((uint32_t)device_count);
    if (!write_all(fd, &count, sizeof(count))) {
        free(devices);
        return false;
    }

    for (size_t i = 0; i < device_count; i++) {
        if (!send_device_with_interfaces(fd, &devices[i])) {
            free(devices);
            return false;
        }
    }

    free(devices);
    return true;
}

static bool handle_import_request(int fd)
{
    char requested_busid[32];
    if (!read_exact(fd, requested_busid, sizeof(requested_busid))) {
        return false;
    }

    usbip_backend_device_t device;
    const bool found = usb_backend_get_device_by_busid(requested_busid, &device);

    if (!send_op_common(fd, USBIP_OP_REP_IMPORT, found ? 0 : 1)) {
        return false;
    }

    if (!found) {
        return true;
    }

    if (!send_device_with_interfaces(fd, &device)) {
        return false;
    }

    ESP_LOGI(TAG, "Client imported %s", device.busid);
    return handle_urb_stream(fd, requested_busid, make_devid(&device));
}

static void handle_client(int fd)
{
    usbip_op_common_t request;
    if (!read_exact(fd, &request, sizeof(request))) {
        return;
    }

    const uint16_t version = ntohs(request.version);
    const uint16_t code = ntohs(request.code);

    if (version != USBIP_VERSION) {
        ESP_LOGW(TAG, "Unsupported USB/IP version 0x%04x", version);
        return;
    }

    if (code == USBIP_OP_REQ_DEVLIST) {
        (void)handle_devlist_request(fd);
        return;
    }

    if (code == USBIP_OP_REQ_IMPORT) {
        (void)handle_import_request(fd);
        return;
    }

    ESP_LOGW(TAG, "Unsupported USB/IP op request 0x%04x", code);
}

static void usbip_server_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(USBIP_TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB/IP server listening on TCP %d", USBIP_TCP_PORT);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            ESP_LOGW(TAG, "accept() failed: errno=%d", errno);
            continue;
        }

        ESP_LOGI(TAG, "Client connected");
        handle_client(client_fd);
        close(client_fd);
        ESP_LOGI(TAG, "Client disconnected");
    }
}

esp_err_t usbip_server_start(void)
{
    if (xTaskCreate(usbip_server_task,
                    "usbip_server",
                    CONFIG_USBIP_SERVER_TASK_STACK,
                    NULL,
                    CONFIG_USBIP_SERVER_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
