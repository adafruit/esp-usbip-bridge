#include "log_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#define LOG_SERVER_PORT         1514
#define LOG_SERVER_MAX_CLIENTS  4
#define LOG_SERVER_QUEUE_DEPTH  64
#define LOG_SERVER_MSG_SIZE     512
#define LOG_SERVER_TASK_STACK   4096
#define LOG_SERVER_TASK_PRIORITY 3

#define SYSLOG_FACILITY_USER    1
#define SYSLOG_VERSION          "1"
#define SYSLOG_HOSTNAME         "esp32"
#define SYSLOG_APPNAME          "esp"
#define SYSLOG_NILVALUE         "-"

static const char *TAG = "log_server";

typedef struct {
    uint16_t len;
    char data[LOG_SERVER_MSG_SIZE];
} log_msg_t;

static QueueHandle_t s_log_queue;
static int s_clients[LOG_SERVER_MAX_CLIENTS];
static int s_client_count;
static vprintf_like_t s_orig_vprintf;

static int esp_level_to_syslog_severity(char level_char)
{
    switch (level_char) {
    case 'E': return 3; /* Error */
    case 'W': return 4; /* Warning */
    case 'I': return 6; /* Informational */
    case 'D': return 7; /* Debug */
    case 'V': return 7; /* Debug */
    default:  return 6;
    }
}

static int log_vprintf_hook(const char *fmt, va_list args)
{
    /* Always forward to the original output first. */
    int ret = s_orig_vprintf(fmt, args);

    if (s_log_queue == NULL) {
        return ret;
    }

    /* Format the raw log text. */
    char raw[LOG_SERVER_MSG_SIZE];
    int raw_len = vsnprintf(raw, sizeof(raw), fmt, args);
    if (raw_len <= 0) {
        return ret;
    }
    if ((size_t)raw_len >= sizeof(raw)) {
        raw_len = sizeof(raw) - 1;
    }

    /* Strip trailing newline — syslog message should not include it. */
    while (raw_len > 0 && (raw[raw_len - 1] == '\n' || raw[raw_len - 1] == '\r')) {
        raw[--raw_len] = '\0';
    }
    if (raw_len == 0) {
        return ret;
    }

    /* Determine severity from the ESP log level character. */
    int severity = esp_level_to_syslog_severity(raw[0]);
    int priority = SYSLOG_FACILITY_USER * 8 + severity;

    /* Build an RFC 5424 timestamp from uptime (no RTC). */
    int64_t us = esp_timer_get_time();
    int secs = (int)(us / 1000000);
    int frac = (int)((us % 1000000) / 1000);
    int hh = (secs / 3600) % 24;
    int mm = (secs / 60) % 60;
    int ss = secs % 60;

    /* Build RFC 5424 syslog message. */
    log_msg_t msg;
    int syslog_len = snprintf(msg.data, sizeof(msg.data),
        "<%d>" SYSLOG_VERSION " 2000-01-01T%02d:%02d:%02d.%03dZ "
        SYSLOG_HOSTNAME " " SYSLOG_APPNAME " " SYSLOG_NILVALUE " "
        SYSLOG_NILVALUE " " SYSLOG_NILVALUE " %s\n",
        priority, hh, mm, ss, frac, raw);

    if (syslog_len <= 0 || (size_t)syslog_len >= sizeof(msg.data)) {
        return ret;
    }

    /* RFC 6587 octet-counting: prepend the length. We build the final
       framed message by shifting the syslog text and prepending "LEN ". */
    char length_prefix[12];
    int prefix_len = snprintf(length_prefix, sizeof(length_prefix), "%d ", syslog_len);

    if ((size_t)(prefix_len + syslog_len) >= sizeof(msg.data)) {
        return ret;
    }

    memmove(msg.data + prefix_len, msg.data, syslog_len + 1);
    memcpy(msg.data, length_prefix, prefix_len);
    msg.len = (uint16_t)(prefix_len + syslog_len);

    /* Non-blocking enqueue — drop the message if the queue is full. */
    xQueueSend(s_log_queue, &msg, 0);

    return ret;
}

static void remove_client(int index)
{
    close(s_clients[index]);
    s_clients[index] = s_clients[--s_client_count];
}

static void log_server_task(void *arg)
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
    addr.sin_port = htons(LOG_SERVER_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_fd, 2) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Log server listening on TCP %d", LOG_SERVER_PORT);

    /* Make the listen socket non-blocking so we can interleave accept()
       with queue reads. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 }; /* 50 ms */
    setsockopt(listen_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (true) {
        /* Try to accept a new client (non-blocking via timeout). */
        if (s_client_count < LOG_SERVER_MAX_CLIENTS) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
            if (fd >= 0) {
                s_clients[s_client_count++] = fd;
                ESP_LOGI(TAG, "Log client connected (%d total)", s_client_count);
            }
        }

        /* Drain the queue and send to all connected clients. */
        log_msg_t msg;
        while (xQueueReceive(s_log_queue, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = s_client_count - 1; i >= 0; i--) {
                int sent = send(s_clients[i], msg.data, msg.len, MSG_NOSIGNAL);
                if (sent < 0) {
                    ESP_LOGW(TAG, "Log client disconnected");
                    remove_client(i);
                }
            }
        }
    }
}

esp_err_t log_server_start(void)
{
    s_client_count = 0;

    s_log_queue = xQueueCreate(LOG_SERVER_QUEUE_DEPTH, sizeof(log_msg_t));
    if (s_log_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Hook into ESP logging. */
    s_orig_vprintf = esp_log_set_vprintf(log_vprintf_hook);

    if (xTaskCreate(log_server_task,
                    "log_server",
                    LOG_SERVER_TASK_STACK,
                    NULL,
                    LOG_SERVER_TASK_PRIORITY,
                    NULL) != pdPASS) {
        esp_log_set_vprintf(s_orig_vprintf);
        vQueueDelete(s_log_queue);
        s_log_queue = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}
