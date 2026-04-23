/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "stdlib.h"

#ifdef CONFIG_AT_UART_COMMAND_SUPPORT
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_at.h"
#include "at_uart.h"

#include "esp_bt.h"
#include "esp_hf_client_api.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

static const char *TAG = "HTTP-AT";

#define RINGBUF_SIZE   (4 * 1024)
#define RECV_CHUNK     1024
#define UART_CHUNK     512
static RingbufHandle_t s_rb = NULL;
static TaskHandle_t s_uart_task = NULL;
static bool s_http_busy = false;  // 并发控制标志


static void uart_tx_task(void *arg)
{
    while (1) {
        size_t item_size;
        uint8_t *data = (uint8_t *)xRingbufferReceive(
            s_rb, &item_size, portMAX_DELAY);

        if (data) {
            int offset = 0;

            while (offset < item_size) {
                int chunk = (item_size - offset > UART_CHUNK) ?
                             UART_CHUNK : (item_size - offset);

                esp_at_port_write_data((uint8_t*)"+HTTPP", 6);
                esp_at_port_write_data(data + offset, chunk);

                offset += chunk;

                // 关键：让出调度，避免卡 AT
                taskYIELD();
            }

            vRingbufferReturnItem(s_rb, data);
        }
    }
}
static void at_http_init(void)
{
    if (!s_rb) {
        s_rb = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
        assert(s_rb);
    }

    if (!s_uart_task) {
        xTaskCreate(uart_tx_task, "uart_tx", 4096, NULL, 4, &s_uart_task);
    }
}
static int http_connect(const char *host, const char *port)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };

    struct addrinfo *res;

    if (getaddrinfo(host, port, &hints, &res) != 0 || !res) {
        ESP_LOGE(TAG, "DNS fail");
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        return -1;
    }

    struct timeval tv = {
        .tv_sec = 10,
        .tv_usec = 0
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "connect fail");
        close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    return sock;
}

// static void rb_send_copy(const uint8_t *data, int len)
// {
//     if (len <= 0 || !s_rb) return;
//     xRingbufferSend(s_rb, data, len, pdMS_TO_TICKS(1000));
// }

static void http_recv_task(void *arg)
{
    int sock = (int)(intptr_t)arg;

    char *header_buf = calloc(1, (RECV_CHUNK));
    if (!header_buf) goto cleanup;
    int header_len = 0;
    bool header_done = false;

    int content_length = -1;
    int received = 0;

    while (1) {

        if (!header_done) {
            int remain = RECV_CHUNK - header_len - 1;
            if (remain <= 0){
                ESP_LOGE(TAG, "header overflow");
                break;
            }
            int r = read(sock,
                         header_buf + header_len,
                         remain);

            if (r <= 0) break;
            header_len += r;
            header_buf[header_len] = '\0';

            char *end = strstr(header_buf, "\r\n\r\n");

            if (end) {
                header_done = true;

                // ===== 首包 body =====
                char *body = end + 4;
                int body_len = header_len - (body - header_buf);

                // ===== 输出 Header =====
                esp_at_port_write_data((uint8_t*)header_buf, (body - header_buf));
                esp_at_port_write_data((uint8_t*)"\r\nOK\r\n", 6);
                vTaskDelay(50/portTICK_PERIOD_MS);

                // ===== Content-Length =====
                char *cl = strcasestr(header_buf, "Content-Length:");
                if (cl) {
                    sscanf(cl, "Content-Length: %d", &content_length);
                }

                if (strcasestr(header_buf, "Transfer-Encoding: chunked")) {
                    ESP_LOGE(TAG, "chunked not supported");
                }


                if (body_len > 0) {
                    if (xRingbufferSend(s_rb, (uint8_t*)body, body_len, pdMS_TO_TICKS(1000)) != pdTRUE) {
                        ESP_LOGE(TAG, "RB send fail");
                        break;
                    }
                    received += body_len;
                }
            }

        } else {

            // memcpy(header_buf, "+HTTPP", 6);
            int r = read(sock, header_buf, RECV_CHUNK); //复用header_buf 用于body传输，不建议这样做
            if (r <= 0) break;
            if (xRingbufferSend(s_rb, header_buf, r, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGE(TAG, "RB send fail");
                break;
            }

            received += r;

            // xRingbufferSendComplete(s_rb, ptr);

            if (content_length > 0 && received >= content_length) {
                ESP_LOGI(TAG, "recv done");
                break;
            }
        }
    }

cleanup:
    if (header_buf) {
        free(header_buf);
        header_buf = NULL;
    }
    close(sock);
    s_http_busy = false; // 释放锁
    vTaskDelete(NULL);
}

static void http_send_request(int sock,
                             const char *host,
                             const char *path)
{
    char req[256];

    int len = snprintf(req, sizeof(req),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "User-Agent: esp-at\r\n"
        "Connection: close\r\n\r\n",
        path, host);

    write(sock, req, len);
}

void at_http_get_start(const char *host,
                       const char *port,
                       const char *path)
{
    if (s_http_busy) {
        esp_at_port_write_data((uint8_t*)"BUSY\r\n", 6);
        return;
    }
    s_http_busy = true;

    int sock = http_connect(host, port);
    if (sock < 0) {
        esp_at_port_write_data((uint8_t*)"ERROR\r\n", 7);
        s_http_busy = false;
        return;
    }

    http_send_request(sock, host, path);

    xTaskCreate(http_recv_task,
                "http_recv",
                4096,
                (void*)sock,
                5,
                NULL);
}


static uint8_t at_query_cmd_httpp(uint8_t *cmd_name)
{
    return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setup_cmd_httpp(uint8_t para_num)
{
    uint8_t *web_url;
    char web_path[256] = {0}; // 擴大 Buffer 避免長路徑溢出
    char web_port[8] = "80";  // 預設 Port 為 80
    char web_host[256] = {0};
    uint8_t cnt = 0;
    char *p_host_start;
    char *p_path_start;

    at_http_init();

    if (esp_at_get_para_as_str(cnt++, &web_url) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if ((web_url == NULL) || (strchr((char *)web_url, '\r') != NULL) || (strchr((char *)web_url, '\n') != NULL)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    // 1. 過濾掉 "://" 前綴 (如果存在)
    p_host_start = strstr((char *)web_url, "://");
    if (p_host_start) {
        p_host_start += 3;
    } else {
        p_host_start = (char *)web_url; 
    }

    // 2. 尋找 Path 的起點 '/'
    p_path_start = strchr(p_host_start, '/');

    size_t host_len;
    if (p_path_start) {
        host_len = p_path_start - p_host_start;
        size_t path_len = strlen(p_path_start);
        if (path_len >= sizeof(web_path)) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
        strncpy(web_path, p_path_start, path_len);
        web_path[path_len] = '\0';
    } else {
        // 沒有 '/' 的情況 (如 http://example.com)
        host_len = strlen(p_host_start);
        strcpy(web_path, "/"); 
    }

    if (host_len == 0 || host_len >= sizeof(web_host)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    strncpy(web_host, p_host_start, host_len);
    web_host[host_len] = '\0';

    // 3. 檢查是否包含自定義 Port (例如 host:8080)
    char *p_port = strchr(web_host, ':');
    if (p_port) {
        *p_port = '\0'; // 截斷 Host 字串
        strncpy(web_port, p_port + 1, sizeof(web_port) - 1);
    }

    at_http_get_start(web_host, web_port, web_path);

    return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setup_cmd_uart_common(uint8_t para_num, bool save_to_flash)
{
    int32_t value = 0;
    int32_t cnt = 0;

    at_uart_config_t config;
    memset(&config, 0x0, sizeof(config));

    // baudrate
    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if (value < AT_UART_BAUD_RATE_MIN || value > AT_UART_BAUD_RATE_MAX) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    config.baudrate = value;

    // data_bits
    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    value -= 5;
    if (value < UART_DATA_5_BITS || value > UART_DATA_8_BITS) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    config.data_bits = value;

    // stop_bits
    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if (value < UART_STOP_BITS_1 || value > UART_STOP_BITS_2) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    config.stop_bits = value;

    // parity
    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if (value < 0 || value > 2) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    config.parity = at_uart_parity_get(value);

    // flow_control
    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if (value < UART_HW_FLOWCTRL_DISABLE || value > UART_HW_FLOWCTRL_CTS_RTS) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    config.flow_control = value;

    if (para_num != cnt) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    // save to flash
    if (save_to_flash) {
        if (!at_nvs_uart_config_set_internal(&config)) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    esp_at_response_result(ESP_AT_RESULT_CODE_OK);

    // set now
    uint8_t uart_port = at_uart_port_get();
    uart_wait_tx_done(uart_port, portMAX_DELAY);
    uart_set_baudrate(uart_port, config.baudrate);
    uart_set_word_length(uart_port, config.data_bits);
    uart_set_stop_bits(uart_port, config.stop_bits);
    uart_set_parity(uart_port, config.parity);
    uart_set_hw_flow_ctrl(uart_port, config.flow_control, 120);

    return ESP_AT_RESULT_CODE_PROCESS_DONE;
}

static uint8_t at_setup_cmd_uart_cur(uint8_t para_num)
{
    return at_setup_cmd_uart_common(para_num, false);
}

static uint8_t at_setup_cmd_uart_def(uint8_t para_num)
{
    return at_setup_cmd_uart_common(para_num, true);
}

static uint8_t at_query_cmd_uart(uint8_t *cmd_name)
{
    uint32_t baudrate = 0;
    uart_word_length_t data_bits = UART_DATA_8_BITS;
    uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
    uart_parity_t parity = UART_PARITY_DISABLE;
    uart_hw_flowcontrol_t flow_control = UART_HW_FLOWCTRL_DISABLE;

    uint8_t uart_port = at_uart_port_get();
    uart_get_baudrate(uart_port, &baudrate);
    uart_get_word_length(uart_port, &data_bits);
    uart_get_stop_bits(uart_port, &stop_bits);
    uart_get_parity(uart_port, &parity);
    uart_get_hw_flow_ctrl(uart_port, &flow_control);

    data_bits += 5;
    if (UART_PARITY_DISABLE == parity) {
        parity = 0;
    } else if (UART_PARITY_ODD == parity) {
        parity = 1;
    } else if (UART_PARITY_EVEN == parity) {
        parity = 2;
    } else {
        parity = 0xff;
    }

    uint8_t buffer[AT_BUFFER_ON_STACK_SIZE] = {0};
    snprintf((char *)buffer, AT_BUFFER_ON_STACK_SIZE, "%s:%d,%d,%d,%d,%d\r\n",
             cmd_name, baudrate, data_bits, stop_bits, parity, flow_control);
    esp_at_port_write_data(buffer, strlen((char *)buffer));

    return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_query_cmd_uart_def(uint8_t *cmd_name)
{
    at_uart_config_t config;
    memset(&config, 0x0, sizeof(config));
    at_nvs_uart_config_get_internal(&config);

    config.data_bits += 5;
    if (UART_PARITY_DISABLE == config.parity) {
        config.parity = 0;
    } else if (UART_PARITY_ODD == config.parity) {
        config.parity = 1;
    } else if (UART_PARITY_EVEN == config.parity) {
        config.parity = 2;
    } else {
        config.parity = 0xff;
    }

    uint8_t buffer[AT_BUFFER_ON_STACK_SIZE] = {0};
    snprintf((char *)buffer, AT_BUFFER_ON_STACK_SIZE, "%s:%d,%d,%d,%d,%d\r\n",
             cmd_name, config.baudrate, config.data_bits, config.stop_bits, config.parity, config.flow_control);
    esp_at_port_write_data(buffer, strlen((char *)buffer));

    return ESP_AT_RESULT_CODE_OK;
}

static const esp_at_cmd_struct at_uart_cmd[] = {
    {"+UART", NULL, at_query_cmd_uart, at_setup_cmd_uart_def, NULL},
    {"+UART_CUR", NULL, at_query_cmd_uart, at_setup_cmd_uart_cur, NULL},
    {"+UART_DEF", NULL, at_query_cmd_uart_def, at_setup_cmd_uart_def, NULL},
    {"+HTTPP", NULL, at_query_cmd_httpp, at_setup_cmd_httpp, NULL},
};

bool esp_at_uart_cmd_regist(void)
{
    return esp_at_custom_cmd_array_regist(at_uart_cmd, sizeof(at_uart_cmd) / sizeof(esp_at_cmd_struct));
}

ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_uart_cmd_regist, 23);

#endif
