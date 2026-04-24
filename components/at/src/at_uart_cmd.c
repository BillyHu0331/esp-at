/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"

#ifdef CONFIG_AT_UART_COMMAND_SUPPORT
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "freertos/FreeRTOS.h"
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

#define HTTPP_PREFIX                             "+HTTPP"
#define HTTPP_PREFIX_LEN                         6
#define HTTPP_SOCKET_RECV_SIZE                   512
#define HTTPP_UART_WRITE_CHUNK_SIZE              64
#define HTTPP_UART_WAIT_MARGIN_MS                50
#define HTTPP_UART_WAIT_TIMEOUT_MAX_MS           15000

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

static uint8_t at_query_cmd_httpp(uint8_t *cmd_name)
{
    return ESP_AT_RESULT_CODE_OK;
}

static int32_t httpp_get_uart_wait_timeout_ms(size_t len, uint32_t baudrate)
{
    if (baudrate == 0) {
        baudrate = AT_UART_BAUD_RATE_DEF;
    }

    uint64_t timeout_ms = ((uint64_t)len * 12 * 1000 + baudrate - 1) / baudrate;
    timeout_ms += HTTPP_UART_WAIT_MARGIN_MS;

    if (timeout_ms < HTTPP_UART_WAIT_MARGIN_MS) {
        timeout_ms = HTTPP_UART_WAIT_MARGIN_MS;
    }
    if (timeout_ms > HTTPP_UART_WAIT_TIMEOUT_MAX_MS) {
        timeout_ms = HTTPP_UART_WAIT_TIMEOUT_MAX_MS;
    }

    return (int32_t)timeout_ms;
}

static bool httpp_uart_write_all(const uint8_t *data, size_t len)
{
    size_t offset = 0;
    uint32_t baudrate = AT_UART_BAUD_RATE_DEF;

    uart_get_baudrate(at_uart_port_get(), &baudrate);

    while (offset < len) {
        size_t chunk_len = len - offset;
        if (chunk_len > HTTPP_UART_WRITE_CHUNK_SIZE) {
            chunk_len = HTTPP_UART_WRITE_CHUNK_SIZE;
        }

        int32_t written = esp_at_port_write_data((uint8_t *)data + offset, chunk_len);
        if (written <= 0) {
            ESP_LOGE(TAG, "uart write failed, offset=%u len=%u written=%d",
                     (unsigned int)offset, (unsigned int)chunk_len, (int)written);
            return false;
        }

        offset += (size_t)written;

        if (!esp_at_port_wait_write_complete(httpp_get_uart_wait_timeout_ms((size_t)written, baudrate))) {
            ESP_LOGE(TAG, "uart wait timeout, written=%d", (int)written);
            return false;
        }
    }

    return true;
}

static bool http_get_param(uint8_t *web_host, uint8_t *web_port, uint8_t *web_path)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;
    struct in_addr  *addr;
    int s = -1, r;

    // 擴大 Request Buffer，防止長 URL 導致棧溢出
    char request[256] = {0}; 
    int req_len = snprintf(request, sizeof(request), "GET %s HTTP/1.0\r\nHost: %s:%s\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n", web_path, web_host, web_port);
    if ((req_len < 0) || (req_len >= sizeof(request))) {
        ESP_LOGE(TAG, "request too long");
        return false;
    }
    size_t request_len = (size_t)req_len;

    int err = getaddrinfo((char*)web_host, (char*)web_port, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGI(TAG, "DNS lookup failed err=%d res=%p", err, res);
        return false;
    }
    addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGI(TAG, "DNS lookup successed. IP=%s", inet_ntoa(*addr));
   
    s = socket(res->ai_family, res->ai_socktype, 0);
    if (s < 0) {
        ESP_LOGE(TAG, "...Failed to allocate socket.");
        goto cleanup;
    }
    
    if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "...socket connect failed error=%d", errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "...connected");
    freeaddrinfo(res);
    res = NULL;

    size_t sent_len = 0;
    while (sent_len < request_len) {
        int write_len = write(s, request + sent_len, request_len - sent_len);
        if (write_len <= 0) {
            ESP_LOGE(TAG, "...socket send failed");
            goto cleanup;
        }
        sent_len += (size_t)write_len;
    }
    ESP_LOGI(TAG, "...socket send success");
    
    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
        ESP_LOGE(TAG, "...failed to set socket receiving timeout");
        goto cleanup;
    }

    // HTTP 解析狀態機配置
    char recv_buf[HTTPP_SOCKET_RECV_SIZE + HTTPP_PREFIX_LEN] = {0};
    char *data_buf = recv_buf + HTTPP_PREFIX_LEN; // 預留前綴空間，保持 +HTTPP 與原始資料連續
    bool header_done = false;
    uint8_t match_state = 0;
    bool success = true;

    // 循環讀取 Socket，直到對端關閉連接或超時
    while ((r = read(s, data_buf, HTTPP_SOCKET_RECV_SIZE)) > 0) {
        if (!header_done) {
            int i;
            for (i = 0; i < r; i++) {
                // 狀態機：精準捕獲 \r\n\r\n 邊界，完美解決跨包截斷問題
                if (data_buf[i] == '\r' && match_state == 0) match_state = 1;
                else if (data_buf[i] == '\n' && match_state == 1) match_state = 2;
                else if (data_buf[i] == '\r' && match_state == 2) match_state = 3;
                else if (data_buf[i] == '\n' && match_state == 3) {
                    match_state = 4;
                    header_done = true;
                    i++; // 將指針移向 Body 的首字節
                    break;
                } else {
                    if (data_buf[i] == '\r') match_state = 1; // 容錯處理 \r\r\n
                    else match_state = 0;
                }
            }

            if (header_done) {
                // 1. 輸出最後一部分的 Header
                if (i > 0) {
                    if (!httpp_uart_write_all((uint8_t *)data_buf, i)) {
                        success = false;
                        goto cleanup;
                    }
                }
                // 2. 插入分隔標記
                if (!httpp_uart_write_all((const uint8_t *)"\r\nOK\r\n", 6)) {
                    success = false;
                    goto cleanup;
                }

                // 3. 如果這個封包已經包含了部分 Body 數據，將其拋出
                if (i < r) {
                    int body_len = r - i;
                    char *body_with_prefix = recv_buf + i;
                    memcpy(body_with_prefix, HTTPP_PREFIX, HTTPP_PREFIX_LEN);
                    if (!httpp_uart_write_all((uint8_t *)body_with_prefix, HTTPP_PREFIX_LEN + body_len)) {
                        success = false;
                        goto cleanup;
                    }
                }
            } else {
                // 整個封包都還是 Header
                if (!httpp_uart_write_all((uint8_t *)data_buf, r)) {
                    success = false;
                    goto cleanup;
                }
            }
            
        } else {
            // Header 已處理完畢，純 Body 透傳
            // 直接在預留的內存前綴寫入 "+HTTPP"，保持原協議格式不變
            memcpy(recv_buf, HTTPP_PREFIX, HTTPP_PREFIX_LEN);
            if (!httpp_uart_write_all((uint8_t *)recv_buf, r + HTTPP_PREFIX_LEN)) {
                success = false;
                goto cleanup;
            }
        }
    }

    ESP_LOGI(TAG, "...done reading from socket. Last read return = %d errno = %d.", r, errno);
    if (r < 0) {
        ESP_LOGE(TAG, "...socket recv failed/timeout error=%d", errno);
        success = false;
    }

cleanup:
    if (res != NULL) {
        freeaddrinfo(res);
    }
    if (s >= 0) {
        close(s);
    }

    return success;
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

    if (!http_get_param((uint8_t *)web_host, (uint8_t *)web_port, (uint8_t *)web_path)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

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
