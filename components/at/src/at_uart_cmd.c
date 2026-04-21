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

static uint8_t at_query_cmd_master(uint8_t *cmd_name)
{
    return ESP_AT_RESULT_CODE_OK;
}

static void http_get_param(uint8_t *web_host, uint8_t *web_port, uint8_t *web_path)
{
    const struct addrinfo hints = {
    .ai_family = AF_INET,
    .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;
    struct in_addr  *addr;
    int s = -1, r, rlen;
    char recv_buf[512 + 6] = {0};
    char request[256] = {0};
    char *p = NULL;
    size_t request_len = 0;

    int req_len = snprintf(request, sizeof(request), "GET %s HTTP/1.0\r\nHost: %s:%s\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n", web_path, web_host, web_port);
    if ((req_len < 0) || (req_len >= sizeof(request))) {
        ESP_LOGE(TAG, "request too long");
        return;
    }
    request_len = (size_t)req_len;
    //ESP_LOGI(TAG, "%s", request);
    //ESP_LOGI(TAG, "%s", REQUEST);
    
    int err = getaddrinfo((char*)web_host, (char*)web_port, &hints, &res);
    if (err != 0 || res == NULL) {
    ESP_LOGI(TAG, "DNS lookup failed err=%d res=%p", err, res);
    return;
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

    {
        size_t sent_len = 0;

        while (sent_len < request_len) {
            int write_len = write(s, request + sent_len, request_len - sent_len);
            if (write_len <= 0) {
                ESP_LOGE(TAG, "...socket send failed");
                goto cleanup;
            }
            sent_len += (size_t)write_len;
        }
    }
    ESP_LOGI(TAG, "...socket send success");
    
    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout, sizeof(receiving_timeout)) < 0) {
    ESP_LOGE(TAG, "...failed to set socket receiving timeout");
    goto cleanup;
    }
    ESP_LOGI(TAG, "...set socket receiving timeout success");

    //parse first packet data
    rlen = 0;
    r = read(s, recv_buf, 512);
    if (r <= 0) {
        ESP_LOGE(TAG, "...socket read failed error=%d", errno);
        goto cleanup;
    }
    p = strstr(recv_buf, "\r\n\r\n");
    if (p != NULL)
    {
    rlen = r - (p + 4 - recv_buf);
    }
    if (rlen < 0)
    rlen = 0;
    if ((r - rlen) > 0) {
        esp_at_port_write_data((uint8_t*)recv_buf, r - rlen);
    }
    esp_at_port_write_data((uint8_t *)"\r\nOK\r\n", 6);
    vTaskDelay(50/portTICK_PERIOD_MS);
    
    //fill second packet data
    memcpy(recv_buf, "+HTTPP", 6);
    if (p != NULL && rlen > 0)
        memcpy(recv_buf + 6, p + 4, rlen); 
    r = read(s, recv_buf + 6 + rlen, sizeof(recv_buf) - 6 - rlen);
    if (r > 0) {
        esp_at_port_write_data((uint8_t*)recv_buf, r + 6 + rlen);
    }
    vTaskDelay(50/portTICK_PERIOD_MS);
    
    do {
    bzero(recv_buf, sizeof(recv_buf));
        memcpy(recv_buf, "+HTTPP", 6);
    r = read(s, recv_buf + 6, sizeof(recv_buf) - 6);
    if (r > 0)
        esp_at_port_write_data((uint8_t*)recv_buf, r + 6);
    vTaskDelay(50/portTICK_PERIOD_MS);
    }while(r > 0);

    ESP_LOGI(TAG, "...done reading from socket. Last read return = %d errno = %d.", r, errno);
cleanup:
    if (res != NULL) {
        freeaddrinfo(res);
    }

    if (s >= 0) {
        close(s);
    }

}

static uint8_t at_setup_cmd_master(uint8_t para_num)
{
    uint8_t *web_url;
    char web_path[128] = {0};
    char web_port[8] = {0};
    char web_host[128] = {0};
    uint8_t cnt = 0;
    char *p1, *p2;

    if (esp_at_get_para_as_str(cnt++, &web_url) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if ((web_url == NULL) || (strchr((char *)web_url, '\r') != NULL) || (strchr((char *)web_url, '\n') != NULL)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    p1 = strstr((char *)web_url, "//");
    if (NULL != p1)
    {
        p2 = strstr((p1 + 2), "/");
        if (NULL != p2){
            size_t host_len = p2 - (p1 + 2);
            if ((host_len == 0) || (host_len >= sizeof(web_host))) {
                return ESP_AT_RESULT_CODE_ERROR;
            }
            memcpy(web_host, p1 + 2, host_len);
            web_host[host_len] = '\0';
        } else {
            return ESP_AT_RESULT_CODE_ERROR;
        }

        size_t path_len = strlen(p2);
        if ((path_len == 0) || (path_len >= sizeof(web_path))) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
        memcpy(web_path, p2, path_len);
        web_path[path_len] = '\0';

        snprintf(web_port, sizeof(web_port), "%s", "80");

    } else {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    http_get_param((uint8_t *)web_host, (uint8_t *)web_port, (uint8_t *)web_path);

    return ESP_AT_RESULT_CODE_OK;
}

static const esp_at_cmd_struct at_uart_cmd[] = {
    {"+UART", NULL, at_query_cmd_uart, at_setup_cmd_uart_def, NULL},
    {"+UART_CUR", NULL, at_query_cmd_uart, at_setup_cmd_uart_cur, NULL},
    {"+UART_DEF", NULL, at_query_cmd_uart_def, at_setup_cmd_uart_def, NULL},
    {"+HTPPP", NULL, at_query_cmd_master, at_setup_cmd_master, NULL},
};

bool esp_at_uart_cmd_regist(void)
{
    return esp_at_custom_cmd_array_regist(at_uart_cmd, sizeof(at_uart_cmd) / sizeof(esp_at_cmd_struct));
}

ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_uart_cmd_regist, 23);

#endif
