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

// 引入 HTTP Client 相關頭文件
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

static const char *TAG = "HTTP-AT";
#define HTTPP_OUT_DELAY_MS  50

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

#define MAX_DATA_PER_CHUNK  512  // 純數據最大 512
#define PREFIX_STR          "+HTTPP"
#define PREFIX_LEN          6
#define HTTPP_OUT_DELAY_MS  50

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_HEADER:
            // Header 部分通常按原樣輸出，方便除錯
            if (evt->header_key && evt->header_value) {
                char head_buf[256];
                int len = snprintf(head_buf, sizeof(head_buf), "%s: %s\r\n", evt->header_key, evt->header_value);
                if (len > 0) {
                    esp_at_port_write_data((uint8_t *)head_buf, len);
                }
            }
            break;

        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) {
                int data_left = evt->data_len;
                uint8_t *data_ptr = (uint8_t *)evt->data;

                // 準備一個足以容納 前綴 + 數據 的緩衝區
                // 使用 static 或從堆分配以減輕棧負擔，這裡演示用棧（請確保任務棧 > 2KB）
                uint8_t send_buf[PREFIX_LEN + MAX_DATA_PER_CHUNK];

                while (data_left > 0) {
                    int copy_len = (data_left > MAX_DATA_PER_CHUNK) ? MAX_DATA_PER_CHUNK : data_left;

                    // 1. 組裝數據包：前綴 + 數據內容
                    memcpy(send_buf, PREFIX_STR, PREFIX_LEN);
                    memcpy(send_buf + PREFIX_LEN, data_ptr, copy_len);

                    // 2. 一次性發送整包 (Prefix + Data)
                    esp_at_port_write_data(send_buf, PREFIX_LEN + copy_len);

                    // 3. 強制延遲，保證接收端處理時間
                    vTaskDelay(pdMS_TO_TICKS(HTTPP_OUT_DELAY_MS));

                    // 指標偏移
                    data_ptr += copy_len;
                    data_left -= copy_len;
                }
            }
            break;

        default:
            break;
    }
    return ESP_OK;
}

static void http_perform_request(const char *url)
{
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
        .method = HTTP_METHOD_GET,
        // 設定 Buffer 為 512，讓底層儘可能以 512 為單位觸發事件
        .buffer_size = MAX_UART_CHUNK_SIZE, 
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client) {
        esp_http_client_perform(client);
        esp_http_client_cleanup(client);
    }
}

/**
 * @brief AT+HTTPP=<url> 指令處理函數
 */
static uint8_t at_setup_cmd_httpp(uint8_t para_num)
{
    uint8_t *url_ptr;
    
    // 從參數中提取 URL 字符串
    if (esp_at_get_para_as_str(0, &url_ptr) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (url_ptr == NULL || strlen((char *)url_ptr) == 0) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    // 呼叫重構後的請求邏輯，並根據結果回傳對應的 AT 狀態
    esp_err_t err = http_perform_request((char *)url_ptr);
    
    if (err == ESP_OK) {
        // 底層框架會自動在最後補上 \r\nOK\r\n
        return ESP_AT_RESULT_CODE_OK; 
    } else {
        // 底層框架會補上 \r\nERROR\r\n
        return ESP_AT_RESULT_CODE_ERROR; 
    }
}

static const esp_at_cmd_struct at_uart_cmd[] = {
    {"+UART", NULL, at_query_cmd_uart, at_setup_cmd_uart_def, NULL},
    {"+UART_CUR", NULL, at_query_cmd_uart, at_setup_cmd_uart_cur, NULL},
    {"+UART_DEF", NULL, at_query_cmd_uart_def, at_setup_cmd_uart_def, NULL},
    {"+HTTPP", NULL, NULL, at_setup_cmd_httpp, NULL},
};

bool esp_at_uart_cmd_regist(void)
{
    return esp_at_custom_cmd_array_regist(at_uart_cmd, sizeof(at_uart_cmd) / sizeof(esp_at_cmd_struct));
}

ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_uart_cmd_regist, 23);

#endif
