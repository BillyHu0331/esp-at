/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
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
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_transport.h"

static const char *TAG = "HTTP-AT";

#define HTTPP_PREFIX                             "+HTTPP"
#define HTTPP_PREFIX_LEN                         6
#define HTTPP_SOCKET_RECV_SIZE                   1024
#define HTTPP_UART_PACKET_GAP_MS                 5
#define HTTPP_BODY_PAYLOAD_CHUNK_SIZE            512
#define HTTPP_UART_WAIT_MARGIN_MS                10
#define HTTPP_UART_WAIT_TIMEOUT_MAX_MS           15000
#define HTTPP_REQUEST_TIMEOUT_MS                 30000
#define HTTPP_HEADER_BUF_MAX_SIZE                4096
#define HTTPP_RESUME_MAX_ATTEMPTS                5
#define HTTPP_KEEPALIVE_IDLE_SEC                 10
#define HTTPP_KEEPALIVE_INTERVAL_SEC             5
#define HTTPP_KEEPALIVE_COUNT                    3

#define HTTPP_DBG_LEVEL_NONE     0
#define HTTPP_DBG_LEVEL_ERROR   1
#define HTTPP_DBG_LEVEL_WARN    2
#define HTTPP_DBG_LEVEL_INFO    3
#define HTTPP_DBG_LEVEL_DEBUG   4

#define HTTPP_DBG_LEVEL          HTTPP_DBG_LEVEL_NONE

#define HTTPP_DBG_PRINTF(fmt, ...) do { \
        char _dbg_buf[256]; \
        int _dbg_len = snprintf(_dbg_buf, sizeof(_dbg_buf), "[HTTPP-DBG] " fmt "\r\n", ##__VA_ARGS__); \
        if (_dbg_len > 0) { \
            esp_at_port_write_data((uint8_t *)_dbg_buf, (size_t)_dbg_len); \
        } \
    } while (0)

#define HTTPP_DBG(level, fmt, ...) do { \
        if ((level) <= HTTPP_DBG_LEVEL) { \
            HTTPP_DBG_PRINTF(fmt, ##__VA_ARGS__); \
        } \
    } while (0)

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

static bool httpp_uart_write_packet(const uint8_t *data, size_t len)
{
    uint32_t baudrate = AT_UART_BAUD_RATE_DEF;

    uart_get_baudrate(at_uart_port_get(), &baudrate);

    int32_t written = esp_at_port_write_data((uint8_t *)data, len);
    if (written != (int32_t)len) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "uart write failed, len=%u written=%d", (unsigned int)len, (int)written);
        return false;
    }

    if (!esp_at_port_wait_write_complete(httpp_get_uart_wait_timeout_ms(len, baudrate))) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "uart wait timeout, len=%u", (unsigned int)len);
        return false;
    }

    return true;
}

static bool httpp_uart_write_body_packets(const uint8_t *data, size_t len, bool add_gap)
{
    uint8_t packet_buf[HTTPP_PREFIX_LEN + HTTPP_BODY_PAYLOAD_CHUNK_SIZE];
    size_t offset = 0;

    memcpy(packet_buf, HTTPP_PREFIX, HTTPP_PREFIX_LEN);

    while (offset < len) {
        size_t payload_len = len - offset;
        if (payload_len > HTTPP_BODY_PAYLOAD_CHUNK_SIZE) {
            payload_len = HTTPP_BODY_PAYLOAD_CHUNK_SIZE;
        }

        memcpy(packet_buf + HTTPP_PREFIX_LEN, data + offset, payload_len);
        if (!httpp_uart_write_packet(packet_buf, HTTPP_PREFIX_LEN + payload_len)) {
            return false;
        }

        offset += payload_len;
    }

    if (add_gap) {
        vTaskDelay(HTTPP_UART_PACKET_GAP_MS / portTICK_PERIOD_MS);
    }

    return true;
}

typedef struct {
    uint8_t *header_buf;
    size_t header_len;
    bool have_last_error;
    esp_err_t last_error;
    int last_tls_code;
    int last_tls_flags;
} httpp_http_ctx_t;

static const char *httpp_http_status_reason_phrase(int status_code)
{
    switch (status_code) {
    case 100: return "Continue";
    case 101: return "Switching Protocols";
    case 200: return "OK";
    case 201: return "Created";
    case 202: return "Accepted";
    case 204: return "No Content";
    case 206: return "Partial Content";
    case 300: return "Multiple Choices";
    case 301: return "Moved Permanently";
    case 302: return "Found";
    case 304: return "Not Modified";
    case 307: return "Temporary Redirect";
    case 308: return "Permanent Redirect";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 408: return "Request Timeout";
    case 409: return "Conflict";
    case 410: return "Gone";
    case 413: return "Payload Too Large";
    case 414: return "URI Too Long";
    case 415: return "Unsupported Media Type";
    case 429: return "Too Many Requests";
    case 500: return "Internal Server Error";
    case 501: return "Not Implemented";
    case 502: return "Bad Gateway";
    case 503: return "Service Unavailable";
    case 504: return "Gateway Timeout";
    default:  return "Unknown";
    }
}

static bool httpp_http_append_header(httpp_http_ctx_t *ctx, const char *data, size_t len)
{
    if (len == 0) {
        return true;
    }

    if (ctx->header_len >= HTTPP_HEADER_BUF_MAX_SIZE) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http header buffer already full, len=%u",
                 (unsigned int)ctx->header_len);
        return false;
    }

    if (len > (HTTPP_HEADER_BUF_MAX_SIZE - ctx->header_len)) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http header too large, current=%u append=%u",
                 (unsigned int)ctx->header_len, (unsigned int)len);
        return false;
    }

    uint8_t *new_buf = realloc(ctx->header_buf, ctx->header_len + len);
    if (new_buf == NULL) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "no mem for http header, header_len=%u append=%u",
                 (unsigned int)ctx->header_len, (unsigned int)len);
        return false;
    }

    ctx->header_buf = new_buf;
    memcpy(ctx->header_buf + ctx->header_len, data, len);
    ctx->header_len += len;
    return true;
}

static bool httpp_http_append_header_line(httpp_http_ctx_t *ctx, const char *key, const char *value)
{
    if (!httpp_http_append_header(ctx, key, strlen(key))) {
        return false;
    }
    if (!httpp_http_append_header(ctx, ": ", 2)) {
        return false;
    }
    if (!httpp_http_append_header(ctx, value, strlen(value))) {
        return false;
    }
    return httpp_http_append_header(ctx, "\r\n", 2);
}

static esp_err_t httpp_http_event_handler(esp_http_client_event_t *evt)
{
    httpp_http_ctx_t *ctx = (httpp_http_ctx_t *)evt->user_data;

    if (ctx == NULL) {
        return ESP_OK;
    }

    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        if (evt->data != NULL) {
            esp_tls_error_handle_t tls_err = (esp_tls_error_handle_t)evt->data;
            ctx->have_last_error = true;
            ctx->last_error = esp_tls_get_and_clear_last_error(tls_err, &ctx->last_tls_code, &ctx->last_tls_flags);
        }
        break;
    case HTTP_EVENT_ON_HEADER:
        if ((evt->header_key == NULL) || (evt->header_value == NULL)) {
            return ESP_OK;
        }

        if (!httpp_http_append_header_line(ctx, evt->header_key, evt->header_value)) {
            return ESP_FAIL;
        }
        break;
    default:
        break;
    }

    return ESP_OK;
}

static bool httpp_http_write_header_packets(esp_http_client_handle_t client, httpp_http_ctx_t *ctx)
{
    const char *reason = httpp_http_status_reason_phrase(esp_http_client_get_status_code(client));
    char status_line[64];
    int status_line_len = snprintf(status_line, sizeof(status_line), "HTTP/1.1 %d %s\r\n",
                                   esp_http_client_get_status_code(client), reason);
    size_t total_len;
    uint8_t *packet_buf;
    bool success = false;

    if ((status_line_len < 0) || (status_line_len >= sizeof(status_line))) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http status line too long");
        return false;
    }

    total_len = (size_t)status_line_len + ctx->header_len + 4;
    packet_buf = malloc(total_len);
    if (packet_buf == NULL) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "no mem for http response header, len=%u", (unsigned int)total_len);
        return false;
    }

    memcpy(packet_buf, status_line, status_line_len);
    if (ctx->header_len > 0) {
        memcpy(packet_buf + status_line_len, ctx->header_buf, ctx->header_len);
    }
    memcpy(packet_buf + status_line_len + ctx->header_len, "\r\n\r\n", 4);

    success = httpp_uart_write_packet(packet_buf, total_len);
    free(packet_buf);

    if (!success) {
        return false;
    }

    return httpp_uart_write_packet((const uint8_t *)"\r\nOK\r\n", 6);
}

static bool httpp_http_download_and_forward_body(esp_http_client_handle_t client, long long *out_read)
{
    char recv_buf[HTTPP_SOCKET_RECV_SIZE] = {0};
    int read_len = 0;
    long long total_read = 0;
    bool is_chunked = esp_http_client_is_chunked_response(client);
    long long content_len = (long long)esp_http_client_get_content_length(client);
    int retry_eagain_count = 0;
    const int retry_eagain_max = 100;

    while (1) {
        read_len = esp_http_client_read(client, recv_buf, sizeof(recv_buf));
        if (read_len > 0) {
            total_read += read_len;
            if (!httpp_uart_write_body_packets((const uint8_t *)recv_buf, (size_t)read_len, true)) {
                goto failed;
            }
            continue;
        }

        if (read_len == 0) {
            if (is_chunked) {
                break;
            }
            if ((content_len >= 0) && ((long long)total_read >= content_len)) {
                break;
            }
            if (content_len < 0) {
                break;
            }
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "server closed connection early, read=%lld expected=%lld",
                     total_read, (long long)content_len);
            goto failed;
        }

        if (read_len == -ESP_ERR_HTTP_EAGAIN) {
            if ((content_len >= 0) && ((long long)total_read >= content_len)) {
                break;
            }
            retry_eagain_count++;
            if (retry_eagain_count >= retry_eagain_max) {
                HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http read EAGAIN exhausted, max=%d retries reached, recv=%lld expected=%lld",
                         retry_eagain_max, total_read, (long long)content_len);
                goto failed;
            }
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "http read EAGAIN (recoverable), retry=%d/%d recv=%lld expected=%lld, retrying...",
                     retry_eagain_count, retry_eagain_max, total_read, (long long)content_len);
            continue;
        }

        if (read_len == -ESP_ERR_HTTP_CONNECTION_CLOSED) {
            if ((content_len >= 0) && ((long long)total_read >= content_len)) {
                break;
            }
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "http connection closed, recv=%lld expected=%lld",
                     total_read, (long long)content_len);
            goto failed;
        }

        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http fatal read error, err=%d errno=%d(%s) recv=%lld expected=%lld",
                 read_len, errno, strerror(errno), total_read, (long long)content_len);
        goto failed;
    }

    if ((!is_chunked) && (content_len >= 0) && ((long long)total_read < content_len)) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http body incomplete, read=%lld expected=%lld",
                 total_read, (long long)content_len);
        goto failed;
    }

    if (out_read != NULL) {
        *out_read = total_read;
    }
    return true;

failed:
    if (out_read != NULL) {
        *out_read = total_read;
    }
    return false;
}

static bool http_get_param(uint8_t *web_host, uint8_t *web_port, uint8_t *web_path, bool use_https)
{
    char request_url[528] = {0};
    const char *scheme = use_https ? "https" : "http";
    int url_len = snprintf(request_url, sizeof(request_url), "%s://%s:%s%s", scheme, web_host, web_port, web_path);
    long long downloaded_total = 0;
    long long expected_total = -1;
    bool header_written = false;
    int resume_attempts = 0;

    if ((url_len < 0) || (url_len >= sizeof(request_url))) {
        HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "request url too long");
        return false;
    }

    while (1) {
        httpp_http_ctx_t http_ctx = {0};
        esp_http_client_handle_t client = NULL;
        bool is_open = false;
        long long one_shot_read = 0;
        char range_header[64] = {0};
        esp_http_client_config_t config = {
            .url = request_url,
            .transport_type = use_https ? HTTP_TRANSPORT_OVER_SSL : HTTP_TRANSPORT_OVER_TCP,
            .event_handler = httpp_http_event_handler,
            .user_data = &http_ctx,
            .timeout_ms = HTTPP_REQUEST_TIMEOUT_MS,
            .buffer_size = HTTPP_SOCKET_RECV_SIZE,
            .max_redirection_count = 10,
            .keep_alive_enable = true,
            .keep_alive_idle = HTTPP_KEEPALIVE_IDLE_SEC,
            .keep_alive_interval = HTTPP_KEEPALIVE_INTERVAL_SEC,
            .keep_alive_count = HTTPP_KEEPALIVE_COUNT,
            .skip_cert_common_name_check = true,
        };

        client = esp_http_client_init(&config);
        if (client == NULL) {
            HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "http client init failed");
            return false;
        }

        esp_http_client_set_method(client, HTTP_METHOD_GET);
        esp_http_client_set_header(client, "User-Agent", "Mozilla/5.0 (compatible; esp32)");
        esp_http_client_set_header(client, "Accept", "*/*");
        if (downloaded_total > 0) {
            int range_len = snprintf(range_header, sizeof(range_header), "bytes=%lld-", downloaded_total);
            if ((range_len < 0) || (range_len >= sizeof(range_header))) {
                HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "range header too long, offset=%lld", downloaded_total);
                esp_http_client_cleanup(client);
                return false;
            }
            esp_http_client_set_header(client, "Range", range_header);
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "resume download from offset=%lld", downloaded_total);
        }

        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "Failed to open HTTP connection: err=0x%x(%s)", err, esp_err_to_name(err));
            if (http_ctx.have_last_error) {
                HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "  underlying TLS error: esp_err=0x%x tls_code=0x%x tls_flags=0x%x",
                         http_ctx.last_error, http_ctx.last_tls_code, http_ctx.last_tls_flags);
            }
            free(http_ctx.header_buf);
            esp_http_client_cleanup(client);
            return false;
        }
        is_open = true;

        int64_t fetch_ret = esp_http_client_fetch_headers(client);
        if ((fetch_ret < 0) && !esp_http_client_is_chunked_response(client)) {
            HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "HTTP client fetch headers failed, fetch_ret=%lld is_chunked=%d",
                     fetch_ret, esp_http_client_is_chunked_response(client));
            free(http_ctx.header_buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }

        int status_code = esp_http_client_get_status_code(client);
        if (status_code < 200 || status_code >= 300) {
            HTTPP_DBG(HTTPP_DBG_LEVEL_ERROR, "HTTP server returned error status, status=%d offset=%lld",
                     status_code, downloaded_total);
            if (downloaded_total == 0) {
                free(http_ctx.header_buf);
                if (client != NULL) {
                    if (is_open) {
                        esp_http_client_close(client);
                    }
                    esp_http_client_cleanup(client);
                }
                return false;
            }
            if (status_code != 206) {
                free(http_ctx.header_buf);
                if (client != NULL) {
                    if (is_open) {
                        esp_http_client_close(client);
                    }
                    esp_http_client_cleanup(client);
                }
                return false;
            }
        }

        if (downloaded_total > 0 && status_code == 200) {
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "server ignored Range header, resetting download (status=%d)", status_code);
            downloaded_total = 0;
            resume_attempts = 0;
            expected_total = -1;
            header_written = false;
        }

        if (!header_written) {
            if (!httpp_http_write_header_packets(client, &http_ctx)) {
                free(http_ctx.header_buf);
                if (client != NULL) {
                    if (is_open) {
                        esp_http_client_close(client);
                    }
                    esp_http_client_cleanup(client);
                }
                return false;
            }
            header_written = true;

            if (!esp_http_client_is_chunked_response(client)) {
                expected_total = (long long)esp_http_client_get_content_length(client);
                HTTPP_DBG(HTTPP_DBG_LEVEL_INFO, "download expected total=%lld", expected_total);
            }
        }

        if (!httpp_http_download_and_forward_body(client, &one_shot_read)) {
            downloaded_total += one_shot_read;

            if ((expected_total < 0) || (downloaded_total <= 0) || (resume_attempts >= HTTPP_RESUME_MAX_ATTEMPTS)) {
                free(http_ctx.header_buf);
                if (client != NULL) {
                    if (is_open) {
                        esp_http_client_close(client);
                    }
                    esp_http_client_cleanup(client);
                }
                return false;
            }

            resume_attempts++;
            HTTPP_DBG(HTTPP_DBG_LEVEL_WARN, "resume attempt %d/%d, downloaded=%lld/%lld",
                     resume_attempts, HTTPP_RESUME_MAX_ATTEMPTS,
                     downloaded_total, expected_total);
        } else {
            downloaded_total += one_shot_read;
            resume_attempts = 0;
        }

        free(http_ctx.header_buf);
        if (client != NULL) {
            if (is_open) {
                esp_http_client_close(client);
            }
            esp_http_client_cleanup(client);
        }

        if ((expected_total < 0) || (downloaded_total >= expected_total)) {
            return true;
        }
    }
}

static uint8_t at_setup_cmd_httpp(uint8_t para_num)
{
    uint8_t *web_url;
    char web_path[256] = {0}; // 擴大 Buffer 避免長路徑溢出
    char web_port[8] = "80";  // 預設 Port 為 80
    char web_host[256] = {0};
    bool use_https = false;
    uint8_t cnt = 0;
    char *p_host_start;
    char *p_path_start;

    if (esp_at_get_para_as_str(cnt++, &web_url) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if ((web_url == NULL) || (strchr((char *)web_url, '\r') != NULL) || (strchr((char *)web_url, '\n') != NULL)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    // 1. 解析 URL 協議與 Host 起點
    if (strncmp((char *)web_url, "https://", 8) == 0) {
        use_https = true;
        strncpy(web_port, "443", sizeof(web_port) - 1);
        web_port[sizeof(web_port) - 1] = '\0';
        p_host_start = (char *)web_url + 8;
    } else if (strncmp((char *)web_url, "http://", 7) == 0) {
        p_host_start = (char *)web_url + 7;
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
        web_port[sizeof(web_port) - 1] = '\0';
    }

    if (!http_get_param((uint8_t *)web_host, (uint8_t *)web_port, (uint8_t *)web_path, use_https)) {
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
