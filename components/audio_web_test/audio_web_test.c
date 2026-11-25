/**
 * @file audio_web_test.c
 * @brief SoftAP 网页服务器和单通道实时 PCM WebSocket 推流实现。
 */

#include "audio_web_test.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/** 链接器为嵌入的网页文件生成起止符号，结束地址不属于文件内容。 */
extern const uint8_t audio_web_index_start[] asm("_binary_index_html_start");
extern const uint8_t audio_web_index_end[] asm("_binary_index_html_end");

static const char *TAG = "audio_web_test";

/** 组件状态集中保存在实例中，避免网页请求与音频任务之间使用全局可变变量。 */
struct audio_web_test_context {
    audio_web_test_config_t config;
    audio_pipeline_subscription_t *subscription;
    SemaphoreHandle_t state_mutex;
    httpd_handle_t server;
    TaskHandle_t audio_task;
    int client_socket;
    uint8_t selected_channel;
    bool streaming;
    bool running;
};

/** @brief 初始化 NVS；发现旧版本或页耗尽时擦除 NVS 分区并重新初始化。 */
static esp_err_t audio_web_test_initialize_nvs(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "擦除不兼容的 NVS 分区失败");
        result = nvs_flash_init();
    }
    return result;
}

/** @brief 配置并启动 ESP32-C3 SoftAP，默认网关由 ESP-NETIF 设置为 192.168.4.1。 */
static esp_err_t audio_web_test_start_access_point(struct audio_web_test_context *context)
{
    ESP_RETURN_ON_ERROR(audio_web_test_initialize_nvs(), TAG, "初始化 NVS 失败");

    esp_err_t result = esp_netif_init();
    ESP_RETURN_ON_FALSE(result == ESP_OK || result == ESP_ERR_INVALID_STATE,
                        result, TAG, "初始化 ESP-NETIF 失败");
    result = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE(result == ESP_OK || result == ESP_ERR_INVALID_STATE,
                        result, TAG, "创建默认事件循环失败");
    ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_ap() != NULL,
                        ESP_FAIL, TAG, "创建 SoftAP 网络接口失败");

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init), TAG, "初始化 Wi-Fi 驱动失败");

    wifi_config_t wifi_config = {0};
    size_t ssid_length = strlen(context->config.ap_ssid);
    memcpy(wifi_config.ap.ssid, context->config.ap_ssid, ssid_length);
    wifi_config.ap.ssid_len = ssid_length;
    size_t password_length = strlen(context->config.ap_password);
    memcpy(wifi_config.ap.password, context->config.ap_password, password_length);
    wifi_config.ap.channel = context->config.ap_channel;
    wifi_config.ap.max_connection = context->config.max_connections;
    wifi_config.ap.authmode = password_length == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "设置 SoftAP 模式失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config),
                        TAG, "写入 SoftAP 配置失败");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "启动 SoftAP 失败");
    ESP_LOGI(TAG, "设备热点已启动: SSID=%s, 地址=http://192.168.4.1",
             context->config.ap_ssid);
    return ESP_OK;
}

/** @brief 返回固件内嵌的完整测试页面。 */
static esp_err_t audio_web_test_index_handler(httpd_req_t *request)
{
    const size_t page_size = (size_t)(audio_web_index_end - audio_web_index_start);
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, (const char *)audio_web_index_start, page_size);
}

/** @brief 向当前 WebSocket 请求发送简短状态，便于网页展示确定的设备状态。 */
static esp_err_t audio_web_test_send_status(httpd_req_t *request,
                                            const char *status,
                                            uint8_t channel)
{
    char response[96];
    int length = snprintf(response, sizeof(response),
                          "{\"status\":\"%s\",\"channel\":%u}", status, channel);
    ESP_RETURN_ON_FALSE(length > 0 && length < (int)sizeof(response),
                        ESP_FAIL, TAG, "生成 WebSocket 状态失败");
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)response,
        .len = (size_t)length,
    };
    return httpd_ws_send_frame(request, &frame);
}

/** @brief 校验并执行网页发来的 start、select、stop JSON 命令。 */
static esp_err_t audio_web_test_process_command(struct audio_web_test_context *context,
                                                httpd_req_t *request,
                                                const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    ESP_RETURN_ON_FALSE(root != NULL, ESP_ERR_INVALID_ARG, TAG, "WebSocket JSON 格式无效");

    esp_err_t result = ESP_OK;
    cJSON *action = cJSON_GetObjectItemCaseSensitive(root, "action");
    if (!cJSON_IsString(action) || action->valuestring == NULL) {
        result = ESP_ERR_INVALID_ARG;
        goto finish;
    }

    bool needs_channel = strcmp(action->valuestring, "start") == 0 ||
                         strcmp(action->valuestring, "select") == 0;
    cJSON *channel = cJSON_GetObjectItemCaseSensitive(root, "channel");
    if (needs_channel &&
        (!cJSON_IsNumber(channel) || channel->valuedouble != channel->valueint ||
         channel->valueint < 0 || channel->valueint >= ES7210_AUDIO_CHANNEL_COUNT)) {
        result = ESP_ERR_INVALID_ARG;
        goto finish;
    }

    xSemaphoreTake(context->state_mutex, portMAX_DELAY);
    if (strcmp(action->valuestring, "start") == 0) {
        context->client_socket = httpd_req_to_sockfd(request);
        context->selected_channel = (uint8_t)channel->valueint;
        context->streaming = true;
    } else if (strcmp(action->valuestring, "select") == 0) {
        context->client_socket = httpd_req_to_sockfd(request);
        context->selected_channel = (uint8_t)channel->valueint;
    } else if (strcmp(action->valuestring, "stop") == 0) {
        context->streaming = false;
    } else {
        result = ESP_ERR_NOT_SUPPORTED;
    }
    uint8_t selected_channel = context->selected_channel;
    bool streaming = context->streaming;
    xSemaphoreGive(context->state_mutex);

    if (result == ESP_OK) {
        result = audio_web_test_send_status(
            request, streaming ? "streaming" : "stopped", selected_channel);
    }

finish:
    cJSON_Delete(root);
    return result;
}

/** @brief 完成 WebSocket 握手，并接收长度受限的文本控制帧。 */
static esp_err_t audio_web_test_websocket_handler(httpd_req_t *request)
{
    struct audio_web_test_context *context = request->user_ctx;
    if (request->method == HTTP_GET) {
        ESP_LOGI(TAG, "网页客户端已建立 WebSocket 连接");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t result = httpd_ws_recv_frame(request, &frame, 0);
    ESP_RETURN_ON_ERROR(result, TAG, "读取 WebSocket 帧长度失败");
    ESP_RETURN_ON_FALSE(frame.type == HTTPD_WS_TYPE_TEXT && frame.len > 0 && frame.len < 256,
                        ESP_ERR_INVALID_ARG, TAG, "WebSocket 控制帧类型或长度无效");

    char payload[256];
    frame.payload = (uint8_t *)payload;
    result = httpd_ws_recv_frame(request, &frame, frame.len);
    ESP_RETURN_ON_ERROR(result, TAG, "读取 WebSocket 控制帧失败");
    payload[frame.len] = '\0';

    result = audio_web_test_process_command(context, request, payload);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "拒绝无效网页命令: %s", esp_err_to_name(result));
        (void)audio_web_test_send_status(request, "invalid_command", 0);
        return ESP_OK;
    }
    return ESP_OK;
}

/** @brief 启动 HTTP 服务器并注册主页和音频 WebSocket 两个端点。 */
static esp_err_t audio_web_test_start_server(struct audio_web_test_context *context)
{
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.stack_size = 6144;
    server_config.lru_purge_enable = true;
    ESP_RETURN_ON_ERROR(httpd_start(&context->server, &server_config),
                        TAG, "启动 HTTP 服务器失败");

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = audio_web_test_index_handler,
        .user_ctx = context,
    };
    const httpd_uri_t websocket_uri = {
        .uri = "/audio",
        .method = HTTP_GET,
        .handler = audio_web_test_websocket_handler,
        .user_ctx = context,
        .is_websocket = true,
        .handle_ws_control_frames = false,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(context->server, &index_uri),
                        TAG, "注册主页处理器失败");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(context->server, &websocket_uri),
                        TAG, "注册音频 WebSocket 处理器失败");
    return ESP_OK;
}

/** @brief 持续消费中间层音频帧，并在启用推流时抽取指定麦克风通道。 */
static void audio_web_test_audio_task(void *argument)
{
    struct audio_web_test_context *context = argument;
    while (context->running) {
        const audio_pipeline_frame_t *source = NULL;
        esp_err_t result = audio_pipeline_receive(
            context->subscription, &source, pdMS_TO_TICKS(100));
        if (result == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "网页测试组件接收音频失败: %s", esp_err_to_name(result));
            continue;
        }

        xSemaphoreTake(context->state_mutex, portMAX_DELAY);
        bool should_send = context->streaming;
        uint8_t channel = context->selected_channel;
        int socket = context->client_socket;
        httpd_handle_t server = context->server;
        xSemaphoreGive(context->state_mutex);

        if (should_send && socket >= 0 && server != NULL) {
            /*
             * 中间层保存四通道交织 PCM。这里生成一帧单通道 PCM16，小端序由 ESP32-C3
             * 的本机字节序保证。httpd_ws_send_data 会等待服务器任务完成发送，因此栈上
             * 的 mono_samples 在函数返回前始终有效。
             */
            int16_t mono_samples[AUDIO_PIPELINE_SAMPLES_PER_CHANNEL];
            for (size_t sample = 0; sample < source->samples_per_channel; ++sample) {
                mono_samples[sample] = source->data[sample * source->channel_count + channel];
            }
            httpd_ws_frame_t websocket_frame = {
                .type = HTTPD_WS_TYPE_BINARY,
                .payload = (uint8_t *)mono_samples,
                .len = source->samples_per_channel * sizeof(mono_samples[0]),
            };
            result = httpd_ws_send_data(server, socket, &websocket_frame);
            if (result != ESP_OK) {
                xSemaphoreTake(context->state_mutex, portMAX_DELAY);
                if (context->client_socket == socket) {
                    context->streaming = false;
                    context->client_socket = -1;
                }
                xSemaphoreGive(context->state_mutex);
                ESP_LOGW(TAG, "网页音频连接已断开: %s", esp_err_to_name(result));
            }
        }
        audio_pipeline_release_frame(context->subscription, source);
    }

    context->audio_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t audio_web_test_init(const audio_web_test_config_t *config,
                              audio_web_test_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL && config->pipeline != NULL &&
                            config->ap_ssid != NULL && config->ap_password != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "网页测试组件配置为空");
    size_t ssid_length = strlen(config->ap_ssid);
    size_t password_length = strlen(config->ap_password);
    ESP_RETURN_ON_FALSE(ssid_length > 0 && ssid_length <= 32 && password_length <= 63 &&
                            (password_length == 0 || password_length >= 8) &&
                            config->ap_channel >= 1 && config->ap_channel <= 13 &&
                            config->max_connections >= 1 && config->max_connections <= 4,
                        ESP_ERR_INVALID_ARG, TAG, "SoftAP 参数超出有效范围");

    *out_handle = NULL;
    struct audio_web_test_context *context = calloc(1, sizeof(*context));
    ESP_RETURN_ON_FALSE(context != NULL, ESP_ERR_NO_MEM, TAG, "分配网页测试上下文失败");
    context->config = *config;
    context->client_socket = -1;
    context->state_mutex = xSemaphoreCreateMutex();
    if (context->state_mutex == NULL) {
        free(context);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = audio_pipeline_subscribe(config->pipeline, 2, &context->subscription);
    if (result != ESP_OK) {
        vSemaphoreDelete(context->state_mutex);
        free(context);
        return result;
    }
    *out_handle = context;
    return ESP_OK;
}

esp_err_t audio_web_test_start(audio_web_test_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "网页测试组件句柄为空");
    ESP_RETURN_ON_FALSE(!handle->running && handle->audio_task == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "网页测试组件已经运行");
    ESP_RETURN_ON_ERROR(audio_web_test_start_access_point(handle), TAG, "启动设备热点失败");
    ESP_RETURN_ON_ERROR(audio_web_test_start_server(handle), TAG, "启动网页服务失败");

    handle->running = true;
    if (xTaskCreate(audio_web_test_audio_task,
                    "audio_web_test",
                    handle->config.task_stack_size,
                    handle,
                    handle->config.task_priority,
                    &handle->audio_task) != pdPASS) {
        handle->running = false;
        (void)httpd_stop(handle->server);
        handle->server = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "四路麦克风网页测试服务已就绪");
    return ESP_OK;
}
