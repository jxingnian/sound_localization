/**
 * @file audio_web_test.h
 * @brief 四路麦克风网页试听测试组件的公开接口。
 *
 * 本组件作为 audio_pipeline 的普通应用层订阅者存在，只消费中间层分发的音频帧。
 * 它不会直接访问 ES7210 或 I2S，因此后续唤醒、录音等应用可以继续独立订阅同一中间层。
 */
#pragma once

#include <stdint.h>

#include "audio_pipeline.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 网页试听组件实例的不透明句柄。 */
typedef struct audio_web_test_context *audio_web_test_handle_t;

/** @brief 网页试听组件的运行参数。 */
typedef struct {
    audio_pipeline_handle_t pipeline; /**< 音频中间层实例，生命周期必须覆盖本组件。 */
    const char *ap_ssid;               /**< 设备热点名称，最大 32 字节。 */
    const char *ap_password;           /**< 设备热点密码，必须为空或至少 8 字节。 */
    uint8_t ap_channel;                /**< Wi-Fi 热点信道，范围为 1 至 13。 */
    uint8_t max_connections;           /**< 热点允许同时连接的终端数量，范围为 1 至 4。 */
    uint32_t task_stack_size;          /**< 音频发送任务栈大小，单位为字节。 */
    UBaseType_t task_priority;         /**< 音频发送任务优先级。 */
} audio_web_test_config_t;

/** 生成可直接用于测试的默认配置。 */
#define AUDIO_WEB_TEST_DEFAULT_CONFIG(pipeline_handle) \
    {                                                   \
        .pipeline = (pipeline_handle),                  \
        .ap_ssid = "SoundLocalization",                \
        .ap_password = "sound1234",                    \
        .ap_channel = 1,                                \
        .max_connections = 2,                           \
        .task_stack_size = 6144,                        \
        .task_priority = 5,                             \
    }

/**
 * @brief 创建中间层订阅和组件同步对象，但暂不启动热点、网页服务器和任务。
 */
esp_err_t audio_web_test_init(const audio_web_test_config_t *config,
                              audio_web_test_handle_t *out_handle);

/**
 * @brief 启动 SoftAP、HTTP/WebSocket 服务和音频消费任务。
 *
 * 成功后，连接默认热点并访问 http://192.168.4.1 即可打开测试页面。
 */
esp_err_t audio_web_test_start(audio_web_test_handle_t handle);

#ifdef __cplusplus
}
#endif
