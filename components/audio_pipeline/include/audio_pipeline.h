/**
 * @file audio_pipeline.h
 * @brief 多应用共享音频的中间层接口。
 *
 * 中间层从 ES7210 驱动获取唯一一份音频帧，再通过引用计数帧池分发给多个订阅者。
 * 声源定位、唤醒词和录音等应用互不依赖，也不会并发读取同一个 I2S 外设。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "es7210_audio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 每帧每个麦克风的采样点数；16 kHz 下对应 30 ms 音频。 */
#define AUDIO_PIPELINE_SAMPLES_PER_CHANNEL 480U

/** 中间层允许同时存在的应用订阅者上限。 */
#define AUDIO_PIPELINE_MAX_SUBSCRIBERS 4U

/**
 * @brief 分发给应用的只读音频帧。
 *
 * 应用不得修改 data，也不得长期持有帧。处理完成后必须调用
 * audio_pipeline_release_frame()，否则帧池最终会耗尽并停止采集。
 */
typedef struct {
    uint32_t sequence;             /**< 从 0 开始单调递增的帧序号。 */
    int64_t timestamp_us;          /**< 驱动完成整帧读取时的系统微秒时间戳。 */
    uint32_t sample_rate_hz;       /**< 每个通道的采样率。 */
    uint16_t samples_per_channel;  /**< 每个通道在本帧中的采样点数。 */
    uint8_t channel_count;         /**< 固定为四个 ES7210 模拟麦克风通道。 */
    int16_t data[AUDIO_PIPELINE_SAMPLES_PER_CHANNEL * ES7210_AUDIO_CHANNEL_COUNT];
} audio_pipeline_frame_t;

/** 音频中间层实例的不透明句柄。 */
typedef struct audio_pipeline_context *audio_pipeline_handle_t;

/** 单个应用订阅关系的不透明句柄。 */
typedef struct audio_pipeline_subscription audio_pipeline_subscription_t;

/** @brief 音频中间层运行参数。 */
typedef struct {
    es7210_audio_handle_t driver;  /**< 唯一的数据源驱动，生命周期必须覆盖中间层。 */
    uint8_t frame_pool_size;       /**< 可同时被应用持有的总帧数，范围为 3 到 12。 */
    uint32_t task_stack_size;      /**< 采集分发任务的栈大小，单位为字节。 */
    UBaseType_t task_priority;     /**< 采集任务优先级，应高于普通应用任务。 */
} audio_pipeline_config_t;

/** 生成适合 ESP32-C3 内存规模的默认中间层配置。 */
#define AUDIO_PIPELINE_DEFAULT_CONFIG(driver_handle) \
    {                                                 \
        .driver = (driver_handle),                    \
        .frame_pool_size = 6,                         \
        .task_stack_size = 4096,                      \
        .task_priority = 8,                           \
    }

/** @brief 创建帧池、订阅表和同步对象，但不启动硬件采集。 */
esp_err_t audio_pipeline_init(const audio_pipeline_config_t *config,
                              audio_pipeline_handle_t *out_pipeline);

/**
 * @brief 为一个应用创建独立接收队列。
 *
 * @param[in] pipeline 已初始化的中间层。
 * @param[in] queue_depth 该应用允许积压的帧数，必须小于帧池大小。
 * @param[out] out_subscription 返回创建的订阅句柄。
 */
esp_err_t audio_pipeline_subscribe(audio_pipeline_handle_t pipeline,
                                   uint8_t queue_depth,
                                   audio_pipeline_subscription_t **out_subscription);

/** @brief 启动唯一的采集与分发任务。 */
esp_err_t audio_pipeline_start(audio_pipeline_handle_t pipeline);

/**
 * @brief 等待订阅队列中的下一帧。
 *
 * 返回成功后，调用方获得一个帧引用，必须用 audio_pipeline_release_frame() 归还。
 */
esp_err_t audio_pipeline_receive(audio_pipeline_subscription_t *subscription,
                                 const audio_pipeline_frame_t **out_frame,
                                 TickType_t wait_ticks);

/** @brief 归还 receive 获得的帧引用，使帧能够重新进入采集池。 */
void audio_pipeline_release_frame(audio_pipeline_subscription_t *subscription,
                                  const audio_pipeline_frame_t *frame);

/** @brief 停止新数据采集并等待分发任务退出。 */
esp_err_t audio_pipeline_stop(audio_pipeline_handle_t pipeline, TickType_t wait_ticks);

/** @brief 删除一个订阅并归还其队列中尚未处理的全部帧。 */
esp_err_t audio_pipeline_unsubscribe(audio_pipeline_subscription_t *subscription);

/** @brief 释放中间层资源；调用前必须停止采集并删除全部订阅。 */
esp_err_t audio_pipeline_deinit(audio_pipeline_handle_t pipeline);

#ifdef __cplusplus
}
#endif
