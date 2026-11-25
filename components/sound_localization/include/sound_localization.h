/**
 * @file sound_localization.h
 * @brief 四麦克风二维远场声源定位应用组件。
 *
 * 组件只依赖 audio_pipeline 订阅接口，不直接访问 ES7210 或 I2S，因此可以与
 * 唤醒词、录音等应用同时工作。算法先估计 MIC1 到其余麦克风的到达时间差，
 * 再根据实际麦克风坐标解二维最小二乘方程，输出 0 到 360 度方位角。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "audio_pipeline.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 声源定位实例的不透明句柄。 */
typedef struct sound_localization_context *sound_localization_handle_t;

/** @brief 单个麦克风声学中心在板卡坐标系中的位置。 */
typedef struct {
    float x_m; /**< 向板卡右侧为正的 X 坐标，单位为米。 */
    float y_m; /**< 向板卡前方为正的 Y 坐标，单位为米。 */
} sound_localization_microphone_position_t;

/** @brief 声源定位运行参数。 */
typedef struct {
    audio_pipeline_handle_t pipeline; /**< 提供四通道同步 PCM 的中间层。 */
    sound_localization_microphone_position_t microphones[ES7210_AUDIO_CHANNEL_COUNT];
    float speed_of_sound_mps;        /**< 当前温度下的声速，室温默认使用 343 m/s。 */
    float minimum_rms;               /**< 低于该 PCM 均方根幅度的帧不输出方向。 */
    float minimum_correlation;       /**< 相关峰值低于该值时不输出方向。 */
    uint32_t task_stack_size;        /**< 定位任务栈大小，单位为字节。 */
    UBaseType_t task_priority;       /**< 定位任务优先级。 */
} sound_localization_config_t;

/**
 * @brief 定位结果。
 *
 * 坐标定义为 X 轴正向 0 度，逆时针增加；+Y 为 90 度，-X 为 180 度。
 */
typedef struct {
    bool valid;                 /**< true 表示能量、相关性和几何约束均通过。 */
    float azimuth_degrees;      /**< 归一化到 [0, 360) 的二维方位角。 */
    float confidence;           /**< 0 到 1 的平均归一化互相关置信度。 */
    float rms;                  /**< 四通道本帧的总体均方根幅度。 */
    float delays_us[ES7210_AUDIO_CHANNEL_COUNT]; /**< 各通道相对 MIC1 的到达延时。 */
    uint32_t source_sequence;   /**< 生成该结果的音频帧序号。 */
    int64_t source_timestamp_us; /**< 对应音频帧的时间戳。 */
} sound_localization_result_t;

/**
 * @brief 创建定位实例并订阅音频中间层。
 *
 * 四个麦克风坐标必须是实测的 PCB 声学中心坐标，且至少形成二维非共线阵列。
 */
esp_err_t sound_localization_init(const sound_localization_config_t *config,
                                  sound_localization_handle_t *out_handle);

/** @brief 启动后台定位任务。 */
esp_err_t sound_localization_start(sound_localization_handle_t handle);

/** @brief 等待最新定位结果；队列始终只保留最近一次结果。 */
esp_err_t sound_localization_wait_result(sound_localization_handle_t handle,
                                         sound_localization_result_t *out_result,
                                         TickType_t wait_ticks);

/** @brief 停止定位任务。 */
esp_err_t sound_localization_stop(sound_localization_handle_t handle,
                                  TickType_t wait_ticks);

/** @brief 删除订阅、结果队列和定位实例。 */
esp_err_t sound_localization_deinit(sound_localization_handle_t handle);

#ifdef __cplusplus
}
#endif
