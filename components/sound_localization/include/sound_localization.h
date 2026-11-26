/**
 * @file sound_localization.h
 * @brief 从四通道音频中选择三只阵列麦克风执行二维远场声源定位。
 *
 * 组件只依赖 audio_pipeline 订阅接口，不直接访问 ES7210 或 I2S，因此可以与
 * 唤醒词、录音等应用同时工作。定位通道由配置明确指定，未被选择的独立麦克风
 * 仍保留在中间层中供其他应用使用。算法计算三只阵列麦克风的全部三对时差，
 * 通过物理时差和闭环一致性检查后求解二维最小二乘方程。
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

/** 三角形声学阵列固定使用三只麦克风。 */
#define SOUND_LOCALIZATION_MICROPHONE_COUNT 3U

/** 三只麦克风形成三组无重复配对：0-1、0-2 和 1-2。 */
#define SOUND_LOCALIZATION_PAIR_COUNT 3U

/** @brief 单个麦克风声学中心在板卡坐标系中的位置。 */
typedef struct {
    float x_m; /**< 向板卡右侧为正的 X 坐标，单位为米。 */
    float y_m; /**< 向板卡前方为正的 Y 坐标，单位为米。 */
} sound_localization_microphone_position_t;

/** @brief 声源定位运行参数。 */
typedef struct {
    audio_pipeline_handle_t pipeline; /**< 提供四通道同步 PCM 的中间层。 */
    /**
     * 三只定位麦克风在四通道 TDM 帧中的索引。索引必须互不相同且小于
     * ES7210_AUDIO_CHANNEL_COUNT；数组顺序同时决定 microphones 的对应关系。
     */
    uint8_t channel_indices[SOUND_LOCALIZATION_MICROPHONE_COUNT];
    /** 三只定位麦克风的 PCB 声学中心坐标，顺序与 channel_indices 完全一致。 */
    sound_localization_microphone_position_t
        microphones[SOUND_LOCALIZATION_MICROPHONE_COUNT];
    float speed_of_sound_mps;        /**< 当前温度下的声速，室温默认使用 343 m/s。 */
    float minimum_rms;               /**< 三只定位麦克风 RMS 低于该值时不输出方向。 */
    float minimum_correlation;       /**< 任意一对相关峰值低于该值时不输出方向。 */
    /** 三对时差闭环误差上限，单位为采样点。 */
    float maximum_closure_error_samples;
    /** 未归一化方向解的最小模长，用于拒绝方向信息不足的结果。 */
    float minimum_direction_norm;
    /** 未归一化方向解的最大模长，用于拒绝违反远场传播模型的结果。 */
    float maximum_direction_norm;
    /** 方位单位向量的低通更新系数，范围为 (0, 1]，1 表示不平滑。 */
    float smoothing_factor;
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
    float confidence;           /**< 三对归一化互相关峰值的平均值。 */
    float rms;                  /**< 三只定位麦克风的总体均方根幅度。 */
    /** 配对 0-1、0-2、1-2 的到达时差，正值表示后一只麦克风更晚收到声音。 */
    float pair_delays_us[SOUND_LOCALIZATION_PAIR_COUNT];
    float closure_error_us;     /**< tau01 + tau12 - tau02 的闭环误差。 */
    float direction_norm;       /**< 归一化前方向解的模长，用于诊断解的物理一致性。 */
    uint32_t source_sequence;   /**< 生成该结果的音频帧序号。 */
    int64_t source_timestamp_us; /**< 对应音频帧的时间戳。 */
} sound_localization_result_t;

/**
 * @brief 创建定位实例并订阅音频中间层。
 *
 * 三个通道索引必须互不相同，三个坐标必须来自 PCB 声学中心并形成非共线阵列。
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
