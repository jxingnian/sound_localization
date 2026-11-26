/**
 * @file app_main.c
 * @brief 声源定位固件的组件装配入口。
 *
 * 主程序只负责创建驱动层、中间层和定位应用层，并输出定位结果。算法和硬件细节
 * 均保留在各自组件内，后续增加唤醒词应用时只需订阅 audio_pipeline。
 */

#include <inttypes.h>

#include "audio_pipeline.h"
#include "es7210_audio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sound_localization.h"

static const char *TAG = "sound_localization_app";

/** 无效状态每十帧输出一次；16 kHz、480 点每帧时对应约 300 ms。 */
#define LOCALIZATION_LOG_INTERVAL_FRAMES 10U

/** 有效角度首帧立即输出，连续声音期间最多每 100 ms 输出一次。 */
#define LOCALIZATION_VALID_LOG_INTERVAL_US 100000LL

/**
 * 实机日志测得静音 RMS 为 10.8 至 16.8，已捕获的声音帧为 43.7 至 160.6。
 * 取 40 可明确排除静音底噪，同时允许实机已经捕获到的声音进入互相关定位阶段。
 */
#define LOCALIZATION_MINIMUM_RMS 40.0f

/** @brief 把失败配对索引转换为与 PCB 器件一致的可读名称。 */
static const char *localization_pair_name(uint8_t pair_index)
{
    static const char *const pair_names[SOUND_LOCALIZATION_PAIR_COUNT] = {
        "MIC1-MIC2",
        "MIC1-MIC4",
        "MIC2-MIC4",
    };
    return pair_index < SOUND_LOCALIZATION_PAIR_COUNT
               ? pair_names[pair_index]
               : "无指定配对";
}

/** @brief 把算法拒绝枚举转换为串口诊断文字。 */
static const char *localization_rejection_name(
    sound_localization_rejection_reason_t reason)
{
    switch (reason) {
        case SOUND_LOCALIZATION_REJECTION_NONE:
            return "通过";
        case SOUND_LOCALIZATION_REJECTION_INVALID_FRAME:
            return "音频帧格式无效";
        case SOUND_LOCALIZATION_REJECTION_LOW_ENERGY:
            return "三麦声音能量低于门限";
        case SOUND_LOCALIZATION_REJECTION_LOW_CORRELATION:
            return "麦克风声音相关性低于门限";
        case SOUND_LOCALIZATION_REJECTION_PHYSICAL_DELAY:
            return "时差超过麦克风间距物理上限";
        case SOUND_LOCALIZATION_REJECTION_CLOSURE_ERROR:
            return "三对时差闭环不一致";
        case SOUND_LOCALIZATION_REJECTION_GEOMETRY:
            return "麦克风几何方程不可解";
        case SOUND_LOCALIZATION_REJECTION_DIRECTION_NORM:
            return "方向解不符合远场模型";
        default:
            return "未知拒绝原因";
    }
}

/**
 * @brief 启动四通道采集、共享分发和三麦克风定位链路。
 *
 * 嘉立创 PCB2 回读确认 U1/MIC1、U2/MIC2、U4/MIC4 构成边长约 34 mm 的
 * 等边三角形，U3/MIC3 是独立麦克风。坐标由 PCB 器件中心换算并以三角形质心
 * 为原点，X 向 PCB 视图右侧为正，Y 向 PCB 视图上方为正。
 */
void app_main(void)
{
    es7210_audio_config_t driver_config = ES7210_AUDIO_DEFAULT_CONFIG();
    es7210_audio_handle_t audio_driver = NULL;
    ESP_ERROR_CHECK(es7210_audio_init(&driver_config, &audio_driver));

    audio_pipeline_config_t pipeline_config =
        AUDIO_PIPELINE_DEFAULT_CONFIG(audio_driver);
    audio_pipeline_handle_t pipeline = NULL;
    ESP_ERROR_CHECK(audio_pipeline_init(&pipeline_config, &pipeline));

    sound_localization_config_t localization_config = {
        .pipeline = pipeline,
        /* TDM 数据仍保留四路；定位层只选择 MIC1、MIC2、MIC4，即通道 0、1、3。 */
        .channel_indices = {0, 1, 3},
        .microphones = {
            {.x_m = 0.004732f, .y_m = 0.018850f},
            {.x_m = 0.014069f, .y_m = -0.013713f},
            {.x_m = -0.018800f, .y_m = -0.005137f},
        },
        .speed_of_sound_mps = 343.0f,
        .minimum_rms = LOCALIZATION_MINIMUM_RMS,
        .minimum_correlation = 0.25f,
        /* 三对时差闭环误差不得超过 0.75 个采样点，拒绝互相矛盾的相关峰。 */
        .maximum_closure_error_samples = 0.75f,
        /* 方向解模长偏离远场单位向量过多时不输出角度。 */
        .minimum_direction_norm = 0.35f,
        .maximum_direction_norm = 1.35f,
        /* 每帧使用 25% 新方向更新单位向量，抑制角度乱跳并保留跟随能力。 */
        .smoothing_factor = 0.25f,
        .task_stack_size = 6144,
        .task_priority = 6,
    };
    sound_localization_handle_t localizer = NULL;
    ESP_ERROR_CHECK(sound_localization_init(&localization_config, &localizer));

    /* 先启动定位消费者，再启动音频生产者，确保第一帧采集数据也能进入定位算法。 */
    ESP_ERROR_CHECK(sound_localization_start(localizer));
    ESP_ERROR_CHECK(audio_pipeline_start(pipeline));
    ESP_LOGI(TAG,
             "三麦克风声源定位已启动: MIC1/TDM0 + MIC2/TDM1 + MIC4/TDM3，"
             "MIC3/TDM2 保留给独立应用");

    int64_t last_valid_log_timestamp_us = 0;
    while (true) {
        sound_localization_result_t result;
        esp_err_t wait_result = sound_localization_wait_result(
            localizer, &result, pdMS_TO_TICKS(1000));
        if (wait_result == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "一秒内未收到定位结果");
            continue;
        }
        ESP_ERROR_CHECK(wait_result);

        if (result.valid &&
            (last_valid_log_timestamp_us == 0 ||
             result.source_timestamp_us - last_valid_log_timestamp_us >=
                 LOCALIZATION_VALID_LOG_INTERVAL_US)) {
            /* 首个有效结果立即输出；连续结果限频，避免串口输出再次占满单核 CPU。 */
            last_valid_log_timestamp_us = result.source_timestamp_us;
            ESP_LOGI(TAG,
                     "帧=%" PRIu32 ", 定位角度=%.1f 度, 置信度=%.2f, RMS=%.1f, "
                     "单麦RMS=[%.1f, %.1f, %.1f], 相关=[%.2f, %.2f, %.2f], "
                     "延时=[MIC1-2 %.1f, MIC1-4 %.1f, MIC2-4 %.1f] us, "
                     "闭环误差=%.1f us, 方向模长=%.2f",
                     result.source_sequence,
                     result.azimuth_degrees,
                     result.confidence,
                     result.rms,
                     result.microphone_rms[0],
                     result.microphone_rms[1],
                     result.microphone_rms[2],
                     result.pair_correlations[0],
                     result.pair_correlations[1],
                     result.pair_correlations[2],
                     result.pair_delays_us[0],
                     result.pair_delays_us[1],
                     result.pair_delays_us[2],
                     result.closure_error_us,
                     result.direction_norm);
        } else if (!result.valid &&
                   (result.source_sequence % LOCALIZATION_LOG_INTERVAL_FRAMES) == 0U) {
            /* 无效状态限频输出，避免静音时串口日志占用实时音频任务。 */
            ESP_LOGI(TAG,
                     "帧=%" PRIu32 ", 拒绝原因=%s, 失败配对=%s, "
                     "总RMS=%.1f, 单麦RMS=[%.1f, %.1f, %.1f], "
                     "相关=[MIC1-2 %.2f, MIC1-4 %.2f, MIC2-4 %.2f], "
                     "延时=[%.1f, %.1f, %.1f] us, 闭环误差=%.1f us, 方向模长=%.2f",
                     result.source_sequence,
                     localization_rejection_name(result.rejection_reason),
                     localization_pair_name(result.rejected_pair_index),
                     result.rms,
                     result.microphone_rms[0],
                     result.microphone_rms[1],
                     result.microphone_rms[2],
                     result.pair_correlations[0],
                     result.pair_correlations[1],
                     result.pair_correlations[2],
                     result.pair_delays_us[0],
                     result.pair_delays_us[1],
                     result.pair_delays_us[2],
                     result.closure_error_us,
                     result.direction_norm);
        }
    }
}
