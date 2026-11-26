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

/**
 * @brief 启动整条四通道音频与定位链路。
 *
 * 坐标以阵列中心为原点，当前配置明确表示 50 mm x 50 mm 方形阵列：
 * MIC1 左后、MIC2 右后、MIC3 左前、MIC4 右前。原理图只给出电气连接，量产 PCB
 * 的声学中心坐标确定后，应把下面四组数值更新为 PCB 实测坐标。
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
        .microphones = {
            {.x_m = -0.025f, .y_m = 0.025f},
            {.x_m = 0.025f, .y_m = 0.025f},
            {.x_m = -0.025f, .y_m = -0.025f},
            {.x_m = 0.025f, .y_m = -0.025f},
        },
        .speed_of_sound_mps = 343.0f,
        .minimum_rms = LOCALIZATION_MINIMUM_RMS,
        .minimum_correlation = 0.25f,
        .task_stack_size = 6144,
        .task_priority = 6,
    };
    sound_localization_handle_t localizer = NULL;
    ESP_ERROR_CHECK(sound_localization_init(&localization_config, &localizer));

    /* 先启动定位消费者，再启动音频生产者，确保第一帧采集数据也能进入定位算法。 */
    ESP_ERROR_CHECK(sound_localization_start(localizer));
    ESP_ERROR_CHECK(audio_pipeline_start(pipeline));
    ESP_LOGI(TAG, "四麦克风声源定位已启动，当前仅使用串口日志输出定位结果");

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
                     "延时=[%.1f, %.1f, %.1f, %.1f] us",
                     result.source_sequence,
                     result.azimuth_degrees,
                     result.confidence,
                     result.rms,
                     result.delays_us[0],
                     result.delays_us[1],
                     result.delays_us[2],
                     result.delays_us[3]);
        } else if (!result.valid &&
                   (result.source_sequence % LOCALIZATION_LOG_INTERVAL_FRAMES) == 0U) {
            /* 无效状态限频输出，避免静音时串口日志占用实时音频任务。 */
            ESP_LOGI(TAG,
                     "帧=%" PRIu32 ", 当前帧未形成有效角度, 置信度=%.2f, RMS=%.1f",
                     result.source_sequence,
                     result.confidence,
                     result.rms);
        }
    }
}
