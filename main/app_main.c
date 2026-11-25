/**
 * @file app_main.c
 * @brief 声源定位固件的组件装配入口。
 *
 * 主程序只负责创建驱动层、中间层和定位应用层，并输出定位结果。算法和硬件细节
 * 均保留在各自组件内，后续增加唤醒词应用时只需订阅 audio_pipeline。
 */

#include <inttypes.h>

#include "audio_web_test.h"
#include "audio_pipeline.h"
#include "es7210_audio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sound_localization.h"

static const char *TAG = "sound_localization_app";

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
        .minimum_rms = 300.0f,
        .minimum_correlation = 0.25f,
        .task_stack_size = 6144,
        .task_priority = 6,
    };
    sound_localization_handle_t localizer = NULL;
    ESP_ERROR_CHECK(sound_localization_init(&localization_config, &localizer));

    /*
     * 网页试听是中间层的第二个独立应用订阅者。它只读取 audio_pipeline，后续新增
     * 唤醒词或录音应用时仍可用相同方式订阅，不需要改动 ES7210 驱动。
     */
    audio_web_test_config_t web_test_config = AUDIO_WEB_TEST_DEFAULT_CONFIG(pipeline);
    audio_web_test_handle_t web_test = NULL;
    ESP_ERROR_CHECK(audio_web_test_init(&web_test_config, &web_test));

    /* 先启动全部消费者，再启动生产者，确保第一帧音频能同时进入定位和网页应用。 */
    ESP_ERROR_CHECK(sound_localization_start(localizer));
    ESP_ERROR_CHECK(audio_web_test_start(web_test));
    ESP_ERROR_CHECK(audio_pipeline_start(pipeline));
    ESP_LOGI(TAG, "四麦克风定位与网页试听链路已启动");

    while (true) {
        sound_localization_result_t result;
        esp_err_t wait_result = sound_localization_wait_result(
            localizer, &result, pdMS_TO_TICKS(1000));
        if (wait_result == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "一秒内未收到定位结果");
            continue;
        }
        ESP_ERROR_CHECK(wait_result);

        if (result.valid) {
            ESP_LOGI(TAG,
                     "帧=%" PRIu32 ", 方位=%.1f 度, 置信度=%.2f, RMS=%.1f, "
                     "延时=[%.1f, %.1f, %.1f, %.1f] us",
                     result.source_sequence,
                     result.azimuth_degrees,
                     result.confidence,
                     result.rms,
                     result.delays_us[0],
                     result.delays_us[1],
                     result.delays_us[2],
                     result.delays_us[3]);
        }
    }
}
