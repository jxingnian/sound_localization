/**
 * @file sound_localization.c
 * @brief 基于归一化互相关 TDOA 和二维最小二乘的四麦克风定位实现。
 */

#include "sound_localization.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "sound_localization";
static const float PI_F = 3.14159265358979323846f;

/** 互相关搜索允许的最大延迟，16 kHz 下 16 点覆盖 34.3 cm 麦克风间距。 */
#define SOUND_LOCALIZATION_MAX_LAG_SAMPLES 16

/** 定位组件私有上下文。 */
struct sound_localization_context {
    sound_localization_config_t config;
    audio_pipeline_subscription_t *subscription;
    QueueHandle_t result_queue;
    TaskHandle_t task;
    TaskHandle_t stop_waiter;
    volatile bool running;
};

/** @brief 把浮点值限制到闭区间，避免数值误差产生越界置信度。 */
static float sound_localization_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

/** @brief 计算四通道整帧 RMS，用于拒绝没有有效声源的静音帧。 */
static float sound_localization_calculate_rms(const audio_pipeline_frame_t *frame)
{
    double energy = 0.0;
    const size_t value_count = (size_t)frame->samples_per_channel * frame->channel_count;
    for (size_t index = 0; index < value_count; ++index) {
        const double value = frame->data[index];
        energy += value * value;
    }
    return value_count == 0 ? 0.0f : (float)sqrt(energy / value_count);
}

/**
 * @brief 计算参考麦克风与目标麦克风在指定整数延迟下的归一化互相关。
 *
 * lag 的定义为“目标通道到达时间减参考通道到达时间”。正值表示同一波形在
 * 目标通道中出现在更晚的采样点。计算时去除各自直流分量，避免 ADC 偏置制造假峰。
 */
static float sound_localization_correlation_at_lag(const audio_pipeline_frame_t *frame,
                                                   uint8_t target_channel,
                                                   int lag)
{
    const int sample_count = frame->samples_per_channel;
    const int begin = lag < 0 ? -lag : 0;
    const int end = lag > 0 ? sample_count - lag : sample_count;
    const int valid_count = end - begin;
    if (valid_count < 8) {
        return 0.0f;
    }

    double reference_mean = 0.0;
    double target_mean = 0.0;
    for (int index = begin; index < end; ++index) {
        reference_mean += frame->data[index * frame->channel_count];
        target_mean += frame->data[(index + lag) * frame->channel_count + target_channel];
    }
    reference_mean /= valid_count;
    target_mean /= valid_count;

    double numerator = 0.0;
    double reference_energy = 0.0;
    double target_energy = 0.0;
    for (int index = begin; index < end; ++index) {
        const double reference =
            frame->data[index * frame->channel_count] - reference_mean;
        const double target =
            frame->data[(index + lag) * frame->channel_count + target_channel] - target_mean;
        numerator += reference * target;
        reference_energy += reference * reference;
        target_energy += target * target;
    }

    const double denominator = sqrt(reference_energy * target_energy);
    return denominator <= 1.0 ? 0.0f : (float)(numerator / denominator);
}

/**
 * @brief 搜索相关峰并通过三点抛物线插值获得亚采样延迟。
 *
 * 亚采样插值显著降低 16 kHz 采样下整数点量化对小尺寸麦克风阵列的影响。
 */
static float sound_localization_estimate_delay(const audio_pipeline_frame_t *frame,
                                               uint8_t target_channel,
                                               int maximum_lag,
                                               float *out_peak)
{
    float correlations[2 * SOUND_LOCALIZATION_MAX_LAG_SAMPLES + 1] = {0};
    int best_lag = 0;
    float best_correlation = -1.0f;
    for (int lag = -maximum_lag; lag <= maximum_lag; ++lag) {
        const float correlation = sound_localization_correlation_at_lag(
            frame, target_channel, lag);
        correlations[lag + SOUND_LOCALIZATION_MAX_LAG_SAMPLES] = correlation;
        if (correlation > best_correlation) {
            best_correlation = correlation;
            best_lag = lag;
        }
    }

    float fractional_offset = 0.0f;
    if (best_lag > -maximum_lag && best_lag < maximum_lag) {
        const float left = correlations[best_lag - 1 + SOUND_LOCALIZATION_MAX_LAG_SAMPLES];
        const float center = correlations[best_lag + SOUND_LOCALIZATION_MAX_LAG_SAMPLES];
        const float right = correlations[best_lag + 1 + SOUND_LOCALIZATION_MAX_LAG_SAMPLES];
        const float curvature = left - 2.0f * center + right;
        if (fabsf(curvature) > 1.0e-6f) {
            fractional_offset = sound_localization_clamp(
                0.5f * (left - right) / curvature, -0.5f, 0.5f);
        }
    }
    *out_peak = sound_localization_clamp(best_correlation, 0.0f, 1.0f);
    return best_lag + fractional_offset;
}

/** @brief 验证麦克风坐标能够唯一解出二维方向。 */
static bool sound_localization_geometry_is_valid(
    const sound_localization_microphone_position_t microphones[ES7210_AUDIO_CHANNEL_COUNT])
{
    double ata_xx = 0.0;
    double ata_xy = 0.0;
    double ata_yy = 0.0;
    for (size_t channel = 1; channel < ES7210_AUDIO_CHANNEL_COUNT; ++channel) {
        const double dx = microphones[channel].x_m - microphones[0].x_m;
        const double dy = microphones[channel].y_m - microphones[0].y_m;
        ata_xx += dx * dx;
        ata_xy += dx * dy;
        ata_yy += dy * dy;
    }
    const double determinant = ata_xx * ata_yy - ata_xy * ata_xy;
    return determinant > 1.0e-12;
}

/** @brief 对一帧四通道音频执行完整定位。 */
static void sound_localization_process_frame(struct sound_localization_context *context,
                                             const audio_pipeline_frame_t *frame,
                                             sound_localization_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->source_sequence = frame->sequence;
    result->source_timestamp_us = frame->timestamp_us;
    result->rms = sound_localization_calculate_rms(frame);
    if (result->rms < context->config.minimum_rms) {
        return;
    }

    double ata_xx = 0.0;
    double ata_xy = 0.0;
    double ata_yy = 0.0;
    double atb_x = 0.0;
    double atb_y = 0.0;
    float correlation_sum = 0.0f;

    for (uint8_t channel = 1; channel < ES7210_AUDIO_CHANNEL_COUNT; ++channel) {
        const float dx = context->config.microphones[channel].x_m -
                         context->config.microphones[0].x_m;
        const float dy = context->config.microphones[channel].y_m -
                         context->config.microphones[0].y_m;
        const float distance = sqrtf(dx * dx + dy * dy);
        int maximum_lag = (int)ceilf(distance * frame->sample_rate_hz /
                                     context->config.speed_of_sound_mps) + 1;
        if (maximum_lag > SOUND_LOCALIZATION_MAX_LAG_SAMPLES) {
            maximum_lag = SOUND_LOCALIZATION_MAX_LAG_SAMPLES;
        }
        if (maximum_lag < 1) {
            maximum_lag = 1;
        }

        float peak = 0.0f;
        const float delay_samples = sound_localization_estimate_delay(
            frame, channel, maximum_lag, &peak);
        const float delay_seconds = delay_samples / frame->sample_rate_hz;
        result->delays_us[channel] = delay_seconds * 1000000.0f;
        correlation_sum += peak;

        /* 波从声源指向阵列传播，因此 A*u = -c*tau，u 指向声源。 */
        const double b = -context->config.speed_of_sound_mps * delay_seconds;
        ata_xx += dx * dx;
        ata_xy += dx * dy;
        ata_yy += dy * dy;
        atb_x += dx * b;
        atb_y += dy * b;
    }

    result->confidence = correlation_sum / (ES7210_AUDIO_CHANNEL_COUNT - 1U);
    if (result->confidence < context->config.minimum_correlation) {
        return;
    }

    const double determinant = ata_xx * ata_yy - ata_xy * ata_xy;
    if (determinant <= 1.0e-12) {
        return;
    }
    double direction_x = (ata_yy * atb_x - ata_xy * atb_y) / determinant;
    double direction_y = (ata_xx * atb_y - ata_xy * atb_x) / determinant;
    const double norm = sqrt(direction_x * direction_x + direction_y * direction_y);
    if (norm < 1.0e-6) {
        return;
    }
    direction_x /= norm;
    direction_y /= norm;

    float degrees = atan2f((float)direction_y, (float)direction_x) * 180.0f / PI_F;
    if (degrees < 0.0f) {
        degrees += 360.0f;
    }
    result->azimuth_degrees = degrees;
    result->valid = true;
}

/** @brief 定位后台任务，持续消费自己的订阅队列。 */
static void sound_localization_task(void *argument)
{
    struct sound_localization_context *context = argument;
    while (context->running) {
        const audio_pipeline_frame_t *frame = NULL;
        esp_err_t receive_result = audio_pipeline_receive(
            context->subscription, &frame, pdMS_TO_TICKS(100));
        if (receive_result == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (receive_result != ESP_OK) {
            ESP_LOGE(TAG, "接收音频帧失败: %s", esp_err_to_name(receive_result));
            continue;
        }

        sound_localization_result_t result;
        sound_localization_process_frame(context, frame, &result);
        audio_pipeline_release_frame(context->subscription, frame);

        /* 单元素队列只保留最近结果，监控方读取速度不会反向阻塞实时定位。 */
        xQueueOverwrite(context->result_queue, &result);
    }

    context->task = NULL;
    if (context->stop_waiter != NULL) {
        xTaskNotifyGive(context->stop_waiter);
    }
    vTaskDelete(NULL);
}

esp_err_t sound_localization_init(const sound_localization_config_t *config,
                                  sound_localization_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL && config->pipeline != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "定位配置无效");
    ESP_RETURN_ON_FALSE(config->speed_of_sound_mps > 0.0f &&
                            config->minimum_correlation >= 0.0f &&
                            config->minimum_correlation <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "声速或相关阈值无效");
    ESP_RETURN_ON_FALSE(sound_localization_geometry_is_valid(config->microphones),
                        ESP_ERR_INVALID_ARG, TAG, "麦克风坐标不能形成二维阵列");

    *out_handle = NULL;
    struct sound_localization_context *context = calloc(1, sizeof(*context));
    ESP_RETURN_ON_FALSE(context != NULL, ESP_ERR_NO_MEM, TAG, "无法分配定位上下文");
    context->config = *config;
    context->result_queue = xQueueCreate(1, sizeof(sound_localization_result_t));
    if (context->result_queue == NULL) {
        free(context);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = audio_pipeline_subscribe(config->pipeline, 2, &context->subscription);
    if (result != ESP_OK) {
        vQueueDelete(context->result_queue);
        free(context);
        return result;
    }
    *out_handle = context;
    return ESP_OK;
}

esp_err_t sound_localization_start(sound_localization_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "定位句柄为空");
    ESP_RETURN_ON_FALSE(!handle->running && handle->task == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "定位任务已经运行");
    handle->running = true;
    if (xTaskCreate(sound_localization_task,
                    "sound_localize",
                    handle->config.task_stack_size,
                    handle,
                    handle->config.task_priority,
                    &handle->task) != pdPASS) {
        handle->running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t sound_localization_wait_result(sound_localization_handle_t handle,
                                         sound_localization_result_t *out_result,
                                         TickType_t wait_ticks)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_result != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "结果接收参数无效");
    return xQueueReceive(handle->result_queue, out_result, wait_ticks) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

esp_err_t sound_localization_stop(sound_localization_handle_t handle,
                                  TickType_t wait_ticks)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "定位句柄为空");
    if (!handle->running) {
        return ESP_OK;
    }
    handle->stop_waiter = xTaskGetCurrentTaskHandle();
    handle->running = false;
    if (ulTaskNotifyTake(pdTRUE, wait_ticks) == 0) {
        handle->stop_waiter = NULL;
        return ESP_ERR_TIMEOUT;
    }
    handle->stop_waiter = NULL;
    return ESP_OK;
}

esp_err_t sound_localization_deinit(sound_localization_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "定位句柄为空");
    ESP_RETURN_ON_FALSE(!handle->running && handle->task == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "必须先停止定位任务");
    ESP_RETURN_ON_ERROR(audio_pipeline_unsubscribe(handle->subscription),
                        TAG, "删除定位订阅失败");
    vQueueDelete(handle->result_queue);
    free(handle);
    return ESP_OK;
}
