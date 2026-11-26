/**
 * @file sound_localization.c
 * @brief 基于三麦克风全配对 TDOA 和二维最小二乘的声源定位实现。
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

/**
 * 每次定位使用的最新采样点数。160 点在 16 kHz 下对应 10 ms，足以覆盖当前阵列
 * 最大约 3.3 个采样点的通道时差，同时把互相关计算量降到原 480 点窗口的三分之一。
 */
#define SOUND_LOCALIZATION_ANALYSIS_SAMPLES 160

/** 定位任务每处理一帧后至少阻塞一个系统 tick，保证单核 IDLE 任务能够运行。 */
#define SOUND_LOCALIZATION_YIELD_TICKS 1

/** 定位组件私有上下文。 */
struct sound_localization_context {
    sound_localization_config_t config;
    audio_pipeline_subscription_t *subscription;
    QueueHandle_t result_queue;
    TaskHandle_t task;
    TaskHandle_t stop_waiter;
    volatile bool running;
    bool direction_filter_initialized;
    float filtered_direction_x;
    float filtered_direction_y;
};

/** 三只阵列麦克风的全部无重复配对，顺序同时定义结果中的 pair_delays_us。 */
static const uint8_t SOUND_LOCALIZATION_PAIRS[SOUND_LOCALIZATION_PAIR_COUNT][2] = {
    {0, 1},
    {0, 2},
    {1, 2},
};

/** @brief 把浮点值限制到闭区间，避免数值误差产生越界置信度。 */
static float sound_localization_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

/** @brief 只计算三只定位麦克风的 RMS，独立麦克风不会触发定位。 */
static float sound_localization_calculate_rms(
    const sound_localization_config_t *config,
    const audio_pipeline_frame_t *frame,
    float out_microphone_rms[SOUND_LOCALIZATION_MICROPHONE_COUNT])
{
    uint64_t total_energy = 0;
    uint64_t microphone_energy[SOUND_LOCALIZATION_MICROPHONE_COUNT] = {0};
    for (size_t sample = 0; sample < frame->samples_per_channel; ++sample) {
        for (size_t microphone = 0;
             microphone < SOUND_LOCALIZATION_MICROPHONE_COUNT;
             ++microphone) {
            const uint8_t channel = config->channel_indices[microphone];
            /* 先扩展到 32 位再相乘，避免 int16_t 平方溢出；总能量使用 64 位累加。 */
            const int32_t value = frame->data[sample * frame->channel_count + channel];
            const uint64_t squared_value = (uint64_t)(value * value);
            microphone_energy[microphone] += squared_value;
            total_energy += squared_value;
        }
    }

    for (size_t microphone = 0;
         microphone < SOUND_LOCALIZATION_MICROPHONE_COUNT;
         ++microphone) {
        out_microphone_rms[microphone] =
            frame->samples_per_channel == 0
                ? 0.0f
                : sqrtf((float)microphone_energy[microphone] /
                        (float)frame->samples_per_channel);
    }
    const size_t value_count =
        (size_t)frame->samples_per_channel * SOUND_LOCALIZATION_MICROPHONE_COUNT;
    return value_count == 0 ? 0.0f
                            : sqrtf((float)total_energy / (float)value_count);
}

/**
 * @brief 计算参考麦克风与目标麦克风在指定整数延迟下的归一化互相关。
 *
 * lag 的定义为“目标通道到达时间减参考通道到达时间”。正值表示同一波形在
 * 目标通道中出现在更晚的采样点。计算时去除各自直流分量，避免 ADC 偏置制造假峰。
 */
static float sound_localization_correlation_at_lag(const audio_pipeline_frame_t *frame,
                                                   uint8_t reference_channel,
                                                   uint8_t target_channel,
                                                   int lag)
{
    const int analysis_count =
        frame->samples_per_channel < SOUND_LOCALIZATION_ANALYSIS_SAMPLES
            ? frame->samples_per_channel
            : SOUND_LOCALIZATION_ANALYSIS_SAMPLES;
    const int window_start = frame->samples_per_channel - analysis_count;
    const int begin = window_start + (lag < 0 ? -lag : 0);
    const int end = window_start + (lag > 0 ? analysis_count - lag : analysis_count);
    const int valid_count = end - begin;
    if (valid_count < 8) {
        return 0.0f;
    }

    int32_t reference_sum = 0;
    int32_t target_sum = 0;
    int64_t reference_square_sum = 0;
    int64_t target_square_sum = 0;
    int64_t product_sum = 0;

    /*
     * 使用一次遍历收集 Σx、Σy、Σx²、Σy²、Σxy，再通过恒等式计算去均值相关：
     * numerator = N*Σxy - Σx*Σy
     * energy_x = N*Σx² - (Σx)²
     * 采样值先以 32 位相乘，再累加到 64 位，既保持 PCM16 全精度，也避免 ESP32-C3
     * 在每个采样点执行软件双精度乘法。
     */
    for (int index = begin; index < end; ++index) {
        const int32_t reference =
            frame->data[index * frame->channel_count + reference_channel];
        const int32_t target =
            frame->data[(index + lag) * frame->channel_count + target_channel];
        reference_sum += reference;
        target_sum += target;
        reference_square_sum += reference * reference;
        target_square_sum += target * target;
        product_sum += reference * target;
    }

    const int64_t numerator = (int64_t)valid_count * product_sum -
                              (int64_t)reference_sum * target_sum;
    const int64_t reference_energy =
        (int64_t)valid_count * reference_square_sum -
        (int64_t)reference_sum * reference_sum;
    const int64_t target_energy =
        (int64_t)valid_count * target_square_sum - (int64_t)target_sum * target_sum;
    if (reference_energy <= 0 || target_energy <= 0) {
        return 0.0f;
    }

    const float denominator = sqrtf((float)reference_energy * (float)target_energy);
    return denominator <= 1.0f ? 0.0f : (float)numerator / denominator;
}

/**
 * @brief 搜索相关峰并通过三点抛物线插值获得亚采样延迟。
 *
 * 亚采样插值显著降低 16 kHz 采样下整数点量化对小尺寸麦克风阵列的影响。
 */
static float sound_localization_estimate_delay(const audio_pipeline_frame_t *frame,
                                               uint8_t reference_channel,
                                               uint8_t target_channel,
                                               int maximum_lag,
                                               float *out_peak)
{
    float correlations[2 * SOUND_LOCALIZATION_MAX_LAG_SAMPLES + 1] = {0};
    int best_lag = 0;
    float best_correlation = -1.0f;
    for (int lag = -maximum_lag; lag <= maximum_lag; ++lag) {
        const float correlation = sound_localization_correlation_at_lag(
            frame, reference_channel, target_channel, lag);
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
    const sound_localization_microphone_position_t
        microphones[SOUND_LOCALIZATION_MICROPHONE_COUNT])
{
    const float dx1 = microphones[1].x_m - microphones[0].x_m;
    const float dy1 = microphones[1].y_m - microphones[0].y_m;
    const float dx2 = microphones[2].x_m - microphones[0].x_m;
    const float dy2 = microphones[2].y_m - microphones[0].y_m;

    /* 叉积是三角形面积的两倍；非零即表示三个声学中心不共线。 */
    return fabsf(dx1 * dy2 - dy1 * dx2) > 1.0e-8f;
}

/** @brief 验证三只定位麦克风映射到三个不同且存在的 TDM 通道。 */
static bool sound_localization_channels_are_valid(
    const uint8_t channel_indices[SOUND_LOCALIZATION_MICROPHONE_COUNT])
{
    for (size_t index = 0; index < SOUND_LOCALIZATION_MICROPHONE_COUNT; ++index) {
        if (channel_indices[index] >= ES7210_AUDIO_CHANNEL_COUNT) {
            return false;
        }
        for (size_t previous = 0; previous < index; ++previous) {
            if (channel_indices[index] == channel_indices[previous]) {
                return false;
            }
        }
    }
    return true;
}

/** @brief 对一帧四通道音频中的三只阵列麦克风执行完整定位。 */
static void sound_localization_process_frame(struct sound_localization_context *context,
                                             const audio_pipeline_frame_t *frame,
                                             sound_localization_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->rejected_pair_index = UINT8_MAX;
    result->source_sequence = frame->sequence;
    result->source_timestamp_us = frame->timestamp_us;
    if (frame->channel_count < ES7210_AUDIO_CHANNEL_COUNT ||
        frame->sample_rate_hz == 0 || frame->samples_per_channel == 0) {
        result->rejection_reason = SOUND_LOCALIZATION_REJECTION_INVALID_FRAME;
        return;
    }

    result->rms = sound_localization_calculate_rms(
        &context->config, frame, result->microphone_rms);
    if (result->rms < context->config.minimum_rms) {
        result->rejection_reason = SOUND_LOCALIZATION_REJECTION_LOW_ENERGY;
        return;
    }

    float ata_xx = 0.0f;
    float ata_xy = 0.0f;
    float ata_yy = 0.0f;
    float atb_x = 0.0f;
    float atb_y = 0.0f;
    float correlation_sum = 0.0f;
    float delay_samples[SOUND_LOCALIZATION_PAIR_COUNT] = {0};

    for (size_t pair = 0; pair < SOUND_LOCALIZATION_PAIR_COUNT; ++pair) {
        const uint8_t reference_microphone = SOUND_LOCALIZATION_PAIRS[pair][0];
        const uint8_t target_microphone = SOUND_LOCALIZATION_PAIRS[pair][1];
        const uint8_t reference_channel =
            context->config.channel_indices[reference_microphone];
        const uint8_t target_channel = context->config.channel_indices[target_microphone];
        const float dx = context->config.microphones[target_microphone].x_m -
                         context->config.microphones[reference_microphone].x_m;
        const float dy = context->config.microphones[target_microphone].y_m -
                         context->config.microphones[reference_microphone].y_m;
        const float distance = sqrtf(dx * dx + dy * dy);
        const float physical_delay_samples =
            distance * frame->sample_rate_hz / context->config.speed_of_sound_mps;

        /*
         * 整数互相关必须覆盖物理时差上限所在的最邻近采样点，因此向上取整即可。
         * 原实现额外加 1 会把搜索范围扩展到物理阵列之外并制造边界假峰。
         */
        int maximum_lag = (int)ceilf(physical_delay_samples);
        if (maximum_lag > SOUND_LOCALIZATION_MAX_LAG_SAMPLES) {
            maximum_lag = SOUND_LOCALIZATION_MAX_LAG_SAMPLES;
        }
        if (maximum_lag < 1) {
            maximum_lag = 1;
        }

        float peak = 0.0f;
        delay_samples[pair] = sound_localization_estimate_delay(
            frame, reference_channel, target_channel, maximum_lag, &peak);
        result->pair_correlations[pair] = peak;
        const float delay_seconds = delay_samples[pair] / frame->sample_rate_hz;
        result->pair_delays_us[pair] = delay_seconds * 1000000.0f;
        correlation_sum += peak;

        /*
         * 亚采样插值的量化允许量取半个采样点。超过“物理时差 + 半采样”的结果
         * 不可能由当前麦克风间距产生，必须在进入方向方程前拒绝。
         */
        if (result->rejection_reason == SOUND_LOCALIZATION_REJECTION_NONE &&
            peak < context->config.minimum_correlation) {
            result->rejection_reason = SOUND_LOCALIZATION_REJECTION_LOW_CORRELATION;
            result->rejected_pair_index = (uint8_t)pair;
        } else if (result->rejection_reason == SOUND_LOCALIZATION_REJECTION_NONE &&
                   fabsf(delay_samples[pair]) > physical_delay_samples + 0.5f) {
            result->rejection_reason = SOUND_LOCALIZATION_REJECTION_PHYSICAL_DELAY;
            result->rejected_pair_index = (uint8_t)pair;
        }

        /* 波从声源指向阵列传播，因此 A*u = -c*tau，u 指向声源。 */
        const float b = -context->config.speed_of_sound_mps * delay_seconds;
        ata_xx += dx * dx;
        ata_xy += dx * dy;
        ata_yy += dy * dy;
        atb_x += dx * b;
        atb_y += dy * b;
    }

    result->confidence = correlation_sum / SOUND_LOCALIZATION_PAIR_COUNT;
    if (result->rejection_reason != SOUND_LOCALIZATION_REJECTION_NONE) {
        return;
    }

    /*
     * 三对时差必须满足 tau01 + tau12 = tau02。该闭环约束不依赖声源方向，
     * 能直接拦截混响、噪声或错误相关峰造成的互相矛盾时差。
     */
    const float closure_error_samples =
        delay_samples[0] + delay_samples[2] - delay_samples[1];
    result->closure_error_us =
        closure_error_samples * 1000000.0f / frame->sample_rate_hz;
    if (fabsf(closure_error_samples) >
        context->config.maximum_closure_error_samples) {
        result->rejection_reason = SOUND_LOCALIZATION_REJECTION_CLOSURE_ERROR;
        return;
    }

    const float determinant = ata_xx * ata_yy - ata_xy * ata_xy;
    if (determinant <= 1.0e-12f) {
        result->rejection_reason = SOUND_LOCALIZATION_REJECTION_GEOMETRY;
        return;
    }
    float direction_x = (ata_yy * atb_x - ata_xy * atb_y) / determinant;
    float direction_y = (ata_xx * atb_y - ata_xy * atb_x) / determinant;
    const float norm = sqrtf(direction_x * direction_x + direction_y * direction_y);
    result->direction_norm = norm;
    if (norm < context->config.minimum_direction_norm ||
        norm > context->config.maximum_direction_norm) {
        result->rejection_reason = SOUND_LOCALIZATION_REJECTION_DIRECTION_NORM;
        return;
    }
    direction_x /= norm;
    direction_y /= norm;

    /*
     * 在单位圆上对方向向量做一阶低通，而不是直接平均角度。这样跨越 0/360 度时
     * 不会错误地跳到 180 度，同时能抑制单帧亚采样估计造成的角度抖动。
     */
    if (!context->direction_filter_initialized) {
        context->filtered_direction_x = direction_x;
        context->filtered_direction_y = direction_y;
        context->direction_filter_initialized = true;
    } else {
        const float alpha = context->config.smoothing_factor;
        const float filtered_x = (1.0f - alpha) * context->filtered_direction_x +
                                 alpha * direction_x;
        const float filtered_y = (1.0f - alpha) * context->filtered_direction_y +
                                 alpha * direction_y;
        const float filtered_norm = sqrtf(filtered_x * filtered_x + filtered_y * filtered_y);
        if (filtered_norm > 1.0e-6f) {
            context->filtered_direction_x = filtered_x / filtered_norm;
            context->filtered_direction_y = filtered_y / filtered_norm;
        } else {
            /* 前后两个方向完全相反时以最新测量重新建立滤波状态。 */
            context->filtered_direction_x = direction_x;
            context->filtered_direction_y = direction_y;
        }
    }

    float degrees = atan2f(context->filtered_direction_y,
                           context->filtered_direction_x) *
                    180.0f / PI_F;
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

        /*
         * 定位结果只对实时方向有意义。若计算期间队列中已经到达更新帧，归还旧帧并
         * 保留最新帧，防止持续说话时逐帧追赶历史数据而形成不可恢复的处理积压。
         */
        while (true) {
            const audio_pipeline_frame_t *newer_frame = NULL;
            esp_err_t drain_result = audio_pipeline_receive(
                context->subscription, &newer_frame, 0);
            if (drain_result == ESP_ERR_TIMEOUT) {
                break;
            }
            if (drain_result != ESP_OK) {
                ESP_LOGE(TAG, "清理过时音频帧失败: %s", esp_err_to_name(drain_result));
                break;
            }
            audio_pipeline_release_frame(context->subscription, frame);
            frame = newer_frame;
        }

        sound_localization_result_t result;
        sound_localization_process_frame(context, frame, &result);
        audio_pipeline_release_frame(context->subscription, frame);

        /* 单元素队列只保留最近结果，监控方读取速度不会反向阻塞实时定位。 */
        xQueueOverwrite(context->result_queue, &result);

        /*
         * ESP32-C3 为单核处理器。即使声音连续触发定位，也必须短暂阻塞定位任务，
         * 让优先级 0 的 IDLE 任务运行并复位任务看门狗。
         */
        vTaskDelay(SOUND_LOCALIZATION_YIELD_TICKS);
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
                            config->minimum_rms >= 0.0f &&
                            config->minimum_correlation >= 0.0f &&
                            config->minimum_correlation <= 1.0f &&
                            config->maximum_closure_error_samples >= 0.0f &&
                            config->minimum_direction_norm > 0.0f &&
                            config->maximum_direction_norm >=
                                config->minimum_direction_norm &&
                            config->smoothing_factor > 0.0f &&
                            config->smoothing_factor <= 1.0f &&
                            config->task_stack_size > 0,
                        ESP_ERR_INVALID_ARG, TAG, "定位阈值或任务参数无效");
    ESP_RETURN_ON_FALSE(
        sound_localization_channels_are_valid(config->channel_indices),
        ESP_ERR_INVALID_ARG, TAG, "定位麦克风通道必须互不相同且位于四通道范围内");
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
    /* 每次重新启动都从首个有效方向重新建立滤波状态，避免继承停止前的旧角度。 */
    handle->direction_filter_initialized = false;
    handle->filtered_direction_x = 0.0f;
    handle->filtered_direction_y = 0.0f;
    xQueueReset(handle->result_queue);
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
