/**
 * @file audio_pipeline.c
 * @brief 单生产者、多订阅者的零拷贝音频中间层实现。
 */

#include "audio_pipeline.h"

#include <stdbool.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "audio_pipeline";

/** 公共帧位于结构体首地址，因此公共帧指针可以安全转换回池帧指针。 */
typedef struct {
    audio_pipeline_frame_t frame;
    uint8_t reference_count;
} audio_pipeline_pool_frame_t;

/** 每个订阅者拥有独立队列；队列满只丢弃该订阅者的新帧，不阻塞采集。 */
struct audio_pipeline_subscription {
    struct audio_pipeline_context *owner;
    QueueHandle_t queue;
    bool active;
};

/** 中间层所有状态都集中在实例中，后续可以建立多个独立音频源。 */
struct audio_pipeline_context {
    audio_pipeline_config_t config;
    audio_pipeline_pool_frame_t *frame_pool;
    QueueHandle_t free_frames;
    SemaphoreHandle_t subscriber_mutex;
    portMUX_TYPE reference_lock;
    struct audio_pipeline_subscription subscribers[AUDIO_PIPELINE_MAX_SUBSCRIBERS];
    TaskHandle_t capture_task;
    TaskHandle_t stop_waiter;
    volatile bool running;
    uint32_t next_sequence;
};

/** @brief 引用数归零时将帧送回空闲队列。 */
static void audio_pipeline_release_pool_frame(struct audio_pipeline_context *pipeline,
                                              audio_pipeline_pool_frame_t *pool_frame)
{
    bool return_to_pool = false;
    portENTER_CRITICAL(&pipeline->reference_lock);
    if (pool_frame->reference_count > 0) {
        --pool_frame->reference_count;
        return_to_pool = pool_frame->reference_count == 0;
    }
    portEXIT_CRITICAL(&pipeline->reference_lock);

    if (return_to_pool) {
        (void)xQueueSend(pipeline->free_frames, &pool_frame, portMAX_DELAY);
    }
}

/** @brief 把同一帧引用非阻塞地发送到所有活动订阅者。 */
static void audio_pipeline_publish(struct audio_pipeline_context *pipeline,
                                   audio_pipeline_pool_frame_t *pool_frame)
{
    /*
     * 发布过程自身先持有一个临时引用。即使高优先级消费者立即收取并归还首个
     * 队列引用，帧也不会在生产者尚未遍历完订阅表时提前回到空闲池。
     */
    portENTER_CRITICAL(&pipeline->reference_lock);
    pool_frame->reference_count = 1;
    portEXIT_CRITICAL(&pipeline->reference_lock);

    xSemaphoreTake(pipeline->subscriber_mutex, portMAX_DELAY);
    for (size_t index = 0; index < AUDIO_PIPELINE_MAX_SUBSCRIBERS; ++index) {
        struct audio_pipeline_subscription *subscriber = &pipeline->subscribers[index];
        if (subscriber->active) {
            /*
             * 必须先增加引用再把指针放入队列。高优先级消费者可以在 xQueueSend()
             * 返回前抢占生产者；先计数可保证消费者归还时永远看见有效引用。
             */
            portENTER_CRITICAL(&pipeline->reference_lock);
            ++pool_frame->reference_count;
            portEXIT_CRITICAL(&pipeline->reference_lock);
            if (xQueueSend(subscriber->queue, &pool_frame, 0) != pdTRUE) {
                portENTER_CRITICAL(&pipeline->reference_lock);
                --pool_frame->reference_count;
                portEXIT_CRITICAL(&pipeline->reference_lock);
            }
        }
    }
    xSemaphoreGive(pipeline->subscriber_mutex);

    /* 释放生产者临时引用；没有成功投递时该操作会立即把帧送回空闲池。 */
    audio_pipeline_release_pool_frame(pipeline, pool_frame);
}

/** @brief 连续从 ES7210 读取完整帧并向应用分发。 */
static void audio_pipeline_capture_task(void *argument)
{
    struct audio_pipeline_context *pipeline = argument;
    while (pipeline->running) {
        audio_pipeline_pool_frame_t *pool_frame = NULL;
        if (xQueueReceive(pipeline->free_frames, &pool_frame, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        esp_err_t result = es7210_audio_read(
            pipeline->config.driver,
            pool_frame->frame.data,
            AUDIO_PIPELINE_SAMPLES_PER_CHANNEL);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "ES7210 音频读取失败: %s", esp_err_to_name(result));
            (void)xQueueSend(pipeline->free_frames, &pool_frame, portMAX_DELAY);
            continue;
        }

        pool_frame->frame.sequence = pipeline->next_sequence++;
        pool_frame->frame.timestamp_us = esp_timer_get_time();
        pool_frame->frame.sample_rate_hz =
            es7210_audio_get_sample_rate(pipeline->config.driver);
        pool_frame->frame.samples_per_channel = AUDIO_PIPELINE_SAMPLES_PER_CHANNEL;
        pool_frame->frame.channel_count = ES7210_AUDIO_CHANNEL_COUNT;
        audio_pipeline_publish(pipeline, pool_frame);
    }

    pipeline->capture_task = NULL;
    if (pipeline->stop_waiter != NULL) {
        xTaskNotifyGive(pipeline->stop_waiter);
    }
    vTaskDelete(NULL);
}

esp_err_t audio_pipeline_init(const audio_pipeline_config_t *config,
                              audio_pipeline_handle_t *out_pipeline)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_pipeline != NULL && config->driver != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "中间层配置无效");
    ESP_RETURN_ON_FALSE(config->frame_pool_size >= 3 && config->frame_pool_size <= 12,
                        ESP_ERR_INVALID_ARG, TAG, "帧池大小必须在 3 到 12 之间");

    *out_pipeline = NULL;
    struct audio_pipeline_context *pipeline = calloc(1, sizeof(*pipeline));
    ESP_RETURN_ON_FALSE(pipeline != NULL, ESP_ERR_NO_MEM, TAG, "无法分配中间层上下文");
    pipeline->config = *config;
    pipeline->reference_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    pipeline->frame_pool = calloc(config->frame_pool_size, sizeof(*pipeline->frame_pool));
    pipeline->free_frames = xQueueCreate(config->frame_pool_size,
                                         sizeof(audio_pipeline_pool_frame_t *));
    pipeline->subscriber_mutex = xSemaphoreCreateMutex();
    if (pipeline->frame_pool == NULL || pipeline->free_frames == NULL ||
        pipeline->subscriber_mutex == NULL) {
        audio_pipeline_deinit(pipeline);
        return ESP_ERR_NO_MEM;
    }

    for (size_t index = 0; index < config->frame_pool_size; ++index) {
        audio_pipeline_pool_frame_t *frame = &pipeline->frame_pool[index];
        (void)xQueueSend(pipeline->free_frames, &frame, 0);
    }
    *out_pipeline = pipeline;
    return ESP_OK;
}

esp_err_t audio_pipeline_subscribe(audio_pipeline_handle_t pipeline,
                                   uint8_t queue_depth,
                                   audio_pipeline_subscription_t **out_subscription)
{
    ESP_RETURN_ON_FALSE(pipeline != NULL && out_subscription != NULL && queue_depth > 0 &&
                            queue_depth < pipeline->config.frame_pool_size,
                        ESP_ERR_INVALID_ARG, TAG, "订阅参数无效");
    *out_subscription = NULL;

    xSemaphoreTake(pipeline->subscriber_mutex, portMAX_DELAY);
    for (size_t index = 0; index < AUDIO_PIPELINE_MAX_SUBSCRIBERS; ++index) {
        struct audio_pipeline_subscription *subscriber = &pipeline->subscribers[index];
        if (!subscriber->active && subscriber->queue == NULL) {
            subscriber->queue = xQueueCreate(queue_depth, sizeof(audio_pipeline_pool_frame_t *));
            if (subscriber->queue == NULL) {
                xSemaphoreGive(pipeline->subscriber_mutex);
                return ESP_ERR_NO_MEM;
            }
            subscriber->owner = pipeline;
            subscriber->active = true;
            *out_subscription = subscriber;
            xSemaphoreGive(pipeline->subscriber_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(pipeline->subscriber_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t audio_pipeline_start(audio_pipeline_handle_t pipeline)
{
    ESP_RETURN_ON_FALSE(pipeline != NULL, ESP_ERR_INVALID_ARG, TAG, "中间层句柄为空");
    ESP_RETURN_ON_FALSE(!pipeline->running && pipeline->capture_task == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "采集任务已经运行");

    pipeline->running = true;
    BaseType_t created = xTaskCreate(audio_pipeline_capture_task,
                                     "audio_capture",
                                     pipeline->config.task_stack_size,
                                     pipeline,
                                     pipeline->config.task_priority,
                                     &pipeline->capture_task);
    if (created != pdPASS) {
        pipeline->running = false;
        pipeline->capture_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t audio_pipeline_receive(audio_pipeline_subscription_t *subscription,
                                 const audio_pipeline_frame_t **out_frame,
                                 TickType_t wait_ticks)
{
    ESP_RETURN_ON_FALSE(subscription != NULL && out_frame != NULL && subscription->active,
                        ESP_ERR_INVALID_ARG, TAG, "订阅接收参数无效");
    audio_pipeline_pool_frame_t *pool_frame = NULL;
    if (xQueueReceive(subscription->queue, &pool_frame, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out_frame = &pool_frame->frame;
    return ESP_OK;
}

void audio_pipeline_release_frame(audio_pipeline_subscription_t *subscription,
                                  const audio_pipeline_frame_t *frame)
{
    if (subscription == NULL || frame == NULL || subscription->owner == NULL) {
        return;
    }
    audio_pipeline_pool_frame_t *pool_frame = (audio_pipeline_pool_frame_t *)frame;
    audio_pipeline_release_pool_frame(subscription->owner, pool_frame);
}

esp_err_t audio_pipeline_stop(audio_pipeline_handle_t pipeline, TickType_t wait_ticks)
{
    ESP_RETURN_ON_FALSE(pipeline != NULL, ESP_ERR_INVALID_ARG, TAG, "中间层句柄为空");
    if (!pipeline->running) {
        return ESP_OK;
    }

    pipeline->stop_waiter = xTaskGetCurrentTaskHandle();
    pipeline->running = false;
    if (ulTaskNotifyTake(pdTRUE, wait_ticks) == 0) {
        pipeline->stop_waiter = NULL;
        return ESP_ERR_TIMEOUT;
    }
    pipeline->stop_waiter = NULL;
    return ESP_OK;
}

esp_err_t audio_pipeline_unsubscribe(audio_pipeline_subscription_t *subscription)
{
    ESP_RETURN_ON_FALSE(subscription != NULL && subscription->owner != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "订阅句柄为空");
    struct audio_pipeline_context *pipeline = subscription->owner;

    xSemaphoreTake(pipeline->subscriber_mutex, portMAX_DELAY);
    subscription->active = false;
    xSemaphoreGive(pipeline->subscriber_mutex);

    audio_pipeline_pool_frame_t *pool_frame = NULL;
    while (xQueueReceive(subscription->queue, &pool_frame, 0) == pdTRUE) {
        audio_pipeline_release_pool_frame(pipeline, pool_frame);
    }
    vQueueDelete(subscription->queue);
    subscription->queue = NULL;
    subscription->owner = NULL;
    return ESP_OK;
}

esp_err_t audio_pipeline_deinit(audio_pipeline_handle_t pipeline)
{
    ESP_RETURN_ON_FALSE(pipeline != NULL, ESP_ERR_INVALID_ARG, TAG, "中间层句柄为空");
    ESP_RETURN_ON_FALSE(!pipeline->running && pipeline->capture_task == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "必须先停止采集任务");

    for (size_t index = 0; index < AUDIO_PIPELINE_MAX_SUBSCRIBERS; ++index) {
        ESP_RETURN_ON_FALSE(!pipeline->subscribers[index].active &&
                                pipeline->subscribers[index].queue == NULL,
                            ESP_ERR_INVALID_STATE, TAG, "必须先删除全部订阅");
    }
    if (pipeline->free_frames != NULL) {
        vQueueDelete(pipeline->free_frames);
    }
    if (pipeline->subscriber_mutex != NULL) {
        vSemaphoreDelete(pipeline->subscriber_mutex);
    }
    free(pipeline->frame_pool);
    free(pipeline);
    return ESP_OK;
}
