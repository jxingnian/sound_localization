/**
 * @file es7210_audio.c
 * @brief 基于 ESP-IDF TDM 驱动和 Espressif 官方 esp_codec_dev 的 ES7210 驱动层。
 */

#include "es7210_audio.h"

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>

#include "audio_codec_ctrl_if.h"
#include "audio_codec_data_if.h"
#include "audio_codec_if.h"
#include "driver/i2s_tdm.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev_vol.h"
#include "esp_log.h"

static const char *TAG = "es7210_audio";

/**
 * @brief 驱动私有上下文。
 *
 * 分别保存每一层接口句柄，确保初始化中途失败或正常退出时都能按依赖逆序释放，
 * 避免 I2C、I2S 和编解码器对象残留。
 */
struct es7210_audio_context {
    es7210_audio_config_t config;
    i2c_master_bus_handle_t i2c_bus;
    i2s_chan_handle_t i2s_rx_channel;
    const audio_codec_ctrl_if_t *control_interface;
    const audio_codec_data_if_t *data_interface;
    const audio_codec_if_t *codec_interface;
    esp_codec_dev_handle_t codec_device;
    bool codec_opened;
};

/** @brief 释放已经成功创建的资源，允许处理半初始化上下文。 */
static void es7210_audio_release_resources(struct es7210_audio_context *context)
{
    if (context == NULL) {
        return;
    }

    if (context->codec_opened && context->codec_device != NULL) {
        (void)esp_codec_dev_close(context->codec_device);
        context->codec_opened = false;
    }
    if (context->codec_device != NULL) {
        esp_codec_dev_delete(context->codec_device);
    }
    if (context->codec_interface != NULL) {
        (void)audio_codec_delete_codec_if(context->codec_interface);
    }
    if (context->data_interface != NULL) {
        (void)audio_codec_delete_data_if(context->data_interface);
    }
    if (context->control_interface != NULL) {
        (void)audio_codec_delete_ctrl_if(context->control_interface);
    }
    if (context->i2s_rx_channel != NULL) {
        (void)i2s_del_channel(context->i2s_rx_channel);
    }
    if (context->i2c_bus != NULL) {
        (void)i2c_del_master_bus(context->i2c_bus);
    }
    free(context);
}

esp_err_t es7210_audio_init(const es7210_audio_config_t *config,
                            es7210_audio_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "配置或输出句柄为空");
    ESP_RETURN_ON_FALSE(config->sample_rate_hz > 0 && config->dma_frame_samples > 0,
                        ESP_ERR_INVALID_ARG, TAG, "采样率或 DMA 帧长度无效");

    *out_handle = NULL;
    /* ESP_GOTO_ON_* 系列宏约定错误变量名必须为 ret。 */
    esp_err_t ret = ESP_OK;
    struct es7210_audio_context *context = calloc(1, sizeof(*context));
    ESP_RETURN_ON_FALSE(context != NULL, ESP_ERR_NO_MEM, TAG, "无法分配驱动上下文");
    context->config = *config;

    /* ESP32-C3 作为 TDM 主机产生 MCLK、BCLK 和 LRCK，ES7210 作为从机同步采样。 */
    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(config->i2s_port,
                                                                  I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 6;
    channel_config.dma_frame_num = config->dma_frame_samples;
    ret = i2s_new_channel(&channel_config, NULL, &context->i2s_rx_channel);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "创建 I2S 接收通道失败");

    i2s_tdm_config_t tdm_config = {
        .clk_cfg = {
            .sample_rate_hz = config->sample_rate_hz,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,
        },
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO,
            I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
        .gpio_cfg = {
            .mclk = config->i2s_mclk_gpio,
            .bclk = config->i2s_bclk_gpio,
            .ws = config->i2s_lrck_gpio,
            .dout = GPIO_NUM_NC,
            .din = config->i2s_data_in_gpio,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ret = i2s_channel_init_tdm_mode(context->i2s_rx_channel, &tdm_config);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "配置四通道 Philips TDM 失败");

    /* 原理图已经放置 4.7 kOhm 上拉，内部上拉只作为上电阶段的补充。 */
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->i2c_sda_gpio,
        .scl_io_num = config->i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&i2c_config, &context->i2c_bus);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "创建 ES7210 I2C 总线失败");

    audio_codec_i2c_cfg_t codec_i2c_config = {
        .port = config->i2c_port,
        .addr = config->i2c_address,
        .bus_handle = context->i2c_bus,
    };
    context->control_interface = audio_codec_new_i2c_ctrl(&codec_i2c_config);
    ESP_GOTO_ON_FALSE(context->control_interface != NULL, ESP_ERR_NO_MEM, fail,
                      TAG, "创建编解码器控制接口失败");

    audio_codec_i2s_cfg_t codec_i2s_config = {
        .port = config->i2s_port,
        .rx_handle = context->i2s_rx_channel,
        .tx_handle = NULL,
    };
    context->data_interface = audio_codec_new_i2s_data(&codec_i2s_config);
    ESP_GOTO_ON_FALSE(context->data_interface != NULL, ESP_ERR_NO_MEM, fail,
                      TAG, "创建编解码器数据接口失败");

    es7210_codec_cfg_t es7210_config = {
        .ctrl_if = context->control_interface,
        .master_mode = false,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 |
                        ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
        .mclk_src = ES7210_MCLK_FROM_PAD,
        .mclk_div = I2S_MCLK_MULTIPLE_256,
    };
    context->codec_interface = es7210_codec_new(&es7210_config);
    ESP_GOTO_ON_FALSE(context->codec_interface != NULL, ESP_ERR_NO_MEM, fail,
                      TAG, "创建 ES7210 编解码器接口失败");

    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = context->codec_interface,
        .data_if = context->data_interface,
    };
    context->codec_device = esp_codec_dev_new(&device_config);
    ESP_GOTO_ON_FALSE(context->codec_device != NULL, ESP_ERR_NO_MEM, fail,
                      TAG, "创建 ES7210 设备失败");

    esp_codec_dev_sample_info_t sample_config = {
        .sample_rate = config->sample_rate_hz,
        .channel = ES7210_AUDIO_CHANNEL_COUNT,
        .bits_per_sample = 16,
        .channel_mask = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 |
                        ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
    };
    ESP_GOTO_ON_FALSE(esp_codec_dev_open(context->codec_device, &sample_config) ==
                          ESP_CODEC_DEV_OK,
                      ESP_FAIL, fail, TAG, "打开 ES7210 四通道采集失败");
    context->codec_opened = true;

    ESP_GOTO_ON_FALSE(esp_codec_dev_set_in_gain(context->codec_device,
                                                 config->microphone_gain_db) ==
                          ESP_CODEC_DEV_OK,
                      ESP_FAIL, fail, TAG, "设置 ES7210 麦克风增益失败");

    ESP_LOGI(TAG,
             "ES7210 已就绪: 地址=0x%02X, %" PRIu32 " Hz, 16 bit, 4 通道 TDM",
             config->i2c_address, config->sample_rate_hz);
    *out_handle = context;
    return ESP_OK;

fail:
    es7210_audio_release_resources(context);
    return ret;
}

esp_err_t es7210_audio_read(es7210_audio_handle_t handle,
                            int16_t *interleaved_pcm,
                            size_t samples_per_channel)
{
    ESP_RETURN_ON_FALSE(handle != NULL && interleaved_pcm != NULL &&
                            samples_per_channel > 0,
                        ESP_ERR_INVALID_ARG, TAG, "读取参数无效");

    const size_t byte_count = samples_per_channel * ES7210_AUDIO_CHANNEL_COUNT *
                              sizeof(interleaved_pcm[0]);
    ESP_RETURN_ON_FALSE(byte_count <= INT_MAX, ESP_ERR_INVALID_SIZE,
                        TAG, "单次读取长度超过驱动限制");
    return esp_codec_dev_read(handle->codec_device, interleaved_pcm,
                              (int)byte_count) == ESP_CODEC_DEV_OK
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t es7210_audio_deinit(es7210_audio_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "驱动句柄为空");
    es7210_audio_release_resources(handle);
    return ESP_OK;
}

uint32_t es7210_audio_get_sample_rate(es7210_audio_handle_t handle)
{
    return handle == NULL ? 0U : handle->config.sample_rate_hz;
}
