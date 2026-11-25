/**
 * @file es7210_audio.h
 * @brief ES7210 四通道音频采集驱动的公开接口。
 *
 * 本组件只负责硬件初始化和同步音频采集，不包含业务算法。上层通过
 * es7210_audio_read() 获取固定顺序的四通道交织 PCM 数据。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "driver/i2s_types.h"
#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ES7210 本机启用的模拟麦克风通道数。 */
#define ES7210_AUDIO_CHANNEL_COUNT 4U

/** ES7210 驱动实例的不透明句柄，调用方不能访问其内部资源。 */
typedef struct es7210_audio_context *es7210_audio_handle_t;

/**
 * @brief ES7210 硬件和采样参数。
 *
 * 引脚必须与原理图网络标签一致。当前板卡连接为 SDA=GPIO4、SCL=GPIO5、
 * MCLK=GPIO6、BCLK=GPIO7、LRCK=GPIO10、DIN=GPIO3，7 位地址为 0x41。
 */
typedef struct {
    i2c_port_num_t i2c_port;       /**< 配置 ES7210 寄存器所用的 I2C 控制器。 */
    int i2s_port;                  /**< 接收四通道 TDM 数据所用的 I2S 控制器编号。 */
    gpio_num_t i2c_sda_gpio;       /**< I2C 数据引脚，对应 ES7210 CDATA。 */
    gpio_num_t i2c_scl_gpio;       /**< I2C 时钟引脚，对应 ES7210 CCLK。 */
    gpio_num_t i2s_mclk_gpio;      /**< 主时钟输出引脚，对应 ES7210 MCLK。 */
    gpio_num_t i2s_bclk_gpio;      /**< TDM 位时钟输出引脚，对应 ES7210 SCLK。 */
    gpio_num_t i2s_lrck_gpio;      /**< TDM 帧时钟输出引脚，对应 ES7210 LRCK。 */
    gpio_num_t i2s_data_in_gpio;   /**< TDM 数据输入引脚，对应 ES7210 SDOUT1。 */
    uint8_t i2c_address_7bit;      /**< ES7210 的 7 位 I2C 地址，不包含读写方向位。 */
    uint32_t sample_rate_hz;       /**< 每个麦克风通道的采样率。 */
    float microphone_gain_db;      /**< 四个模拟麦克风通道的统一前置增益。 */
    size_t dma_frame_samples;      /**< 单个 DMA 描述符容纳的每通道采样点数。 */
} es7210_audio_config_t;

/**
 * @brief 生成与当前原理图完全一致的默认配置。
 *
 * 默认采样格式为 16 kHz、16 bit、有符号 PCM、四通道 Philips TDM。
 */
#define ES7210_AUDIO_DEFAULT_CONFIG()            \
    {                                            \
        .i2c_port = I2C_NUM_0,                   \
        .i2s_port = I2S_NUM_0,                   \
        .i2c_sda_gpio = GPIO_NUM_4,              \
        .i2c_scl_gpio = GPIO_NUM_5,              \
        .i2s_mclk_gpio = GPIO_NUM_6,             \
        .i2s_bclk_gpio = GPIO_NUM_7,             \
        .i2s_lrck_gpio = GPIO_NUM_10,            \
        .i2s_data_in_gpio = GPIO_NUM_3,           \
        .i2c_address_7bit = 0x41,                 \
        .sample_rate_hz = 16000,                  \
        .microphone_gain_db = 30.0f,              \
        .dma_frame_samples = 480,                 \
    }

/**
 * @brief 初始化 I2C、I2S TDM 和 ES7210。
 *
 * @param[in] config 硬件及采样配置，函数返回后可以释放该对象。
 * @param[out] out_handle 创建成功的驱动句柄。
 * @return ESP_OK 表示硬件和编解码器均已打开，其余值表示明确的初始化错误。
 */
esp_err_t es7210_audio_init(const es7210_audio_config_t *config,
                            es7210_audio_handle_t *out_handle);

/**
 * @brief 读取一帧四通道交织 PCM 数据。
 *
 * 输出排列固定为 `[MIC1, MIC2, MIC3, MIC4]` 循环，每个元素均为 int16_t。
 * 缓冲区元素数量必须等于 `samples_per_channel * 4`。
 *
 * @param[in] handle 已初始化的驱动句柄。
 * @param[out] interleaved_pcm 接收交织 PCM 数据的缓冲区。
 * @param[in] samples_per_channel 本次希望读取的每通道采样点数。
 * @return ESP_OK 表示整帧读取成功；驱动超时或总线错误会返回 ESP_FAIL。
 */
esp_err_t es7210_audio_read(es7210_audio_handle_t handle,
                            int16_t *interleaved_pcm,
                            size_t samples_per_channel);

/**
 * @brief 关闭 ES7210 并释放驱动创建的全部资源。
 *
 * 调用前必须停止所有正在执行的 es7210_audio_read()。
 */
esp_err_t es7210_audio_deinit(es7210_audio_handle_t handle);

/** @brief 返回驱动配置的每通道采样率。 */
uint32_t es7210_audio_get_sample_rate(es7210_audio_handle_t handle);

#ifdef __cplusplus
}
#endif
