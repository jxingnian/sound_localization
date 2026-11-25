idf_component_register(
    SRCS "es7210_audio.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_codec_dev esp_driver_i2c esp_driver_i2s
)
