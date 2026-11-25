idf_component_register(
    SRCS "audio_pipeline.c"
    INCLUDE_DIRS "include"
    REQUIRES es7210_audio esp_timer freertos
)
