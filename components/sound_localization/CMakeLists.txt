idf_component_register(
    SRCS "sound_localization.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_pipeline freertos
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
