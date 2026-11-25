idf_component_register(
    SRCS "audio_web_test.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_pipeline esp_wifi esp_event esp_netif nvs_flash esp_http_server cjson
    EMBED_FILES "web/index.html"
)
