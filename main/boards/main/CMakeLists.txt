idf_component_register(
    SRCS "main.c" "wifi_manager.c" "webserver.c" "robot_control.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_http_server driver esp_timer
    PRIV_REQUIRES esp_event
)
