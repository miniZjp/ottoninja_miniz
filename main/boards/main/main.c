/*
 * Otto Ninja Robot - ESP-IDF Version
 * Main Application Entry Point
 * 
 * Converted from Arduino Framework to ESP-IDF
 * 
 * GPIO Pins:
 * - Right Leg: 39
 * - Right Foot: 38 -> Actually 18 in code
 * - Left Leg: 17
 * - Left Foot: 18 -> Actually 38 in code
 * - Left Arm: 8
 * - Right Arm: 12
 * - Head: 13
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "robot_control.h"
#include "wifi_manager.h"
#include "webserver.h"

static const char *TAG = "otto_main";

void app_main(void) {
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "   Otto Ninja Robot - ESP-IDF");
    ESP_LOGI(TAG, "   Converted from Arduino");
    ESP_LOGI(TAG, "====================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    // Initialize robot control (servos)
    robot_control_init();
    ESP_LOGI(TAG, "Robot control initialized");
    
    // Initialize servos to home position
    go_home();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Set default mode to WALK
    ninja_set_walk();
    
    // Initialize WiFi
    wifi_init();
    
    // Start web server
    httpd_handle_t server = webserver_start();
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to start web server!");
    }
    
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Otto Ninja Robot READY!");
    ESP_LOGI(TAG, "====================================");
    
    // Create robot control task
    xTaskCreate(robot_control_task, "robot_ctrl", 4096, NULL, 5, NULL);
    
    // Main loop - nothing to do here as everything runs in tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
