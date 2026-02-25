/*
 * Otto Ninja Robot - ESP-IDF Version
 * WiFi Manager Header
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include "esp_netif.h"

// Initialize WiFi
void wifi_init(void);

// Check if connected to WiFi
bool wifi_is_connected(void);

// Get IP address string
const char* wifi_get_ip_address(void);

#endif // WIFI_MANAGER_H
