/*
 * Otto Ninja Robot - ESP-IDF Version
 * Web Server Header
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_http_server.h"

// Start the web server
httpd_handle_t webserver_start(void);

// Stop the web server
void webserver_stop(httpd_handle_t server);

#endif // WEBSERVER_H
