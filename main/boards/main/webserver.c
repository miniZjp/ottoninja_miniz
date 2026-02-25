/*
 * Otto Ninja Robot - ESP-IDF Version
 * Web Server Implementation
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "robot_control.h"

static const char *TAG = "webserver";

// Timing for auto-save
static uint32_t last_setting_change = 0;
static bool settings_changed = false;
#define SAVE_DELAY_MS 1000

// HTML Web UI (embedded)
static const char html_content[] = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no\">"
"<title>Otto Ninja Control</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}"
"body{font-family:Arial,sans-serif;background:#2c3e50;color:#ecf0f1;padding:10px;text-align:center;min-height:100vh;display:flex;flex-direction:column;justify-content:flex-start;overflow-x:hidden}"
"h1{margin:10px 0 15px;font-size:1.5em}"
".joystick-container{width:min(280px,70vw);height:min(280px,70vw);margin:10px auto;position:relative;background:#34495e;border-radius:50%;touch-action:none;box-shadow:0 4px 8px rgba(0,0,0,0.3)}"
".joystick{width:25%;height:25%;background:#3498db;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);cursor:pointer;box-shadow:0 2px 6px rgba(0,0,0,0.4)}"
".buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:400px;margin:15px auto;padding:0 10px}"
".btn{padding:15px 10px;font-size:1em;font-weight:bold;border:none;border-radius:10px;cursor:pointer;color:white;touch-action:manipulation;min-height:50px}"
".btn-walk{background:#27ae60}"
".btn-roll{background:#e67e22}"
".btn-left{background:#3498db}"
".btn-right{background:#9b59b6}"
".values{margin:10px;font-family:monospace;font-size:0.9em;padding:8px;background:#34495e;border-radius:8px;display:inline-block}"
".calibration{max-width:500px;margin:15px auto;background:#34495e;padding:15px;border-radius:10px;max-height:50vh;overflow-y:auto}"
".calibration h2{font-size:1.1em;margin-bottom:8px;color:#3498db;border-bottom:2px solid #2c3e50;padding-bottom:8px}"
".cal-section{margin-bottom:15px}"
".cal-section h3{font-size:0.95em;margin-bottom:8px;color:#e67e22}"
".cal-item{margin-bottom:10px}"
".cal-item label{display:block;margin-bottom:4px;font-size:0.8em;color:#bdc3c7}"
".cal-row{display:flex;gap:8px;align-items:center}"
".cal-row input[type=\"range\"]{flex:1;height:6px;background:#2c3e50;border-radius:3px;outline:none}"
".cal-row input[type=\"range\"]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;background:#3498db;border-radius:50%;cursor:pointer}"
".cal-value{min-width:35px;text-align:center;font-family:monospace;background:#2c3e50;padding:5px;border-radius:5px;font-size:0.8em}"
".btn-apply{width:100%;padding:12px;font-size:1em;font-weight:bold;border:none;border-radius:8px;cursor:pointer;background:#27ae60;color:white;margin-top:10px}"
".btn-apply:active{background:#229954}"
"@media(max-width:480px){h1{font-size:1.3em}"
".btn{font-size:0.9em;padding:12px 8px;min-height:45px}"
".values{font-size:0.85em}}"
"</style>"
"</head>"
"<body>"
"<h1>🤖 OTTO NINJA (ESP-IDF)</h1>"
"<button class=\"btn-apply\" id=\"btnSaveTop\" style=\"max-width:400px;margin:5px auto 10px auto;\">💾 Save All Settings</button>"
"<div class=\"joystick-container\" id=\"joystickZone\">"
"<div class=\"joystick\" id=\"joystick\"></div>"
"</div>"
"<div class=\"values\">X: <span id=\"xVal\">0</span> | Y: <span id=\"yVal\">0</span></div>"
"<div class=\"buttons\">"
"<button class=\"btn btn-walk\" id=\"btnY\">🚶 WALK (Y)</button>"
"<button class=\"btn btn-roll\" id=\"btnX\">⚙️ ROLL (X)</button>"
"<button class=\"btn btn-left\" id=\"btnA\">👈 Left Arm (A)</button>"
"<button class=\"btn btn-right\" id=\"btnB\">👉 Right Arm (B)</button>"
"</div>"
"<div class=\"buttons\" style=\"margin-top:10px\">"
"<button class=\"btn\" style=\"background:#16a085\" id=\"btnTestFwd\">⬆️ TEST Forward</button>"
"<button class=\"btn\" style=\"background:#c0392b\" id=\"btnTestBack\">⬇️ TEST Backward</button>"
"<button class=\"btn\" style=\"background:#3498db\" id=\"btnHome\">🏠 HOME</button>"
"</div>"
"<div class=\"buttons\" style=\"margin-top:10px\">"
"<button class=\"btn\" style=\"background:#9b59b6\" id=\"btnTestLF\">🦶 L Foot</button>"
"<button class=\"btn\" style=\"background:#9b59b6\" id=\"btnTestRF\">🦶 R Foot</button>"
"<button class=\"btn\" style=\"background:#27ae60\" id=\"btnTestBoth\">🦶🦶 BOTH 3s</button>"
"<button class=\"btn\" style=\"background:#e74c3c\" id=\"btnStopFoot\">⏹️ STOP</button>"
"</div>"
"<div class=\"calibration\">"
"<h2>🎛️ Robot Calibration</h2>"
"<p style=\"color:#888;font-size:12px;margin:5px 0 15px 0;\">💡 Điều chỉnh các thông số rồi nhấn Apply. Test sau mỗi lần thay đổi.</p>"
"<div class=\"cal-section\">"
"<h3>⚖️ Foot Neutral (Điểm dừng)</h3>"
"<p style=\"color:#f39c12;font-size:11px;margin:0 0 8px 0;\">⚠️ Điều chỉnh nếu chân quay không đều.</p>"
"<div class=\"cal-item\">"
"<label>Left Foot Neutral</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lfn\" min=\"80\" max=\"100\" value=\"90\"><span class=\"cal-value\" id=\"lfnDisp\">90</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Foot Neutral</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rfn\" min=\"80\" max=\"100\" value=\"90\"><span class=\"cal-value\" id=\"rfnDisp\">90</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🚶 Walk Speed (Tốc độ bước)</h3>"
"<div class=\"cal-item\">"
"<label>Left Forward</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lff\" min=\"5\" max=\"40\" value=\"18\"><span class=\"cal-value\" id=\"lffDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Forward</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rff\" min=\"5\" max=\"40\" value=\"18\"><span class=\"cal-value\" id=\"rffDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Left Backward</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lfb\" min=\"5\" max=\"40\" value=\"18\"><span class=\"cal-value\" id=\"lfbDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Backward</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rfb\" min=\"5\" max=\"40\" value=\"18\"><span class=\"cal-value\" id=\"rfbDisp\">18</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🦵 Standing Position</h3>"
"<div class=\"cal-item\">"
"<label>Left Leg (LA0)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"la0\" min=\"0\" max=\"180\" value=\"60\"><span class=\"cal-value\" id=\"la0Disp\">60</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Leg (RA0)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ra0\" min=\"0\" max=\"180\" value=\"135\"><span class=\"cal-value\" id=\"ra0Disp\">135</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>↔️ Leg Tilt - Walk</h3>"
"<div class=\"cal-item\">"
"<label>Left Tilt Left (LATL)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"latl\" min=\"0\" max=\"180\" value=\"100\"><span class=\"cal-value\" id=\"latlDisp\">100</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Tilt Left (RATL)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ratl\" min=\"0\" max=\"180\" value=\"175\"><span class=\"cal-value\" id=\"ratlDisp\">175</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Left Tilt Right (LATR)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"latr\" min=\"0\" max=\"180\" value=\"5\"><span class=\"cal-value\" id=\"latrDisp\">5</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Tilt Right (RATR)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ratr\" min=\"0\" max=\"180\" value=\"80\"><span class=\"cal-value\" id=\"ratrDisp\">80</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>⚙️ Roll Position</h3>"
"<div class=\"cal-item\">"
"<label>Left Leg (LA1)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"la1\" min=\"0\" max=\"180\" value=\"160\"><span class=\"cal-value\" id=\"la1Disp\">160</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>Right Leg (RA1)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ra1\" min=\"0\" max=\"180\" value=\"25\"><span class=\"cal-value\" id=\"ra1Disp\">25</span></div>"
"</div>"
"</div>"
"<button class=\"btn-apply\" id=\"btnApply\">✅ Apply All Settings</button>"
"</div>"
"<script>"
"let jx=0,jy=0,bA=0,bB=0,bX=0,bY=0;"
"const js=document.getElementById('joystick'),jz=document.getElementById('joystickZone');"
"let drag=false;const md=70;"
"let lastSendTime=0;"
"let pendingUpdate=false;"
"function sendControl(){"
"fetch('/control?x='+jx+'&y='+jy+'&a='+bA+'&b='+bB+'&x2='+bX+'&y2='+bY).catch(e=>console.log('Req failed'));"
"lastSendTime=Date.now();"
"pendingUpdate=false;"
"}"
"function uj(e){if(!drag)return;"
"const r=jz.getBoundingClientRect(),cx=r.width/2,cy=r.height/2;"
"let x=(e.touches?e.touches[0].clientX:e.clientX)-r.left-cx,"
"y=(e.touches?e.touches[0].clientY:e.clientY)-r.top-cy;"
"const d=Math.sqrt(x*x+y*y);if(d>md){x=(x/d)*md;y=(y/d)*md;}"
"js.style.left=(cx+x)+'px';js.style.top=(cy+y)+'px';"
"let rawX=Math.round((x/md)*100);"
"let rawY=Math.round(-(y/md)*100);"
"jx=(Math.abs(rawX)<5)?0:rawX;"
"jy=(Math.abs(rawY)<5)?0:rawY;"
"document.getElementById('xVal').textContent=jx;"
"document.getElementById('yVal').textContent=jy;"
"if(Date.now()-lastSendTime>100){sendControl();}else{pendingUpdate=true;}"
"}"
"setInterval(()=>{if(pendingUpdate&&Date.now()-lastSendTime>100){sendControl();}},50);"
"function rj(){drag=false;js.style.left='50%';js.style.top='50%';"
"jx=0;jy=0;document.getElementById('xVal').textContent='0';"
"document.getElementById('yVal').textContent='0';"
"sendControl();}"
"js.addEventListener('mousedown',()=>drag=true);"
"js.addEventListener('touchstart',()=>drag=true);"
"document.addEventListener('mousemove',uj);"
"document.addEventListener('touchmove',uj);"
"document.addEventListener('mouseup',rj);"
"document.addEventListener('touchend',rj);"
"document.getElementById('btnY').onclick=()=>{bY=1;setTimeout(()=>bY=0,100);sendControl();};"
"document.getElementById('btnX').onclick=()=>{bX=1;setTimeout(()=>bX=0,100);sendControl();};"
"document.getElementById('btnA').onmousedown=()=>{bA=1;sendControl();};"
"document.getElementById('btnA').onmouseup=()=>{bA=0;sendControl();};"
"document.getElementById('btnB').onmousedown=()=>{bB=1;sendControl();};"
"document.getElementById('btnB').onmouseup=()=>{bB=0;sendControl();};"
"document.getElementById('btnTestFwd').onclick=()=>{fetch('/control?x=0&y=80&fwd=1&back=0&test=1').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnTestBack').onclick=()=>{fetch('/control?x=0&y=-80&fwd=0&back=1&test=1').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnHome').onclick=()=>{fetch('/home').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnTestLF').onclick=()=>{fetch('/testfoot?foot=left').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnTestRF').onclick=()=>{fetch('/testfoot?foot=right').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnTestBoth').onclick=()=>{fetch('/testfoot?foot=both').catch(e=>console.log('Req failed'));};"
"document.getElementById('btnStopFoot').onclick=()=>{fetch('/testfoot?foot=stop').catch(e=>console.log('Req failed'));};"
"const sliders=['lfn','rfn','lff','rff','lfb','rfb','la0','ra0','latl','ratl','latr','ratr','la1','ra1'];"
"sliders.forEach(id=>{"
"const el=document.getElementById(id);"
"el.oninput=()=>{document.getElementById(id+'Disp').textContent=el.value;};"
"});"
"document.getElementById('btnApply').onclick=()=>{"
"const params=sliders.map(id=>id+'='+document.getElementById(id).value).join('&');"
"fetch('/calibrate?'+params).then(r=>r.text()).then(s=>{alert('✓ '+s);});};"
"document.getElementById('btnSaveTop').onclick=()=>{"
"const params=sliders.map(id=>id+'='+document.getElementById(id).value).join('&');"
"fetch('/calibrate?'+params).then(r=>r.text()).then(s=>{alert('✓ '+s);});};"
"fetch('/getCal').then(r=>r.json()).then(d=>{"
"sliders.forEach(id=>{"
"if(d[id]!==undefined){"
"document.getElementById(id).value=d[id];"
"document.getElementById(id+'Disp').textContent=d[id];"
"}"
"});"
"});"
"</script>"
"</body></html>";

// Helper function to get query parameter value
static esp_err_t get_query_param(httpd_req_t *req, const char *key, char *value, size_t value_len) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char *buf = malloc(buf_len);
        if (!buf) return ESP_ERR_NO_MEM;
        
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, key, value, value_len) == ESP_OK) {
                free(buf);
                return ESP_OK;
            }
        }
        free(buf);
    }
    return ESP_FAIL;
}

static int get_query_param_int(httpd_req_t *req, const char *key, int default_val) {
    char value[16];
    if (get_query_param(req, key, value, sizeof(value)) == ESP_OK) {
        return atoi(value);
    }
    return default_val;
}

// Handler for root page
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_content, strlen(html_content));
    return ESP_OK;
}

// Handler for control endpoint
static esp_err_t control_handler(httpd_req_t *req) {
    control_state_t *state = get_control_state();
    
    char val[16];
    if (get_query_param(req, "x", val, sizeof(val)) == ESP_OK) {
        state->j_x = atoi(val);
    }
    if (get_query_param(req, "y", val, sizeof(val)) == ESP_OK) {
        state->j_y = atoi(val);
    }
    if (get_query_param(req, "a", val, sizeof(val)) == ESP_OK) {
        state->button_a = atoi(val);
    }
    if (get_query_param(req, "b", val, sizeof(val)) == ESP_OK) {
        state->button_b = atoi(val);
    }
    if (get_query_param(req, "x2", val, sizeof(val)) == ESP_OK) {
        state->button_x = atoi(val);
    }
    if (get_query_param(req, "y2", val, sizeof(val)) == ESP_OK) {
        state->button_y = atoi(val);
    }
    
    // Test mode
    if (get_query_param(req, "test", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
        state->test_mode_active = true;
        state->test_cycles_remaining = 5;
        ESP_LOGI(TAG, "[TEST MODE] Starting 5 cycles...");
    }
    
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// Handler for calibration endpoint
static esp_err_t calibrate_handler(httpd_req_t *req) {
    calibration_t *cal = get_calibration();
    
    cal->lf_neutral = get_query_param_int(req, "lfn", cal->lf_neutral);
    cal->rf_neutral = get_query_param_int(req, "rfn", cal->rf_neutral);
    cal->lffwrs = get_query_param_int(req, "lff", cal->lffwrs);
    cal->rffwrs = get_query_param_int(req, "rff", cal->rffwrs);
    cal->lfbwrs = get_query_param_int(req, "lfb", cal->lfbwrs);
    cal->rfbwrs = get_query_param_int(req, "rfb", cal->rfbwrs);
    cal->la0 = get_query_param_int(req, "la0", cal->la0);
    cal->ra0 = get_query_param_int(req, "ra0", cal->ra0);
    cal->latl = get_query_param_int(req, "latl", cal->latl);
    cal->ratl = get_query_param_int(req, "ratl", cal->ratl);
    cal->latr = get_query_param_int(req, "latr", cal->latr);
    cal->ratr = get_query_param_int(req, "ratr", cal->ratr);
    cal->la1 = get_query_param_int(req, "la1", cal->la1);
    cal->ra1 = get_query_param_int(req, "ra1", cal->ra1);
    
    // Mark for auto-save
    settings_changed = true;
    last_setting_change = esp_log_timestamp();
    
    ESP_LOGI(TAG, "Cal: LFN=%d RFN=%d LFF=%d RFF=%d",
             cal->lf_neutral, cal->rf_neutral, cal->lffwrs, cal->rffwrs);
    
    // Save immediately for now
    save_calibration_to_nvs();
    
    httpd_resp_sendstr(req, "Settings applied and saved!");
    return ESP_OK;
}

// Handler for get calibration endpoint
static esp_err_t get_cal_handler(httpd_req_t *req) {
    calibration_t *cal = get_calibration();
    
    char json[512];
    snprintf(json, sizeof(json),
             "{\"lfn\":%d,\"rfn\":%d,\"lff\":%d,\"rff\":%d,"
             "\"lfb\":%d,\"rfb\":%d,\"la0\":%d,\"ra0\":%d,"
             "\"latl\":%d,\"ratl\":%d,\"latr\":%d,\"ratr\":%d,"
             "\"la1\":%d,\"ra1\":%d}",
             cal->lf_neutral, cal->rf_neutral, cal->lffwrs, cal->rffwrs,
             cal->lfbwrs, cal->rfbwrs, cal->la0, cal->ra0,
             cal->latl, cal->ratl, cal->latr, cal->ratr,
             cal->la1, cal->ra1);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// Handler for home endpoint
static esp_err_t home_handler(httpd_req_t *req) {
    go_home();
    httpd_resp_sendstr(req, "Robot returned to HOME position");
    return ESP_OK;
}

// Handler for test foot endpoint
static esp_err_t testfoot_handler(httpd_req_t *req) {
    char foot[16];
    if (get_query_param(req, "foot", foot, sizeof(foot)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing foot param");
        return ESP_FAIL;
    }
    
    if (strcmp(foot, "left") == 0) {
        test_left_foot();
        httpd_resp_sendstr(req, "Left foot running");
    } else if (strcmp(foot, "right") == 0) {
        test_right_foot();
        httpd_resp_sendstr(req, "Right foot running");
    } else if (strcmp(foot, "both") == 0) {
        test_both_feet();
        httpd_resp_sendstr(req, "Both feet running for 3s");
    } else if (strcmp(foot, "stop") == 0) {
        stop_feet_test();
        httpd_resp_sendstr(req, "Both feet stopped");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid foot param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// URI handlers
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_control = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = control_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_calibrate = {
    .uri = "/calibrate",
    .method = HTTP_GET,
    .handler = calibrate_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_get_cal = {
    .uri = "/getCal",
    .method = HTTP_GET,
    .handler = get_cal_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_home = {
    .uri = "/home",
    .method = HTTP_GET,
    .handler = home_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_testfoot = {
    .uri = "/testfoot",
    .method = HTTP_GET,
    .handler = testfoot_handler,
    .user_ctx = NULL
};

httpd_handle_t webserver_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 10;
    
    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_control);
        httpd_register_uri_handler(server, &uri_calibrate);
        httpd_register_uri_handler(server, &uri_get_cal);
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_testfoot);
        
        ESP_LOGI(TAG, "Web server started!");
        return server;
    }
    
    ESP_LOGE(TAG, "Failed to start web server!");
    return NULL;
}

void webserver_stop(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
}
