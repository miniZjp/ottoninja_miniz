/*
 * Otto Ninja Robot - ESP-IDF Version
 * Robot Control Implementation
 * 
 * Servo control using LEDC PWM and walking logic
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "robot_control.h"

static const char *TAG = "robot_ctrl";

// Servo configuration
#define SERVO_MIN_PULSEWIDTH_US 544
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE        0
#define SERVO_MAX_DEGREE        180
#define SERVO_FREQ_HZ           50
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_FULL_DUTY          ((1 << 14) - 1)

// GPIO pin assignments for each servo channel
static const int servo_gpio[SERVO_CH_MAX] = {
    [SERVO_CH_LEFT_FOOT]  = SERVO_LEFT_FOOT_PIN,
    [SERVO_CH_LEFT_LEG]   = SERVO_LEFT_LEG_PIN,
    [SERVO_CH_RIGHT_FOOT] = SERVO_RIGHT_FOOT_PIN,
    [SERVO_CH_RIGHT_LEG]  = SERVO_RIGHT_LEG_PIN,
    [SERVO_CH_LEFT_ARM]   = SERVO_LEFT_ARM_PIN,
    [SERVO_CH_RIGHT_ARM]  = SERVO_RIGHT_ARM_PIN,
    [SERVO_CH_HEAD]       = SERVO_HEAD_PIN,
};

// Track which servos are attached
static bool servo_is_attached[SERVO_CH_MAX] = {false};

// Current calibration settings
static calibration_t calibration = {
    .lf_neutral = CAL_DEFAULT_LF_NEUTRAL,
    .rf_neutral = CAL_DEFAULT_RF_NEUTRAL,
    .lffwrs = CAL_DEFAULT_LFFWRS,
    .rffwrs = CAL_DEFAULT_RFFWRS,
    .lfbwrs = CAL_DEFAULT_LFBWRS,
    .rfbwrs = CAL_DEFAULT_RFBWRS,
    .la0 = CAL_DEFAULT_LA0,
    .ra0 = CAL_DEFAULT_RA0,
    .latl = CAL_DEFAULT_LATL,
    .ratl = CAL_DEFAULT_RATL,
    .latr = CAL_DEFAULT_LATR,
    .ratr = CAL_DEFAULT_RATR,
    .la1 = CAL_DEFAULT_LA1,
    .ra1 = CAL_DEFAULT_RA1,
};

// Current control state
static control_state_t control_state = {
    .j_x = 0,
    .j_y = 0,
    .button_a = 0,
    .button_b = 0,
    .button_x = 0,
    .button_y = 0,
    .test_mode_active = false,
    .test_cycles_remaining = 0,
};

// Robot mode
static robot_mode_t current_mode = MODE_WALK;

// Walking state tracking
static bool was_moving = false;
static bool is_at_home = true;
static bool walk_cal_logged = false;
static bool walk_cycle_reset = false;
static uint32_t current_millis1 = 0;

// Convert angle to PWM duty
static uint32_t angle_to_duty(int angle) {
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    // Calculate pulse width in microseconds
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_DEGREE;
    
    // Convert to duty cycle (14-bit resolution, 50Hz = 20ms period)
    // duty = (pulse_width_us / 20000us) * full_duty
    uint32_t duty = (pulse_width * LEDC_FULL_DUTY) / 20000;
    
    return duty;
}

// Initialize servo PWM
void robot_control_init(void) {
    ESP_LOGI(TAG, "Initializing robot control...");
    
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
    
    // Configure LEDC channels for each servo
    for (int ch = 0; ch < SERVO_CH_MAX; ch++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = servo_gpio[ch],
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)ch,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
        servo_is_attached[ch] = false;
    }
    
    // Load calibration from NVS
    load_calibration_from_nvs();
    
    ESP_LOGI(TAG, "Robot control initialized");
}

void servo_write(servo_channel_t channel, int angle) {
    if (channel >= SERVO_CH_MAX) return;
    
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel);
}

void servo_attach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    servo_is_attached[channel] = true;
}

void servo_detach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    
    // Set duty to 0 to stop PWM signal
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, 0);
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel);
    servo_is_attached[channel] = false;
}

bool servo_attached(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return false;
    return servo_is_attached[channel];
}

// ========== NVS FUNCTIONS ==========

void save_calibration_to_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("otto", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_set_i32(handle, "lfn", calibration.lf_neutral);
    nvs_set_i32(handle, "rfn", calibration.rf_neutral);
    nvs_set_i32(handle, "lff", calibration.lffwrs);
    nvs_set_i32(handle, "rff", calibration.rffwrs);
    nvs_set_i32(handle, "lfb", calibration.lfbwrs);
    nvs_set_i32(handle, "rfb", calibration.rfbwrs);
    nvs_set_i32(handle, "la0", calibration.la0);
    nvs_set_i32(handle, "ra0", calibration.ra0);
    nvs_set_i32(handle, "latl", calibration.latl);
    nvs_set_i32(handle, "ratl", calibration.ratl);
    nvs_set_i32(handle, "latr", calibration.latr);
    nvs_set_i32(handle, "ratr", calibration.ratr);
    nvs_set_i32(handle, "la1", calibration.la1);
    nvs_set_i32(handle, "ra1", calibration.ra1);
    
    nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "💾 Settings saved to NVS!");
}

void load_calibration_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("otto", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not found, using defaults");
        return;
    }
    
    int32_t val;
    if (nvs_get_i32(handle, "lfn", &val) == ESP_OK) calibration.lf_neutral = val;
    if (nvs_get_i32(handle, "rfn", &val) == ESP_OK) calibration.rf_neutral = val;
    if (nvs_get_i32(handle, "lff", &val) == ESP_OK) calibration.lffwrs = val;
    if (nvs_get_i32(handle, "rff", &val) == ESP_OK) calibration.rffwrs = val;
    if (nvs_get_i32(handle, "lfb", &val) == ESP_OK) calibration.lfbwrs = val;
    if (nvs_get_i32(handle, "rfb", &val) == ESP_OK) calibration.rfbwrs = val;
    if (nvs_get_i32(handle, "la0", &val) == ESP_OK) calibration.la0 = val;
    if (nvs_get_i32(handle, "ra0", &val) == ESP_OK) calibration.ra0 = val;
    if (nvs_get_i32(handle, "latl", &val) == ESP_OK) calibration.latl = val;
    if (nvs_get_i32(handle, "ratl", &val) == ESP_OK) calibration.ratl = val;
    if (nvs_get_i32(handle, "latr", &val) == ESP_OK) calibration.latr = val;
    if (nvs_get_i32(handle, "ratr", &val) == ESP_OK) calibration.ratr = val;
    if (nvs_get_i32(handle, "la1", &val) == ESP_OK) calibration.la1 = val;
    if (nvs_get_i32(handle, "ra1", &val) == ESP_OK) calibration.ra1 = val;
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "📂 Settings loaded from NVS:");
    ESP_LOGI(TAG, "  Neutral: LF=%d, RF=%d", calibration.lf_neutral, calibration.rf_neutral);
    ESP_LOGI(TAG, "  Forward: LFF=%d, RFF=%d", calibration.lffwrs, calibration.rffwrs);
    ESP_LOGI(TAG, "  Backward: LFB=%d, RFB=%d", calibration.lfbwrs, calibration.rfbwrs);
    ESP_LOGI(TAG, "  Stand: LA0=%d, RA0=%d", calibration.la0, calibration.ra0);
    ESP_LOGI(TAG, "  Tilt: LATL=%d, RATL=%d, LATR=%d, RATR=%d", 
             calibration.latl, calibration.ratl, calibration.latr, calibration.ratr);
    ESP_LOGI(TAG, "  Roll: LA1=%d, RA1=%d", calibration.la1, calibration.ra1);
}

calibration_t* get_calibration(void) {
    return &calibration;
}

void set_calibration(const calibration_t* cal) {
    if (cal) {
        memcpy(&calibration, cal, sizeof(calibration_t));
    }
}

control_state_t* get_control_state(void) {
    return &control_state;
}

robot_mode_t get_robot_mode(void) {
    return current_mode;
}

// ========== MOVEMENT FUNCTIONS ==========

void go_home(void) {
    ESP_LOGI(TAG, "🏠 Going to HOME position...");
    
    // Stop foot servos at neutral position first
    servo_attach(SERVO_CH_LEFT_FOOT);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Detach foot servos
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    
    // Move legs to HOME
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la0);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra0);
    vTaskDelay(pdMS_TO_TICKS(400));
    
    // Arms to position
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 180);
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Head to center
    servo_attach(SERVO_CH_HEAD);
    servo_write(SERVO_CH_HEAD, 90);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Detach arms and head
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    servo_detach(SERVO_CH_HEAD);
    
    ESP_LOGI(TAG, "🏠 HOME: LeftLeg=%d, RightLeg=%d, Arms=180/0, Head=90",
             calibration.la0, calibration.ra0);
    ESP_LOGI(TAG, "🏠 HOME position reached!");
    
    is_at_home = true;
}

void ninja_set_walk(void) {
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 90);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
    vTaskDelay(pdMS_TO_TICKS(200));
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la0);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
    
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 180);
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    
    current_mode = MODE_WALK;
    ESP_LOGI(TAG, "Mode: WALK");
}

void ninja_set_roll(void) {
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 90);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
    vTaskDelay(pdMS_TO_TICKS(200));
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la1);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra1);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
    
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 180);
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    
    current_mode = MODE_ROLL;
    ESP_LOGI(TAG, "Mode: ROLL");
}

void ninja_left_arm_up(void) {
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 90);
}

void ninja_left_arm_down(void) {
    servo_write(SERVO_CH_LEFT_ARM, 180);
}

void ninja_right_arm_up(void) {
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
}

void ninja_right_arm_down(void) {
    servo_write(SERVO_CH_RIGHT_ARM, 0);
}

void ninja_walk_stop(void) {
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la0);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra0);
}

void ninja_roll_stop(void) {
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
}

// Helper function: map value from one range to another
static int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int constrain_value(int x, int min_val, int max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

// Get current time in ms
static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ========== WALKING LOGIC ==========

void ninja_walk(void) {
    int8_t j_x = control_state.j_x;
    int8_t j_y = control_state.j_y;
    
    // IMMEDIATE STOP when X=0 and Y=0 (except in test mode)
    if (j_x == 0 && j_y == 0 && !control_state.test_mode_active) {
        ninja_walk_stop();
        servo_detach(SERVO_CH_LEFT_FOOT);
        servo_detach(SERVO_CH_RIGHT_FOOT);
        servo_detach(SERVO_CH_LEFT_LEG);
        servo_detach(SERVO_CH_RIGHT_LEG);
        walk_cal_logged = false;
        walk_cycle_reset = false;
        
        if (was_moving && !is_at_home) {
            was_moving = false;
            is_at_home = true;
            go_home();
        }
        return;
    }
    
    // Reset timing when starting new walk session
    if (!walk_cycle_reset) {
        current_millis1 = millis();
        walk_cycle_reset = true;
    }
    
    was_moving = true;
    is_at_home = false;
    
    // FORWARD WALKING - Y >= 0
    if (j_y >= 0) {
        int lt = map_value(j_x, -100, 100, 200, 700);
        int rt = map_value(j_x, -100, 100, 700, 200);
        
        int interval1 = 250;
        int interval2 = 250 + rt;
        int interval3 = 250 + rt + 250;
        int interval4 = 250 + rt + 250 + lt;
        int interval5 = 250 + rt + 250 + lt + 50;
        
        static int last_phase = 0;
        int current_phase = 0;
        uint32_t elapsed = millis() - current_millis1;
        
        if (!walk_cal_logged) {
            ESP_LOGI(TAG, "========== WALKING CALIBRATION VALUES ==========");
            ESP_LOGI(TAG, "LF_NEUTRAL=%d RF_NEUTRAL=%d", calibration.lf_neutral, calibration.rf_neutral);
            ESP_LOGI(TAG, "LFFWRS=%d RFFWRS=%d LFBWRS=%d RFBWRS=%d",
                     calibration.lffwrs, calibration.rffwrs, calibration.lfbwrs, calibration.rfbwrs);
            walk_cal_logged = true;
        }
        
        if (elapsed <= interval1) current_phase = 1;
        else if (elapsed <= interval2) current_phase = 2;
        else if (elapsed <= interval3) current_phase = 3;
        else if (elapsed <= interval4) current_phase = 4;
        else current_phase = 5;
        
        if (millis() > current_millis1 + interval5) {
            current_millis1 = millis();
            last_phase = 0;
            
            if (control_state.test_mode_active && control_state.test_cycles_remaining > 0) {
                control_state.test_cycles_remaining--;
                ESP_LOGI(TAG, "[TEST MODE] Forward cycle done, %d remaining", control_state.test_cycles_remaining);
                if (control_state.test_cycles_remaining == 0) {
                    control_state.test_mode_active = false;
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    ninja_walk_stop();
                    servo_detach(SERVO_CH_LEFT_FOOT);
                    servo_detach(SERVO_CH_RIGHT_FOOT);
                    servo_detach(SERVO_CH_LEFT_LEG);
                    servo_detach(SERVO_CH_RIGHT_LEG);
                    go_home();
                }
            }
        }
        
        elapsed = millis() - current_millis1;
        
        // Phase 1: Tilt to right
        if (elapsed <= interval1) {
            if (current_phase != last_phase) {
                ESP_LOGD(TAG, "[FWD] Phase1: LeftLeg=%d RightLeg=%d (Tilt Right)",
                         calibration.latr, calibration.ratr);
                last_phase = current_phase;
            }
            servo_attach(SERVO_CH_LEFT_LEG);
            servo_attach(SERVO_CH_RIGHT_LEG);
            servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
        }
        
        // Phase 2: Right foot rotation FORWARD
        elapsed = millis() - current_millis1;
        if (elapsed >= interval1 && elapsed <= interval2) {
            int right_foot_angle = calibration.rf_neutral - calibration.rffwrs;
            if (current_phase != last_phase) {
                ESP_LOGD(TAG, "[FWD] Phase2: RightFoot=%d", right_foot_angle);
                last_phase = current_phase;
                servo_attach(SERVO_CH_RIGHT_FOOT);
            }
            servo_write(SERVO_CH_RIGHT_FOOT, right_foot_angle);
        }
        
        // Phase 3: Stop right foot, tilt to left
        elapsed = millis() - current_millis1;
        if (elapsed >= interval2 && elapsed <= interval3) {
            if (current_phase != last_phase) {
                ESP_LOGD(TAG, "[FWD] Phase3: LeftLeg=%d RightLeg=%d (Tilt Left)",
                         calibration.latl, calibration.ratl);
                last_phase = current_phase;
                servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
            }
            servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
        }
        
        // Phase 4: Left foot rotation FORWARD
        elapsed = millis() - current_millis1;
        if (elapsed >= interval3 && elapsed <= interval4) {
            int left_foot_angle = calibration.lf_neutral + calibration.lffwrs;
            if (current_phase != last_phase) {
                ESP_LOGD(TAG, "[FWD] Phase4: LeftFoot=%d", left_foot_angle);
                last_phase = current_phase;
                servo_attach(SERVO_CH_LEFT_FOOT);
            }
            servo_write(SERVO_CH_LEFT_FOOT, left_foot_angle);
        }
        
        // Phase 5: Stop left foot
        elapsed = millis() - current_millis1;
        if (elapsed >= interval4 && elapsed <= interval5) {
            if (current_phase != last_phase) {
                last_phase = current_phase;
                servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
            }
        }
    }
    // BACKWARD WALKING - Y < 0
    else if (j_y < 0) {
        int lt = map_value(j_x, -100, 100, 200, 700);
        int rt = map_value(j_x, -100, 100, 700, 200);
        
        int interval1 = 250;
        int interval2 = 250 + rt;
        int interval3 = 250 + rt + 250;
        int interval4 = 250 + rt + 250 + lt;
        int interval5 = 250 + rt + 250 + lt + 50;
        
        static int last_back_phase = 0;
        int current_phase = 0;
        uint32_t elapsed = millis() - current_millis1;
        
        if (elapsed <= interval1) current_phase = 1;
        else if (elapsed <= interval2) current_phase = 2;
        else if (elapsed <= interval3) current_phase = 3;
        else if (elapsed <= interval4) current_phase = 4;
        else current_phase = 5;
        
        if (millis() > current_millis1 + interval5) {
            current_millis1 = millis();
            last_back_phase = 0;
            
            if (control_state.test_mode_active && control_state.test_cycles_remaining > 0) {
                control_state.test_cycles_remaining--;
                if (control_state.test_cycles_remaining == 0) {
                    control_state.test_mode_active = false;
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    ninja_walk_stop();
                    servo_detach(SERVO_CH_LEFT_FOOT);
                    servo_detach(SERVO_CH_RIGHT_FOOT);
                    servo_detach(SERVO_CH_LEFT_LEG);
                    servo_detach(SERVO_CH_RIGHT_LEG);
                    go_home();
                }
            }
        }
        
        elapsed = millis() - current_millis1;
        
        // Phase 1: Tilt to right
        if (elapsed <= interval1) {
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
            }
            servo_attach(SERVO_CH_LEFT_LEG);
            servo_attach(SERVO_CH_RIGHT_LEG);
            servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
        }
        
        // Phase 2: Right foot rotation BACKWARD
        elapsed = millis() - current_millis1;
        if (elapsed >= interval1 && elapsed <= interval2) {
            int right_foot_angle = calibration.rf_neutral + calibration.rfbwrs;
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_attach(SERVO_CH_RIGHT_FOOT);
            }
            servo_write(SERVO_CH_RIGHT_FOOT, right_foot_angle);
        }
        
        // Phase 3: Stop right foot, tilt to left
        elapsed = millis() - current_millis1;
        if (elapsed >= interval2 && elapsed <= interval3) {
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
            }
            servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
        }
        
        // Phase 4: Left foot rotation BACKWARD
        elapsed = millis() - current_millis1;
        if (elapsed >= interval3 && elapsed <= interval4) {
            int left_foot_angle = calibration.lf_neutral - calibration.lfbwrs;
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_attach(SERVO_CH_LEFT_FOOT);
            }
            servo_write(SERVO_CH_LEFT_FOOT, left_foot_angle);
        }
        
        // Phase 5: Stop left foot
        elapsed = millis() - current_millis1;
        if (elapsed >= interval4 && elapsed <= interval5) {
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
            }
        }
    }
}

// ========== ROLLING LOGIC ==========

void ninja_roll(void) {
    int8_t j_x = control_state.j_x;
    int8_t j_y = control_state.j_y;
    
    // Dead zone
    if ((j_x >= -10) && (j_x <= 10) && (j_y >= -10) && (j_y <= 10)) {
        ninja_roll_stop();
        return;
    }
    
    // Attach both servos
    if (!servo_attached(SERVO_CH_LEFT_FOOT)) {
        servo_attach(SERVO_CH_LEFT_FOOT);
    }
    if (!servo_attached(SERVO_CH_RIGHT_FOOT)) {
        servo_attach(SERVO_CH_RIGHT_FOOT);
    }
    
    // Roll mode mapping
    int lws = map_value(j_y, 100, -100, 135, 45);
    int rws = map_value(j_y, 100, -100, 45, 135);
    int lwd = map_value(j_x, 100, -100, 45, 0);
    int rwd = map_value(j_x, 100, -100, 0, -45);
    
    int left_speed = constrain_value(lws + lwd, 45, 135);
    int right_speed = constrain_value(rws + rwd, 45, 135);
    
    servo_write(SERVO_CH_LEFT_FOOT, left_speed);
    servo_write(SERVO_CH_RIGHT_FOOT, right_speed);
}

// ========== TEST FUNCTIONS ==========

void test_left_foot(void) {
    servo_attach(SERVO_CH_LEFT_FOOT);
    int angle = calibration.lf_neutral + calibration.lffwrs;
    servo_write(SERVO_CH_LEFT_FOOT, angle);
    ESP_LOGI(TAG, "[TEST LEFT FOOT] Angle=%d (Neutral=%d + Speed=%d)",
             angle, calibration.lf_neutral, calibration.lffwrs);
}

void test_right_foot(void) {
    servo_attach(SERVO_CH_RIGHT_FOOT);
    int angle = calibration.rf_neutral - calibration.rffwrs;
    servo_write(SERVO_CH_RIGHT_FOOT, angle);
    ESP_LOGI(TAG, "[TEST RIGHT FOOT] Angle=%d (Neutral=%d - Speed=%d)",
             angle, calibration.rf_neutral, calibration.rffwrs);
}

void test_both_feet(void) {
    int left_angle = calibration.lf_neutral + calibration.lffwrs;
    int right_angle = calibration.rf_neutral - calibration.rffwrs;
    
    servo_attach(SERVO_CH_LEFT_FOOT);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    servo_write(SERVO_CH_LEFT_FOOT, left_angle);
    servo_write(SERVO_CH_RIGHT_FOOT, right_angle);
    
    ESP_LOGI(TAG, "[TEST BOTH FEET] Left=%d Right=%d", left_angle, right_angle);
    
    // Run for 3 seconds then stop
    vTaskDelay(pdMS_TO_TICKS(3000));
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    ESP_LOGI(TAG, "[TEST BOTH FEET] 3 seconds completed - STOPPED");
    go_home();
}

void stop_feet_test(void) {
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    ESP_LOGI(TAG, "[TEST] Both feet stopped");
    go_home();
}

// ========== MAIN CONTROL TASK ==========

void robot_control_task(void *pvParameters) {
    static int8_t last_j_x = 0;
    static int8_t last_j_y = 0;
    
    while (1) {
        // Debug: Print joystick values when they change
        if (control_state.j_x != last_j_x || control_state.j_y != last_j_y) {
            const char *direction = "";
            
            if (control_state.j_x == 0 && control_state.j_y == 0) {
                direction = "STOP";
            } else if (control_state.j_y >= 0) {
                if (control_state.j_x > 20) direction = "FORWARD + TURN RIGHT";
                else if (control_state.j_x < -20) direction = "FORWARD + TURN LEFT";
                else if (control_state.j_y == 0 && control_state.j_x != 0) {
                    direction = (control_state.j_x > 0) ? "TURN RIGHT" : "TURN LEFT";
                } else direction = "FORWARD STRAIGHT";
            } else {
                if (control_state.j_x > 20) direction = "BACKWARD + TURN RIGHT";
                else if (control_state.j_x < -20) direction = "BACKWARD + TURN LEFT";
                else direction = "BACKWARD STRAIGHT";
            }
            
            ESP_LOGI(TAG, "Joystick: X=%d, Y=%d | Mode=%s | %s",
                     control_state.j_x, control_state.j_y,
                     current_mode == MODE_WALK ? "WALK" : "ROLL", direction);
            last_j_x = control_state.j_x;
            last_j_y = control_state.j_y;
        }
        
        // Handle mode buttons
        if (control_state.button_x == 1) {
            ninja_set_roll();
            control_state.button_x = 0;
        }
        if (control_state.button_y == 1) {
            ninja_set_walk();
            control_state.button_y = 0;
        }
        
        // Handle arm buttons
        if (control_state.button_a == 1) {
            ninja_left_arm_up();
        } else {
            ninja_left_arm_down();
        }
        
        if (control_state.button_b == 1) {
            ninja_right_arm_up();
        } else {
            ninja_right_arm_down();
        }
        
        // Process movement based on mode
        if (current_mode == MODE_WALK) {
            ninja_walk();
        } else {
            ninja_roll();
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
