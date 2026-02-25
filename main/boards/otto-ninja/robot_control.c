/*
 * Otto Ninja Robot - ESP-IDF Version
 * Robot Control Implementation
 * 
 * Servo control using LEDC PWM and walking logic
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "robot_control.h"
#include "led_strip.h"

static const char *TAG = "robot_ctrl";

// Servo configuration
#define SERVO_MIN_PULSEWIDTH_US 544
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE        0
#define SERVO_MAX_DEGREE        180
#define SERVO_FREQ_HZ           50
#define LEDC_TIMER              LEDC_TIMER_1  // Changed from TIMER_0 to avoid conflict with backlight
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
    .roll_lf_fwd_speed = CAL_DEFAULT_ROLL_LF_FWD,
    .roll_lf_bwd_speed = CAL_DEFAULT_ROLL_LF_BWD,
    .roll_rf_fwd_speed = CAL_DEFAULT_ROLL_RF_FWD,
    .roll_rf_bwd_speed = CAL_DEFAULT_ROLL_RF_BWD,
    .transform_ll_speed = CAL_DEFAULT_TRANSFORM_LL,
    .transform_rl_speed = CAL_DEFAULT_TRANSFORM_RL,
    .turn_left_speed = CAL_DEFAULT_TURN_L,
    .turn_right_speed = CAL_DEFAULT_TURN_R,
    .combo_lf_speed = CAL_DEFAULT_COMBO_LF,
    .combo_rf_speed = CAL_DEFAULT_COMBO_RF,
    .battery_alert_enabled = CAL_DEFAULT_BATTERY_ALERT,
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
    .manual_mode = false,
    .move_duration_ms = 0,  // 0 = unlimited duration
};

// Robot mode
static robot_mode_t current_mode = MODE_WALK;

// Walking state tracking
static bool was_moving = false;
static bool is_at_home = true;
static bool walk_cal_logged = false;
// Tracked arm positions for smooth go_home transitions
static int g_arm_left_pos = 180;  // home = 180
static int g_arm_right_pos = 0;   // home = 0
static bool walk_cycle_reset = false;
static uint32_t current_millis1 = 0;

// Single-step walk control (1 cycle per activation)
static bool walk_cycle_active = false;  // Currently executing 1 cycle
static bool walk_trigger_armed = true;  // Ready to start new cycle

// Forward declarations for static helper functions used in go_home
static void smooth_transform_legs(int ll_target, int rl_target);
static void servo_smooth_move(servo_channel_t ch, int from_angle, int to_angle, int duration_ms);

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
    // Start from channel 1 to avoid conflict with backlight (channel 0)
    for (int ch = 0; ch < SERVO_CH_MAX; ch++) {
        // Skip servos with no GPIO assigned (pin = -1)
        if (servo_gpio[ch] < 0) {
            servo_is_attached[ch] = false;
            ESP_LOGW(TAG, "Servo channel %d has no GPIO assigned, skipping", ch);
            continue;
        }
        ledc_channel_config_t channel_conf = {
            .gpio_num = servo_gpio[ch],
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)(ch + 1),  // Offset +1 to avoid channel 0 (backlight)
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
    
    // Load saved actions from NVS (3 slots)
    ESP_LOGI(TAG, "Loading saved actions from NVS...");
    for (int i = 0; i < MAX_ACTION_SLOTS; i++) {
        load_actions_from_nvs(i);
    }

    // Load built-in default dance into slot 1 if empty
    load_default_dance();

    // Initialize LED strip
    ninja_led_init();
    
    ESP_LOGI(TAG, "Robot control initialized");
}

void servo_write(servo_channel_t channel, int angle) {
    if (channel >= SERVO_CH_MAX) return;
    
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)(channel + 1), duty);  // +1 offset
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)(channel + 1));
}

void servo_attach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    servo_is_attached[channel] = true;
}

void servo_detach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    
    // Set duty to 0 to stop PWM signal
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)(channel + 1), 0);  // +1 offset
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)(channel + 1));
    servo_is_attached[channel] = false;
}

bool servo_attached(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return false;
    return servo_is_attached[channel];
}

// ========== NVS FUNCTIONS ==========

void reset_calibration_to_defaults(void) {
    calibration.lf_neutral = CAL_DEFAULT_LF_NEUTRAL;
    calibration.rf_neutral = CAL_DEFAULT_RF_NEUTRAL;
    calibration.lffwrs = CAL_DEFAULT_LFFWRS;
    calibration.rffwrs = CAL_DEFAULT_RFFWRS;
    calibration.lfbwrs = CAL_DEFAULT_LFBWRS;
    calibration.rfbwrs = CAL_DEFAULT_RFBWRS;
    calibration.la0 = CAL_DEFAULT_LA0;
    calibration.ra0 = CAL_DEFAULT_RA0;
    calibration.latl = CAL_DEFAULT_LATL;
    calibration.ratl = CAL_DEFAULT_RATL;
    calibration.latr = CAL_DEFAULT_LATR;
    calibration.ratr = CAL_DEFAULT_RATR;
    calibration.la1 = CAL_DEFAULT_LA1;
    calibration.ra1 = CAL_DEFAULT_RA1;
    calibration.roll_lf_fwd_speed = CAL_DEFAULT_ROLL_LF_FWD;
    calibration.roll_lf_bwd_speed = CAL_DEFAULT_ROLL_LF_BWD;
    calibration.roll_rf_fwd_speed = CAL_DEFAULT_ROLL_RF_FWD;
    calibration.roll_rf_bwd_speed = CAL_DEFAULT_ROLL_RF_BWD;
    calibration.turn_left_speed = CAL_DEFAULT_TURN_L;
    calibration.turn_right_speed = CAL_DEFAULT_TURN_R;
    calibration.combo_lf_speed = CAL_DEFAULT_COMBO_LF;
    calibration.combo_rf_speed = CAL_DEFAULT_COMBO_RF;
    calibration.battery_alert_enabled = CAL_DEFAULT_BATTERY_ALERT;
    
    ESP_LOGI(TAG, "🔄 Calibration reset to factory defaults!");
}

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
    nvs_set_i32(handle, "rlff", calibration.roll_lf_fwd_speed);
    nvs_set_i32(handle, "rlfb", calibration.roll_lf_bwd_speed);
    nvs_set_i32(handle, "rrff", calibration.roll_rf_fwd_speed);
    nvs_set_i32(handle, "rrfb", calibration.roll_rf_bwd_speed);
    nvs_set_i32(handle, "tll", calibration.transform_ll_speed);
    nvs_set_i32(handle, "trl", calibration.transform_rl_speed);
    nvs_set_i32(handle, "tls", calibration.turn_left_speed);
    nvs_set_i32(handle, "trs", calibration.turn_right_speed);
    nvs_set_i32(handle, "clf", calibration.combo_lf_speed);
    nvs_set_i32(handle, "crf", calibration.combo_rf_speed);
    nvs_set_i32(handle, "balert", calibration.battery_alert_enabled ? 1 : 0);
    
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
    if (nvs_get_i32(handle, "rlff", &val) == ESP_OK) calibration.roll_lf_fwd_speed = val;
    if (nvs_get_i32(handle, "rlfb", &val) == ESP_OK) calibration.roll_lf_bwd_speed = val;
    if (nvs_get_i32(handle, "rrff", &val) == ESP_OK) calibration.roll_rf_fwd_speed = val;
    if (nvs_get_i32(handle, "rrfb", &val) == ESP_OK) calibration.roll_rf_bwd_speed = val;
    if (nvs_get_i32(handle, "tll", &val) == ESP_OK) calibration.transform_ll_speed = val;
    if (nvs_get_i32(handle, "trl", &val) == ESP_OK) calibration.transform_rl_speed = val;
    if (nvs_get_i32(handle, "tls", &val) == ESP_OK) calibration.turn_left_speed = val;
    if (nvs_get_i32(handle, "trs", &val) == ESP_OK) calibration.turn_right_speed = val;
    if (nvs_get_i32(handle, "clf", &val) == ESP_OK) calibration.combo_lf_speed = val;
    if (nvs_get_i32(handle, "crf", &val) == ESP_OK) calibration.combo_rf_speed = val;
    if (nvs_get_i32(handle, "balert", &val) == ESP_OK) calibration.battery_alert_enabled = (val != 0);
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "📂 Settings loaded from NVS:");
    ESP_LOGI(TAG, "  Neutral: LF=%d, RF=%d", calibration.lf_neutral, calibration.rf_neutral);
    ESP_LOGI(TAG, "  Forward: LFF=%d, RFF=%d", calibration.lffwrs, calibration.rffwrs);
    ESP_LOGI(TAG, "  Backward: LFB=%d, RFB=%d", calibration.lfbwrs, calibration.rfbwrs);
    ESP_LOGI(TAG, "  Stand: LA0=%d, RA0=%d", calibration.la0, calibration.ra0);
    ESP_LOGI(TAG, "  Tilt: LATL=%d, RATL=%d, LATR=%d, RATR=%d", 
             calibration.latl, calibration.ratl, calibration.latr, calibration.ratr);
    ESP_LOGI(TAG, "  Roll: LA1=%d, RA1=%d", calibration.la1, calibration.ra1);
    ESP_LOGI(TAG, "  Roll Speed: LF_FWD=%d, LF_BWD=%d, RF_FWD=%d, RF_BWD=%d", 
             calibration.roll_lf_fwd_speed, calibration.roll_lf_bwd_speed,
             calibration.roll_rf_fwd_speed, calibration.roll_rf_bwd_speed);
    ESP_LOGI(TAG, "  Transform Speed: LL=%dms, RL=%dms", calibration.transform_ll_speed, calibration.transform_rl_speed);
    ESP_LOGI(TAG, "  Turn Speed: L=%dms, R=%dms", calibration.turn_left_speed, calibration.turn_right_speed);
    ESP_LOGI(TAG, "  Combo Speed: LF=%dms, RF=%dms", calibration.combo_lf_speed, calibration.combo_rf_speed);
}

calibration_t* get_calibration(void) {
    return &calibration;
}

void set_calibration(const calibration_t* cal) {
    if (cal) {
        memcpy(&calibration, cal, sizeof(calibration_t));
    }
}

bool is_battery_alert_enabled(void) {
    return calibration.battery_alert_enabled;
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
    
    // Smoothly move legs to HOME (smooth_transform_legs tracks current position)
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    smooth_transform_legs(calibration.la0, calibration.ra0);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Smoothly move arms to home using tracked positions
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_smooth_move(SERVO_CH_LEFT_ARM, g_arm_left_pos, 180, 350);
    servo_smooth_move(SERVO_CH_RIGHT_ARM, g_arm_right_pos, 0, 350);
    g_arm_left_pos = 180;
    g_arm_right_pos = 0;
    
    // Head to center (head is always 90, just ensure)
    servo_attach(SERVO_CH_HEAD);
    servo_write(SERVO_CH_HEAD, 90);
    vTaskDelay(pdMS_TO_TICKS(80));
    
    // Detach arms and head
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    servo_detach(SERVO_CH_HEAD);
    
    ESP_LOGI(TAG, "🏠 HOME: LeftLeg=%d, RightLeg=%d, Arms=180/0, Head=90",
             calibration.la0, calibration.ra0);
    ESP_LOGI(TAG, "🏠 HOME position reached!");
    
    is_at_home = true;
}

// Smooth transform legs with individual speeds for LL and RL
// Both legs move simultaneously but with independent delay per step
// Speed value = ms delay per degree (lower = faster)
static void smooth_transform_legs(int ll_target, int rl_target) {
    static int ll_current = -1;
    static int rl_current = -1;
    
    // Initialize current positions on first call
    if (ll_current < 0) ll_current = calibration.la0;
    if (rl_current < 0) rl_current = calibration.ra0;
    
    // Calculate directions
    int ll_dir = (ll_target > ll_current) ? 1 : -1;
    int rl_dir = (rl_target > rl_current) ? 1 : -1;
    
    // Calculate total steps needed for each leg
    int ll_steps_remaining = abs(ll_target - ll_current);
    int rl_steps_remaining = abs(rl_target - rl_current);
    
    ESP_LOGI(TAG, "  🦿 Smooth transform: LL %d→%d (speed=%dms/deg), RL %d→%d (speed=%dms/deg)",
             ll_current, ll_target, calibration.transform_ll_speed,
             rl_current, rl_target, calibration.transform_rl_speed);
    
    // Move both legs with their own speed
    int ll_timer = 0;
    int rl_timer = 0;
    int max_iterations = 5000; // Safety limit
    
    while ((ll_steps_remaining > 0 || rl_steps_remaining > 0) && max_iterations > 0) {
        max_iterations--;
        
        // Move LL if timer reached
        if (ll_steps_remaining > 0) {
            ll_timer++;
            if (ll_timer >= calibration.transform_ll_speed) {
                ll_current += ll_dir;
                servo_write(SERVO_CH_LEFT_LEG, ll_current);
                ll_steps_remaining--;
                ll_timer = 0;
            }
        }
        
        // Move RL if timer reached
        if (rl_steps_remaining > 0) {
            rl_timer++;
            if (rl_timer >= calibration.transform_rl_speed) {
                rl_current += rl_dir;
                servo_write(SERVO_CH_RIGHT_LEG, rl_current);
                rl_steps_remaining--;
                rl_timer = 0;
            }
        }
        
        // Delay 1ms per iteration
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Ensure final positions
    servo_write(SERVO_CH_LEFT_LEG, ll_target);
    servo_write(SERVO_CH_RIGHT_LEG, rl_target);
    ll_current = ll_target;
    rl_current = rl_target;
    
    ESP_LOGI(TAG, "  🦿 Transform complete: LL=%d, RL=%d", ll_current, rl_current);
}

void ninja_set_walk(void) {
    ESP_LOGI(TAG, ">>> ninja_set_walk() START - Đứng dậy chế độ đi bộ");
    
    // Step 1: Attach ALL servos first for synchronized movement
    ESP_LOGI(TAG, "  Step 1: Attaching all servos...");
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    // Step 2: Arms to neutral 90° first
    ESP_LOGI(TAG, "  Step 2: Arms to neutral 90°");
    servo_write(SERVO_CH_LEFT_ARM, 90);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
    g_arm_left_pos = 90;
    g_arm_right_pos = 90;
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Step 3: SMOOTH TRANSFORM - BOTH LEGS with individual speeds
    ESP_LOGI(TAG, "  Step 3: SMOOTH transform to WALK position");
    smooth_transform_legs(calibration.la0, calibration.ra0);
    
    // Step 4: Arms to final position
    ESP_LOGI(TAG, "  Step 4: Arms to final position LA=180, RA=0");
    servo_write(SERVO_CH_LEFT_ARM, 180);
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    g_arm_left_pos = 180;
    g_arm_right_pos = 0;
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Detach all servos at the end
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
    
    current_mode = MODE_WALK;
    ESP_LOGI(TAG, "<<< ninja_set_walk() COMPLETED - Mode: WALK");
}

void ninja_set_roll(void) {
    ESP_LOGI(TAG, ">>> ninja_set_roll() START - Biến hình thành xe");
    
    // Step 1: Attach ALL servos first for synchronized movement
    ESP_LOGI(TAG, "  Step 1: Attaching all servos...");
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    // Step 2: Arms to neutral 90° first
    ESP_LOGI(TAG, "  Step 2: Arms to neutral 90°");
    servo_write(SERVO_CH_LEFT_ARM, 90);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
    g_arm_left_pos = 90;
    g_arm_right_pos = 90;
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Step 3: SMOOTH TRANSFORM - BOTH LEGS with individual speeds
    ESP_LOGI(TAG, "  Step 3: SMOOTH transform to ROLL position");
    smooth_transform_legs(calibration.la1, calibration.ra1);
    
    // Step 4: Arms down to final position
    ESP_LOGI(TAG, "  Step 4: Arms to final position LA=180, RA=0");
    servo_write(SERVO_CH_LEFT_ARM, 180);
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    g_arm_left_pos = 180;
    g_arm_right_pos = 0;
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Detach all servos at the end
    servo_detach(SERVO_CH_LEFT_ARM);
    servo_detach(SERVO_CH_RIGHT_ARM);
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
    
    current_mode = MODE_ROLL;
    ESP_LOGI(TAG, "<<< ninja_set_roll() COMPLETED - Mode: ROLL");
}

void ninja_left_arm_up(void) {
    servo_attach(SERVO_CH_LEFT_ARM);
    servo_write(SERVO_CH_LEFT_ARM, 90);
    g_arm_left_pos = 90;
}

void ninja_left_arm_down(void) {
    servo_write(SERVO_CH_LEFT_ARM, 180);
    g_arm_left_pos = 180;
}

void ninja_right_arm_up(void) {
    servo_attach(SERVO_CH_RIGHT_ARM);
    servo_write(SERVO_CH_RIGHT_ARM, 90);
    g_arm_right_pos = 90;
}

void ninja_right_arm_down(void) {
    servo_write(SERVO_CH_RIGHT_ARM, 0);
    g_arm_right_pos = 0;
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
    
    // Joystick returned to neutral - ARM trigger for next cycle
    if (j_x == 0 && j_y == 0 && !control_state.test_mode_active) {
        // Stop any ongoing movement
        if (walk_cycle_active) {
            ninja_walk_stop();
            servo_detach(SERVO_CH_LEFT_FOOT);
            servo_detach(SERVO_CH_RIGHT_FOOT);
            servo_detach(SERVO_CH_LEFT_LEG);
            servo_detach(SERVO_CH_RIGHT_LEG);
            walk_cycle_active = false;
            walk_cal_logged = false;
            walk_cycle_reset = false;
            ESP_LOGI(TAG, "✅ Walk cycle completed - Trigger ARMED for next step");
        }
        
        // Arm trigger for next cycle
        walk_trigger_armed = true;
        
        if (was_moving && !is_at_home) {
            was_moving = false;
            is_at_home = true;
            go_home();
        }
        return;
    }
    
    // Joystick activated - START new cycle ONLY if trigger is armed
    if (!walk_cycle_active && walk_trigger_armed) {
        ESP_LOGI(TAG, "🎯 Walk trigger FIRED - Starting 1 cycle (X=%d, Y=%d)", j_x, j_y);
        walk_cycle_active = true;
        walk_trigger_armed = false;  // Disarm until joystick returns to neutral
        walk_cycle_reset = false;
    }
    
    // If cycle is not active (already completed), do nothing
    if (!walk_cycle_active) {
        return;
    }
    
    // Reset timing when starting new walk cycle
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
            // Cycle completed - Continue walking if joystick still held
            ESP_LOGI(TAG, "✅ Forward walk cycle COMPLETED - Starting next cycle");
            last_phase = 0;
            
            // Reset for next cycle immediately (continuous walking)
            current_millis1 = millis();
            walk_cycle_reset = true;
            
            // Handle test mode
            if (control_state.test_mode_active && control_state.test_cycles_remaining > 0) {
                control_state.test_cycles_remaining--;
                ESP_LOGI(TAG, "[TEST MODE] Forward cycle done, %d remaining", control_state.test_cycles_remaining);
                if (control_state.test_cycles_remaining == 0) {
                    control_state.test_mode_active = false;
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    walk_cycle_active = false;
                    walk_cycle_reset = false;
                    ninja_walk_stop();
                    servo_detach(SERVO_CH_LEFT_FOOT);
                    servo_detach(SERVO_CH_RIGHT_FOOT);
                    servo_detach(SERVO_CH_LEFT_LEG);
                    servo_detach(SERVO_CH_RIGHT_LEG);
                    go_home();
                    return;
                }
            }
            // Continue to next cycle without returning
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
            // Cycle completed - Continue walking if joystick still held
            ESP_LOGI(TAG, "✅ Backward walk cycle COMPLETED - Starting next cycle");
            last_back_phase = 0;
            
            // Reset for next cycle immediately (continuous walking)
            current_millis1 = millis();
            walk_cycle_reset = true;
            
            // Handle test mode
            if (control_state.test_mode_active && control_state.test_cycles_remaining > 0) {
                control_state.test_cycles_remaining--;
                ESP_LOGI(TAG, "[TEST MODE] Backward cycle done, %d remaining", control_state.test_cycles_remaining);
                if (control_state.test_cycles_remaining == 0) {
                    control_state.test_mode_active = false;
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    walk_cycle_active = false;
                    walk_cycle_reset = false;
                    ninja_walk_stop();
                    servo_detach(SERVO_CH_LEFT_FOOT);
                    servo_detach(SERVO_CH_RIGHT_FOOT);
                    servo_detach(SERVO_CH_LEFT_LEG);
                    servo_detach(SERVO_CH_RIGHT_LEG);
                    go_home();
                    return;
                }
            }
            // Continue to next cycle without returning
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
    
    static bool roll_logged = false;
    
    // Dead zone
    if ((j_x >= -10) && (j_x <= 10) && (j_y >= -10) && (j_y <= 10)) {
        ninja_roll_stop();
        roll_logged = false;  // Reset logging flag when stopped
        return;
    }
    
    // Attach both servos
    if (!servo_attached(SERVO_CH_LEFT_FOOT)) {
        servo_attach(SERVO_CH_LEFT_FOOT);
        ESP_LOGI(TAG, "🚗 [ROLL] Attached LEFT_FOOT servo");
    }
    if (!servo_attached(SERVO_CH_RIGHT_FOOT)) {
        servo_attach(SERVO_CH_RIGHT_FOOT);
        ESP_LOGI(TAG, "🚗 [ROLL] Attached RIGHT_FOOT servo");
    }
    
    // Use separate configurable roll speeds for LF and RF
    int lf_fwd_speed = calibration.roll_lf_fwd_speed;  // Left Foot forward speed (default 45)
    int lf_bwd_speed = calibration.roll_lf_bwd_speed;  // Left Foot backward speed (default 45)
    int rf_fwd_speed = calibration.roll_rf_fwd_speed;  // Right Foot forward speed (default 45)
    int rf_bwd_speed = calibration.roll_rf_bwd_speed;  // Right Foot backward speed (default 45)
    
    // Calculate speed offset based on direction for each foot
    // Forward: j_y > 0, Backward: j_y < 0
    int lf_speed_offset = (j_y >= 0) ? lf_fwd_speed : lf_bwd_speed;
    int rf_speed_offset = (j_y >= 0) ? rf_fwd_speed : rf_bwd_speed;
    
    // Map joystick Y to servo speed for each foot independently
    // Left Foot: 90 = stop, 90+speed = forward, 90-speed = backward
    int lws = map_value(j_y, 100, -100, 90 + lf_speed_offset, 90 - lf_speed_offset);
    // Right Foot: 90 = stop, 90-speed = forward, 90+speed = backward
    int rws = map_value(j_y, 100, -100, 90 - rf_speed_offset, 90 + rf_speed_offset);
    
    // Steering adjustment using average of both speeds
    int avg_speed = (lf_speed_offset + rf_speed_offset) / 2;
    int lwd = map_value(j_x, 100, -100, avg_speed, 0);
    int rwd = map_value(j_x, 100, -100, 0, -avg_speed);
    
    // Apply steering with individual speed limits
    int left_speed = constrain_value(lws + lwd, 90 - lf_speed_offset, 90 + lf_speed_offset);
    int right_speed = constrain_value(rws + rwd, 90 - rf_speed_offset, 90 + rf_speed_offset);
    
    // Log roll details once per activation
    if (!roll_logged) {
        const char* direction = "";
        if (j_y >= 20) direction = "FORWARD";
        else if (j_y <= -20) direction = "BACKWARD";
        else direction = "NEUTRAL";
        
        const char* steering = "";
        if (j_x >= 20) steering = " + RIGHT TURN";
        else if (j_x <= -20) steering = " + LEFT TURN";
        else steering = "";
        
        ESP_LOGI(TAG, "🚗 [ROLL] === MOVEMENT START ===");
        ESP_LOGI(TAG, "🚗 [ROLL] Joystick: X=%d, Y=%d (%s%s)", j_x, j_y, direction, steering);
        ESP_LOGI(TAG, "🚗 [ROLL] Speed Config: LF_FWD=%d LF_BWD=%d RF_FWD=%d RF_BWD=%d", 
                 lf_fwd_speed, lf_bwd_speed, rf_fwd_speed, rf_bwd_speed);
        ESP_LOGI(TAG, "🚗 [ROLL] Active Offsets: LF=%d RF=%d (AVG=%d)", 
                 lf_speed_offset, rf_speed_offset, avg_speed);
        ESP_LOGI(TAG, "🚗 [ROLL] Servo Angles: LEFT=%d RIGHT=%d", left_speed, right_speed);
        ESP_LOGI(TAG, "🚗 [ROLL] === ROLLING NOW ===");
        roll_logged = true;
    }
    
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
        // Skip normal control if in manual mode
        if (control_state.manual_mode) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
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
        
        // Update LED strip animation
        ninja_led_update();
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz is sufficient for servo robot, gives IDLE0 time to run
    }
}

// ========== TILT FUNCTIONS ==========

void ninja_tilt_left(void) {
    control_state.manual_mode = true;  // Enable manual mode
    ESP_LOGI(TAG, "🔄 Tilt LEFT and HOLD (manual mode ON)");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
}

void ninja_tilt_right(void) {
    control_state.manual_mode = true;  // Enable manual mode
    ESP_LOGI(TAG, "🔄 Tilt RIGHT and HOLD (manual mode ON)");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
}

// ========== DIRECT SERVO CONTROL ==========

void servo_direct_write(int channel, int angle) {
    if (channel < 0 || channel >= SERVO_CH_MAX) return;
    control_state.manual_mode = true;  // Enable manual mode
    servo_attach((servo_channel_t)channel);
    servo_write((servo_channel_t)channel, angle);
    ESP_LOGI(TAG, "Direct servo CH%d = %d° (manual mode ON)", channel, angle);
}

void set_manual_mode(bool enable) {
    control_state.manual_mode = enable;
    if (!enable) {
        ESP_LOGI(TAG, "Manual mode OFF - returning to normal control");
    }
}

// ========== COMBO FUNCTIONS ==========

void ninja_combo1(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "🎯 COMBO 1 START");
    
    // Get speed from calibration (default 1000 if not set)
    int lf_speed_ms = (calibration.combo_lf_speed > 0) ? calibration.combo_lf_speed : 1000;
    
    // Step 1: Tilt left and wait 1s
    ESP_LOGI(TAG, "  Step 1: Tilt LEFT...");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Wave Right Leg (RL) 3 times: 135 -> 155 -> 180
    ESP_LOGI(TAG, "  Step 2: Wave Right Leg 3 times (135->155->180)");
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_RIGHT_LEG, 135);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 155);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 180);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "    Wave %d/3 complete", i + 1);
    }
    
    // Step 3: Rotate Left Foot (LF) using configured speed
    ESP_LOGI(TAG, "  Step 3: Rotate Left Foot for %dms", lf_speed_ms);
    servo_attach(SERVO_CH_LEFT_FOOT);
    int lf_angle = calibration.lf_neutral + calibration.lffwrs;
    servo_write(SERVO_CH_LEFT_FOOT, lf_angle);
    vTaskDelay(pdMS_TO_TICKS(lf_speed_ms));
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    
    // Step 4: Return to home
    ESP_LOGI(TAG, "  Step 4: Return to HOME");
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "🎯 COMBO 1 COMPLETE");
}

void ninja_combo2(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "🎯 COMBO 2 START");
    
    // Get speed from calibration (default 1000 if not set)
    int rf_speed_ms = (calibration.combo_rf_speed > 0) ? calibration.combo_rf_speed : 1000;
    
    // Step 1: Tilt right and wait 1s
    ESP_LOGI(TAG, "  Step 1: Tilt RIGHT...");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Wave Left Leg (LL) 3 times: 10 -> 30 -> 75
    ESP_LOGI(TAG, "  Step 2: Wave Left Leg 3 times (10->30->75)");
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_LEFT_LEG, 10);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 30);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 75);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "    Wave %d/3 complete", i + 1);
    }
    
    // Step 3: Rotate Right Foot (RF) using configured speed
    ESP_LOGI(TAG, "  Step 3: Rotate Right Foot for %dms", rf_speed_ms);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    int rf_angle = calibration.rf_neutral - calibration.rffwrs;
    servo_write(SERVO_CH_RIGHT_FOOT, rf_angle);
    vTaskDelay(pdMS_TO_TICKS(rf_speed_ms));
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    
    // Step 4: Return to home
    ESP_LOGI(TAG, "  Step 4: Return to HOME");
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "🎯 COMBO 2 COMPLETE");
}

// ========== RHYTHM/DANCE FUNCTIONS ==========

// Helper function for smooth servo movement
static void servo_smooth_move(servo_channel_t ch, int from_angle, int to_angle, int duration_ms) {
    int steps = 20;  // Number of interpolation steps
    int delay_per_step = duration_ms / steps;
    if (delay_per_step < 10) delay_per_step = 10;
    
    for (int i = 0; i <= steps; i++) {
        int angle = from_angle + ((to_angle - from_angle) * i) / steps;
        servo_write(ch, angle);
        vTaskDelay(pdMS_TO_TICKS(delay_per_step));
    }
}

// ========== WALK PHASE CONTROL FUNCTIONS ==========

// Phase 1: Tilt to right (prepare for right foot step)
void ninja_walk_phase1(void) {
    ESP_LOGI(TAG, "[PHASE 1] Tilt to right");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
    vTaskDelay(pdMS_TO_TICKS(300));
}

// Phase 2: Right foot rotation forward
void ninja_walk_phase2(void) {
    ESP_LOGI(TAG, "[PHASE 2] Right foot forward");
    servo_attach(SERVO_CH_RIGHT_FOOT);
    int right_foot_angle = calibration.rf_neutral - calibration.rffwrs;
    servo_write(SERVO_CH_RIGHT_FOOT, right_foot_angle);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Phase 3: Tilt to left (prepare for left foot step)
void ninja_walk_phase3(void) {
    ESP_LOGI(TAG, "[PHASE 3] Tilt to left");
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
    vTaskDelay(pdMS_TO_TICKS(300));
}

// Phase 4: Left foot rotation forward
void ninja_walk_phase4(void) {
    ESP_LOGI(TAG, "[PHASE 4] Left foot forward");
    servo_attach(SERVO_CH_LEFT_FOOT);
    int left_foot_angle = calibration.lf_neutral + calibration.lffwrs;
    servo_write(SERVO_CH_LEFT_FOOT, left_foot_angle);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Phase 5: Stop and return to neutral
void ninja_walk_phase5(void) {
    ESP_LOGI(TAG, "[PHASE 5] Return neutral");
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    vTaskDelay(pdMS_TO_TICKS(100));
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
}

// Walk Combo 1: Phases 1+2+3 (Tilt right -> RF forward -> Tilt left)
void ninja_walk_combo_123(void) {
    ESP_LOGI(TAG, "🎯 WALK COMBO 1-2-3");
    ninja_walk_phase1();  // Tilt right
    ninja_walk_phase2();  // RF forward
    ninja_walk_phase3();  // Tilt left
    ESP_LOGI(TAG, "✅ COMBO 1-2-3 complete");
}

// Walk Combo 2: Phases 3+4+5 (Tilt left -> LF forward -> Neutral)
void ninja_walk_combo_345(void) {
    ESP_LOGI(TAG, "🎯 WALK COMBO 3-4-5");
    ninja_walk_phase3();  // Tilt left
    ninja_walk_phase4();  // LF forward
    ninja_walk_phase5();  // Return neutral
    ESP_LOGI(TAG, "✅ COMBO 3-4-5 complete");
}

// ========== RHYTHM/DANCE FUNCTIONS ==========

void left_leg_rhythm(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "💃 LEFT LEG RHYTHM START");
    
    servo_attach(SERVO_CH_LEFT_LEG);
    
    // Target positions: 34 -> 45 -> 65 degrees
    int positions[] = {34, 45, 65};
    int num_positions = 3;
    int repeat_each = 3;
    int neutral = calibration.la0;  // Standing position
    int current_angle = neutral;
    
    // Smooth and slow rhythm with interpolation
    for (int i = 0; i < num_positions; i++) {
        ESP_LOGI(TAG, "  Position %d: %d°", i + 1, positions[i]);
        for (int j = 0; j < repeat_each; j++) {
            // Smooth move to target position (300ms)
            servo_smooth_move(SERVO_CH_LEFT_LEG, current_angle, positions[i], 300);
            current_angle = positions[i];
            vTaskDelay(pdMS_TO_TICKS(100));  // Hold at position
            
            // Smooth return to neutral (300ms)
            servo_smooth_move(SERVO_CH_LEFT_LEG, current_angle, neutral, 300);
            current_angle = neutral;
            vTaskDelay(pdMS_TO_TICKS(100));  // Hold at neutral
            
            ESP_LOGI(TAG, "    Beat %d/%d", j + 1, repeat_each);
        }
    }
    
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "💃 LEFT LEG RHYTHM COMPLETE");
}

void right_leg_rhythm(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "💃 RIGHT LEG RHYTHM START");
    
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    // Target positions: 140 -> 150 -> 170 degrees
    int positions[] = {140, 150, 170};
    int num_positions = 3;
    int repeat_each = 3;
    int neutral = calibration.ra0;  // Standing position
    int current_angle = neutral;
    
    // Smooth and slow rhythm with interpolation
    for (int i = 0; i < num_positions; i++) {
        ESP_LOGI(TAG, "  Position %d: %d°", i + 1, positions[i]);
        for (int j = 0; j < repeat_each; j++) {
            // Smooth move to target position (300ms)
            servo_smooth_move(SERVO_CH_RIGHT_LEG, current_angle, positions[i], 300);
            current_angle = positions[i];
            vTaskDelay(pdMS_TO_TICKS(100));  // Hold at position
            
            // Smooth return to neutral (300ms)
            servo_smooth_move(SERVO_CH_RIGHT_LEG, current_angle, neutral, 300);
            current_angle = neutral;
            vTaskDelay(pdMS_TO_TICKS(100));  // Hold at neutral
            
            ESP_LOGI(TAG, "    Beat %d/%d", j + 1, repeat_each);
        }
    }
    
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "💃 RIGHT LEG RHYTHM COMPLETE");
}

// ========== ACTION RECORDING SYSTEM ==========

// Action slots storage
static action_slot_t action_slots[MAX_ACTION_SLOTS] = {0};

// Recording state
static recording_state_t recording_state = {
    .is_recording = false,
    .current_slot = 0,
    .step_count = 0
};

recording_state_t* get_recording_state(void) {
    return &recording_state;
}

action_slot_t* get_action_slot(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return NULL;
    return &action_slots[slot];
}

void start_recording(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) {
        ESP_LOGE(TAG, "❌ Invalid slot %d (max %d)", slot, MAX_ACTION_SLOTS - 1);
        return;
    }
    
    recording_state.is_recording = true;
    recording_state.current_slot = slot;
    recording_state.step_count = 0;
    
    // Clear the slot
    memset(&action_slots[slot], 0, sizeof(action_slot_t));
    
    ESP_LOGI(TAG, "🎬 START RECORDING to Slot %d (max %d actions)", slot + 1, MAX_ACTIONS);
}

void stop_recording(void) {
    if (!recording_state.is_recording) {
        ESP_LOGW(TAG, "⚠️ Not recording!");
        return;
    }
    
    uint8_t slot = recording_state.current_slot;
    action_slots[slot].count = recording_state.step_count;
    
    ESP_LOGI(TAG, "⏹️ STOP RECORDING - Slot %d saved %d/%d actions", 
             slot + 1, recording_state.step_count, MAX_ACTIONS);
    
    // Save to NVS
    save_actions_to_nvs(slot);
    
    recording_state.is_recording = false;
}

void record_action(action_type_t type, int16_t param1, int16_t param2, uint16_t duration) {
    if (!recording_state.is_recording) {
        ESP_LOGW(TAG, "⚠️ Not in recording mode");
        return;
    }
    if (recording_state.step_count >= MAX_ACTIONS) {
        ESP_LOGW(TAG, "⚠️ Recording FULL! (%d/%d actions)", MAX_ACTIONS, MAX_ACTIONS);
        return;
    }
    
    uint8_t slot = recording_state.current_slot;
    uint8_t idx = recording_state.step_count;
    
    action_slots[slot].steps[idx].type = type;
    action_slots[slot].steps[idx].param1 = param1;
    action_slots[slot].steps[idx].param2 = param2;
    action_slots[slot].steps[idx].duration_ms = duration;
    
    recording_state.step_count++;
    action_slots[slot].count = recording_state.step_count;  // Update count immediately
    
    ESP_LOGI(TAG, "📝 Recorded [%d/%d]: type=%d p1=%d p2=%d dur=%dms", 
             recording_state.step_count, MAX_ACTIONS, type, param1, param2, duration);
}

void play_action(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) {
        ESP_LOGE(TAG, "❌ Invalid slot %d", slot);
        return;
    }
    
    action_slot_t* actions = &action_slots[slot];
    if (actions->count == 0) {
        ESP_LOGW(TAG, "⚠️ Slot %d is EMPTY - nothing to play", slot + 1);
        return;
    }
    
    ESP_LOGI(TAG, "▶️ PLAYING Slot %d (%d actions)...", slot + 1, actions->count);
    
    control_state.manual_mode = true;
    
    for (int i = 0; i < actions->count; i++) {
        action_step_t* step = &actions->steps[i];
        
        ESP_LOGI(TAG, "  Action %d: type=%d", i + 1, step->type);
        
        switch (step->type) {
            case ACTION_JOYSTICK:
                control_state.j_x = step->param1;
                control_state.j_y = step->param2;
                control_state.manual_mode = false;  // Allow control task to process
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms));
                control_state.j_x = 0;
                control_state.j_y = 0;
                control_state.manual_mode = true;
                break;
                
            case ACTION_BUTTON_A:
                ninja_left_arm_up();
                vTaskDelay(pdMS_TO_TICKS(300));
                ninja_left_arm_down();
                break;
                
            case ACTION_BUTTON_B:
                ninja_right_arm_up();
                vTaskDelay(pdMS_TO_TICKS(300));
                ninja_right_arm_down();
                break;
                
            case ACTION_WALK_MODE:
                ninja_set_walk();
                break;
                
            case ACTION_ROLL_MODE:
                ninja_set_roll();
                break;
                
            case ACTION_HOME:
                control_state.manual_mode = false;
                go_home();
                control_state.manual_mode = true;
                break;
                
            case ACTION_TILT_LEFT:
                ninja_tilt_left();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 500));
                break;
                
            case ACTION_TILT_RIGHT:
                ninja_tilt_right();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 500));
                break;
                
            case ACTION_SERVO:
                servo_direct_write(step->param1, step->param2);
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 200));
                break;
                
            case ACTION_DELAY:
                vTaskDelay(pdMS_TO_TICKS(step->param1));
                break;
            
            case ACTION_COMBO1:
                ESP_LOGI(TAG, "  Playing COMBO 1...");
                ninja_combo1();
                break;
                
            case ACTION_COMBO2:
                ESP_LOGI(TAG, "  Playing COMBO 2...");
                ninja_combo2();
                break;
            
            case ACTION_RHYTHM_LEFT:
                ESP_LOGI(TAG, "  Playing LEFT LEG RHYTHM...");
                left_leg_rhythm();
                break;
                
            case ACTION_RHYTHM_RIGHT:
                ESP_LOGI(TAG, "  Playing RIGHT LEG RHYTHM...");
                right_leg_rhythm();
                break;
            
            case ACTION_WALK_COMBO_123:
                ESP_LOGI(TAG, "  Playing WALK COMBO 1-2-3...");
                ninja_walk_combo_123();
                break;
            
            case ACTION_WALK_COMBO_345:
                ESP_LOGI(TAG, "  Playing WALK COMBO 3-4-5...");
                ninja_walk_combo_345();
                break;
            
            case ACTION_TEST_LF:
                ESP_LOGI(TAG, "  Playing TEST LEFT FOOT...");
                test_left_foot();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 1000));
                stop_feet_test();
                break;
            
            case ACTION_TEST_RF:
                ESP_LOGI(TAG, "  Playing TEST RIGHT FOOT...");
                test_right_foot();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 1000));
                stop_feet_test();
                break;
            
            case ACTION_TEST_BOTH:
                ESP_LOGI(TAG, "  Playing TEST BOTH FEET...");
                test_both_feet();
                break;
            
            case ACTION_TEST_STOP:
                ESP_LOGI(TAG, "  Playing STOP FEET TEST...");
                stop_feet_test();
                break;
            
            case ACTION_MOVE_FWD:
                ESP_LOGI(TAG, "  Playing MOVE FORWARD (%dms)...", step->param1);
                control_state.j_y = 80;
                control_state.j_x = 0;
                control_state.manual_mode = false;
                vTaskDelay(pdMS_TO_TICKS(step->param1 > 0 ? step->param1 : 1000));
                control_state.j_y = 0;
                control_state.j_x = 0;
                control_state.manual_mode = true;
                break;
            
            case ACTION_MOVE_BWD:
                ESP_LOGI(TAG, "  Playing MOVE BACKWARD (%dms)...", step->param1);
                control_state.j_y = -80;
                control_state.j_x = 0;
                control_state.manual_mode = false;
                vTaskDelay(pdMS_TO_TICKS(step->param1 > 0 ? step->param1 : 1000));
                control_state.j_y = 0;
                control_state.j_x = 0;
                control_state.manual_mode = true;
                break;
            
            case ACTION_MOVE_STOP:
                ESP_LOGI(TAG, "  Playing MOVE STOP...");
                control_state.j_y = 0;
                control_state.j_x = 0;
                break;
            
            case ACTION_TURN_LEFT: {
                int spd = step->param1 > 0 ? step->param1 : 500;
                ESP_LOGI(TAG, "  Playing TURN LEFT (speed=%dms)...", spd);
                robot_mode_t mode = get_robot_mode();
                if (mode == MODE_WALK) {
                    ninja_tilt_left();
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    calibration_t *cal = get_calibration();
                    int lf_angle = cal->lf_neutral + cal->lffwrs;
                    servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    servo_direct_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
                    control_state.manual_mode = false;
                    go_home();
                    control_state.manual_mode = true;
                } else {
                    control_state.j_x = -75;
                    control_state.j_y = -64;
                    control_state.manual_mode = false;
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    control_state.manual_mode = true;
                }
                break;
            }
            
            case ACTION_TURN_RIGHT: {
                int spd = step->param1 > 0 ? step->param1 : 500;
                ESP_LOGI(TAG, "  Playing TURN RIGHT (speed=%dms)...", spd);
                robot_mode_t mode = get_robot_mode();
                if (mode == MODE_WALK) {
                    ninja_tilt_right();
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    calibration_t *cal = get_calibration();
                    int rf_angle = cal->rf_neutral - cal->rffwrs;
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
                    control_state.manual_mode = false;
                    go_home();
                    control_state.manual_mode = true;
                } else {
                    control_state.j_x = 51;
                    control_state.j_y = -81;
                    control_state.manual_mode = false;
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    control_state.manual_mode = true;
                }
                break;
            }
                
            case ACTION_SPIN_IN_PLACE: {
                int dur = step->param1 > 0 ? step->param1 : 2000;
                ESP_LOGI(TAG, "  Playing SPIN IN PLACE (dur=%dms)...", dur);
                control_state.j_x = 0;
                control_state.j_y = 0;
                vTaskDelay(pdMS_TO_TICKS(80));
                control_state.j_x = 100;
                control_state.j_y = 5;
                vTaskDelay(pdMS_TO_TICKS(dur));
                control_state.j_x = 0;
                control_state.j_y = 0;
                break;
            }

            case ACTION_SPIN_RF: {
                int dur = step->param1 > 0 ? step->param1 : 2000;
                int offset = step->param2 > 0 ? step->param2 : 30;
                calibration_t *cal = get_calibration();
                ESP_LOGI(TAG, "  Playing SPIN RF (dur=%dms offset=%d)...", dur, offset);
                control_state.manual_mode = true;
                servo_attach(SERVO_CH_RIGHT_FOOT);
                int rf_angle = cal->rf_neutral - offset;
                if (rf_angle < 0) rf_angle = 0;
                servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                vTaskDelay(pdMS_TO_TICKS(dur));
                servo_direct_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
                break;
            }

            case ACTION_SPIN_LF: {
                int dur = step->param1 > 0 ? step->param1 : 2000;
                int offset = step->param2 > 0 ? step->param2 : 30;
                calibration_t *cal = get_calibration();
                ESP_LOGI(TAG, "  Playing SPIN LF (dur=%dms offset=%d)...", dur, offset);
                control_state.manual_mode = true;
                servo_attach(SERVO_CH_LEFT_FOOT);
                int lf_angle = cal->lf_neutral + offset;
                if (lf_angle > 180) lf_angle = 180;
                servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                vTaskDelay(pdMS_TO_TICKS(dur));
                servo_direct_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
                break;
            }

            case ACTION_WAVE_RIGHT_LEG: {
                ESP_LOGI(TAG, "  Playing WAVE RIGHT LEG...");
                control_state.manual_mode = true;
                servo_attach(SERVO_CH_RIGHT_LEG);
                for (int i = 0; i < 3; i++) {
                    servo_write(SERVO_CH_RIGHT_LEG, 135);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_RIGHT_LEG, 155);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_RIGHT_LEG, 180);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                break;
            }

            case ACTION_WAVE_LEFT_LEG: {
                ESP_LOGI(TAG, "  Playing WAVE LEFT LEG...");
                control_state.manual_mode = true;
                servo_attach(SERVO_CH_LEFT_LEG);
                for (int i = 0; i < 3; i++) {
                    servo_write(SERVO_CH_LEFT_LEG, 10);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_LEFT_LEG, 30);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_LEFT_LEG, 75);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                break;
            }

            default:
                break;
        }  // end switch
    }  // end for

    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "Play slot done");
}

void load_default_dance(void) {
    if (action_slots[0].count > 0) {
        ESP_LOGI(TAG, "💃 Default dance: Slot 1 already has %d steps, skipping", action_slots[0].count);
        return;
    }
    ESP_LOGI(TAG, "💃 Loading default dance into slot 1...");
    memset(&action_slots[0], 0, sizeof(action_slot_t));
    action_step_t *s = action_slots[0].steps;
    int i = 0;
    s[i++] = (action_step_t){ACTION_HOME,          0,    0,  0};
    s[i++] = (action_step_t){ACTION_TILT_LEFT,     0,    0,  500};
    s[i++] = (action_step_t){ACTION_WAVE_LEFT_LEG, 0,    0,  0};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 11, 0};
    s[i++] = (action_step_t){ACTION_SPIN_LF,       500,  5,  0};
    s[i++] = (action_step_t){ACTION_TILT_RIGHT,    0,    0,  500};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 8,  0};
    s[i++] = (action_step_t){ACTION_WAVE_RIGHT_LEG,0,    0,  0};
    s[i++] = (action_step_t){ACTION_TILT_LEFT,     0,    0,  500};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 10, 0};
    s[i++] = (action_step_t){ACTION_WAVE_LEFT_LEG, 0,    0,  0};
    s[i++] = (action_step_t){ACTION_TILT_RIGHT,    0,    0,  500};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 8,  0};
    s[i++] = (action_step_t){ACTION_WAVE_RIGHT_LEG,0,    0,  0};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 17, 0};
    s[i++] = (action_step_t){ACTION_SPIN_IN_PLACE, 1500, 29, 0};
    action_slots[0].count = (uint8_t)i;
    save_actions_to_nvs(0);
    ESP_LOGI(TAG, "💃 Default dance saved to NVS slot 1 (%d steps)", action_slots[0].count);
}

void save_actions_to_nvs(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_actions", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for actions");
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "action_%d", slot);
    
    err = nvs_set_blob(nvs_handle, key, &action_slots[slot], sizeof(action_slot_t));
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "💾 Saved %d actions to NVS slot %d (size=%d bytes)", 
                 action_slots[slot].count, slot + 1, sizeof(action_slot_t));
    } else {
        ESP_LOGE(TAG, "❌ Failed to save actions: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
}

void load_actions_from_nvs(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_actions", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "📂 NVS 'robot_actions' not found - slots empty");
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "action_%d", slot);
    
    size_t required_size = sizeof(action_slot_t);
    err = nvs_get_blob(nvs_handle, key, &action_slots[slot], &required_size);
    if (err == ESP_OK) {
        // Validate loaded count
        if (action_slots[slot].count > MAX_ACTIONS) {
            ESP_LOGW(TAG, "⚠️ Invalid count %d in slot %d, resetting", 
                     action_slots[slot].count, slot + 1);
            action_slots[slot].count = 0;
        }
        ESP_LOGI(TAG, "📂 Loaded slot %d: %d/%d actions", 
                 slot + 1, action_slots[slot].count, MAX_ACTIONS);
    } else {
        ESP_LOGW(TAG, "📂 Slot %d empty", slot + 1);
        memset(&action_slots[slot], 0, sizeof(action_slot_t));
    }
    
    nvs_close(nvs_handle);
}

// ========== LED STRIP CONTROL ==========

static led_strip_handle_t led_strip = NULL;
static led_state_t led_state = {
    .r = 255,
    .g = 255,
    .b = 255,
    .brightness = 128,
    .mode = LED_MODE_SOLID,
    .speed = 50
};

static bool led_initialized = false;
static uint32_t led_animation_step = 0;
static uint32_t led_last_update = 0;

// Initialize LED strip
void ninja_led_init(void) {
    if (led_initialized) return;
    
    ESP_LOGI(TAG, "🌈 Initializing LED strip on GPIO %d with %d LEDs", LED_STRIP_PIN, LED_STRIP_COUNT);
    
    // LED strip configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_PIN,
        .max_leds = LED_STRIP_COUNT,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    
    // RMT configuration
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to create LED strip: %s", esp_err_to_name(ret));
        return;
    }
    
    led_initialized = true;
    
    // Load saved state
    load_led_state_from_nvs();
    
    // Apply initial state
    ninja_led_update();
    
    ESP_LOGI(TAG, "✅ LED strip initialized! Mode=%d, R=%d G=%d B=%d, Brightness=%d", 
             led_state.mode, led_state.r, led_state.g, led_state.b, led_state.brightness);
}

// Apply brightness to a color component
static uint8_t apply_brightness(uint8_t color, uint8_t brightness) {
    return (uint8_t)((color * brightness) / 255);
}

// Set all LEDs to a single color
void ninja_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    led_state.r = r;
    led_state.g = g;
    led_state.b = b;
    led_state.mode = LED_MODE_SOLID;
    
    ESP_LOGI(TAG, "🎨 LED color set: R=%d G=%d B=%d", r, g, b);
}

// Set individual LED pixel
void ninja_led_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!led_initialized || !led_strip || index >= LED_STRIP_COUNT) return;
    
    uint8_t br = apply_brightness(r, led_state.brightness);
    uint8_t bg = apply_brightness(g, led_state.brightness);
    uint8_t bb = apply_brightness(b, led_state.brightness);
    
    led_strip_set_pixel(led_strip, index, br, bg, bb);
}

// Set brightness
void ninja_led_set_brightness(uint8_t brightness) {
    led_state.brightness = brightness;
    ESP_LOGI(TAG, "💡 LED brightness: %d", brightness);
}

// Set LED mode
void ninja_led_set_mode(led_mode_t mode) {
    led_state.mode = mode;
    led_animation_step = 0;
    ESP_LOGI(TAG, "🎯 LED mode: %d", mode);
}

// Set animation speed
void ninja_led_set_speed(uint16_t speed_ms) {
    led_state.speed = speed_ms;
    ESP_LOGI(TAG, "⚡ LED speed: %d ms", speed_ms);
}

// Turn off all LEDs
void ninja_led_off(void) {
    if (!led_initialized || !led_strip) return;
    
    led_state.mode = LED_MODE_OFF;
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "💤 LED strip off");
}

// HSV to RGB conversion for rainbow effect
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t region = h / 60;
    uint8_t remainder = (h - (region * 60)) * 255 / 60;
    
    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - ((s * remainder) / 255))) / 255;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;
    
    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

// Update LED strip based on current mode
void ninja_led_update(void) {
    if (!led_initialized || !led_strip) return;
    
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    
    // Rate-limit ALL modes to avoid calling led_strip_refresh too frequently
    if (now - led_last_update < led_state.speed) {
        return;
    }
    led_last_update = now;
    
    // For non-animated modes, cap refresh at 50ms (20Hz) to reduce CPU load
    if (led_state.mode == LED_MODE_SOLID || led_state.mode == LED_MODE_OFF) {
        static uint32_t last_solid_refresh = 0;
        if (now - last_solid_refresh < 50) {
            return;
        }
        last_solid_refresh = now;
    }
    
    uint8_t br, bg, bb;
    
    switch (led_state.mode) {
        case LED_MODE_OFF:
            led_strip_clear(led_strip);
            break;
            
        case LED_MODE_SOLID:
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            break;
            
        case LED_MODE_RAINBOW: {
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                uint16_t hue = (led_animation_step + i * 360 / LED_STRIP_COUNT) % 360;
                uint8_t r, g, b;
                hsv_to_rgb(hue, 255, 255, &r, &g, &b);
                br = apply_brightness(r, led_state.brightness);
                bg = apply_brightness(g, led_state.brightness);
                bb = apply_brightness(b, led_state.brightness);
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step = (led_animation_step + 5) % 360;
            break;
        }
            
        case LED_MODE_BREATHING: {
            // Sine wave breathing effect
            float phase = (led_animation_step % 100) / 100.0f * 3.14159f * 2;
            float breath = (sinf(phase) + 1.0f) / 2.0f;
            uint8_t breath_brightness = (uint8_t)(breath * led_state.brightness);
            
            br = apply_brightness(led_state.r, breath_brightness);
            bg = apply_brightness(led_state.g, breath_brightness);
            bb = apply_brightness(led_state.b, breath_brightness);
            
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step++;
            break;
        }
            
        case LED_MODE_CHASE: {
            // Running light effect
            led_strip_clear(led_strip);
            int lit_led = led_animation_step % LED_STRIP_COUNT;
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            led_strip_set_pixel(led_strip, lit_led, br, bg, bb);
            // Trail effect
            int trail1 = (lit_led - 1 + LED_STRIP_COUNT) % LED_STRIP_COUNT;
            int trail2 = (lit_led - 2 + LED_STRIP_COUNT) % LED_STRIP_COUNT;
            led_strip_set_pixel(led_strip, trail1, br/2, bg/2, bb/2);
            led_strip_set_pixel(led_strip, trail2, br/4, bg/4, bb/4);
            led_animation_step++;
            break;
        }
            
        case LED_MODE_BLINK: {
            // Simple blinking
            bool on = (led_animation_step % 2) == 0;
            if (on) {
                br = apply_brightness(led_state.r, led_state.brightness);
                bg = apply_brightness(led_state.g, led_state.brightness);
                bb = apply_brightness(led_state.b, led_state.brightness);
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    led_strip_set_pixel(led_strip, i, br, bg, bb);
                }
            } else {
                led_strip_clear(led_strip);
            }
            led_animation_step++;
            break;
        }
    }
    
    led_strip_refresh(led_strip);
}

// Get LED state
led_state_t* get_led_state(void) {
    return &led_state;
}

// Save LED state to NVS
void save_led_state_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_led", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for LED");
        return;
    }
    
    nvs_set_u8(nvs_handle, "led_r", led_state.r);
    nvs_set_u8(nvs_handle, "led_g", led_state.g);
    nvs_set_u8(nvs_handle, "led_b", led_state.b);
    nvs_set_u8(nvs_handle, "led_br", led_state.brightness);
    nvs_set_u8(nvs_handle, "led_mode", (uint8_t)led_state.mode);
    nvs_set_u16(nvs_handle, "led_speed", led_state.speed);
    
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "💾 LED state saved to NVS");
}

// Load LED state from NVS
void load_led_state_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_led", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No saved LED state in NVS, using defaults");
        return;
    }
    
    uint8_t val8;
    uint16_t val16;
    
    if (nvs_get_u8(nvs_handle, "led_r", &val8) == ESP_OK) led_state.r = val8;
    if (nvs_get_u8(nvs_handle, "led_g", &val8) == ESP_OK) led_state.g = val8;
    if (nvs_get_u8(nvs_handle, "led_b", &val8) == ESP_OK) led_state.b = val8;
    if (nvs_get_u8(nvs_handle, "led_br", &val8) == ESP_OK) led_state.brightness = val8;
    if (nvs_get_u8(nvs_handle, "led_mode", &val8) == ESP_OK) led_state.mode = (led_mode_t)val8;
    if (nvs_get_u16(nvs_handle, "led_speed", &val16) == ESP_OK) led_state.speed = val16;
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "📂 LED state loaded from NVS: Mode=%d R=%d G=%d B=%d Br=%d Speed=%d",
             led_state.mode, led_state.r, led_state.g, led_state.b, led_state.brightness, led_state.speed);
}
