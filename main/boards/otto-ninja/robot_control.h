/*
 * Otto Ninja Robot - ESP-IDF Version
 * Robot Control Header
 * 
 * Servo control and walking logic
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO Pins for Servos
#define SERVO_LEFT_FOOT_PIN     17
#define SERVO_LEFT_LEG_PIN      18
#define SERVO_RIGHT_FOOT_PIN    38
#define SERVO_RIGHT_LEG_PIN     39
#define SERVO_LEFT_ARM_PIN      -1  // Not connected (was 18 - conflict with LEFT_LEG)
#define SERVO_RIGHT_ARM_PIN     -1  // Not connected (was 17 - conflict with LEFT_FOOT)
#define SERVO_HEAD_PIN          13

// GPIO Pin for LED strip (WS2812)
#define LED_STRIP_PIN           8
#define LED_STRIP_COUNT         8

// Servo channel assignments (LEDC)
typedef enum {
    SERVO_CH_LEFT_FOOT = 0,
    SERVO_CH_LEFT_LEG,
    SERVO_CH_RIGHT_FOOT,
    SERVO_CH_RIGHT_LEG,
    SERVO_CH_LEFT_ARM,
    SERVO_CH_RIGHT_ARM,
    SERVO_CH_HEAD,
    SERVO_CH_MAX
} servo_channel_t;

// Robot mode
typedef enum {
    MODE_WALK = 0,
    MODE_ROLL = 1
} robot_mode_t;

// Calibration settings structure
typedef struct {
    // Foot neutral points
    int lf_neutral;     // Left Foot Neutral (default 90)
    int rf_neutral;     // Right Foot Neutral (default 90)
    
    // Walking speeds
    int lffwrs;         // Left Foot Forward Walking Rotation Speed
    int rffwrs;         // Right Foot Forward Walking Rotation Speed
    int lfbwrs;         // Left Foot Backward Walking Rotation Speed
    int rfbwrs;         // Right Foot Backward Walking Rotation Speed
    
    // Standing positions
    int la0;            // Left Leg standing Position
    int ra0;            // Right Leg standing position
    
    // Walking tilt positions
    int latl;           // Left Leg tilt left walking position
    int ratl;           // Right Leg tilt left walking position
    int latr;           // Left Leg tilt right walking position
    int ratr;           // Right Leg tilt right walking position
    
    // Roll positions
    int la1;            // Left Leg roll Position
    int ra1;            // Right Leg roll position
    
    // Roll speed settings (servo angle offset from neutral)
    int roll_lf_fwd_speed; // Left Foot roll forward speed (default 45)
    int roll_lf_bwd_speed; // Left Foot roll backward speed (default 45)
    int roll_rf_fwd_speed; // Right Foot roll forward speed (default 45)
    int roll_rf_bwd_speed; // Right Foot roll backward speed (default 45)
    
    // Transform leg speeds (delay in ms, lower = faster)
    int transform_ll_speed; // Left Leg transform speed (default 5ms per step)
    int transform_rl_speed; // Right Leg transform speed (default 5ms per step)
    
    // Turn and Combo speeds (in ms)
    int turn_left_speed;  // Turn left speed (default 500ms)
    int turn_right_speed; // Turn right speed (default 500ms)
    int combo_lf_speed;   // Combo LF rotation speed (default 1000ms)
    int combo_rf_speed;   // Combo RF rotation speed (default 1000ms)
    
    // Battery alert settings
    bool battery_alert_enabled; // Enable/disable low battery popup and sound (default true)
} calibration_t;

// Control state structure
typedef struct {
    int8_t j_x;         // Joystick X (-100 to 100)
    int8_t j_y;         // Joystick Y (-100 to 100)
    uint8_t button_a;   // Left Arm button
    uint8_t button_b;   // Right Arm button
    uint8_t button_x;   // Roll mode
    uint8_t button_y;   // Walk mode
    bool test_mode_active;
    int test_cycles_remaining;
    bool manual_mode;   // Manual/direct servo control mode
    int move_duration_ms; // Custom movement duration (0 = unlimited)
} control_state_t;

// Default calibration values
#define CAL_DEFAULT_LF_NEUTRAL  90
#define CAL_DEFAULT_RF_NEUTRAL  90
#define CAL_DEFAULT_LFFWRS      18
#define CAL_DEFAULT_RFFWRS      18
#define CAL_DEFAULT_LFBWRS      18
#define CAL_DEFAULT_RFBWRS      18
#define CAL_DEFAULT_LA0         60
#define CAL_DEFAULT_RA0         135
#define CAL_DEFAULT_LATL        100
#define CAL_DEFAULT_RATL        175
#define CAL_DEFAULT_LATR        5
#define CAL_DEFAULT_RATR        80
#define CAL_DEFAULT_LA1         160
#define CAL_DEFAULT_RA1         25
#define CAL_DEFAULT_ROLL_LF_FWD 45
#define CAL_DEFAULT_ROLL_LF_BWD 45
#define CAL_DEFAULT_ROLL_RF_FWD 45
#define CAL_DEFAULT_ROLL_RF_BWD 45
#define CAL_DEFAULT_TRANSFORM_LL 5   // ms per step (lower = faster)
#define CAL_DEFAULT_TRANSFORM_RL 5   // ms per step (lower = faster)
#define CAL_DEFAULT_TURN_L      500
#define CAL_DEFAULT_TURN_R      500
#define CAL_DEFAULT_COMBO_LF    1000
#define CAL_DEFAULT_COMBO_RF    1000
#define CAL_DEFAULT_BATTERY_ALERT true

// Initialize robot control
void robot_control_init(void);

// Servo control functions
void servo_write(servo_channel_t channel, int angle);
void servo_attach(servo_channel_t channel);
void servo_detach(servo_channel_t channel);
bool servo_attached(servo_channel_t channel);

// Movement functions
void go_home(void);
void ninja_set_walk(void);
void ninja_set_roll(void);
void ninja_walk(void);
void ninja_roll(void);
void ninja_walk_stop(void);
void ninja_roll_stop(void);

// Arm functions
void ninja_left_arm_up(void);
void ninja_left_arm_down(void);
void ninja_right_arm_up(void);
void ninja_right_arm_down(void);

// Test functions
void test_left_foot(void);
void test_right_foot(void);
void test_both_feet(void);
void stop_feet_test(void);

// Calibration functions
calibration_t* get_calibration(void);
void set_calibration(const calibration_t* cal);
void reset_calibration_to_defaults(void);
void save_calibration_to_nvs(void);
void load_calibration_from_nvs(void);

// Battery alert setting
bool is_battery_alert_enabled(void);

// Control state
control_state_t* get_control_state(void);
robot_mode_t get_robot_mode(void);

// Tilt functions (hold position)
void ninja_tilt_left(void);   // Tilt left and hold
void ninja_tilt_right(void);  // Tilt right and hold

// Direct servo control for real-time adjustment
void servo_direct_write(int channel, int angle);

// Manual mode control
void set_manual_mode(bool enable);

// Combo functions
void ninja_combo1(void);  // Tilt left, wave RL, rotate LF, go home
void ninja_combo2(void);  // Tilt right, wave LL, rotate RF, go home

// ========== ACTION RECORDING ==========
#define MAX_ACTIONS 100
#define MAX_ACTION_SLOTS 3

// Action types
typedef enum {
    ACTION_NONE = 0,
    ACTION_JOYSTICK,      // j_x, j_y values
    ACTION_BUTTON_A,      // Left arm
    ACTION_BUTTON_B,      // Right arm
    ACTION_WALK_MODE,     // Switch to walk
    ACTION_ROLL_MODE,     // Switch to roll
    ACTION_HOME,          // Go home
    ACTION_TILT_LEFT,     // Tilt left
    ACTION_TILT_RIGHT,    // Tilt right
    ACTION_SERVO,         // Direct servo control
    ACTION_DELAY,         // Wait time
    ACTION_COMBO1,        // Combo 1 sequence
    ACTION_COMBO2,        // Combo 2 sequence
    ACTION_RHYTHM_LEFT,   // Left leg rhythm dance
    ACTION_RHYTHM_RIGHT,  // Right leg rhythm dance
    ACTION_WALK_COMBO_123,// Walk combo phases 1+2+3
    ACTION_WALK_COMBO_345,// Walk combo phases 3+4+5
    ACTION_TEST_LF,       // Test left foot
    ACTION_TEST_RF,       // Test right foot
    ACTION_TEST_BOTH,     // Test both feet
    ACTION_TEST_STOP,     // Stop feet test
    ACTION_MOVE_FWD,      // Move forward (param1=duration_ms)
    ACTION_MOVE_BWD,      // Move backward (param1=duration_ms)
    ACTION_MOVE_STOP,     // Stop movement
    ACTION_TURN_LEFT,     // Turn left (param1=speed_ms)
    ACTION_TURN_RIGHT,    // Turn right (param1=speed_ms)
    ACTION_SPIN_IN_PLACE, // Spin in place (param1=duration_ms, param2=speed_offset)
    ACTION_SPIN_RF,       // Spin Right Foot  (param1=duration_ms, param2=speed_offset)
    ACTION_SPIN_LF,       // Spin Left Foot   (param1=duration_ms, param2=speed_offset)
    ACTION_WAVE_RIGHT_LEG, // Wave right leg x3 (no params)
    ACTION_WAVE_LEFT_LEG   // Wave left leg x3  (no params)
} action_type_t;

// Single action step
typedef struct {
    action_type_t type;
    int16_t param1;       // j_x or servo channel or delay_ms
    int16_t param2;       // j_y or servo angle
    uint16_t duration_ms; // Duration for joystick actions
} action_step_t;

// Action slot (sequence of actions)
typedef struct {
    uint8_t count;        // Number of actions recorded
    action_step_t steps[MAX_ACTIONS];
} action_slot_t;

// Recording state
typedef struct {
    bool is_recording;
    uint8_t current_slot;  // 0, 1, or 2
    uint8_t step_count;
} recording_state_t;

// Action recording functions
void start_recording(uint8_t slot);
void stop_recording(void);
void record_action(action_type_t type, int16_t param1, int16_t param2, uint16_t duration);
void play_action(uint8_t slot);
void load_default_dance(void);  // Load built-in dance into slot 1 if empty
void save_actions_to_nvs(uint8_t slot);
void load_actions_from_nvs(uint8_t slot);
recording_state_t* get_recording_state(void);
action_slot_t* get_action_slot(uint8_t slot);

// Rhythm/Dance functions
void left_leg_rhythm(void);   // Left leg oscillate: 34->45->65 degrees, 3 times each
void right_leg_rhythm(void);  // Right leg oscillate: 140->150->170 degrees, 3 times each

// Walk phase control functions (for testing individual phases)
void ninja_walk_phase1(void);  // Phase 1: Tilt to right
void ninja_walk_phase2(void);  // Phase 2: Right foot rotation forward
void ninja_walk_phase3(void);  // Phase 3: Tilt to left
void ninja_walk_phase4(void);  // Phase 4: Left foot rotation forward
void ninja_walk_phase5(void);  // Phase 5: Stop and return neutral

// Walk phase combos
void ninja_walk_combo_123(void);  // Combo: Phase 1+2+3 (Tilt R -> RF Fwd -> Tilt L)
void ninja_walk_combo_345(void);  // Combo: Phase 3+4+5 (Tilt L -> LF Fwd -> Neutral)

// ========== LED STRIP CONTROL ==========
// LED modes
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID,       // Single solid color
    LED_MODE_RAINBOW,     // Rainbow cycling
    LED_MODE_BREATHING,   // Breathing effect
    LED_MODE_CHASE,       // Chase/running effect
    LED_MODE_BLINK        // Blinking effect
} led_mode_t;

// LED state structure
typedef struct {
    uint8_t r;            // Red (0-255)
    uint8_t g;            // Green (0-255)
    uint8_t b;            // Blue (0-255)
    uint8_t brightness;   // Brightness (0-255)
    led_mode_t mode;      // Current LED mode
    uint16_t speed;       // Animation speed (ms per step)
} led_state_t;

// LED control functions
void ninja_led_init(void);
void ninja_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void ninja_led_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void ninja_led_set_brightness(uint8_t brightness);
void ninja_led_set_mode(led_mode_t mode);
void ninja_led_set_speed(uint16_t speed_ms);
void ninja_led_off(void);
void ninja_led_update(void);
led_state_t* get_led_state(void);
void save_led_state_to_nvs(void);
void load_led_state_from_nvs(void);

// Main control loop task
void robot_control_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONTROL_H
