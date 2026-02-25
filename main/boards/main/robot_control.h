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

// GPIO Pins for Servos
#define SERVO_LEFT_FOOT_PIN     38
#define SERVO_LEFT_LEG_PIN      17
#define SERVO_RIGHT_FOOT_PIN    18
#define SERVO_RIGHT_LEG_PIN     39
#define SERVO_LEFT_ARM_PIN      8
#define SERVO_RIGHT_ARM_PIN     12
#define SERVO_HEAD_PIN          13

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
void save_calibration_to_nvs(void);
void load_calibration_from_nvs(void);

// Control state
control_state_t* get_control_state(void);
robot_mode_t get_robot_mode(void);

// Main control loop task
void robot_control_task(void *pvParameters);

#endif // ROBOT_CONTROL_H
