#ifndef __ROBOT_MCP_CONTROLLER_H__
#define __ROBOT_MCP_CONTROLLER_H__

#include "mcp_server.h"
#include "robot_control.h"
#include <esp_log.h>
#include <cJSON.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG_MCP "RobotMCP"

// Helper struct to pass parameters to task
typedef struct {
    int action_type;  // 0=forward, 1=backward, 2=turn_left, 3=turn_right, 4=roll_go
    int duration_ms;
    int speed_ms;
    int speed_level;  // Roll speed: 1=slow(30), 2=medium(65), 3=fast(100)
    int walk_steps;   // Walk mode: number of complete walk cycles (0 = use duration)
} robot_action_params_t;

// Background task for play_action to avoid blocking MCP callback
typedef struct {
    uint8_t slot;
} play_action_params_t;

static void play_action_bg_task(void* pvParameters) {
    play_action_params_t* params = (play_action_params_t*)pvParameters;
    ESP_LOGI(TAG_MCP, "🎭 play_action_bg_task: starting slot %d", params->slot + 1);
    play_action(params->slot);
    ESP_LOGI(TAG_MCP, "🎭 play_action_bg_task: slot %d completed", params->slot + 1);
    free(params);
    vTaskDelete(NULL);
}

// ========== ACTION TYPE <-> STRING CONVERSION ==========
static const char* action_type_to_string(action_type_t type) {
    switch (type) {
        case ACTION_NONE: return "none";
        case ACTION_JOYSTICK: return "joystick";
        case ACTION_BUTTON_A: return "button_a";
        case ACTION_BUTTON_B: return "button_b";
        case ACTION_WALK_MODE: return "walk_mode";
        case ACTION_ROLL_MODE: return "roll_mode";
        case ACTION_HOME: return "home";
        case ACTION_TILT_LEFT: return "tilt_left";
        case ACTION_TILT_RIGHT: return "tilt_right";
        case ACTION_SERVO: return "servo";
        case ACTION_DELAY: return "delay";
        case ACTION_COMBO1: return "combo1";
        case ACTION_COMBO2: return "combo2";
        case ACTION_RHYTHM_LEFT: return "rhythm_left";
        case ACTION_RHYTHM_RIGHT: return "rhythm_right";
        case ACTION_WALK_COMBO_123: return "walk_combo_123";
        case ACTION_WALK_COMBO_345: return "walk_combo_345";
        case ACTION_TEST_LF: return "test_lf";
        case ACTION_TEST_RF: return "test_rf";
        case ACTION_TEST_BOTH: return "test_both";
        case ACTION_TEST_STOP: return "test_stop";
        case ACTION_MOVE_FWD: return "move_fwd";
        case ACTION_MOVE_BWD: return "move_bwd";
        case ACTION_MOVE_STOP: return "move_stop";
        case ACTION_TURN_LEFT: return "turn_left";
        case ACTION_TURN_RIGHT: return "turn_right";
        case ACTION_SPIN_IN_PLACE: return "spin_in_place";
        case ACTION_SPIN_RF: return "spin_rf";
        case ACTION_SPIN_LF: return "spin_lf";
        case ACTION_WAVE_RIGHT_LEG: return "wave_right_leg";
        case ACTION_WAVE_LEFT_LEG: return "wave_left_leg";
        default: return "unknown";
    }
}

static action_type_t string_to_action_type(const char* str) {
    if (!str) return ACTION_NONE;
    if (strcmp(str, "joystick") == 0) return ACTION_JOYSTICK;
    if (strcmp(str, "button_a") == 0) return ACTION_BUTTON_A;
    if (strcmp(str, "button_b") == 0) return ACTION_BUTTON_B;
    if (strcmp(str, "walk_mode") == 0) return ACTION_WALK_MODE;
    if (strcmp(str, "roll_mode") == 0) return ACTION_ROLL_MODE;
    if (strcmp(str, "home") == 0) return ACTION_HOME;
    if (strcmp(str, "tilt_left") == 0) return ACTION_TILT_LEFT;
    if (strcmp(str, "tilt_right") == 0) return ACTION_TILT_RIGHT;
    if (strcmp(str, "servo") == 0) return ACTION_SERVO;
    if (strcmp(str, "delay") == 0) return ACTION_DELAY;
    if (strcmp(str, "combo1") == 0) return ACTION_COMBO1;
    if (strcmp(str, "combo2") == 0) return ACTION_COMBO2;
    if (strcmp(str, "rhythm_left") == 0) return ACTION_RHYTHM_LEFT;
    if (strcmp(str, "rhythm_right") == 0) return ACTION_RHYTHM_RIGHT;
    if (strcmp(str, "walk_combo_123") == 0) return ACTION_WALK_COMBO_123;
    if (strcmp(str, "walk_combo_345") == 0) return ACTION_WALK_COMBO_345;
    if (strcmp(str, "test_lf") == 0) return ACTION_TEST_LF;
    if (strcmp(str, "test_rf") == 0) return ACTION_TEST_RF;
    if (strcmp(str, "test_both") == 0) return ACTION_TEST_BOTH;
    if (strcmp(str, "test_stop") == 0) return ACTION_TEST_STOP;
    if (strcmp(str, "move_fwd") == 0) return ACTION_MOVE_FWD;
    if (strcmp(str, "move_bwd") == 0) return ACTION_MOVE_BWD;
    if (strcmp(str, "move_stop") == 0) return ACTION_MOVE_STOP;
    if (strcmp(str, "turn_left") == 0) return ACTION_TURN_LEFT;
    if (strcmp(str, "turn_right") == 0) return ACTION_TURN_RIGHT;
    if (strcmp(str, "spin_in_place") == 0) return ACTION_SPIN_IN_PLACE;
    if (strcmp(str, "spin_rf") == 0) return ACTION_SPIN_RF;
    if (strcmp(str, "spin_lf") == 0) return ACTION_SPIN_LF;
    if (strcmp(str, "wave_right_leg") == 0) return ACTION_WAVE_RIGHT_LEG;
    if (strcmp(str, "wave_left_leg") == 0) return ACTION_WAVE_LEFT_LEG;
    return ACTION_NONE;
}

// Background task for robot actions
static void robot_action_task(void* pvParameters) {
    robot_action_params_t* params = (robot_action_params_t*)pvParameters;
    control_state_t* state = get_control_state();
    
    ESP_LOGI(TAG_MCP, "🤖 Action task started: type=%d, duration=%d", params->action_type, params->duration_ms);
    
    // Clear manual mode and joystick before any action
    state->manual_mode = false;
    state->j_x = 0;
    state->j_y = 0;
    
    robot_mode_t mode = get_robot_mode();
    ESP_LOGI(TAG_MCP, "📊 Current robot mode: %d (%s)", mode, mode == MODE_WALK ? "WALK" : mode == MODE_ROLL ? "ROLL" : "UNKNOWN");
    
    // Map speed_level to joystick intensity for roll mode
    int roll_joy_value = 100; // default full speed
    if (params->speed_level == 1) roll_joy_value = 30;       // slow
    else if (params->speed_level == 2) roll_joy_value = 65;  // medium
    else roll_joy_value = 100;                                // fast
    
    switch (params->action_type) {
        case 0: // Forward
            if (mode == MODE_WALK && params->walk_steps > 0) {
                // WALK mode with step count: use test_mode for exact phase cycles
                ESP_LOGI(TAG_MCP, "🚶 [MCP-FORWARD-WALK] Walking %d steps (phases)", params->walk_steps);
                state->test_mode_active = true;
                state->test_cycles_remaining = params->walk_steps;
                state->j_x = 0;
                state->j_y = 100;
                // Wait for all cycles to complete (each cycle ~1250ms max)
                int max_wait_ms = params->walk_steps * 2000 + 1000;
                int waited = 0;
                while (state->test_mode_active && waited < max_wait_ms) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    waited += 50;
                }
                state->j_x = 0;
                state->j_y = 0;
                state->test_mode_active = false;
                ESP_LOGI(TAG_MCP, "🚶 [MCP-FORWARD-WALK] Completed %d steps", params->walk_steps);
            } else {
                // ROLL mode or duration-based: use joystick with speed level
                int joy_y = (mode == MODE_ROLL) ? roll_joy_value : 100;
                ESP_LOGI(TAG_MCP, "⬆️ [MCP-FORWARD] Setting joystick: X=0, Y=%d for %dms (speed_level=%d)", joy_y, params->duration_ms, params->speed_level);
                state->j_x = 0;
                state->j_y = joy_y;
                vTaskDelay(pdMS_TO_TICKS(params->duration_ms));
                state->j_x = 0;
                state->j_y = 0;
                ESP_LOGI(TAG_MCP, "⬆️ [MCP-FORWARD] Stopped: j_x=0, j_y=0");
            }
            break;
        case 1: // Backward
            if (mode == MODE_WALK && params->walk_steps > 0) {
                // WALK mode with step count: use test_mode for exact phase cycles
                ESP_LOGI(TAG_MCP, "🚶 [MCP-BACKWARD-WALK] Walking backward %d steps (phases)", params->walk_steps);
                state->test_mode_active = true;
                state->test_cycles_remaining = params->walk_steps;
                state->j_x = 0;
                state->j_y = -100;
                // Wait for all cycles to complete
                int max_wait_ms = params->walk_steps * 2000 + 1000;
                int waited = 0;
                while (state->test_mode_active && waited < max_wait_ms) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    waited += 50;
                }
                state->j_x = 0;
                state->j_y = 0;
                state->test_mode_active = false;
                ESP_LOGI(TAG_MCP, "🚶 [MCP-BACKWARD-WALK] Completed %d steps", params->walk_steps);
            } else {
                // ROLL mode or duration-based: use joystick with speed level
                int joy_y = (mode == MODE_ROLL) ? -roll_joy_value : -100;
                ESP_LOGI(TAG_MCP, "⬇️ [MCP-BACKWARD] Setting joystick: X=0, Y=%d for %dms (speed_level=%d)", joy_y, params->duration_ms, params->speed_level);
                state->j_x = 0;
                state->j_y = joy_y;
                vTaskDelay(pdMS_TO_TICKS(params->duration_ms));
                state->j_x = 0;
                state->j_y = 0;
                ESP_LOGI(TAG_MCP, "⬇️ [MCP-BACKWARD] Stopped: j_x=0, j_y=0");
            }
            break;
        case 4: // Roll and go
            ninja_set_roll();
            vTaskDelay(pdMS_TO_TICKS(800));
            state->j_x = 0;
            state->j_y = roll_joy_value;
            vTaskDelay(pdMS_TO_TICKS(params->duration_ms));
            state->j_x = 0;
            state->j_y = 0;
            break;
        case 5: // Set roll mode only (biến hình thành xe)
            ESP_LOGI(TAG_MCP, "🚗 Executing ninja_set_roll()...");
            ninja_set_roll();
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for servo to complete
            ESP_LOGI(TAG_MCP, "🚗 Roll mode transformation completed! Mode=%d", get_robot_mode());
            break;
        case 6: // Set walk mode only (đứng dậy)
            ESP_LOGI(TAG_MCP, "🚶 Executing ninja_set_walk()...");
            ninja_set_walk();
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for servo to complete
            ESP_LOGI(TAG_MCP, "🚶 Walk mode transformation completed! Mode=%d", get_robot_mode());
            break;
        case 7: // Tilt left (nghiêng trái)
            ESP_LOGI(TAG_MCP, "↙️ Executing ninja_tilt_left()...");
            ninja_tilt_left();
            ESP_LOGI(TAG_MCP, "↙️ Tilt left completed!");
            break;
        case 8: // Tilt right (nghiêng phải)
            ESP_LOGI(TAG_MCP, "↘️ Executing ninja_tilt_right()...");
            ninja_tilt_right();
            ESP_LOGI(TAG_MCP, "↘️ Tilt right completed!");
            break;
    }
    
    ESP_LOGI(TAG_MCP, "🤖 Action task completed");
    free(params);
    vTaskDelete(NULL);
}

class RobotMcpController {
public:
    RobotMcpController() {
        auto& mcp_server = McpServer::GetInstance();
        
        // Tool 1: Move forward (tiến lên phía trước)
        // Supports: duration (seconds), speed (1-3), steps (walk mode)
        mcp_server.AddTool("robot.move_forward", 
            "[Otto Ninja Robot] 🚶 DI CHUYỂN TIẾN về phía trước. "
            "KHI NÀO GỌI: người dùng nói 'tiến', 'forward', 'đi tới', 'đi lên', 'tiến lên', 'đi thẳng'. "
            "THAM SỐ BẮT BUỘC PHẢI TRÍCH XUẤT TỪ LỜI NÓI: "
            "- 'duration': SỐ GIÂY di chuyển. Ví dụ: '1 giây'→duration=1, '2 giây'→duration=2, '5s'→duration=5, '1s'→duration=1. LUÔN trích xuất số giây nếu người dùng nói! "
            "- 'speed': Tốc độ: 'chậm'→speed=1, 'vừa/bình thường'→speed=2, 'nhanh'→speed=3. "
            "- 'steps': Chế độ WALK (đi bộ): số bước đi (1 bước = 1 chu kỳ). 'tiến 2 bước'→steps=2. "
            "VÍ DỤ: 'tiến 1 giây'→duration=1; 'chạy nhanh 5 giây'→speed=3,duration=5; 'tiến chậm 2 giây'→speed=1,duration=2.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 1, 1, 20),
                Property("duration", kPropertyTypeInteger, 3, 1, 30),
                Property("speed", kPropertyTypeInteger, 2, 1, 3)
            }), 
            [](const PropertyList& properties) -> ReturnValue {
                robot_mode_t mode = get_robot_mode();
                
                int steps = 1;
                int duration = 3;
                int speed = 2;
                try { steps = properties["steps"].value<int>(); } catch (...) {}
                try { duration = properties["duration"].value<int>(); } catch (...) {}
                try { speed = properties["speed"].value<int>(); } catch (...) {}
                
                ESP_LOGI(TAG_MCP, "🏃 >>> TOOL: move_forward - mode=%s, steps=%d, duration=%d, speed=%d",
                         mode == MODE_WALK ? "WALK" : "ROLL", steps, duration, speed);
                
                // Create background task to avoid blocking main loop
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 0; // forward
                params->duration_ms = duration * 1000;
                params->speed_level = speed;
                params->walk_steps = steps;
                xTaskCreate(robot_action_task, "mcp_fwd", 4096, params, 5, NULL);
                
                if (mode == MODE_WALK) {
                    return std::string("✅ SUCCESS: Robot Ninja đang đi bộ tiến ") + 
                           std::to_string(steps) + " bước!";
                } else {
                    const char* speed_text = (speed == 1) ? "chậm" : (speed == 2) ? "vừa" : "nhanh";
                    return std::string("✅ SUCCESS: Robot Ninja đang lăn tiến ") + 
                           std::to_string(duration) + " giây với tốc độ " + speed_text + "!";
                }
            });
        
        // Tool 2: Move backward (lùi)
        // Supports: duration (seconds), speed (1-3), steps (walk mode)
        mcp_server.AddTool("robot.move_backward", 
            "[Otto Ninja Robot] 🔙 DI CHUYỂN LÙI về phía sau. "
            "KHI NÀO GỌI: người dùng nói 'lùi', 'back', 'đi lui', 'lùi lại', 'backward', 'đi xuống'. "
            "THAM SỐ BẮT BUỘC PHẢI TRÍCH XUẤT TỪ LỜI NÓI: "
            "- 'duration': SỐ GIÂY di chuyển. Ví dụ: '1 giây'→duration=1, '2 giây'→duration=2, '5s'→duration=5, '1s'→duration=1. LUÔN trích xuất số giây nếu người dùng nói! "
            "- 'speed': Tốc độ: 'chậm'→speed=1, 'vừa/bình thường'→speed=2, 'nhanh'→speed=3. "
            "- 'steps': Chế độ WALK (đi bộ): số bước lùi (1 bước = 1 chu kỳ). 'lùi 3 bước'→steps=3. "
            "VÍ DỤ: 'lùi 1 giây'→duration=1; 'lùi nhanh 5 giây'→speed=3,duration=5; 'lùi chậm 2 giây'→speed=1,duration=2.",
            PropertyList({
                Property("steps", kPropertyTypeInteger, 1, 1, 20),
                Property("duration", kPropertyTypeInteger, 3, 1, 30),
                Property("speed", kPropertyTypeInteger, 2, 1, 3)
            }), 
            [](const PropertyList& properties) -> ReturnValue {
                robot_mode_t mode = get_robot_mode();
                
                int steps = 1;
                int duration = 3;
                int speed = 2;
                try { steps = properties["steps"].value<int>(); } catch (...) {}
                try { duration = properties["duration"].value<int>(); } catch (...) {}
                try { speed = properties["speed"].value<int>(); } catch (...) {}
                
                ESP_LOGI(TAG_MCP, "🔙 >>> TOOL: move_backward - mode=%s, steps=%d, duration=%d, speed=%d",
                         mode == MODE_WALK ? "WALK" : "ROLL", steps, duration, speed);
                
                // Create background task to avoid blocking main loop
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 1; // backward
                params->duration_ms = duration * 1000;
                params->speed_level = speed;
                params->walk_steps = steps;
                xTaskCreate(robot_action_task, "mcp_back", 4096, params, 5, NULL);
                
                if (mode == MODE_WALK) {
                    return std::string("✅ SUCCESS: Robot Ninja đang đi lùi ") + 
                           std::to_string(steps) + " bước!";
                } else {
                    const char* speed_text = (speed == 1) ? "chậm" : (speed == 2) ? "vừa" : "nhanh";
                    return std::string("✅ SUCCESS: Robot Ninja đang lùi ") + 
                           std::to_string(duration) + " giây với tốc độ " + speed_text + "!";
                }
            });
        
        // Tool 3: Turn left (quay trái)
        // WALK mode: Nghiêng trái (giống bấm nút Web UI) + Đợi 1s + Rotate LF 0.5s
        // ROLL mode: X=-75, Y=-64
        mcp_server.AddTool("robot.turn_left", 
            "[Otto Ninja Robot] ⬅️ QUAY SANG TRÁI. "
            "KHI NÀO GỌI: người dùng nói 'trái', 'quay trái', 'left', 'rẽ trái', 'xoay trái'. "
            "Chế độ WALK: nghiêng trái, đợi, xoay chân trái. "
            "Chế độ ROLL: sử dụng joystick để quay trái.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                calibration_t* cal = get_calibration();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Turn left - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                // Use speed from calibration (saved to NVS)
                int speed_ms = cal->turn_left_speed;
                
                if (mode == MODE_WALK) {
                    // WALK mode: Call ninja_tilt_left() exactly like Web UI button
                    ninja_tilt_left();  // This sets manual_mode = true internally
                    
                    // Wait using configured speed
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    
                    // Rotate Left Foot using configured speed
                    int lf_angle = cal->lf_neutral + cal->lffwrs;
                    servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    servo_direct_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
                    
                    // Return to home
                    state->manual_mode = false;
                    go_home();
                    
                    return std::string("✅ SUCCESS: Robot Ninja quay trái xong (nghiêng + xoay LF). Hành động hoàn thành!");
                } else {
                    // ROLL mode: X=-75, Y=-64
                    state->j_x = -75;
                    state->j_y = -64;
                    
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    
                    state->j_x = 0;
                    state->j_y = 0;
                    
                    return std::string("✅ SUCCESS: Robot Ninja quay trái xong (chế độ lăn). Hành động hoàn thành!");
                }
            });
        
        // Tool 4: Turn right (quay phải)
        // WALK mode: Nghiêng phải (giống bấm nút Web UI) + Đợi 1s + Rotate RF 0.5s
        // ROLL mode: X=51, Y=-81
        mcp_server.AddTool("robot.turn_right", 
            "[Otto Ninja Robot] ➡️ QUAY SANG PHẢI. "
            "KHI NÀO GỌI: người dùng nói 'phải', 'quay phải', 'right', 'rẽ phải', 'xoay phải'. "
            "Chế độ WALK: nghiêng phải, đợi, xoay chân phải. "
            "Chế độ ROLL: sử dụng joystick để quay phải.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                control_state_t* state = get_control_state();
                calibration_t* cal = get_calibration();
                robot_mode_t mode = get_robot_mode();
                
                ESP_LOGI(TAG_MCP, "Turn right - Mode: %s", mode == MODE_WALK ? "WALK" : "ROLL");
                
                // Use speed from calibration (saved to NVS)
                int speed_ms = cal->turn_right_speed;
                
                if (mode == MODE_WALK) {
                    // WALK mode: Call ninja_tilt_right() exactly like Web UI button
                    ninja_tilt_right();  // This sets manual_mode = true internally
                    
                    // Wait using configured speed
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    
                    // Rotate Right Foot using configured speed
                    int rf_angle = cal->rf_neutral - cal->rffwrs;
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
                    
                    // Return to home
                    state->manual_mode = false;
                    go_home();
                    
                    return std::string("✅ SUCCESS: Robot Ninja quay phải xong (nghiêng + xoay RF). Hành động hoàn thành!");
                } else {
                    // ROLL mode: X=51, Y=-81
                    state->j_x = 51;
                    state->j_y = -81;
                    
                    vTaskDelay(pdMS_TO_TICKS(speed_ms));
                    
                    state->j_x = 0;
                    state->j_y = 0;
                    
                    return std::string("✅ SUCCESS: Robot Ninja quay phải xong (chế độ lăn). Hành động hoàn thành!");
                }
            });
        
        // Tool 5: Get robot mode (kiểm tra mode hiện tại)
        mcp_server.AddTool("robot.get_mode", 
            "[Otto Ninja Robot] ❓ KIỂM TRA CHẾ ĐỘ HIỆN TẠI. "
            "Trả về WALK (đi bộ bằng chân) hoặc ROLL (lăn bánh xe). "
            "KHI NÀO GỌI: cần biết robot đang ở chế độ nào trước khi thực hiện lệnh.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                robot_mode_t mode = get_robot_mode();
                if (mode == MODE_WALK) {
                    return std::string("{\"success\": true, \"mode\": \"walk\", \"description\": \"Robot đang ở chế độ ĐI BỘ - đi bằng 2 chân\"}");
                } else {
                    return std::string("{\"success\": true, \"mode\": \"roll\", \"description\": \"Robot đang ở chế độ LĂN - lăn bằng bánh xe\"}");
                }
            });
        
        // Tool 6: Set mode to walk
        mcp_server.AddTool("robot.set_walk_mode", 
            "[Otto Ninja Robot] 🚶 CHUYỂN SANG CHẾ ĐỘ ĐI BỘ (WALK) - ĐỨNG DẬY. "
            "Robot sẽ biến hình: đứng thẳng bằng 2 chân, giơ tay lên. "
            "KHI NÀO GỌI: người dùng nói 'đứng', 'đi bộ', 'walk', 'đứng dậy', 'chế độ đi', 'đứng lên'.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "🚶 >>> TOOL: set_walk_mode - Creating background task for 100%% execution");
                
                // Create background task to ensure 100% execution
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 6; // set_walk_mode
                params->duration_ms = 0;
                xTaskCreate(robot_action_task, "mcp_walk", 4096, params, 5, NULL);
                
                return std::string("✅ SUCCESS: Robot Ninja đang ĐỨNG DẬY và chuyển sang chế độ ĐI BỘ!");
            });
        
        // Tool 7: Set mode to roll
        mcp_server.AddTool("robot.set_roll_mode", 
            "[Otto Ninja Robot] 🚗 CHUYỂN SANG CHẾ ĐỘ LĂN XE / LĂNG XE / BIẾN HÌNH THÀNH XE. "
            "Robot sẽ biến hình: hạ thấp xuống thành xe có bánh, sẵn sàng lăn đi. "
            "LƯU Ý: 'lăn xe' và 'lăng xe' là CÙNG MỘT NGHĨA - đều là biến thành xe! "
            "KHI NÀO GỌI: người dùng nói 'lăn', 'lăng', 'roll', 'lăn xe', 'lăng xe', 'lên xe', 'chế độ xe', "
            "'bánh xe', 'biến hình', 'xe bánh', 'thành xe', 'hóa xe'.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "🚗 >>> TOOL: set_roll_mode - Creating background task for 100%% execution");
                
                // Create background task to ensure 100% execution
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 5; // set_roll_mode
                params->duration_ms = 0;
                xTaskCreate(robot_action_task, "mcp_roll_m", 4096, params, 5, NULL);
                
                return std::string("✅ SUCCESS: Robot Ninja đang BIẾN HÌNH thành XE! Chế độ lăn xe đã được kích hoạt!");
            });
        
        // Tool 8: Go home position
        mcp_server.AddTool("robot.go_home", 
            "[Otto Ninja Robot] 🏠 VỀ VỊ TRÍ HOME (đứng thẳng trung tính). "
            "Robot sẽ dừng mọi hoạt động và trở về tư thế đứng chuẩn. "
            "KHI NÀO GỌI: người dùng nói 'home', 'về', 'dừng', 'stop', 'đứng im', 'reset'.",
            PropertyList(), 
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "Going home");
                go_home();
                return std::string("✅ SUCCESS: Robot Ninja đã về vị trí HOME. Hành động hoàn thành!");
            });
        
        // Tool 9a: Tilt left (nghiêng trái)
        mcp_server.AddTool("robot.tilt_left",
            "[Otto Ninja Robot] ↙️ NGHIÊNG SANG TRÁI - giữ nguyên tư thế nghiêng trái. "
            "Robot sẽ nghiêng thân sang bên trái và GIỮ tư thế đó. "
            "KHI NÀO GỌI: người dùng nói 'nghiêng trái', 'ngả trái', 'tilt left', "
            "'nghiêng sang trái', 'ngã trái', 'đổ trái', 'lean left'.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "↙️ >>> TOOL: tilt_left");
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 7; // tilt_left
                params->duration_ms = 0;
                xTaskCreate(robot_action_task, "mcp_tilt_l", 4096, params, 5, NULL);
                return std::string("✅ SUCCESS: Robot Ninja đang NGHIÊNG SANG TRÁI và giữ tư thế!");
            });
        
        // Tool 9b: Tilt right (nghiêng phải)
        mcp_server.AddTool("robot.tilt_right",
            "[Otto Ninja Robot] ↘️ NGHIÊNG SANG PHẢI - giữ nguyên tư thế nghiêng phải. "
            "Robot sẽ nghiêng thân sang bên phải và GIỮ tư thế đó. "
            "KHI NÀO GỌI: người dùng nói 'nghiêng phải', 'ngả phải', 'tilt right', "
            "'nghiêng sang phải', 'ngã phải', 'đổ phải', 'lean right'.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "↘️ >>> TOOL: tilt_right");
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 8; // tilt_right
                params->duration_ms = 0;
                xTaskCreate(robot_action_task, "mcp_tilt_r", 4096, params, 5, NULL);
                return std::string("✅ SUCCESS: Robot Ninja đang NGHIÊNG SANG PHẢI và giữ tư thế!");
            });
        
        // ========== Tool 9: Play recorded action slot ==========
        mcp_server.AddTool(
            "robot.play_slot",
            "[Otto Ninja Robot] 🎭 PHÁT LẠI HÀNH ĐỘNG ĐÃ GHI từ slot (1-3). "
            "Slot 1: thường dùng cho múa/nhảy/dance/ballet. "
            "Slot 2-3: cho các bài khác. "
            "KHI NÀO GỌI: người dùng nói 'múa', 'nhảy', 'dance', 'ba lê', 'múa bài 2', 'nhảy slot 3'... "
            "QUAN TRỌNG: Nếu slot trống, hãy thông báo 'Slot chưa có hành động được lưu'.",
            PropertyList({Property("slot", kPropertyTypeInteger, 1, 1, 3)}), 
            [](const PropertyList& properties) -> ReturnValue {
                int slot_num = 1;
                try {
                    slot_num = properties["slot"].value<int>();
                } catch (...) {
                    slot_num = 1;
                }
                
                int slot_idx = slot_num - 1;  // Convert to 0-based index
                if (slot_idx < 0 || slot_idx >= 3) {
                    return std::string("❌ FAILED: Slot không hợp lệ! Chỉ có slot 1, 2, 3.");
                }
                
                action_slot_t* slot = get_action_slot(slot_idx);
                if (!slot || slot->count == 0) {
                    return std::string("❌ FAILED: Slot ") + std::to_string(slot_num) + " trống, chưa có hành động nào được lưu. Hãy ghi hành động vào slot trước!";
                }
                
                int action_count = slot->count;
                ESP_LOGI(TAG_MCP, "🎭 Playing slot %d (%d actions) - launching bg task", slot_num, action_count);
                
                // Run play_action in background task to avoid blocking MCP callback
                // (play_action uses vTaskDelay which would block the WebSocket task causing timeout)
                play_action_params_t* params = (play_action_params_t*)malloc(sizeof(play_action_params_t));
                if (!params) {
                    return std::string("❌ FAILED: Không đủ bộ nhớ để thực hiện hành động slot ") + std::to_string(slot_num) + ".";
                }
                params->slot = (uint8_t)slot_idx;
                BaseType_t task_ret = xTaskCreate(play_action_bg_task, "mcp_play_slot", 4096, params, 5, NULL);
                if (task_ret != pdPASS) {
                    free(params);
                    return std::string("❌ FAILED: Không thể tạo task thực hiện slot ") + std::to_string(slot_num) + ". Thử lại!";
                }
                
                return std::string("✅ SUCCESS: Robot đang thực hiện ") + std::to_string(action_count) + " hành động từ slot " + std::to_string(slot_num) + ". Điệu múa/nhảy đang diễn ra!";
            });
        
        // ========== Tool 10: Roll and Move (biến hình + lăn đi luôn) ==========
        mcp_server.AddTool(
            "robot.roll_and_go",
            "[Otto Ninja Robot] 🚗💨 BIẾN HÌNH THÀNH XE VÀ LĂN ĐI LUÔN. "
            "Kết hợp: chuyển sang chế độ LĂN + tự động di chuyển tiến về phía trước. "
            "KHI NÀO GỌI: người dùng nói 'lăn đi', 'biến hình rồi chạy', 'chạy xe đi', 'roll and go', 'lăn tiến'. "
            "THAM SỐ: 'duration'=số giây lăn (1-30), 'speed'=tốc độ (1=chậm,2=vừa,3=nhanh). "
            "VÍ DỤ: 'lăn đi 5 giây'→duration=5; 'biến hình chạy nhanh 2 giây'→speed=3,duration=2.",
            PropertyList({
                Property("duration", kPropertyTypeInteger, 3, 1, 30),
                Property("speed", kPropertyTypeInteger, 2, 1, 3)
            }), 
            [](const PropertyList& properties) -> ReturnValue {
                int duration = 3;
                int speed = 2;
                try { duration = properties["duration"].value<int>(); } catch (...) {}
                try { speed = properties["speed"].value<int>(); } catch (...) {}
                
                ESP_LOGI(TAG_MCP, "🚗💨 >>> TOOL: roll_and_go - duration=%d, speed=%d", duration, speed);
                
                // Create background task to avoid blocking main loop
                robot_action_params_t* params = (robot_action_params_t*)malloc(sizeof(robot_action_params_t));
                params->action_type = 4; // roll_and_go
                params->duration_ms = duration * 1000;
                params->speed_level = speed;
                xTaskCreate(robot_action_task, "mcp_roll", 4096, params, 5, NULL);
                
                const char* speed_text = (speed == 1) ? "chậm" : (speed == 2) ? "vừa" : "nhanh";
                return std::string("✅ SUCCESS: Robot Ninja đang BIẾN HÌNH thành xe và LĂN TIẾN ") + 
                       std::to_string(duration) + " giây với tốc độ " + speed_text + "!";
            });
        
        // ========== LED CONTROL TOOLS ==========
        
        // Tool 11: Set LED color
        mcp_server.AddTool(
            "robot.led.set_color",
            "[Otto Ninja Robot] 🌈 ĐẶT MÀU LED cho 8 đèn WS2812. "
            "Robot có 8 LED RGB có thể đổi màu. "
            "KHI NÀO GỌI: người dùng nói 'đổi màu led', 'bật đèn đỏ', 'led xanh', 'đèn màu vàng', 'sáng màu tím'... "
            "Nhập màu RGB (0-255 cho mỗi kênh Red/Green/Blue).",
            PropertyList({
                Property("red", kPropertyTypeInteger, 0, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 0, 255)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                uint8_t r = static_cast<uint8_t>(properties["red"].value<int>());
                uint8_t g = static_cast<uint8_t>(properties["green"].value<int>());
                uint8_t b = static_cast<uint8_t>(properties["blue"].value<int>());
                
                ESP_LOGI(TAG_MCP, "🎨 Setting LED color: R=%d G=%d B=%d", r, g, b);
                
                ninja_led_set_color(r, g, b);
                ninja_led_set_mode(LED_MODE_SOLID);
                ninja_led_update();
                
                return std::string("✅ SUCCESS: Đã đổi màu LED sang RGB(") + 
                       std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")!";
            });
        
        // Tool 12: Set LED mode
        mcp_server.AddTool(
            "robot.led.set_mode",
            "[Otto Ninja Robot] 🎯 ĐẶT CHẾ ĐỘ HIỆU ỨNG LED. "
            "Có 6 chế độ: off (tắt), solid (màu cố định), rainbow (cầu vồng), "
            "breathing (nhấp nhô thở), chase (chạy đuổi), blink (nháy). "
            "KHI NÀO GỌI: người dùng nói 'led cầu vồng', 'đèn chạy', 'led nháy', 'hiệu ứng thở', 'tắt đèn'...",
            PropertyList({
                Property("mode", kPropertyTypeString)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string mode_str = properties["mode"].value<std::string>();
                led_mode_t mode = LED_MODE_SOLID;
                std::string mode_name = "Solid";
                
                if (mode_str == "off" || mode_str == "tắt") {
                    mode = LED_MODE_OFF;
                    mode_name = "Tắt";
                } else if (mode_str == "solid" || mode_str == "cố định") {
                    mode = LED_MODE_SOLID;
                    mode_name = "Màu cố định";
                } else if (mode_str == "rainbow" || mode_str == "cầu vồng") {
                    mode = LED_MODE_RAINBOW;
                    mode_name = "Cầu vồng";
                } else if (mode_str == "breathing" || mode_str == "thở") {
                    mode = LED_MODE_BREATHING;
                    mode_name = "Nhấp nhô thở";
                } else if (mode_str == "chase" || mode_str == "chạy") {
                    mode = LED_MODE_CHASE;
                    mode_name = "Chạy đuổi";
                } else if (mode_str == "blink" || mode_str == "nháy") {
                    mode = LED_MODE_BLINK;
                    mode_name = "Nháy";
                }
                
                ESP_LOGI(TAG_MCP, "🎯 Setting LED mode: %s (%d)", mode_name.c_str(), mode);
                
                ninja_led_set_mode(mode);
                ninja_led_update();
                
                return std::string("✅ SUCCESS: Đã chuyển LED sang chế độ ") + mode_name + "!";
            });
        
        // Tool 13: Set LED brightness
        mcp_server.AddTool(
            "robot.led.set_brightness",
            "[Otto Ninja Robot] 💡 ĐẶT ĐỘ SÁNG LED (0-255). "
            "Điều chỉnh độ sáng của 8 đèn LED. "
            "KHI NÀO GỌI: người dùng nói 'tăng độ sáng led', 'giảm sáng đèn', 'led sáng 50%', 'đèn tối đi'... "
            "0 = tắt, 128 = 50%, 255 = sáng tối đa.",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 0, 255)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                uint8_t brightness = static_cast<uint8_t>(properties["brightness"].value<int>());
                
                ESP_LOGI(TAG_MCP, "💡 Setting LED brightness: %d", brightness);
                
                ninja_led_set_brightness(brightness);
                ninja_led_update();
                
                int percent = (brightness * 100) / 255;
                return std::string("✅ SUCCESS: Đã đặt độ sáng LED = ") + std::to_string(brightness) + 
                       " (" + std::to_string(percent) + "%)!";
            });
        
        // Tool 14: Set LED animation speed
        mcp_server.AddTool(
            "robot.led.set_speed",
            "[Otto Ninja Robot] ⚡ ĐẶT TỐC ĐỘ HIỆU ỨNG LED (10-500ms). "
            "Điều chỉnh tốc độ chuyển động của hiệu ứng LED (rainbow, chase, breathing, blink). "
            "KHI NÀO GỌI: người dùng nói 'led chạy nhanh hơn', 'đèn chậm lại', 'tốc độ hiệu ứng'... "
            "Giá trị nhỏ = nhanh, giá trị lớn = chậm.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 10, 10, 500)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                uint16_t speed = static_cast<uint16_t>(properties["speed"].value<int>());
                
                ESP_LOGI(TAG_MCP, "⚡ Setting LED speed: %d ms", speed);
                
                ninja_led_set_speed(speed);
                ninja_led_update();
                
                return std::string("✅ SUCCESS: Đã đặt tốc độ hiệu ứng LED = ") + std::to_string(speed) + "ms!";
            });
        
        // Tool 15: Get LED state
        mcp_server.AddTool(
            "robot.led.get_state",
            "[Otto Ninja Robot] ℹ️ KIỂM TRA TRẠNG THÁI LED HIỆN TẠI. "
            "Trả về màu sắc, độ sáng, chế độ hiệu ứng và tốc độ của LED. "
            "KHI NÀO GỌI: cần biết LED đang ở trạng thái gì.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                led_state_t* state = get_led_state();
                
                std::string mode_name;
                switch (state->mode) {
                    case LED_MODE_OFF: mode_name = "Tắt"; break;
                    case LED_MODE_SOLID: mode_name = "Màu cố định"; break;
                    case LED_MODE_RAINBOW: mode_name = "Cầu vồng"; break;
                    case LED_MODE_BREATHING: mode_name = "Nhấp nhô thở"; break;
                    case LED_MODE_CHASE: mode_name = "Chạy đuổi"; break;
                    case LED_MODE_BLINK: mode_name = "Nháy"; break;
                    default: mode_name = "Không rõ"; break;
                }
                
                return std::string("{\"success\": true, \"color\": {\"r\": ") + std::to_string(state->r) + 
                       ", \"g\": " + std::to_string(state->g) + ", \"b\": " + std::to_string(state->b) + 
                       "}, \"brightness\": " + std::to_string(state->brightness) + 
                       ", \"mode\": \"" + mode_name + "\", \"speed\": " + std::to_string(state->speed) + 
                       ", \"description\": \"LED đang ở chế độ " + mode_name + 
                       " với màu RGB(" + std::to_string(state->r) + "," + std::to_string(state->g) + "," + std::to_string(state->b) + 
                       "), độ sáng " + std::to_string((state->brightness * 100) / 255) + "%\"}";
            });
        
        // Tool 16: Turn off all LEDs
        mcp_server.AddTool(
            "robot.led.off",
            "[Otto Ninja Robot] 💤 TẮT TẤT CẢ LED. "
            "Tắt hết 8 đèn LED. "
            "KHI NÀO GỌI: người dùng nói 'tắt đèn', 'tắt led', 'led off', 'tắt hết đèn'...",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "💤 Turning off all LEDs");
                
                ninja_led_off();
                
                return std::string("✅ SUCCESS: Đã tắt hết LED!");
            });
        
        // Tool 17: Save LED state to NVS
        mcp_server.AddTool(
            "robot.led.save",
            "[Otto Ninja Robot] 💾 LƯU TRẠNG THÁI LED vào bộ nhớ NVS. "
            "Lưu cấu hình LED hiện tại để khôi phục khi khởi động lại. "
            "KHI NÀO GỌI: người dùng nói 'lưu cài đặt led', 'nhớ màu đèn này', 'save led'...",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "💾 Saving LED state to NVS");
                
                save_led_state_to_nvs();
                
                return std::string("✅ SUCCESS: Đã lưu trạng thái LED vào bộ nhớ!");
            });
        
        // ========== WAVE LEG TOOLS (tách từ combo) ==========
        
        // Tool 18: Wave Right Leg (vẫy chân phải - từ combo1)
        mcp_server.AddTool(
            "robot.wave_right_leg",
            "[Otto Ninja Robot] 🦵 VẪY CHÂN PHẢI. "
            "Robot vẫy chân phải 3 lần (135→155→180°) và giữ nguyên tư thế. "
            "KHI NÀO GỌI: người dùng nói 'vẫy chân phải', 'giơ chân phải', 'đá chân phải', 'wave right leg'.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "🦵 >>> TOOL: wave_right_leg");
                control_state_t* state = get_control_state();
                
                state->manual_mode = true;
                
                // Attach and wave Right Leg 3 times: 135 -> 155 -> 180
                servo_attach(SERVO_CH_RIGHT_LEG);
                for (int i = 0; i < 3; i++) {
                    servo_write(SERVO_CH_RIGHT_LEG, 135);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_RIGHT_LEG, 155);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_RIGHT_LEG, 180);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                
                return std::string("✅ SUCCESS: Robot đã vẫy chân phải 3 lần!");
            });
        
        // Tool 19: Wave Left Leg (vẫy chân trái - từ combo2)
        mcp_server.AddTool(
            "robot.wave_left_leg",
            "[Otto Ninja Robot] 🦵 VẪY CHÂN TRÁI. "
            "Robot vẫy chân trái 3 lần (10→30→75°) và giữ nguyên tư thế. "
            "KHI NÀO GỌI: người dùng nói 'vẫy chân trái', 'giơ chân trái', 'đá chân trái', 'wave left leg'.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_MCP, "🦵 >>> TOOL: wave_left_leg");
                control_state_t* state = get_control_state();
                
                state->manual_mode = true;
                
                // Attach and wave Left Leg 3 times: 10 -> 30 -> 75
                servo_attach(SERVO_CH_LEFT_LEG);
                for (int i = 0; i < 3; i++) {
                    servo_write(SERVO_CH_LEFT_LEG, 10);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_LEFT_LEG, 30);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    servo_write(SERVO_CH_LEFT_LEG, 75);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                
                return std::string("✅ SUCCESS: Robot đã vẫy chân trái 3 lần!");
            });
        
        // ========== ACTION JSON EXPORT/IMPORT TOOLS ==========
        
        // Tool 20: Export action slot to JSON
        mcp_server.AddTool(
            "robot.action.export",
            "[Otto Ninja Robot] 📤 XUẤT HÀNH ĐỘNG ĐÃ LƯU RA JSON. "
            "Xuất toàn bộ hành động từ slot (1-3) ra chuỗi JSON để chia sẻ. "
            "KHI NÀO GỌI: người dùng nói 'xuất hành động', 'export action', 'chia sẻ động tác', "
            "'lưu json', 'xuất slot 1', 'export slot'.",
            PropertyList({Property("slot", kPropertyTypeInteger, 1, 1, 3)}),
            [](const PropertyList& properties) -> ReturnValue {
                int slot_num = 1;
                try { slot_num = properties["slot"].value<int>(); } catch (...) {}
                
                int slot_idx = slot_num - 1;
                if (slot_idx < 0 || slot_idx >= 3) {
                    return std::string("❌ FAILED: Slot không hợp lệ! Chỉ có slot 1, 2, 3.");
                }
                
                action_slot_t* slot = get_action_slot(slot_idx);
                if (!slot || slot->count == 0) {
                    return std::string("❌ FAILED: Slot ") + std::to_string(slot_num) + " trống!";
                }
                
                // Build JSON
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "name", (std::string("Otto Ninja Action Slot ") + std::to_string(slot_num)).c_str());
                cJSON_AddNumberToObject(root, "slot", slot_num);
                cJSON_AddNumberToObject(root, "count", slot->count);
                
                cJSON* steps_arr = cJSON_CreateArray();
                for (int i = 0; i < slot->count; i++) {
                    action_step_t* step = &slot->steps[i];
                    cJSON* step_obj = cJSON_CreateObject();
                    cJSON_AddStringToObject(step_obj, "type", action_type_to_string(step->type));
                    cJSON_AddNumberToObject(step_obj, "param1", step->param1);
                    cJSON_AddNumberToObject(step_obj, "param2", step->param2);
                    cJSON_AddNumberToObject(step_obj, "duration_ms", step->duration_ms);
                    cJSON_AddItemToArray(steps_arr, step_obj);
                }
                cJSON_AddItemToObject(root, "steps", steps_arr);
                
                char* json_str = cJSON_Print(root);
                std::string result(json_str);
                cJSON_free(json_str);
                cJSON_Delete(root);
                
                ESP_LOGI(TAG_MCP, "📤 Exported slot %d: %d actions", slot_num, slot->count);
                
                return std::string("✅ JSON xuất từ Slot ") + std::to_string(slot_num) + 
                       " (" + std::to_string(slot->count) + " hành động):\n" + result;
            });
        
        // Tool 21: Import action slot from JSON
        mcp_server.AddTool(
            "robot.action.import",
            "[Otto Ninja Robot] 📥 NẠP HÀNH ĐỘNG TỪ JSON VÀO SLOT. "
            "Nhập chuỗi JSON hành động (đã xuất trước đó) vào slot (1-3). "
            "KHI NÀO GỌI: người dùng nói 'nạp hành động', 'import action', 'nhập json', "
            "'nạp động tác', 'import slot', 'load json'. "
            "THAM SỐ: 'slot' = slot đích (1-3), 'json' = chuỗi JSON hành động.",
            PropertyList({
                Property("slot", kPropertyTypeInteger, 1, 1, 3),
                Property("json", kPropertyTypeString)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int slot_num = 1;
                std::string json_str;
                try { slot_num = properties["slot"].value<int>(); } catch (...) {}
                try { json_str = properties["json"].value<std::string>(); } catch (...) {
                    return std::string("❌ FAILED: Thiếu dữ liệu JSON!");
                }
                
                if (json_str.empty()) {
                    return std::string("❌ FAILED: Chuỗi JSON trống!");
                }
                
                int slot_idx = slot_num - 1;
                if (slot_idx < 0 || slot_idx >= 3) {
                    return std::string("❌ FAILED: Slot không hợp lệ! Chỉ có slot 1, 2, 3.");
                }
                
                // Parse JSON
                cJSON* root = cJSON_Parse(json_str.c_str());
                if (!root) {
                    return std::string("❌ FAILED: JSON không hợp lệ! Lỗi parse.");
                }
                
                cJSON* steps_arr = cJSON_GetObjectItem(root, "steps");
                cJSON* count_item = cJSON_GetObjectItem(root, "count");
                if (!steps_arr || !cJSON_IsArray(steps_arr)) {
                    cJSON_Delete(root);
                    return std::string("❌ FAILED: JSON thiếu mảng 'steps'!");
                }
                
                int count = cJSON_GetArraySize(steps_arr);
                if (count_item && cJSON_IsNumber(count_item)) {
                    int declared_count = count_item->valueint;
                    if (declared_count < count) count = declared_count;
                }
                if (count > MAX_ACTIONS) count = MAX_ACTIONS;
                if (count <= 0) {
                    cJSON_Delete(root);
                    return std::string("❌ FAILED: JSON không có hành động nào!");
                }
                
                // Load into slot
                action_slot_t* slot = get_action_slot(slot_idx);
                memset(slot, 0, sizeof(action_slot_t));
                slot->count = count;
                
                for (int i = 0; i < count; i++) {
                    cJSON* step_obj = cJSON_GetArrayItem(steps_arr, i);
                    if (!step_obj) break;
                    
                    cJSON* type_item = cJSON_GetObjectItem(step_obj, "type");
                    cJSON* p1_item = cJSON_GetObjectItem(step_obj, "param1");
                    cJSON* p2_item = cJSON_GetObjectItem(step_obj, "param2");
                    cJSON* dur_item = cJSON_GetObjectItem(step_obj, "duration_ms");
                    
                    if (type_item && cJSON_IsString(type_item)) {
                        slot->steps[i].type = string_to_action_type(type_item->valuestring);
                    }
                    if (p1_item && cJSON_IsNumber(p1_item)) {
                        slot->steps[i].param1 = (int16_t)p1_item->valueint;
                    }
                    if (p2_item && cJSON_IsNumber(p2_item)) {
                        slot->steps[i].param2 = (int16_t)p2_item->valueint;
                    }
                    if (dur_item && cJSON_IsNumber(dur_item)) {
                        slot->steps[i].duration_ms = (uint16_t)dur_item->valueint;
                    }
                }
                
                cJSON_Delete(root);
                
                // Save to NVS
                save_actions_to_nvs(slot_idx);
                
                ESP_LOGI(TAG_MCP, "📥 Imported %d actions into slot %d", count, slot_num);
                
                return std::string("✅ SUCCESS: Đã nạp ") + std::to_string(count) + 
                       " hành động vào Slot " + std::to_string(slot_num) + " và lưu vào NVS!";
            });
        
        ESP_LOGI(TAG_MCP, "Robot MCP tools registered successfully (21 tools: 14 robot + 7 LED)");
    }
};

#endif // __ROBOT_MCP_CONTROLLER_H__
