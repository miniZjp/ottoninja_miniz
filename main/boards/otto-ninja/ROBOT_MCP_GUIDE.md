# Robot MCP Controller

## Mô tả
Robot MCP Controller đăng ký các MCP tools để điều khiển robot Otto Ninja bằng giọng nói thông qua AI.

## Các MCP Tools đã đăng ký

### 1. `robot.step_forward`
**Mô tả:** Tiến tới 1 bước
- Robot sẽ kiểm tra mode hiện tại (walk hoặc roll)
- Thực hiện di chuyển tiến với mode đó trong 500ms

**Ví dụ lệnh giọng nói:**
- "Tiến tới 1 bước"
- "Đi thẳng"
- "Move forward"

### 2. `robot.step_backward`
**Mô tả:** Lùi 1 bước
- Robot sẽ kiểm tra mode hiện tại
- Thực hiện di chuyển lùi với mode đó trong 500ms

**Ví dụ lệnh giọng nói:**
- "Lùi lại"
- "Đi lùi"
- "Step backward"

### 3. `robot.turn_left`
**Mô tả:** Quay trái
- Robot sẽ kiểm tra mode hiện tại
- Thực hiện quay trái với mode đó trong 500ms

**Ví dụ lệnh giọng nói:**
- "Quay trái"
- "Rẽ trái"
- "Turn left"

### 4. `robot.turn_right`
**Mô tả:** Quay phải
- Robot sẽ kiểm tra mode hiện tại
- Thực hiện quay phải với mode đó trong 500ms

**Ví dụ lệnh giọng nói:**
- "Quay phải"
- "Rẽ phải"
- "Turn right"

### 5. `robot.get_mode`
**Mô tả:** Lấy thông tin mode hiện tại
- Trả về JSON: `{"mode": "walk"}` hoặc `{"mode": "roll"}`

**Ví dụ lệnh giọng nói:**
- "Robot đang ở mode nào?"
- "Kiểm tra mode hiện tại"

### 6. `robot.set_walk_mode`
**Mô tả:** Chuyển sang mode đi bộ (walk)

**Ví dụ lệnh giọng nói:**
- "Chuyển sang mode walk"
- "Đi bộ"

### 7. `robot.set_roll_mode`
**Mô tả:** Chuyển sang mode lăn (roll)

**Ví dụ lệnh giọng nói:**
- "Chuyển sang mode roll"
- "Lăn đi"

### 8. `robot.go_home`
**Mô tả:** Trở về vị trí home
- Tất cả servo về vị trí mặc định

**Ví dụ lệnh giọng nói:**
- "Về nhà"
- "Go home"
- "Trở về vị trí ban đầu"

## Cách hoạt động

1. **Kiểm tra mode tự động:** Mỗi khi thực hiện lệnh di chuyển (tiến, lùi, quay), robot tự động kiểm tra mode hiện tại (walk/roll) và thực hiện hành động phù hợp với mode đó.

2. **Control state:** Các lệnh MCP điều khiển thông qua `control_state_t` - cập nhật joystick values (j_x, j_y) giống như điều khiển qua web UI.

3. **Task loop:** `robot_control_task` sẽ đọc control_state và thực hiện hành động tương ứng:
   - Mode WALK: Gọi `ninja_walk()`
   - Mode ROLL: Gọi `ninja_roll()`

## Implementation

File: `robot_mcp_controller.h`

```cpp
class RobotMcpController {
public:
    RobotMcpController() {
        // Đăng ký các MCP tools với McpServer
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(...);
    }
};
```

Được khởi tạo trong `compact_wifi_board_lcd.cc`:

```cpp
void InitializeTools() {
    static LampController lamp(LAMP_GPIO);
    static RobotMcpController robot_mcp;  // Khởi tạo MCP tools
}
```

## Test

1. Build và flash firmware
2. Kết nối với AI assistant (có hỗ trợ MCP)
3. Nói các lệnh như: "Tiến tới 1 bước", "Quay trái", "Lùi lại"
4. Robot sẽ tự động kiểm tra mode và thực hiện hành động

## Log

Khi thực hiện lệnh, log sẽ hiển thị:
```
I (12345) RobotMCP: Step forward - Mode: WALK
I (12345) RobotMCP: Turn left - Mode: ROLL
I (12345) RobotMCP: Robot MCP tools registered successfully
```
