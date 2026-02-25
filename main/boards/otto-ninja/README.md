# Otto Ninja Robot Board

Otto Ninja là board robot chuyên dụng với khả năng biến hình giữa 2 chế độ:
- **Walk Mode (Chế độ đi bộ)**: Robot đứng thẳng bằng 2 chân, giơ tay lên
- **Roll Mode (Chế độ lăn)**: Robot hạ thấp xuống thành xe có bánh, lăn đi

## Tính năng

### 🤖 Robot Control
- **7 Servo motors**: 
  - Left Foot (LF) - Bánh trái
  - Right Foot (RF) - Bánh phải
  - Left Leg (LL) - Chân trái
  - Right Leg (RL) - Chân phải
  - Left Arm (LA) - Tay trái
  - Right Arm (RA) - Tay phải
  - Head - Đầu

- **Chế độ điều khiển**:
  - Web UI với joystick và calibration
  - Voice control qua LLM (Model Context Protocol)
  - 10 MCP tools cho AI assistant

### 🎮 Calibration Features
- **Walk Speed**: Tốc độ di chuyển khi đi bộ (LF/RF forward/backward)
- **Roll Speed**: Tốc độ bánh xe khi lăn (4 speeds riêng biệt)
- **Transform Speed**: Tốc độ biến hình LL/RL (1-20ms per degree)
- **Leg Positions**: Điều chỉnh góc chân đứng/lăn/nghiêng
- **Turn Speed**: Tốc độ quay trái/phải

### 📡 Connectivity
- WiFi AP mode: SSID "OTTO NINJA", Password "12345678"
- Web UI: http://192.168.4.1
- WebSocket support cho real-time control
- NVS storage cho lưu settings

### 🖥️ Display
- LCD 240x320 SPI (ST7789, GC9A01, ILI9341)
- Backlight PWM control
- Custom UI cho robot status

## Hardware Configuration

### Servo Pins (ESP32-S3)
```
Left Foot:   GPIO 13 (D7)
Right Foot:  GPIO 0  (D3)
Left Leg:    GPIO 15 (D8)
Right Leg:   GPIO 2  (D4)
Left Arm:    GPIO 16 (D0)
Right Arm:   GPIO 3  (RX)
Head:        GPIO 1  (TX)
```

### LCD Pins
```
MOSI: GPIO 11
CLK:  GPIO 12
CS:   GPIO 10
DC:   GPIO 13
RST:  GPIO 14
BL:   GPIO 21
```

### Other Pins
```
Boot Button: GPIO 0
LED:         GPIO 48
Battery ADC: GPIO 4 (ADC1_CH3)
```

## Build & Flash

1. Select board type:
```bash
idf.py menuconfig
# Board Configuration -> Board Type -> Otto Ninja Robot
```

2. Build:
```bash
idf.py build
```

3. Flash:
```bash
idf.py flash monitor
```

## Web UI Usage

1. Connect to WiFi "OTTO NINJA" (password: 12345678)
2. Open browser to http://192.168.4.1
3. Use joystick to control robot
4. Calibrate servo positions in Settings tab
5. Press X button to switch to Roll mode
6. Press Y button to switch to Walk mode

## Voice Control (MCP)

Robot hỗ trợ 10 MCP tools:
- `robot.move_forward` - Tiến về phía trước
- `robot.move_backward` - Lùi về phía sau
- `robot.roll_and_go` - Biến hình và lăn đi
- `robot.set_roll_mode` - Chuyển sang chế độ lăn
- `robot.set_walk_mode` - Chuyển sang chế độ đi bộ
- `robot.go_home` - Về vị trí home
- `robot.play_slot` - Chơi action đã lưu
- `robot.start_recording` - Bắt đầu record action
- `robot.stop_recording` - Dừng record
- `robot.record_current_position` - Lưu vị trí hiện tại

## Advanced Features

### Single-Step Walk Mode
Walk cycle thực thi 1 lần duy nhất mỗi lần kích hoạt:
- Joystick về neutral (0,0) → arm trigger
- Joystick activated → execute 1 cycle → stop
- Phải release joystick về neutral trước bước tiếp theo

### Independent Roll Speeds
4 tốc độ riêng biệt cho roll mode:
- LF Forward Speed (10-60)
- LF Backward Speed (10-60)
- RF Forward Speed (10-60)
- RF Backward Speed (10-60)

### Smooth Transform
LL và RL di chuyển mượt mà khi biến hình:
- Transform LL Speed (1-20ms per degree)
- Transform RL Speed (1-20ms per degree)
- Cả 2 chân di chuyển đồng thời với tốc độ riêng

## Based On

Board này được phát triển dựa trên:
- [Otto DIY Ninja](https://www.ottodiy.com/ninja) - Original Otto Ninja design
- [OttoDIY/OttoNinja](https://github.com/OttoDIY/OttoNinja) - Arduino libraries
- Xiaozhi ESP32 framework - ESP-IDF implementation

## Credits

- Original Otto Ninja: Sebastian Coddington
- ESP32 Port: Xiaozhi Project
- Enhancements: Custom calibration, smooth transform, MCP integration
