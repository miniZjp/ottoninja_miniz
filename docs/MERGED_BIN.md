# Tạo Merged Binary File

## Giới thiệu

Merged binary file là file nhị phân đã merge tất cả các phần (bootloader, partition table, app) vào một file duy nhất. Điều này giúp quá trình flash đơn giản hơn, chỉ cần flash 1 file thay vì phải flash nhiều file ở nhiều offset khác nhau.

## Cách sử dụng

### Phương pháp 1: Sử dụng batch script (Windows)

1. Build project trước:
   ```bash
   idf.py build
   ```

2. Chạy script:
   ```bash
   create_merged_bin.bat
   ```

3. File merged sẽ được tạo trong thư mục `build/` với tên `xiaozhi-merged.bin`

### Phương pháp 2: Sử dụng Python script trực tiếp

```bash
python scripts/create_merged_bin.py
```

### Phương pháp 3: Sử dụng CMake target

```bash
idf.py merged-bin
```

## Flash Merged Binary

Sau khi tạo merged binary, flash bằng lệnh sau:

```bash
esptool.py --chip esp32s3 --port COM3 write_flash 0x0 build/xiaozhi-merged.bin
```

Thay `COM3` bằng cổng COM của thiết bị ESP32 của bạn.

### Sử dụng idf.py (nếu đã config port)

```bash
python %IDF_PATH%/components/esptool_py/esptool/esptool.py --chip esp32s3 -p COM3 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 build/xiaozhi-merged.bin
```

## Lợi ích của Merged Binary

1. **Đơn giản**: Chỉ cần flash 1 file duy nhất
2. **Nhanh chóng**: Không cần chỉ định nhiều offset
3. **Ít lỗi**: Giảm nguy cơ flash sai offset
4. **Dễ phân phối**: Chỉ cần gửi 1 file cho người dùng cuối

## Cấu trúc Merged Binary

File merged bao gồm:
- **0x0** - Bootloader (second stage bootloader)
- **0x8000** - Partition table
- **0x10000** - Application binary
- **0xXXXXX** - Other data partitions (if any)

## Lưu ý

- File merged chỉ cần tạo lại khi build lại project
- Kích thước file merged thường khoảng 2-4 MB tùy vào cấu hình
- Flash mode, freq và size phải khớp với cấu hình sdkconfig
