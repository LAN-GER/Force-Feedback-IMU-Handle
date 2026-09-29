# software — ESP32-C3FH4 固件

固件采用 APP → SVC → DRV → BSP 分层：

1. **引脚**：见 `components/bsp/include/bsp/bsp_pin.h`（与 `hardware/pinmap.md` 同步）
2. **调度**：单核，`app_sam` 优先级 7、`app_telem` 优先级 5、`app_idle` 优先级 4
3. **状态灯**：GPIO4 WS2812
4. **目标**：`esp32c3`，Flash 默认 4 MB

## GPIO 分配

| 功能 | C3 GPIO |
|------|----------|
| SPI SCK | GPIO8 |
| SPI MISO | GPIO6 |
| BOOT，禁止接 MISO | GPIO9 |
| SPI MOSI | GPIO10 |
| BMI088 CS_ACC | GPIO5 |
| BMI088 CS_GYRO | GPIO7 |
| IMU 加热 Gate | GPIO21 |
| MOTOR IN1 / IN2 | GPIO2 / GPIO3 |
| MOTOR nSLEEP | GPIO20 |
| ENC A / B | GPIO0 / GPIO1 |
| WS2812 | GPIO4 |
| 用户键 / BOOT | GPIO9 |
| 原生 USB D− / D+ | GPIO18 / GPIO19 |

## 编译烧录

推荐从仓库根目录构建：

```powershell
cd E:\SeeedWork\Force-Feedback-IMU-Handle
idf.py build
idf.py -p COM3 flash monitor
```

上电后串口应看到 `bsp_spi: SPI Mode3 8000000 Hz` 与 `bmi088: acc_id=0x1E gyro_id=0x0F`。
