# Force-Feedback-IMU-Handle

基于自研 **ESP32-C3FH4** + **BMI088（SPI）** + **GA12-N20 霍尔编码减速电机** + **DRV8833** 的力反馈 IMU 手柄样机。

工程结构与已验证的 CAN_Magnetic_Encoder 对齐：ESP-IDF 分层 APP / SVC / DRV / BSP。

## 能力摘要

- BMI088 六轴原始数据（Acc/Gyro 独立片选，SPI Mode 3，8 MHz）
- 单轴 N20 正交编码器测角 / 测速
- 阻抗力反馈环：`tau = K*(theta_des - theta) - B*omega`，PWM 限幅
- USB-Serial/JTAG JSON 调参与遥测（见 `protocol/usb_protocol.md`）

## 仓库结构

```text
Force-Feedback-IMU-Handle/
├── hardware/      # 引脚表、接线说明、BOM 草案
├── software/      # ESP-IDF：ESP32-C3FH4（单核）
├── protocol/      # USB JSON 协议
├── tools/         # PC 调试脚本（占位）
├── tests/         # Bring-up 清单
└── docs/          # 方案与固件说明
```

## 硬件组合

| 模块 | 型号 | 说明 |
|------|------|------|
| 主控 | ESP32-C3FH4 | 片内 4 MB Flash；SPI MISO 使用 GPIO6 |
| IMU | BMI088 | SPI，CS_ACC=GPIO5，CS_GYRO=GPIO7 |
| 加热 | 100Ω + YJL3400A | GPIO21 PWM，抑制温漂 |
| 电机 | GA12-N20 + 霍尔编码器 | ENC_A/B = GPIO0/1 |
| 驱动 | DRV8833 | IN1/IN2 = GPIO2/3，nSLEEP = GPIO20 |

## 快速入口

| 要做什么 | 看哪里 |
|----------|--------|
| 引脚 / 接线 | `hardware/pinmap.md` |
| 编译烧录 | `docs/getting_started.md` |
| 固件分层 | `docs/firmware.md` |
| USB JSON | `protocol/usb_protocol.md` |
| 上电调试 | `tests/bringup_checklist.md` |

## 默认引脚（与 CAN 磁编 SPI 一致）

| 功能 | C3 GPIO |
|------|---------|
| SPI SCK / MISO / MOSI | 8 / 6 / 10 |
| BMI088 CS_ACC / CS_GYRO | 5 / 7 |
| IMU 加热（YJL3400A） | 21 |
| MOTOR IN1 / IN2 / nSLEEP | 2 / 3 / 20 |
| ENC A / B | 0 / 1 |
| WS2812 | 4 |
| 用户键 / BOOT | 9 |
| USB D− / D+ | 18 / 19 |

## 开发约定

- ESP-IDF **6.x**，目标 `esp32c3`，只允许 APP → SVC → DRV → BSP 向下依赖
- 单核：采样任务优先级 7，高于 USB 遥测
- 电机电源与 3.3 V 逻辑分离；编码器供电用 3.3 V
