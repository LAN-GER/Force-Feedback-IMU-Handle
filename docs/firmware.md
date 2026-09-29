# 固件架构

## 分层

```text
APP (main)     app_main / app_sampler / app_usb
    ↓
SVC            params / ff_impedance / imu_ff / indicate
    ↓
DRV            bmi088 / drv8833 / n20_encoder
    ↓
BSP            spi / motor_pwm / encoder / usb / ws2812 / gpio
```

只允许向下依赖。

## 任务

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| `app_sam` | 7 | 1 kHz | IMU 读 + 编码器 + 阻抗环 → PWM |
| `app_telem` | 5 | telem_hz | USB 遥测推送 |
| `app_idle` | 4 | 10 ms | USB 收包、WS2812 灯色 |

## SPI

- `SPI2_HOST`，Mode 3，8 MHz
- 软件片选：`CS_ACC=GPIO5`，`CS_GYRO=GPIO7`
- Acc 读寄存器丢弃 1 个 dummy byte

## 编码器

- ESP32-C3 **无硬件 PCNT**，`bsp_encoder` 用 GPIO 双边沿中断 + 正交状态机做四倍频

## 力反馈

```text
tau = K * (theta_des - theta) - B * omega
duty = clamp(tau, -duty_max, +duty_max)
```

软限位 `theta_min/max` 外额外回推。
