# software — ESP32-C3FH4 固件

ESP-IDF 6.x 固件，目标 `esp32c3`、4 MB Flash。当前硬件为 BMI088、DRV8876 PH/EN + IPROPI 和 GA12-N20 编码器；GPIO4 用于电流采样，本板无 WS2812。

## 源码入口

| 层 | 模块 | 职责 |
|----|------|------|
| APP | `main/app_main.c`、`app_sampler.c`、`app_usb.c`、`app_telem_json.c`、`app_wifi.c` | 初始化、任务调度、共享 JSON 命令、轻量遥测组包、USB 配网和 Wi-Fi WebSocket |
| SVC | `params`、`imu_ff`、`imu_kalman`、`imu_attitude` | NVS、采样与遥测汇总、六轴降噪、板端四元数姿态融合 |
| SVC | `ff_impedance`、`motor_pid`、`motor_torque`、`torque_dob` | 阻抗、位置、电流控制及力矩估计 |
| DRV | `bmi088`、`drv8876`、`n20_encoder` | 设备驱动、编码器物理量换算 |
| BSP | SPI、PWM、ADC、GPIO、编码器、USB、加热 | ESP32-C3 板级接口 |

采样任务 `app_sam` 优先级 7，名义周期 1 ms；遥测任务 `app_telem` 优先级 5，默认 50 Hz；USB 收包及按键任务 `app_idle` 优先级 4，每 10 ms 轮询。采样使用 `ESP_TIMER_TASK` 回调唤醒，实际节拍受单核调度影响。详细机制见 [固件架构](../docs/firmware.md)。

## GPIO 分配

| 功能 | C3 GPIO |
|------|---------|
| BMI088 SCK / MISO / MOSI | 8 / 6 / 10 |
| BMI088 CS_ACC / CS_GYRO | 5 / 7 |
| IMU 加热 Gate | 21 |
| MOTOR EN / PH / nSLEEP | 2 / 3 / 20 |
| IPROPI ADC1_CH4 | 4 |
| 编码器 A / B | 0 / 1 |
| 用户键 / BOOT | 9 |
| 原生 USB D− / D+ | 18 / 19 |

[bsp_pin.h](components/bsp/include/bsp/bsp_pin.h) 必须与 [硬件引脚表](../hardware/pinmap.md) 同步。GPIO9 不接 MISO。

## 编译与烧录

推荐在仓库根目录、已加载 ESP-IDF 的终端执行：

```powershell
idf.py build
idf.py -p COM3 flash
```

`COM3` 替换为实际端口。需要查看启动日志时执行 `idf.py -p COM3 monitor`，退出后再使用网页；monitor、网页和诊断脚本不能同时占用同一串口。仓库根构建生成 `build/ff_imu_handle.bin`；完整环境说明见 [快速上手](../docs/getting_started.md)。

## 运行行为

- 模式 0 开环／停机，1 阻抗，2 位置 PID，3 力矩／电流 PI。`duty=0` 切回模式 0 并清除开环输出。
- 编码器为 A/B 双边沿四倍频；固件内置默认 `enc_cpr=8344` 对应原 298:1 电机。当前更换的 `JGA12-N20-30` 为 30:1、6 V 空载标称 500 r/min，暂按 7 PPR 配置 `enc_cpr=7 × 4 × 30=840` 并保存 NVS。参数表未给出 PPR，仍需实测确认；换算时不再额外乘四或除以减速比。步骤见 [电机更换说明](../README.md#电机更换与编码器参数)。
- 板端 IMU 卡尔曼默认开启，原始 `acc`/`gyro` 与滤波 `acc_f`/`gyro_f` 分别上报；`imu_filter` 独立开关，`save` 保存。
- 板端姿态融合优先使用有效卡尔曼六轴，否则使用原始六轴；输出 `quat=[w,x,y,z]`，以 `quat_ok` 判断有效、`quat_kf` 判断输入来源。航向为相对值，网页不再对 USB 六轴重新融合。
- `save` 保存当前模式、目标、增益、编码器标定、滤波开关及加热占空比。IMU 初始化成功时恢复存档；初始化失败时本次运行关闭初始闭环和加热，但不覆盖存档。开环占空比不存档。
- 原 298:1 电机的旧默认编码器存档按版本迁移为 8344，其他自定义值（包括新电机设置的 840）保留；规则见 [编码器输出轴换算](../docs/firmware.md#编码器输出轴换算)。
- IMU 初始化失败仍启动 USB 服务，`imu_ok=0`；`diag` 读取缓存诊断，`imu_probe` 仅在初始化尚未成功时探测总线。

USB 命令和字段见 [协议](../protocol/usb_protocol.md)，网页使用见 [tools](../tools/README.md)，实板验证见 [Bring-up 清单](../tests/bringup_checklist.md)。

遥测使用独立 `esp_timer` 按 `telem_hz` 唤醒发送任务，100 Hz 对应 10 ms；修改频率同步调整定时器，启动使用 NVS 恢复值。组包或发送时间不再与额外周期延时叠加，过载时丢弃积压通知而不补发。遥测采用轻量十进制组包，诊断提供分段耗时用于区分组包负载与通信等待；字段和单位不变。用户已确认本板 USB 配置 100 Hz 而仅接收约 50 帧/s 的问题在更新后解决；具体实测数值未记录。Wi-Fi 和其他负载下的速率仍需单独测量，见 [遥测验证记录](../docs/firmware.md#遥测验证记录)。

## Wi-Fi

`wifi_set` / `wifi_get` / `wifi_enable` / `wifi_forget` 通过 USB 配置、查询、开关及清除网络。首次默认关闭，保存配置不自动开启；配置和开关由独立 `ffh_wifi` NVS 保存，重启后恢复。开启时 STA 连接 2.4 GHz 网络，HTTP 端口 80 的 `/ws` 提供共享控制及遥测；关闭时停止 HTTP、注销 Wi-Fi 事件回调、释放无线驱动和 STA 网口，保留配置与密钥。优先级 3 的 Wi-Fi 管理任务在关闭时阻塞，无周期重连；首次启用后的 lwIP 和系统事件循环保留。USB / Wi-Fi 独立订阅，网络遥测通过 HTTP 任务发送，不在采样任务中发包。初始化或切换失败不会关闭 USB；通过 `busy` / `error` / `running` 查询异步结果。

`Kconfig.projbuild` 自动选中 WebSocket 及握手回调支持，兼容已有 sdkconfig；ESP-IDF 6 的 cJSON 由 `main/idf_component.yml` 声明，首次编译需下载官方组件。完整协议、调度及连接限制见 [Wi-Fi 说明](../docs/wifi.md)。
