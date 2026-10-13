# USB JSON 行协议

当前固件另支持 `wifi_get`、`wifi_set`、`wifi_enable`、`wifi_forget`，仅通过 USB 修改网络配置和无线开关；关闭时保留账号密码并释放无线驱动与网络服务。Wi-Fi WebSocket 复用下列控制命令与遥测字段，帧格式和接入密钥见 [Wi-Fi 协议](../docs/wifi.md)。

设备通过原生 USB-Serial/JTAG 收发。主机发送一行 JSON（`\n` 结尾），设备应答以 `>>` 前缀。

## 控制模式 `ctrl_mode`

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | OFF | 开环 `duty` 或停机 |
| 1 | IMPEDANCE | 阻抗力反馈 |
| 2 | POS_PID | **位置 PID 闭环** |
| 3 | TORQUE | **力矩闭环**（IPROPI 电流 PI） |

## 命令

| cmd | 说明 |
|-----|------|
| `get` | 读当前参数 |
| `set` | 改参数（可只含部分字段） |
| `save` | 写入 NVS |
| `zero` | 编码器清零 + 复位 PID/DOB/力矩积分 |
| `sub` / `unsub` | 订阅 / 取消遥测 |
| `duty` | 开环占空比（强制 `ctrl_mode=0`） |
| `heater` | IMU 加热：`{"cmd":"heater","v":0.4}` |

### set 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `ctrl_mode` | 0–3 | 控制模式 |
| `ff_enable` | 0/1 | 兼容：1→阻抗，0→若当前为阻抗则 OFF |
| `k` / `b` | float | 阻抗刚度 / 阻尼 |
| `theta_des` | float | 目标角 rad（阻抗 / 位置 PID） |
| `duty_max` | float | 占空比上限 |
| `theta_min` / `theta_max` | float | 软限位 |
| `enc_cpr` / `enc_dir` | int | 输出轴每转四倍频计数（含减速比，默认 7 × 4 × 298 = 8344；7 PPR 由粗测推断）/ 编码器方向 |
| `telem_hz` | 1–200 | 遥测频率 |
| `heater_duty` | float | 加热 0~1 |
| `pid_kp` / `pid_ki` / `pid_kd` | float | 位置 PID |
| `tau_des` | float | 目标力矩 N·m |
| `tq_kp` / `tq_ki` | float | 电流环 PI |
| `i_max` | float | 电流限幅 A |
| `kt` | float | 力矩系数 N·m/A（与 `dob_kt` 同步） |
| `dob_*` | — | DOB 参数（见下） |

示例：

```json
{"cmd":"set","ctrl_mode":2,"theta_des":0.5,"pid_kp":2.0,"pid_ki":0.5,"pid_kd":0.05}
{"cmd":"set","ctrl_mode":3,"tau_des":0.01,"tq_kp":1.5,"tq_ki":8,"i_max":1.0,"kt":0.02}
{"cmd":"set","ctrl_mode":1,"k":1.2,"b":0.08,"ff_enable":1}
{"cmd":"duty","v":0.25}
{"cmd":"sub"}
```

### 只读诊断 `diag`

发送 `{"cmd":"diag"}`，回复 `{"t":"ok","cmd":"diag","diag":{...}}`。诊断只查询初始化缓存及运行统计，不访问 SPI、不复位 IMU、不修改控制参数；旧固件不支持此命令时会返回 `unknown`。

| 字段 | 说明 |
|------|------|
| `firmware_sha` | 当前固件 ELF SHA256 的前 16 个十六进制字符 |
| `uptime_ms` / `reset_reason` | 运行时间 ms / ESP-IDF `esp_reset_reason_t` 数值 |
| `sample_seq` | 当前采样序号，连续查询可确认采样任务是否运行 |
| `telem_hz` | 当前配置遥测频率 Hz，默认 50 |
| `telem_work_us` / `telem_work_max_us` | 最近一次 / 本次启动最高的快照获取、JSON 组包、USB 写入和 Wi-Fi 入队总耗时，单位微秒；包含调度抢占和阻塞等待，未发送时为 0 |
| `telem_snapshot_us` / `telem_format_us` / `telem_usb_us` / `telem_wifi_us` | 最近一帧的快照读取 / JSON 组包 / USB 写入 / Wi-Fi 入队分段耗时，单位微秒；含抢占和等待。Wi-Fi 只统计提交队列，不代表网络送达耗时 |
| `telem_format_errors` | 本次启动累计组包失败次数，失败帧不发送；通常应为 0 |
| `telem_skipped` | 本次启动累计丢弃的遥测任务通知数；32 位计数，非 USB 丢包数，也不包含网络队列主动跳过的帧 |
| `imu_ready` / `imu_init` | 初始化是否成功 / `esp_err_to_name` 错误名称 |
| `acc_id` / `gyro_id` | 初始化缓存的芯片 ID，正常十进制 30 / 15 |
| `acc_spi` / `gyro_spi` | 初始化期间 ID 读取的 SPI 返回状态；未执行读取为 `ESP_ERR_INVALID_STATE` |

SPI 返回 `ESP_OK` 仅代表主控传输成功，不能代替正确芯片 ID；修复接线后仍需重新上电初始化。

### SPI 电气探测 `imu_probe`

发送 `{"cmd":"imu_probe"}`，回复 `{"t":"ok","cmd":"imu_probe","probe":{...}}`。仅允许 IMU 初始化失败时执行，避免与正常采样共用 SPI；正常 IMU 返回 `ESP_ERR_INVALID_STATE`。探测仅发送 CHIP_ID 读操作，不写配置、不复位传感器、不改变控制参数；它会临时改变 SPI 模式/速率和 MISO 弱上/下拉，结束时恢复原有 MISO 偏置、释放两个片选，后续常规事务使用正常速率。

`probe` 包含：

- `miso_up` / `miso_down`：两个片选均为高时，MISO 在弱上拉/弱下拉下的电平；测试出错时为 −1。
- `miso_err`：电平测试返回码，0=ESP_OK；MISO 输入未启用或 GPIO 软件控制输出使能时返回 ESP_ERR_INVALID_STATE（259）。非零时不能用 −1 判断线路电平。
- `miso_input` / `miso_oe` / `miso_periph`：GPIO6 的输入使能、OE 寄存器、输出使能是否受外设控制。SPI 驱动为双线模式预留输出路径时，OE 寄存器可能为 1，但实际输出使能受 SPI 外设控制；不能仅凭 OE 寄存器判为普通输出。旧版探测会误拒绝这种正常配置，新版已修正。
- `cases`：Mode0/Mode3 各测试 8 MHz、1 MHz、250 kHz，共六组。每组 `mode`、`hz`、`acc` 三个原始接收字节、`gyro` 两个字节、`ae`/`ge` SPI 返回码。所有 ID 读取临时启用 MISO 弱上拉；Acc 第 3 字节正常为 30，Gyro 第 2 字节正常为 15。

空闲电平 1/0 只说明未被强驱动，不能证明连通；启用弱上拉后的 ID 全为 255，可能来自无人驱动、片选/供电问题或断路；空闲上拉仍为 0，则应排查 MISO 被拉低。探测结果不修改初始化诊断及 `imu_ok`，不能当作传感器已恢复。

### IMU 滤波开关 `imu_filter`

发送 `{"cmd":"imu_filter","v":1}` 开启、`v:0` 关闭；只接受 0/1。回复为 `{"t":"ok","cmd":"imu_filter","live":{...}}`，`live.imu_kf_en` 表示确认后的使能状态；`get` 同样返回此字段。默认开启，`save` 可保存至 NVS。该命令不改电机模式，不重置 PID／电流环；开关不使用通用 `set`。关闭、无效样本或超时数据导致滤波状态失效，重新开启后从新样本初始化。

### 遥测 `t=telem` 字段

正常数值保留现有小数精度。异常非有限浮点值输出 JSON `null`，二次开发时须检查有效性；异常大有限值可能采用科学计数法。组包超出缓冲区时整帧不发送，并增加 `diag.telem_format_errors`。

| 字段 | 说明 |
|------|------|
| `mode` | 当前 `ctrl_mode` |
| `imu_ok` | 1=IMU 本帧有效；0=不可用，勿把 acc/gyro 的零值当成实测 |
| `imu_acc_id` / `imu_gyro_id` | 初始化读到的芯片 ID（十进制）；正常为 30 / 15，即 0x1E / 0x0F |
| `acc` / `gyro` | 原始加速度 [X,Y,Z] g / 原始角速度 [X,Y,Z] °/s，保留旧协议含义 |
| `imu_kf_en` / `imu_kf` | 卡尔曼使能 / 本帧滤波结果有效标志（0/1） |
| `acc_f` / `gyro_f` | 同帧卡尔曼加速度 g / 角速度 °/s；仅 `imu_ok=1`、`imu_kf_en=1`、`imu_kf=1` 时可用，否则数组是无效占位值 |
| `quat` | 板端融合的单位四元数 `[w,x,y,z]`，右手系，传感器坐标→世界坐标（世界 +Z 向上）；航向以板端姿态初始化时为零，只是相对航向 |
| `quat_ok` / `quat_kf` | 姿态有效／本次融合使用有效卡尔曼输入；仅 `imu_ok=1` 且 `quat_ok=1` 时使用 quat，无效数组为 `[1,0,0,0]` 占位；关闭卡尔曼后仍可用原始六轴融合 |
| `theta` / `omega` | rad / rad/s |
| `duty` | 实际占空比 |
| `tau` | 阻抗指令（模式 1） |
| `pid_err` | 位置误差（模式 2） |
| `i_meas` | IPROPI 有符号电流 A |
| `i_des` / `tau_des` / `tau_meas` | 力矩环目标/实测 |
| `i_hat` / `tau_m_hat` / `tau_ext_hat` | DOB（优先用实测电流） |

标定见 [`docs/torque_estimation.md`](../docs/torque_estimation.md)；阻容见 [`hardware/drv8876_bom.md`](../hardware/drv8876_bom.md)。
