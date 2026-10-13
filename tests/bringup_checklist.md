# Bring-up Checklist

## 上电前

- [ ] 3.3 V / GND / 电机电源（**VM=5 V**）极性正确，与逻辑共地
- [ ] BMI088：SCK=8、MISO=6、MOSI=10、CS_ACC=5、CS_GYRO=7
- [ ] GPIO9 未接任何传感器信号
- [ ] DRV8876：EN=2、PH=3、nSLEEP=20、IPROPI=4；\(R_{IPROPI}=2.2\,\mathrm{k}\Omega\)；VM 已供电
- [ ] 编码器：A=0、B=1、VCC=3.3 V；必要时外接上拉

## 固件

- [ ] `idf.py build` 成功
- [ ] 日志出现 `acc_id=0x1E gyro_id=0x0F`
- [ ] 日志出现 `DRV8876 PH/EN` 与 `IPROPI ADC1_CH4`

## IMU

- [ ] `{"cmd":"sub"}` 后 `acc`/`gyro` 随姿态变化
- [ ] 静止时 az ≈ ±1 g（安装方向相关）
- [ ] 遥测 `quat_ok=1`，`quat` 长度 4、顺序 `[w,x,y,z]`、模长约 1；绕三个轴转动时网页方块直接跟随板端姿态
- [ ] 卡尔曼开启并就绪时 `quat_kf=1`；关闭后 `quat_kf=0`，有效原始六轴仍可生成姿态；网页姿态归零不修改板端四元数或编码器
- [ ] `{"cmd":"heater","v":0.4}` 后加热电阻微温；`v:0` 关闭
- [ ] 运行中短按 GPIO9，日志 `encoder zero`，`theta` 回零（上电勿按住）

## 电流 / 力矩估计

- [ ] `{"cmd":"duty","v":0.2}` + `sub`，`i_meas` 明显非零，与电流表大致一致
- [ ] 可见 `tau_m_hat` / `tau_ext_hat`；手拧轴时 `tau_ext_hat` 有响应
- [ ] 标定流程见 [`docs/torque_estimation.md`](../docs/torque_estimation.md)

## 电机开环

- [ ] `{"cmd":"duty","v":0.2}` 正转；`-0.2` 反转；`0` 停
- [ ] 转动时 `counts` / `theta` 变化
- [ ] 使用默认标定时，`get` 返回 `enc_cpr=8344`（7 PPR × 4 × 298；7 PPR 由粗测推断，待实物规格确认）
- [ ] 停机、清零后，输出轴完整一圈约为 8344 个计数、`theta≈2π`，网页约 360°；用多圈累计计数校验或进一步标定
- [ ] 未带版本的旧 NVS 600、`enc_ver<2` 的旧默认 3576、版本 2 的粗测默认 8292 首次启动迁移为 8344；其他自定义值保留，包括版本 1 的 600/8292、版本 2 的 3576、版本 3 的 8292；保存并重启后数值不变

## 闭环

- [ ] `{"cmd":"zero"}` 后 `theta≈0`
- [ ] 位置 PID：`{"cmd":"set","ctrl_mode":2,"theta_des":0.3,"pid_kp":2}`
- [ ] 力矩环：`{"cmd":"set","ctrl_mode":3,"tau_des":0.01,"kt":0.02}`（小目标）
- [ ] 阻抗：`{"cmd":"set","ctrl_mode":1,"k":0.5,"theta_des":0}` 有回中力
- [ ] `{"cmd":"set","ctrl_mode":0}` 或 `duty:0` 释放

## 安全

- [ ] 初调 `duty_max≤0.6`，`i_max` 低于硬件限流（约 1.15 A）
- [ ] 异常立即 `ctrl_mode=0` 或断电
- [ ] 勿长时间满占空比堵转

## Wi-Fi 补充检查

- [ ] 无 Wi-Fi 配置首次启动，USB `wifi_get` 为 ready=1、enabled=0、running=0；保存 WPA2 2.4 GHz 网络不自动开启，USB 打开开关后等待 busy=0、error=""、connected=1 和有效 IP。
- [ ] 开启时断电重启后自动连接；关闭时重启不启动无线驱动与 HTTP。USB 读取的接入密钥保持一致；不在通信日志中暴露 password/token。
- [ ] USB 关闭开关，确认 enabled=0、busy=0、error=""、running=0；HTTP 无法连接，空闲堆内存恢复，SSID/password/token 保留；再次开启无需重填密码，多次开关无持续内存损失。
- [ ] 网页停机并断开 USB 后选择 Wi-Fi，IMU、滤波、quat、编码器和电机状态更新；CSV source 为 wifi。
- [ ] 用小 PWM 验证无线控制和停止，再按原清单验证三个闭环；确认没有改变角度、限幅或力矩单位。
- [ ] 错误接入密钥和第二个 Wi-Fi 客户端被拒绝；不能通过 Wi-Fi 修改开关、修改或清除网络配置。
- [ ] 暂停订阅后断开网络，连接最终被清理；重连只读取/订阅，不自动重发启动命令。
- [ ] 网络中断时网页标记历史数据，USB 仍可停机和读取状态；不要把断线当作板端自动停机。
- [ ] USB `wifi_forget` 后关闭无线连接，下次上电不自动连接，接入密钥更换；电机参数不受影响。

使用步骤与协议见 [Wi-Fi 说明](../docs/wifi.md)。

## 遥测补充检查

- [ ] USB 读取 `telem_hz=100`，连续接收不少于 10 秒并记录实际帧率；测试 10/25/50/100 Hz 切换、暂停/恢复订阅、保存并重启；正常负载下检查每周期发送，不再叠加一段发送后的周期延时。Wi-Fi 同步验证，拥堵时不出现连续补发。

- [ ] 遥测组包主机测试：在 Windows + MSVC 环境运行 `powershell -File tests/telem_json_test.ps1`，检查随机浮点与 libc 小数输出一致、极端值、完整协议帧和缓冲区边界。
- [ ] USB 100 Hz 实板性能：烧录后订阅至少 10 s，读取诊断确认组包失败为 0；记录快照/组包/USB 写入/Wi-Fi 入队分段耗时、总耗时和跳过通知增量。若仍约 50 帧/s，按耗时最大的分段继续定位；主机测试通过不代表板端达到 100 Hz。

本板联调反馈（2026-10-12）：用户更新后确认 USB 配置 100 Hz 却仅接收约 50 帧/s 的问题已解决，未提供新的具体速率和耗时。此反馈不替代上述全部检查；Wi-Fi、重启保存及其他频率须分别记录，见 [验证记录](../docs/firmware.md#遥测验证记录)。
