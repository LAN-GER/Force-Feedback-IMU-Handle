# 固件架构

## 分层

```text
APP (main)     app_main / app_sampler / app_usb / app_telem_json / app_wifi
    ↓
SVC            params / imu_ff / imu_kalman / imu_attitude / ff_impedance / motor_pid / motor_torque / torque_dob
    ↓
DRV            bmi088 / drv8876 / n20_encoder
    ↓
BSP            spi / motor_pwm(PH/EN) / adc_current / encoder / usb / gpio / heater
```

设备调用按 APP → SVC → DRV → BSP 组织；当前 `imu_ff` 通过 APP 维护的全局 `g_params` 读取运行参数。

## 任务

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| `app_sam` | 7 | 名义 1 kHz | IMU + 卡尔曼 + 编码器 + 电流 + 控制环 + DOB → 遥测快照 |
| `app_telem` | 5 | 默认 50 Hz | 按 `telem_hz` 输出 USB 遥测并提交 Wi-Fi 遥测 |
| `app_idle` | 4 | 10 ms | USB 收包、用户键 |
| `app_wifi` | 3 | 配网通知 / 开启时约 2 秒重试 | STA 配置、开关及重连；关闭时阻塞 |
| `httpd` | 5 | 网络事件 / 发包队列 | WebSocket 握手、命令及遥测发送；Wi-Fi 关闭时删除 |

1 ms 的 `ESP_TIMER_TASK` 回调通过任务通知唤醒采样任务；积压通知会合并，实际采样率受调度影响。位置 PID、电流 PI 和 DOB 当前使用固定 `dt=0.001s`，编码器测速、IMU 滤波及板端姿态融合使用实际时间。`seq` 是采样快照更新序号，网页只用它辅助检测重启，不用它积分姿态。

## 遥测频率与性能

遥测由独立 `esp_timer` 定期通知 `app_telem`，周期为 `1000000 / telem_hz` 微秒（100 Hz 对应 10 ms），回调不组包或发包。启动使用 NVS 恢复值，`set.telem_hz` 重设定时器周期；不再在每次组包/发送完成后额外延时一个周期，避免处理耗时叠加使频率降低。无订阅时不唤醒遥测任务；一次发送期间积累的通知丢弃，等下一个时间槽发送，避免追赶积压帧。RTOS tick 保持 100 Hz，遥测周期不受 tick 整数换算限制；单核负载、USB 写入阻塞及网络拥堵仍可能使实际接收频率低于配置，200 Hz 只是参数上限。

遥测 JSON 由 `main/app_telem_json.c` 使用有边界检查的定点十进制组包：对 binary32 的尾数做整数缩放和舍入，避免逐字段调用浮点 `snprintf`。保留字段、单位和原有小数位；非有限值输出 JSON `null`，异常大有限值使用科学计数法。组包溢出不发送残缺帧，诊断失败计数递增。

网页「读取诊断」显示最近及最高总耗时、快照/组包/USB 写入/Wi-Fi 入队分段耗时、组包失败和累计丢弃通知数。100 Hz 的周期为 10 ms；最近一次工作耗时超过 10 ms 且丢弃通知持续增长时，说明发送任务可能赶不上配置周期。耗时包含抢占和等待，不只计算 CPU 组包时间，详见 [diag 字段](../protocol/usb_protocol.md#只读诊断-diag)。定时器接口见 [ESP Timer 官方说明](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32c3/api-reference/system/esp_timer.html)。

### 遥测验证记录

2026-10-12，本板 USB 联调曾出现 `get.live.telem_hz=100`、网页实际约 50 帧/s。旧版诊断最近一次总耗时为 13.07 ms、最高 19.02 ms，超过 100 Hz 的 10 ms 周期；周期内通知积压后被丢弃。该记录能确认任务赶不上周期，当时尚无分段耗时，不能据此单独认定 USB 写入或浮点组包占据全部时间。

当前版本采用独立周期定时通知及轻量遥测组包，并增加四段耗时诊断。用户更新后反馈该问题已解决；没有提供更新后的具体帧率、分段耗时或测量时长。本次确认范围为该板 USB 连接，Wi-Fi、USB / Wi-Fi 同时订阅以及 200 Hz 的性能需另外验证。

该版本构建已通过 ESP-IDF v6.0.2 编译；主机组包测试通过 70 万次随机小数输出对照、IEEE 边界值、完整协议帧、缓冲区容量和标志位检查；网页 31 项检查通过，包括分段诊断和按实际到达帧数显示速率。主机检查使用模拟数据，实板反馈另列如上。

Windows 安装完整 MSVC x86 工具集后，可在仓库根目录复现组包主机测试：

```powershell
powershell -File tests/telem_json_test.ps1
```

板端更新后重新订阅并观察接收速率，读取 `diag` 记录总耗时、分段耗时、组包失败及跳过通知增量；`telem_work_max_us` 和累计计数保留本次启动的历史，不能仅凭一次最高值判断当前持续性能。详细操作见 [遥测设置](getting_started.md#遥测频率与保存) 和 [实板检查](../tests/bringup_checklist.md#遥测补充检查)。

## 启动、存档与诊断

`app_main` 读取 NVS，初始化 BSP、IMU 和驱动，再应用控制参数并启动 USB 与采样任务。IMU 初始化成功时会恢复保存的闭环模式、目标及加热占空比；开环 duty 初始为 0，不存档。

IMU 初始化失败时，本次运行将模式设为 OFF、关闭加热，仍启动 USB 和编码器诊断，不覆盖原存档。运行期间 IMU 读数无效只标记遥测和滤波无效，当前代码不因单帧读取失败自动停止电机控制。

`get` 返回当前运行参数 `live`，`save` 将当前参数复制到存档快照并写入 NVS；协议不单独返回存档快照。编码器升级迁移另见下节。`diag` 返回缓存初始化错误、芯片 ID、复位原因、运行时间及固件标识，不重新初始化传感器。`imu_probe` 仅在 IMU 初始化未成功时测试 SPI，不修改控制参数；修复焊接或供电后复位重新初始化。

USB / Wi-Fi 共用命令处理器，用互斥锁防止回复缓冲和控制命令并发覆盖；回复仅发送给请求所在链路，订阅状态独立。Wi-Fi 配置经 USB 写入独立 NVS 命名空间，网络初始化失败时 USB 和采样继续运行。网络遥测最多排队一帧，拥堵时丢弃新帧。完整接口与连接限制见 [Wi-Fi 说明](wifi.md)。

`wifi_enable.v=0/1` 仅 USB 可切换，开关持久化；首次默认关闭，保存 SSID/password 保留原开关值。关闭时停止 HTTP 和无线、释放驱动和网口；再次开启重建资源。配置回复是异步入队确认，`wifi_get` 的 `busy=0,error="",running=0` 才确认关闭完成。关闭仍保留阻塞的管理任务和少量管理对象；首次开启后共享 lwIP 与事件循环保留。

## 电机驱动（DRV8876 PH/EN）

- EN=GPIO2 PWM，PH=GPIO3 方向，nSLEEP=GPIO20
- IPROPI=GPIO4 ADC，\(I=V/(0.001\times R)\)，\(R=2.2\,\mathrm{k}\Omega\)

## IMU 卡尔曼降噪

`svc/imu_kalman` 为加速度 XYZ（g）和角速度 XYZ（°/s）分别维护一维随机游走卡尔曼滤波器，不做姿态融合或陀螺零偏估计。`acc` / `gyro` 保留驱动原始物理值，滤波值单独放入遥测 `acc_f` / `gyro_f`；电机仍使用编码器反馈。

预测 `P⁻ = P + Q·dt`，更新 `K = P⁻/(P⁻+R)`、`x = x + K·(z-x)`、`P = R·P⁻/(P⁻+R)`。首个有效样本直接初始化 x，P=R；加速度默认 Q=0.04 g²/s、R=0.0025 g²，角速度 Q=4 (°/s)²/s、R=1 (°/s)²。这是初始降噪配置，可在 `imu_kalman.h` 调整；增加 Q 或减小 R 会加快跟踪，也会保留更多噪声。滤波有响应滞后，需用实板前后对比评估。

采样任务为 1 kHz，加速度 ODR=100 Hz、陀螺 ODR=1 kHz。加速度仅在读取数据前的 ACC_STATUS bit7=1 时更新，重复读取保持估计和协方差；陀螺按采样时间更新，间隔小于 500µs 的重复轮询跳过，Q 使用实际时间间隔。新数据标志定义见 [Bosch BMI088 数据手册 §5.3.3](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。滤波状态由采样任务独占维护。

默认开启，`imu_filter` 命令只改变滤波使能，不重新配置电机或重置电机积分器。关闭、读取失败或非有限输入清空滤波状态；数据超过 250ms 未更新或时间倒退时重新初始化。`imu_kf=0` 的滤波数组为无效占位值，网页数值和曲线不显示、CSV 留空。使用 `save` 保存开启状态至 NVS。

实际 C 模块可在主机编译验证，例如已安装 GCC 时：

```sh
gcc -std=c11 -Wall -Wextra -Werror -Isoftware/components/svc/include tests/imu_kalman_test.c software/components/svc/src/imu_kalman.c -lm -o build/imu_kalman_test
./build/imu_kalman_test
```

## 板端四元数姿态

`svc/imu_attitude` 在采样任务中使用陀螺四元数积分和比例重力校正，优先输入有效卡尔曼六轴，滤波关闭或尚未就绪时使用原始六轴。它是姿态融合模块，独立于六轴卡尔曼降噪。输出 `quat=[w,x,y,z]`，为右手系、传感器坐标到世界坐标的单位四元数；世界 +Z 对齐重力参考的向上方向，首次有效姿态的航向为 0，无绝对北向参考。

初始横滚／俯仰直接从重力定向，不从零缓慢爬升。加速度模长 0.7～1.3 g 时参与重力校正，增益为 2.5 s⁻¹；陀螺输入从 °/s 转为 rad/s，每个积分子步最多 10 ms，并归一化。少于 500 µs 的重复轮询不重复积分。无效样本暂停；超过 250 ms 间隔或恢复采样时，重新按重力初始化倾斜并保留相对航向，不积分丢失时段。时间倒退则重置姿态。

遥测 `quat_ok=1` 表示可用，`quat_kf=1` 表示本次融合用了有效滤波输入；无效 `quat` 是 `[1,0,0,0]` 占位，不能视为实测水平姿态。初始重力不可用时等待有效样本。航向仍受陀螺零偏影响，剧烈线性加速度可干扰倾斜，当前不估计陀螺零偏、不存储姿态到 NVS。

主机测试针对同一 C 模块验证三轴转动、实际时间间隔、归一化、重力收敛、航向保持及异常输入：

```sh
gcc -std=c11 -Wall -Wextra -Werror -Isoftware/components/svc/include tests/imu_attitude_test.c software/components/svc/src/imu_attitude.c -lm -o build/imu_attitude_test
./build/imu_attitude_test
```

## 控制模式

| mode | 算法 |
|------|------|
| 0 OFF | 开环 duty |
| 1 阻抗 | `u = K(θd−θ) − Bω`，加限位回推后直接限幅为 duty |
| 2 位置 PID | `duty = Kp e + Ki∫e − Kd ω` |
| 3 力矩 | `ides=τdes/Kt`，电流 PI → duty |

阻抗的 `tau` 字段是直接用于占空比的控制量，不能视为已标定 N·m。位置和阻抗模式使用角度软限位；力矩模式仅执行电流及 PWM 限幅。开环 duty 限制在 −1～1，不使用闭环 `duty_max`。`duty=0` 切回 OFF 并清除开环输出；只设置 `ctrl_mode=0` 不清除已有开环目标。

USB `zero` 清零编码器、重置 DOB、位置 PID 和电流环状态；GPIO9 短按只清零编码器并重置 DOB，二者不会修改目标角。

## 编码器输出轴换算

减速比为 298:1，BSP 对 A/B 两相进行四倍频计数。输出轴一圈粗测约 8292 个累计计数，反算电机轴 `8292 / (298 × 4) ≈ 6.956`，据此推断编码器为 7 PPR。默认采用理论输出轴每转计数 `7 × 4 × 298 = 8344`，统一定义在 `bsp_encoder.h`。7 PPR 仍需实际电机规格确认；8344 与粗测值相差约 0.62%。[N20 编码器规格书示例](https://cdn.sparkfun.com/assets/a/3/9/9/9/28633_datasheet_N20_Motor_With_Encoder_and_Cable_pair.pdf)列出了 7 PPR 型号，但不能据此确认本机型号。

`theta = counts × 2π / enc_cpr`；角速度使用同样的每转计数换算并经过低通。`enc_cpr` 已经包含减速比，不再额外除以 298。位置 PID、阻抗反馈及 DOB 使用这个输出轴角度/角速度。

升级时，未带编码器标定版本的 NVS 旧值 600、`enc_ver<2` 的旧默认 3576，以及 `enc_ver=2` 的粗测默认 8292，会迁移为 8344，并单独保存编码器参数。其他自定义正值保留，包括版本 1 的 600、版本 2 的 3576、版本 0/1 的 8292；新版保存带 `enc_ver=3`，之后明确设置的上述值也会保留。迁移写入失败时本次运行仍采用 8344，并在日志中提示，下一次启动重试。

在 7 PPR、298:1 参数正确且无丢计数时，停机并清零后，输出轴完整一圈应产生约 8344 个计数、显示约 360°；可连续测量多圈，用累计计数除以圈数进一步校验或标定 `enc_cpr`，通过 USB 保存后重启仍保留。

## 力矩估计（DOB）

当前采样流程读取 IPROPI 幅值，按 duty 合成符号，计算 `τ̂_m = dob_kt · I_meas`，再估计外力矩。力矩环的 `tau_meas` 使用 `kt`，DOB 使用独立的 `dob_kt`，标定时应同步核对。

DOB 模块支持 `have_i=false` 时使用电枢方程，但当前采样调用固定传入 `true`；ADC 读取失败返回 0，不会自动切换估计路径。电流和力矩未提供独立的有效状态字段，不能将读数为 0 当作采样正常的依据。细节见 [力矩估计](torque_estimation.md)。

## USB 与网页边界

JSON 每行一帧，固件回复带 `>>` 前缀；上层 JSON 容量为 `BSP_USB_JSON_CAPACITY=1024`，接收正文不超过 255 字节。遥测使用最新快照，并非每个 1 kHz 样本都上传。协议见 [USB JSON](../protocol/usb_protocol.md)。

网页分别显示原始、滤波及独立卡尔曼波形。USB 三维方块直接使用板端 `quat`，网页只进行参考四元数变换、欧拉角显示和 CSS 旋转，不重新积分六轴。姿态归零只改变显示参考，与电机编码器清零及板端姿态无关。只有无板演示保留模拟六轴融合，旧固件没有 `quat` 时提示更新。操作、有效性和测试见 [工具说明](../tools/README.md)。
