# 带电流采样的电机力矩估计与参数辨识

本文面向本工程硬件：**GA12-N20 + DRV8876（IPROPI）+ ESP32-C3 ADC**。  
目标：用实测电流得到电机力矩 \(\tau_m\)，再用机械 DOB 估计外力矩 \(\tau_{ext}\)，并说明如何标定 \(K_t,J,B\) 等。

当前采样流程使用 IPROPI。DOB 模块支持电枢方程估计，但调用端固定传入 `have_i=true`，ADC 故障不会自动切换到回退路径。本文中的力矩值依赖标定，更新周期不代表已验证的控制带宽或测量精度。

---

## 1. 硬件能提供什么

| 量 | 来源 | 说明 |
|----|------|------|
| 位置 \(\theta\) | 编码器正交计数 | 输出轴 rad，`n20_encoder` |
| 速度 \(\omega\) | \(\theta\) 差分 + 低通 | 同上 |
| 加速度 \(\dot{\omega}\) | \(\omega\) 再差分 | 噪声大，DOB 内低通 |
| 电流幅值 \(\lvert I\rvert\) | DRV8876 **IPROPI** → GPIO4 ADC | \(R_{IPROPI}=2.2\,\mathrm{k}\Omega\) |
| 有符号电流 \(I\) | 幅值 × \(\mathrm{sign}(duty)\) | 固件合成 |
| 电压指令 \(u\) | \(u \approx d\cdot V_m\) | 电枢估计模型，\(V_m\) 默认 **5 V** |

输出轴默认 `enc_cpr=8344`，按 7 PPR × 4 × 298 计算；7 PPR 由粗测推断。开始辨识前先用多圈实测核对编码器计数及方向，详见 [固件换算说明](firmware.md#编码器输出轴换算)。

电流换算（手册 \(A_{IPROPI}=1000\,\mu\mathrm{A/A}\)）：

\[
V_{IPROPI} = I_{OUT}\times 0.001\times R_{IPROPI}
\quad\Rightarrow\quad
\hat{I}_{mag} = \frac{V_{ADC}}{0.001\times 2200} = \frac{V_{ADC}}{2.2}
\]

阻容与量程见 [`hardware/drv8876_bom.md`](../hardware/drv8876_bom.md)。

---

## 2. 方法 A：实测电流 → 电机力矩（主路径）

### 2.1 原理

\[
\hat{\tau}_m = K_t \cdot I_{meas}
\]

- \(I_{meas}\)：IPROPI 有符号电流（A）
- \(K_t\)：输出轴当量力矩系数，需标定。力矩环 `tau_meas` 使用 `kt`，DOB `tau_m_hat` 使用独立的 `dob_kt`，两者应同步核对；默认均为 0.02，不能当作已标定值。

### 2.2 响应速度

| 项目 | 量级 |
|------|------|
| 电流采样 / 控制 | 名义 1 kHz；每次采样在控制前后分别读取，控制前读数供 PI，控制后读数供遥测／DOB |
| USB 上报 | 默认 50 Hz，上传最新快照；不逐帧上传全部 1 kHz 样本 |
| 响应 / 带宽 | 受 ADC、PWM、软件调度、滤波和机械系统影响，尚未给出实板带宽测量 |
| 力矩环（`ctrl_mode=3`） | 电流 PI，固定积分步长 0.001s；具备电流和 PWM 限幅，不执行角度软限位 |

### 2.3 优点 / 注意

- 不依赖 \(R,K_e\) 温漂；堵转时电流仍可读  
- IPROPI 为幅值输出，符号靠方向合成；零点需空载校准（固件启动时采偏置）  
- 小电流区有固定偏置（手册说明），软件已做零点；仍建议用电流表核对比例  
- \(K_t\) 含减速比/效率时，才是“输出轴 N·m”

### 2.4 回退：电枢方程

DOB 的 `have_i=false` 分支采用：

\[
\hat{I} = \frac{d\cdot V_m - K_e\,\omega}{R},\quad
\hat{\tau}_m = K_t\,\hat{I}
\]

参数为 `dob_r` / `dob_ke` / `dob_vm`，结果受参数误差影响。当前 `imu_ff.c` 调用固定使用 `have_i=true`，而 ADC 读取失败返回 0；此分支不会自动启用。遥测也没有电流有效性字段，读数为 0 不能证明电流采样正常。

---

## 3. 方法 B：机械 DOB 估外力矩

### 3.1 原理

\[
J\dot{\omega} = \tau_m - \tau_{ext} - B\omega - \tau_f(\omega)
\]

\[
\hat{\tau}_{ext}
\approx
\hat{\tau}_m - J\dot{\omega} - B\omega - \hat{\tau}_f
\quad\xrightarrow{\;G_f(s)\;}
\hat{\tau}_{ext}^{filt}
\]

本工程 \(\hat{\tau}_m\) 来自 IPROPI 路径。`dob_fc_hz` 默认 30 Hz，分别用于速度差分得到的角加速度和外力矩的一阶低通；它是滤波参数，不等于整机实测控制带宽。

力反馈更常关心 \(\hat{\tau}_{ext}\)（手在推多大），力矩闭环更关心 \(\hat{\tau}_m\) / \(I\)。

### 3.2 响应与调参

| 调整 | 预期影响 |
|------|----------|
| 减小 `dob_fc_hz` | 加强平滑，增加跟踪滞后 |
| 增大 `dob_fc_hz` | 减少平滑，可能放大差分噪声 |
| 修改 `dob_j` / `dob_b_visc` / `dob_tau_c` | 改变惯性、粘滞及库仑摩擦补偿，需实际辨识 |

实际效果通过低速运动、已知负载和网页曲线／CSV 评估，尚无已验证的 DOB 带宽数值。

---

## 4. 与控制模式的关系

```text
IPROPI I_meas ──┬──► 力矩环 ctrl_mode=3：i_des=τ_des/Kt，PI → duty
               └──► DOB：τ̂_m = dob_kt·I
编码器 θ,ω ────┬──► 阻抗 ctrl_mode=1 / 位置 PID ctrl_mode=2
               └──► DOB：惯性、粘滞及摩擦补偿 → τ̂_ext
手动 duty ─────────► 开环 ctrl_mode=0
```

DOB 是并行估计支路，不反馈到当前控制器；力矩模式以电流 PI 跟踪由 \(\tau_{des}/K_t\) 得到并限幅的目标电流。阻抗模式计算结果直接作为占空比，遥测 `tau` 不能当作实测或标定力矩。

---

## 5. 离线参数辨识

用 USB `duty` 开环 + `sub` 遥测，或网页的小占空比 PWM 与 CSV 导出，观察 `i_meas` / `tau_m_hat` / `theta`。注意堵转发热；按 BOM 的 VREF 与电阻计算，硬件限流约 **1.15 A**，实际值需核对所装器件。

| 实验 | 操作 | 得到 |
|------|------|------|
| ADC 比例核对 | 电流表串电机，对比 `i_meas` | 确认 \(R_{IPROPI}\)/零点 |
| 准静态加载 | 已知砝码/弹簧力臂，记 \(I\) 与负载力矩 | \(K_t \approx \tau / I\) |
| 恒速空载多组 | 多组 \(d\)，稳态 \(\omega\) | \(K_e\)、粘滞（电枢回退用） |
| 阶跃加速 | 阶跃 \(d\)，看 \(\omega(t)\) | \(J\)（配合已知 \(\tau_m=K_t I\)） |
| 低速往复 | 启动门槛 | 库仑摩擦 \(\tau_f\)（`dob_tau_c`） |

注意：

- 辨识轴（电机轴 / 输出轴）与 \(K_t,J,B\) 单位必须一致  
- \(V_m=5\,\mathrm{V}\) 要稳；`enc_cpr` / `enc_dir` 先校准  
- 标定后在网页填写 `kt`、`dob_kt`、`dob_j` 等参数，发送修改，再 `save`；修改控制参数会重新应用控制器，宜停机时操作

---

## 6. 对比（本板）

| 路径 | 估什么 | 响应 | 精度 | 本工程 |
|------|--------|------|------|--------|
| IPROPI → \(K_t I\) | \(\hat{\tau}_m\) | 名义 1 kHz 更新 | 依赖 ADC 比例、符号和 Kt 标定 | 当前路径 |
| DOB | \(\hat{\tau}_{ext}\) | 名义 1 kHz 更新，默认 30 Hz 低通 | 依赖机械参数及差分噪声 | 已实现，带宽未实测 |
| 电枢方程 | \(\hat{I},\hat{\tau}_m\) | DOB 模块中的备选分支 | 依赖 R、Ke、Vm、Kt | 当前调用不自动启用 |
| 力矩环 `ctrl_mode=3` | 跟踪 \(\tau_{des}\) | 电流 PI | 依赖电流与 Kt 标定 | 已实现，带宽未实测 |

---

## 7. 建议联调顺序

1. **核对编码器与电流**：校验输出轴计数和方向，再用 `duty` + 电流表确认 `i_meas`  
2. **标定 \(K_t\)**：准静态加载  
3. **开 DOB**：调 `dob_j` / `dob_b_visc` / `dob_fc_hz`，看手推时 `tau_ext_hat`  
4. **力矩环**：`ctrl_mode=3`，小 `tau_des`，调 `tq_kp`/`tq_ki`  
5. **位置 / 阻抗**：再开 PID 或阻抗模式做手感

USB 字段见 [`protocol/usb_protocol.md`](../protocol/usb_protocol.md)。

---

## 8. 安全

- 初调 `duty_max≤0.6`、`i_max` 低于硬件 \(I_{TRIP}\)  
- 堵转时依赖 DRV8876 限流；仍勿长时间满占空比堵转  
- IPROPI 开路会导致电流读数为 0，力矩环失控风险 → 上电检查 `bsp_i` 日志与 `i_meas`

---

## 相关路径

| 路径 | 说明 |
|------|------|
| `software/components/bsp/src/bsp_adc_current.c` | IPROPI ADC |
| `software/components/drv/src/drv8876.c` | 驱动 + 有符号电流 |
| `software/components/svc/src/torque_dob.c` | \(\tau_m\) / DOB |
| `software/components/svc/src/motor_torque.c` | 力矩电流环 |
| `hardware/drv8876_bom.md` | \(R_{IPROPI}\)、\(V_{REF}\) |
| `hardware/pinmap.md` | GPIO4 ← IPROPI |
