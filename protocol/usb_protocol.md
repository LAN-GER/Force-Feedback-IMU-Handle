# USB JSON 行协议

设备通过原生 USB-Serial/JTAG 收发。主机发送一行 JSON（`\n` 结尾），设备应答以 `>>` 前缀：

```text
>>{"t":"ok","cmd":"get",...}
```

## 命令

| cmd | 说明 |
|-----|------|
| `get` | 读当前参数 |
| `set` | 改参数（可只含部分字段） |
| `save` | 写入 NVS |
| `zero` | 编码器清零 |
| `sub` / `unsub` | 订阅 / 取消遥测流 |
| `duty` | 开环占空比测试，同时关闭闭环 |
| `heater` | 设置 IMU 加热占空比：`{"cmd":"heater","v":0.4}` |

### set 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `k` | float | 刚度 |
| `b` | float | 阻尼 |
| `theta_des` | float | 目标角 rad |
| `duty_max` | float | 占空比上限 0~1 |
| `theta_min` / `theta_max` | float | 软限位 rad |
| `enc_cpr` | int | 输出轴每转计数 |
| `enc_dir` | 0/1 | 方向 |
| `ff_enable` | 0/1 | 闭环使能 |
| `telem_hz` | 1~200 | 遥测频率 |
| `heater_duty` | float | IMU 加热 0~1（YJL3400A + 100Ω） |

示例：

```json
{"cmd":"set","k":1.2,"b":0.08,"theta_des":0.5,"ff_enable":1}
{"cmd":"heater","v":0.35}
{"cmd":"duty","v":0.25}
{"cmd":"sub"}
```

### 遥测 `t=telem`

| 字段 | 说明 |
|------|------|
| `acc` | [ax,ay,az] g |
| `gyro` | [gx,gy,gz] °/s |
| `theta` / `omega` | rad / rad/s |
| `counts` | 编码器累计 |
| `tau` / `duty` | 阻抗输出与占空比 |
| `ff_en` / `lim` | 使能 / 限位标志 |
