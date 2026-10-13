# 文档导航

本文档对应当前 DRV8876 + IPROPI 固件和 USB / Wi-Fi 网页调试台。推荐先完成烧录与连接，再校验编码器、电流，最后调试闭环。

| 文档 | 内容 |
|------|------|
| [getting_started.md](getting_started.md) | 环境、烧录、网页串口选择、首次联调与故障排查 |
| [interface_sdk.md](interface_sdk.md) | 二次开发接口、IMU／电机遥测、控制 API、Python SDK 与浏览器接入 |
| [wifi.md](wifi.md) | USB 配网、Wi-Fi 状态、WebSocket 接入密钥、双链路语义与 SDK |
| [firmware.md](firmware.md) | 分层、任务、遥测定时与组包优化、分段诊断、验证记录、IMU／电机及 NVS |
| [torque_estimation.md](torque_estimation.md) | IPROPI 电流、力矩／DOB、参数标定及当前实现限制 |
| [网页与工具说明](../tools/README.md) | 四种电机模式、滤波对比、独立波形、三维方块、CSV、诊断及测试 |
| [硬件引脚表](../hardware/pinmap.md) | BMI088、DRV8876、电流、编码器、加热和 USB 接线 |
| [USB JSON 协议](../protocol/usb_protocol.md) | 命令、参数、诊断与遥测字段 |
| [实板联调清单](../tests/bringup_checklist.md) | 上电、IMU、编码器、电流、闭环、遥测与 Wi-Fi 逐项检查 |

当前更换为 `JGA12-N20-30`，参数表标称 6 V 空载 500 r/min、减速比 30:1；暂按 7 PPR × 4 × 30 设置输出轴每转 840 个计数。固件内置默认仍为原 298:1 电机的 8344，更换后需设置并保存 NVS，见 [电机更换说明](../README.md#电机更换与编码器参数)。7 PPR 沿用原电机粗测推断，参数表没有提供编码器 PPR，仍需确认。板端卡尔曼做六轴降噪，板端姿态融合输出 `quat=[w,x,y,z]`，网页直接消费四元数；相对航向可能漂移。力矩数值依赖 Kt 和机械参数标定。

遥测默认 50 Hz，网页最高可选 100 Hz；修改频率后需要「保存参数至 NVS」确认才能在重启后保留。USB 100 Hz 接收异常的本板反馈见 [验证记录](firmware.md#遥测验证记录)，设置步骤见 [快速上手](getting_started.md#遥测频率与保存)。
