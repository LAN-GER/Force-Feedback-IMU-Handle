# 快速上手

## 环境

- ESP-IDF **6.x**（与 CAN_Magnetic_Encoder 一致，当前建议 v6.0.2）
- 目标芯片：`esp32c3`

## 编译烧录

```powershell
cd E:\SeeedWork\Force-Feedback-IMU-Handle
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

## 期望日志

```text
bsp_spi: SPI Mode3 8000000 Hz CS_ACC=5 CS_GYRO=7
bsp_motor: LEDC 20000 Hz ...
bsp_enc: PCNT quad ENC_A=0 ENC_B=1
bmi088: acc_id=0x1E gyro_id=0x0F
app: 力反馈 IMU 手柄启动
```

## USB 联调

串口发送（每行一条）：

```json
{"cmd":"get"}
{"cmd":"duty","v":0.2}
{"cmd":"zero"}
{"cmd":"set","ff_enable":1,"k":0.8,"theta_des":0}
{"cmd":"sub"}
```

应答行以 `>>` 开头，见 [`protocol/usb_protocol.md`](../protocol/usb_protocol.md)。

## 故障排查

| 现象 | 检查 |
|------|------|
| Acc/Gyro ID 不对 | CS 接线、SPI Mode3、MISO 是否误接 GPIO9 |
| 编码器不动 | A/B 上拉、编码器 VCC=3.3 V、共地 |
| 电机不转 | nSLEEP=高、VM 电源、`duty`/`ff_enable` |
| USB 无 JSON | 确认 USB-Serial/JTAG 口，而非 UART0 |
