# 硬件说明

## 引脚

当前硬件为 ESP32-C3FH4 + BMI088 + DRV8876 + GA12-N20，使用 PH/EN 控制和 IPROPI 电流采样。以 [`pinmap.md`](pinmap.md) 为准，软件 [`bsp_pin.h`](../software/components/bsp/include/bsp/bsp_pin.h) 必须同步。GPIO4 用于 ADC，本板无 WS2812。

DRV8876 阻容选型见 [`drv8876_bom.md`](drv8876_bom.md)。

## 接线要点

1. **BMI088 SPI**：SCK/MISO/MOSI = GPIO8/6/10；双 CS = GPIO5/7；各串 22 Ω。
2. **DRV8876 PH/EN**：EN=GPIO2，PH=GPIO3，nSLEEP=GPIO20，IPROPI=GPIO4。
3. **VM=5 V**，与 3.3 V 逻辑分开；GND/EP 共地。
4. **N20 编码器**：VCC=3.3 V，A/B→GPIO0/1。
5. **IMU 加热**：GPIO21 → YJL3400A 栅极；按所提供原理图，5 V 经九颗并联 100 Ω 电阻接 MOSFET Drain，Source 接 GND。固件只控制 PWM，不提供温度闭环。
6. **用户键**：GPIO9（BOOT 共用）；本板无 RGB。

## BOM 草案

| 位号 | 型号 | 备注 |
|------|------|------|
| U1 | ESP32-C3FH4 最小系统 | 自研板 |
| U2 | BMI088 | SPI；VDD/VDDIO=3.3 V，PS 接 GND，SDO1/SDO2 共接 MISO |
| U3 | **DRV8876PWPR** | 电流检测 H 桥 |
| — | 见 drv8876_bom.md | \(R_{IPROPI}\)/VREF/CBulk 等 |
| Q1 | YJL3400A | 加热 |
| Rheat | 九颗 100 Ω 并联 | 按所提供原理图，5 V 加热支路 |
| M1 | JGA12-N20-30 + 编码器 | 当前更换为 30:1；6 V 空载标称 500 r/min |

## 编码器与联调

编码器 A/B 双边沿四倍频。当前 500 r/min 电机按所提供参数表为 `JGA12-N20-30`、减速比 30:1，暂按 7 PPR 计算输出轴每转 `7 × 4 × 30 = 840` 个计数。参数表没有编码器 PPR，需实物规格或多圈精测确认；不要再次对 `enc_cpr` 乘四或除以减速比。

固件内置默认仍为原 298:1 电机的 8344，新电机需通过网页设置 `enc_cpr=840` 并保存 NVS；步骤见 [电机更换说明](../README.md#电机更换与编码器参数)。500 r/min 是 6 V 空载标称值，本板 VM=5 V 时实际转速需测量。

BMI088 芯片 ID 应为 Acc `0x1E`、Gyro `0x0F`。ID 不匹配时检查供电、双片选、MISO 共线及焊接；本次板子的 IMU 故障已通过修复虚焊解决。SPI 主控返回 ESP_OK 不能证明传感器响应正确。

烧录和网页连接见 [快速上手](../docs/getting_started.md)，实板检查见 [Bring-up 清单](../tests/bringup_checklist.md)。电流、力矩参数和响应需要实测标定，不能仅由 BOM 值确定。
