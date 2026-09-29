# 硬件说明

## 引脚

以 [`pinmap.md`](pinmap.md) 为准，软件 `bsp_pin.h` 必须同步。

## 接线要点

1. **BMI088 SPI**：SCK/MISO/MOSI 与 CAN 磁编工程相同（8/6/10）；双 CS 接 GPIO5/7。
2. **SPI 阻尼**：SCK/MISO/MOSI/CS 各串 **22 Ω**；推荐 **4×22 Ω 独立型排阻**（勿用共端型）。双 CS 时 CSB2 再加一颗 22 Ω。见 [`pinmap.md`](pinmap.md)。
3. **SDO1 与 SDO2** 并联到 MISO（经上述串阻之后）。
4. **DRV8833 ↔ GA12-N20**：电机红/白接 AOUT1/AOUT2；编码器黑→3.3 V、蓝→GND、绿/黄→GPIO0/1；详见 [`pinmap.md`](pinmap.md)「MCU ↔ DRV8833 + GA12-N20」。
5. **DRV8833 电源**：逻辑 3.3 V；VM=电机额定电压（常见 6 V）；GND 与 MCU 共地。
6. **N20 编码器**：VCC=3.3 V；A/B 无板上拉时各外接 4.7 kΩ 到 3.3 V。
7. **IMU 加热**：100 Ω + YJL3400A，栅极接 GPIO21；见 [`pinmap.md`](pinmap.md)。
8. **用户键**：GPIO9（与 BOOT 共用），按下接 GND；上电勿按住。
9. **GPIO9** 禁止接任何传感器 MISO。

## BOM 草案

| 位号 | 型号 | 备注 |
|------|------|------|
| U1 | ESP32-C3FH4 最小系统 | 自研板 |
| U2 | BMI088 模块 | SPI |
| U3 | DRV8833 | 或兼容双 H 桥 |
| RN1 | 4×22 Ω 独立型排阻 | SPI：SCK/MISO/MOSI/CSB1 |
| Rcs2 | 22 Ω | CSB2（GPIO7），或并入第二颗排阻 |
| Q1 | YJL3400A | N-MOS 低侧开关，加热 |
| Rheat | 100 Ω | 贴片，贴 IMU 附近 |
| Rg / Rpd | 100 Ω / 10 kΩ | 栅极串阻 / 下拉 |
| M1 | GA12-N20 + 霍尔编码器 | 减速比按实物改 CPR |
| D1 | WS2812 | GPIO4 |
