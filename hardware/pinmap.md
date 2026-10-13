# ESP32-C3FH4 引脚分配

软件 `software/components/bsp/include/bsp/bsp_pin.h` 必须与本表一致。
BMI088 MISO 使用 GPIO6；GPIO9 是 BOOT strapping，禁止连接 MISO。
本板：**DRV8876**（PH/EN + IPROPI），**无 WS2812**。

## MCU ↔ BMI088（SPI Mode 3）

| ESP32-C3 GPIO | 方向 | BMI088 | 说明 |
|----------------|------|--------|------|
| GPIO8 | 出 | SCK | strapping；串 22 Ω 阻尼 |
| GPIO6 | 入 | SDO1 + SDO2 | Acc/Gyro MISO 并联；串 22 Ω；避开 GPIO9 |
| GPIO10 | 出 | SDI | MOSI；串 22 Ω |
| GPIO5 | 出 | CSB1 | 加速度计片选，空闲高；串 22 Ω |
| GPIO7 | 出 | CSB2 | 陀螺仪片选，空闲高；串 22 Ω |

VDD / VDDIO = 3.3 V。PS 脚接 GND 选 SPI（若模块有 PS）。

### SPI 串联阻尼（推荐）

MCU 与 BMI088 之间，**SCK / MISO / MOSI / CS** 各串 **22 Ω**；按所提供原理图，RN1 连接 CSB2、CSB1、SDO、SDI，R24 连接 SCK。

## MCU ↔ IMU 加热（抗温漂）

| ESP32-C3 GPIO | 方向 | 连接 | 说明 |
|----------------|------|------|------|
| GPIO21 | 出 | YJL3400A Gate | LEDC PWM，1 kHz；高=导通加热 |

按所提供原理图，5 V 经九颗并联 100 Ω 加热电阻连接 Drain；Source — GND。栅极串 100 Ω、对地 10 kΩ。固件控制加热占空比，不读取温度，也不执行温度闭环。

## MCU ↔ DRV8876 + GA12-N20

硬件阻容选型见 [`drv8876_bom.md`](drv8876_bom.md)。**PMODE=GND、IMODE=GND** → PH/EN。

### 1）MCU → DRV8876

| ESP32-C3 GPIO | 方向 | DRV8876 | 说明 |
|----------------|------|---------|------|
| GPIO2 | 出 | **EN/IN1** | LEDC PWM（20 kHz） |
| GPIO3 | 出 | **PH/IN2** | 方向：高=正转，低=反转 |
| GPIO20 | 出 | nSLEEP | 高=唤醒；低=休眠/高阻 |
| GPIO4 | 入 | **IPROPI** | ADC1_CH4；经 \(R_{IPROPI}=2.2\,\mathrm{k}\Omega\) 到 GND |
| — | — | nFAULT | 可选，本板可悬空 |
| 3V3 | — | VREF 分压 / 上拉 | \(R_{REF}=10\,\mathrm{k}/33\,\mathrm{k}\) → ≈2.53 V |
| GND | — | GND / EP | 与电机电源共地；散热焊盘接地 |

软件约定：`duty > 0` → PH=1 + EN=PWM；`duty < 0` → PH=0 + EN=PWM。

### 2）电源 → DRV8876

| 电源 | DRV8876 | 说明 |
|------|---------|------|
| 电机电源正极（**5 V**） | **VM** | 与 3.3 V 分开 |
| 电机电源负极 | **GND** | 与 MCU 共地 |
| — | VM 旁路 | 100 nF + \(C_{Bulk}\)（推荐 100 µF/16 V） |

### 3）DRV8876 → N20

| DRV8876 | GA12-N20（常见线色） |
|---------|----------------------|
| OUT1 | 电机 M+（红） |
| OUT2 | 电机 M−（白） |

### 4）N20 编码器 → MCU

| GA12-N20 | 接到 |
|----------|------|
| 编码器 VCC（黑） | 3.3 V |
| 编码器 GND（蓝） | GND |
| A 相（绿） | GPIO0 |
| B 相（黄） | GPIO1 |

### 5）总连接示意

本机减速比 **298:1**。编码器按 A/B 正交四倍频计数，输出轴一圈粗测约 **8292 个计数**，反算约 6.956 PPR，据此暂按电机轴 **7 PPR** 计算，理论 `enc_cpr = 7 × 4 × 298 = 8344`。7 PPR 待实际电机规格确认。`enc_cpr` 已经包含减速比和四倍频，无需再次乘除；后续可通过 USB 按多圈精确实测结果标定覆盖。

```text
  ESP32-C3
  GPIO2  ---- PWM ----> EN/IN1
  GPIO3  ---- DIR ----> PH/IN2      DRV8876
  GPIO20 ---- --------> nSLEEP
  GPIO4  <--- IPROPI -- IPROPI --[2.2k]-- GND
  GND    -------------- GND/EP
  电机 5V ------------> VM (+100nF + Cbulk)
                 OUT1 ---> 电机红
                 OUT2 ---> 电机白

  GPIO0/1 <--- 编码器 A/B
  GPIO9  ---- 用户键 → GND
  GPIO21 ---- 加热 MOS 栅极
```

## USB / 用户键

| ESP32-C3 GPIO | 连接 |
|----------------|------|
| GPIO18 / GPIO19 | USB D− / D+ |
| GPIO9 | 用户键（与 BOOT 共用）；短按=编码器清零 |

本板 **无 RGB LED**。
