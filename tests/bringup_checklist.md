# Bring-up Checklist

## 上电前

- [ ] 3.3 V / GND / 电机电源极性正确，电机电源与逻辑共地
- [ ] BMI088：SCK=8、MISO=6、MOSI=10、CS_ACC=5、CS_GYRO=7
- [ ] GPIO9 未接任何信号
- [ ] DRV8833：IN1=2、IN2=3、nSLEEP=20；VM 已供电
- [ ] 编码器：A=0、B=1、VCC=3.3 V；必要时外接上拉

## 固件

- [ ] `idf.py build` 成功
- [ ] 日志出现 `acc_id=0x1E gyro_id=0x0F`
- [ ] WS2812 亮蓝（就绪未使能）或绿（闭环使能）

## IMU

- [ ] `{"cmd":"sub"}` 后 `acc`/`gyro` 数值随姿态变化
- [ ] 静止时 az ≈ ±1 g（安装方向相关）
- [ ] `{"cmd":"heater","v":0.4}` 后加热电阻微温；`v:0` 关闭
- [ ] 运行中短按 GPIO9 用户键，日志 `encoder zero`，`theta` 回零（上电时勿按住）

## 电机开环

- [ ] `{"cmd":"duty","v":0.2}` 电机缓慢正转
- [ ] `{"cmd":"duty","v":-0.2}` 反转
- [ ] `{"cmd":"duty","v":0}` 停转
- [ ] 转动时 `counts` / `theta` 变化

## 闭环

- [ ] `{"cmd":"zero"}` 后 `theta≈0`
- [ ] `{"cmd":"set","ff_enable":1,"k":0.5,"theta_des":0}` 手拨轴有回中力
- [ ] 增大 `k` 手感变硬；增大 `b` 粘滞感增强
- [ ] `{"cmd":"set","ff_enable":0}` 电机释放

## 安全

- [ ] `duty_max` 不超过 0.6（初调）
- [ ] 堵转立即 `ff_enable=0` 或断电
- [ ] 长时间堵转勿测试（无电流环）
