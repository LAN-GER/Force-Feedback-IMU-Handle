# 板端接口协议与 SDK 二次开发说明

本文面向通过 PC、上位机或浏览器开发 IMU 采集和电机控制程序。板子支持 ESP32-C3 原生 USB-Serial/JTAG 的 JSON 行协议，以及 Wi-Fi WebSocket 的 JSON 文本帧。USB 还负责配置与开关 Wi-Fi、读取 IP 与接入密钥；`wifi_enable.v=0/1` 持久化开关，关闭保留账号密码并释放无线资源，`wifi_set` 保存配置不自动开启。异步操作需查询 `busy` / `error` / `running` 确认完成。下方 Python SDK 使用 USB，Wi-Fi SDK 与帧格式见 [Wi-Fi 协议](wifi.md)。示例对应当前工程固件，完整字段定义见 [USB 协议](../protocol/usb_protocol.md)。

## 1. 连接与帧格式

| 项目 | 约定 |
|------|------|
| 接口 | 板子原生 USB，Windows 中选择实际 COM 端口；本工程曾使用 COM3，但不可固定假设 |
| 上位机配置 | 115200，8N1，无流控；串口同一时刻由一个程序占用 |
| 编码 | UTF-8，每条 JSON 以 LF（`\n`）结束；固件也接受 CRLF |
| 请求 | 扁平 JSON 对象，`cmd` 为字符串，控制参数为有限数值，Wi-Fi SSID/password 为字符串；使能字段使用数字 0/1，不使用 JSON true/false |
| 请求长度 | JSON 正文最多 255 UTF-8 字节，不含换行；超长行被整行丢弃，可能没有错误回复 |
| 板端输出 | `>>` + JSON + LF；同一端口还输出普通启动／运行日志 |
| 回复容量 | 当前固件 JSON 缓冲为 1024 字节，上位机应留出前缀和换行空间 |
| 并发 | 无请求 ID、无 CRC、无协议版本协商；命令串行发送，遥测可与回复交错 |

网页需要桌面 Chrome／Edge、安全上下文和用户通过端口选择弹窗授权。Python／C#／C++ 等直接使用系统串口接口即可。打开前先关闭网页连接和 IDF monitor；初始化 DTR/RTS 后再打开端口，避免主动切换复位信号。

以下请求发送后读取到的回复为独立的一行：

```text
TX {"cmd":"sub"}\n
RX >>{"t":"ok","cmd":"sub"}\n
```

成功类型分为简单确认和带数据回复；失败为 `t=err`：

```json
{"t":"ok","cmd":"sub"}
{"t":"ok","cmd":"imu_filter","live":{"imu_kf_en":1}}
{"t":"err","cmd":"imu_filter","msg":"v must be 0 or 1"}
```

带 `live` 的示例只展示相关字段，实际回复包含完整运行参数。`get` 不返回独立 NVS 存档副本；`diag` 返回 `diag` 对象，`imu_probe` 返回 `probe` 对象。

解析必须先积累字节到完整换行，再解码 UTF-8、移除 `>>`、解析 JSON。不要将一次串口 read 当成一帧；日志可忽略或另存。将 `t=telem` 路由到数据处理，将 `t=ok/err` 按 `cmd` 匹配当前请求，忽略未知字段以兼容后续扩展。

## 2. 命令 API

| SDK 操作 | 请求 | 回复和副作用 |
|----------|------|--------------|
| 读取参数 | `{"cmd":"get"}` | `ok + live`，只读 |
| 修改参数 | `{"cmd":"set","enc_cpr":8344}` | `ok + live`，立即应用参数；可能重置正在运行的控制器 |
| 开始遥测 | `{"cmd":"sub"}` | `ok`，随后持续输出 `telem` |
| 停止遥测 | `{"cmd":"unsub"}` | `ok`，只停遥测，不停止电机；接收端仍可能读到已排队的旧帧 |
| 停止电机 | `{"cmd":"duty","v":0}` | `ok`，模式切为 0 并清除开环输出 |
| 开环 PWM | `{"cmd":"duty","v":0.1}` | `ok`，切换模式 0，正负值控制方向 |
| 编码器清零 | `{"cmd":"zero"}` | `ok`，重置计数、测速、PID／电流环及 DOB 状态，不修改目标角 |
| IMU 卡尔曼 | `{"cmd":"imu_filter","v":1}` | `ok + live`，0 关闭、1 开启，不改变电机模式，不重置电机积分器 |
| 加热占空比 | `{"cmd":"heater","v":0}` | `ok + live`，范围 0～1，无温度闭环 |
| 保存参数 | `{"cmd":"save"}` | `ok`，保存当前参数至 NVS，不用于每帧更新 |
| 缓存诊断 | `{"cmd":"diag"}` | `ok + diag`，不重新初始化 IMU、不访问 SPI |
| SPI 探测 | `{"cmd":"imu_probe"}` | `ok + probe`；仅 IMU 初始化未成功时可用，正常初始化后拒绝 |

`set` 对未知字段或部分非法字段可能忽略后仍回复 `ok`，不同服务也可能拒绝非法增益而保留旧值。上位机应先检查范围，再核对回复中的 `live` 和遥测；确认回复只表明命令处理完成，不表示电机已到达目标。

超时表示执行结果未知。不要自动重复闭环启动、清零或保存命令；先检查通信及运行状态。当前协议无法识别超时后迟到的同名回复；示例 SDK 将该会话标为不确定，停止普通请求。

## 3. IMU 与电机遥测

`sub` 后每帧为 `t=telem`，包含 IMU、编码器、电流、控制状态和 DOB。默认 50 Hz；可发送 `{"cmd":"set","telem_hz":50}` 修改。参数接受 1～200，独立 `esp_timer` 按 `1000000 / telem_hz` 微秒定期唤醒发送，不在组包/发送后额外延时一个周期；实际频率仍受单核负载、USB、网络及接收端调度影响。板端采样名义为 1 kHz，遥测只发送最新快照，处理过慢时跳过积压通知，不保证上传全部样本。`diag` 提供 `telem_work_us` / `telem_work_max_us`（最近/最高工作耗时）和 `telem_skipped`（累计丢弃通知）辅助排查，新版还提供快照、组包、USB 写入、Wi-Fi 入队分段耗时及组包失败计数。字段说明见 [只读诊断](../protocol/usb_protocol.md#只读诊断-diag)。

当前遥测组包保留字段、单位和小数位；非有限浮点值为 `null`，异常大有限值可为科学计数法，接收端需要检查数值有效性。USB 100 Hz 的本板反馈与验证范围见 [遥测验证记录](firmware.md#遥测验证记录)。

以下是完整字段示例，数值为示意数据：

```json
{"t":"telem","seq":1000,"imu_ok":1,"mode":0,"imu_acc_id":30,"imu_gyro_id":15,"acc":[0,0,1],"gyro":[0,0,0],"imu_kf_en":1,"imu_kf":1,"acc_f":[0,0,1],"gyro_f":[0,0,0],"theta":0,"omega":0,"counts":0,"tau":0,"duty":0,"ff_en":0,"lim":0,"pid_err":0,"i_meas":0,"i_des":0,"tau_des":0,"tau_meas":0,"i_hat":0,"tau_m_hat":0,"tau_ext_hat":0,"alpha_hat":0,"quat_ok":1,"quat_kf":1,"quat":[1,0,0,0]}
```

| 字段 | 单位／含义 | 使用条件 |
|------|-----------|----------|
| `seq` | uint32 采样快照序号 | 会回绕，重启后重新计数；不是毫秒时间戳 |
| `imu_ok` | 本帧 IMU 有效标志 0/1 | 为 0 时不使用原始或滤波六轴 |
| `imu_acc_id` / `imu_gyro_id` | 初始化缓存 ID | 正常十进制 30 / 15，即 0x1E / 0x0F |
| `acc` / `acc_f` | XYZ 加速度，g | 前者原始，后者卡尔曼；换算 m/s² 可乘 9.80665 |
| `gyro` / `gyro_f` | XYZ 角速度，°/s | 转 rad/s 乘 π/180；不是姿态角 |
| `imu_kf_en` / `imu_kf` | 滤波开关／本帧滤波有效标志 | 仅三标志 `imu_ok`、`imu_kf_en`、`imu_kf` 全为 1 才使用滤波数组 |
| `quat` | 板端姿态单位四元数 `[w,x,y,z]` | 右手系，传感器→世界，世界 +Z 向上；无量纲；不要按 `[x,y,z,w]` 读取 |
| `quat_ok` / `quat_kf` | 姿态有效／本次融合输入为有效卡尔曼六轴 | `imu_ok=1`、`quat_ok=1` 才使用 quat；`quat_kf=0` 不表示姿态无效，可能正在用原始六轴 |
| `counts` | 有符号累计四倍频计数 | 编码器可多圈累计，清零不是寻找机械绝对零点 |
| `theta` / `omega` | 输出轴 rad / rad/s | 与 IMU 轴姿态无关；theta 不限制在一圈内 |
| `mode` / `duty` | 当前模式／实际施加的有符号占空比 | 以遥测判断板端运行状态 |
| `ff_en` / `lim` | 阻抗开启／控制限幅标志 | `lim` 是控制器限幅，不是驱动器 nFAULT |
| `tau` | 阻抗控制量 | 仅模式 1 有意义，直接用于 duty，不能当作标定 N·m |
| `pid_err` | 位置误差 rad | 仅模式 2 有意义 |
| `i_meas` | IPROPI 电流 A | 幅值按 duty 合成符号，无独立 ADC 有效标志 |
| `i_des` / `tau_des` / `tau_meas` | 力矩环目标电流 A／目标及估计力矩 N·m | 主要用于模式 3，依赖 Kt 标定 |
| `i_hat` / `tau_m_hat` / `tau_ext_hat` | DOB 电流 A／电机及外力矩估计 N·m | 依赖 DOB 使能和机械参数标定 |
| `alpha_hat` | DOB 角加速度估计 rad/s² | 速度差分后低通 |

IMU 关闭滤波、读数无效或旧固件缺少新字段时，先检查状态标志，不将占位值当成数据。板端六轴卡尔曼负责降噪，姿态模块另行融合并输出 `quat`；无效 quat 是 `[1,0,0,0]` 占位，不能当作实测水平姿态。网页 [imu-attitude.mjs](../tools/web/imu-attitude.mjs) 直接使用板端四元数做显示与参考归零，不再融合 USB 六轴。初始倾斜参考重力、相对航向为零；无磁力计，航向可能漂移。

四元数为传感器到世界的旋转：`v_world = q ⊗ [0,v_sensor] ⊗ conjugate(q)`；初始航向参考取决于板端姿态初始化，重连不会重新初始化板端。用于 SDK／游戏引擎时请核对坐标轴、左右手系和四元数顺序。需要显示角时可按 ZYX 约定计算（结果为 rad）：`roll=atan2(2(wx+yz),1−2(x²+y²))`、`pitch=asin(clamp(2(wy−zx),−1,1))`、`yaw=atan2(2(wz+xy),1−2(y²+z²))`。姿态本身传输为四元数，不传欧拉角数组。

输出轴默认 `enc_cpr=8344`，按 7 PPR × 4 × 298 计算；7 PPR 由约 8292 的粗测推断，仍需规格或多圈校验。`theta = counts × 2π / enc_cpr`，不再次除以 298。位置及速度均使用此标定，详见 [编码器换算及升级迁移](firmware.md#编码器输出轴换算)。

## 4. 四种电机控制模式

所有控制示例均立即生效。角度通过协议统一使用 rad，例如 30° 应发送 `30 × π / 180 ≈ 0.523599`。上位机可在输入层显示度，但不要将度直接填入 `theta_des`。

| 模式 | 主要参数 | 算法及限幅 |
|------|----------|------------|
| 0 开环 | `duty` 命令的 `v`，−1～1 | 固定 PWM，不使用 `duty_max` 或角度软限位 |
| 1 阻抗 | `k`、`b`、`theta_des`、`duty_max`、`theta_min/max` | 角度回中及速度阻尼，含软限位回推 |
| 2 位置 PID | `pid_kp/ki/kd`、`theta_des`、`duty_max`、`theta_min/max` | 位置反馈，积分限幅，微分项使用角速度 |
| 3 力矩／电流 PI | `tau_des`、`kt`、`tq_kp/ki`、`i_max`、`duty_max` | `i_des=tau_des/kt`，电流及 PWM 限幅；不执行角度软限位 |

以下各行是独立请求，选定一种模式发送，不应把四种模式当作初始化序列：

```json
{"cmd":"duty","v":0.1}
{"cmd":"set","ctrl_mode":1,"k":0.5,"b":0.05,"theta_des":0,"duty_max":0.2,"theta_min":-1.5,"theta_max":1.5}
{"cmd":"set","ctrl_mode":2,"theta_des":0.3,"pid_kp":2,"pid_ki":0.5,"pid_kd":0.05,"duty_max":0.2,"theta_min":-1.5,"theta_max":1.5}
{"cmd":"set","ctrl_mode":3,"tau_des":0.001,"kt":0.02,"tq_kp":1.5,"tq_ki":8,"i_max":0.1,"duty_max":0.2}
{"cmd":"duty","v":0}
```

建议校验：所有数值有限；增益非负；`0<duty_max≤1`；`theta_min<theta_max`，位置目标在限位内；`kt>1e-6`、`i_max>0.05`。电流和力矩系数需校准，默认值不代表实物精度。`set` 中设置 `kt` 或 `dob_kt` 会同时更新二者；同时发送不同值时，固件后处理的 `dob_kt` 覆盖两者。

闭环模式切换应将模式和该模式参数放在同一条 `set` 中，避免先按旧目标启动再改参数。完整位置请求长度小于 255 字节；超长配置可先停机，分批设置参数，最后发送完整启动请求。参数设置在命令处理函数中依次应用，固件没有多任务事务锁，不能把确认当作采样周期内的原子切换保证。

停止使用 `duty=0`。单独设置 `ctrl_mode=0` 只切换模式，不能保证清除已有开环目标。`zero` 不修改目标，在闭环中清零会改变位置误差；一般先停机再清零。关闭串口、取消订阅或上位机进程退出不会自动停机，固件当前没有 USB 断联看门狗。

`save` 保存模式、目标、参数、滤波开关及加热占空比；IMU 初始化成功时上电恢复，开环 duty 不存档。需要上电停机时，停止电机、关闭加热后再保存。IMU 运行时读取失败只使数据无效，不自动停止现有电机控制。

## 5. Python SDK 示例

下面的同步示例可复制保存为 `handle_sdk.py`；需 Python 3 和 pyserial（例如 `python -m pip install pyserial`）。只允许一个线程调用该客户端，无后台接收任务；若需要多线程、ROS 或 Qt 集成，将所有串口读写交给同一个工作线程，再通过队列分发数据。

SDK 用单一读缓冲处理分包、日志及遥测交错；请求期间收到的遥测存入最多 256 帧的队列，满后保留最新数据。`close()` 只关闭端口，不发送控制命令。超时后标记会话不确定；`stop_unconfirmed()` 可尽力发送停机，但不宣称收到确认。

```python
import json
import math
import time
from collections import deque

import serial


class BoardError(RuntimeError):
    pass


class HandleSDK:
    def __init__(self, port, baudrate=115200):
        self.port = serial.Serial(port=None, baudrate=baudrate,
                                  timeout=0.05, write_timeout=1,
                                  xonxoff=False, rtscts=False, dsrdtr=False)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = port
        self.port.open()
        self.buffer = bytearray()
        self.telemetry = deque(maxlen=256)
        self.uncertain = False

    @staticmethod
    def _encode(cmd, params):
        if not isinstance(cmd, str) or not cmd or not cmd.isascii():
            raise ValueError("cmd must be a nonempty ASCII string")
        for value in params.values():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("parameters must be finite numbers; use 0/1")
        body = json.dumps(dict(params, cmd=cmd), ensure_ascii=True,
                          separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(body) > 255:
            raise ValueError("JSON body exceeds 255 bytes")
        return body + b"\n"

    def _write(self, payload):
        try:
            if self.port.write(payload) != len(payload):
                raise OSError("partial serial write; execution unknown")
        except (OSError, serial.SerialException):
            self.uncertain = True
            raise

    def _read_message(self, deadline):
        while True:
            while b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n")
                self.buffer[:] = rest
                text = line.decode("utf-8", errors="replace").strip()
                prefix = text.find(">>")
                if prefix < 0:
                    continue  # Ordinary board logs are not protocol replies.
                try:
                    msg = json.loads(text[prefix + 2:])
                except json.JSONDecodeError:
                    continue
                if isinstance(msg, dict):
                    return msg
            if len(self.buffer) > 8192:
                self.uncertain = True
                raise BoardError("receive line too long; reopen the session")
            if time.monotonic() >= deadline:
                raise TimeoutError("no matching frame before deadline")
            try:
                chunk = self.port.read(min(self.port.in_waiting or 1, 4096))
            except (OSError, serial.SerialException):
                self.uncertain = True
                raise
            self.buffer.extend(chunk)

    def request(self, cmd, timeout=3.0, **params):
        if self.uncertain:
            raise BoardError("previous command is uncertain; do not retry blindly")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive")
        self._write(self._encode(cmd, params))
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                msg = self._read_message(deadline)
                if msg.get("t") == "telem":
                    self.telemetry.append(msg)
                elif msg.get("cmd") == cmd and msg.get("t") in ("ok", "err"):
                    if msg["t"] == "err":
                        raise BoardError(msg.get("msg", "board rejected command"))
                    return msg
            raise TimeoutError(f"{cmd}: execution result unknown")
        except TimeoutError:
            self.uncertain = True
            raise

    def next_telemetry(self, timeout=1.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive")
        if self.telemetry:
            return self.telemetry.popleft()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._read_message(deadline)
            if msg.get("t") == "telem":
                return msg
        raise TimeoutError("no telemetry; confirm subscription and connection")

    def stop_unconfirmed(self):
        self.uncertain = True
        self._write(self._encode("duty", {"v": 0}))

    def close(self):
        self.port.close()
```

此示例是应用层封装，不提供重连、自动重试、重启检测或电机到位等待。队列帧可能是历史数据，实时控制应记录接收时间、清理积压并检查新鲜度。新会话也不能凭重连就断定前一条命令未执行，应重新读取参数和遥测核对。

### 采集 IMU 和电机数据

下例只读取、订阅和取消订阅，不主动改变当前电机或加热状态：

```python
from handle_sdk import HandleSDK

board = HandleSDK("COM3")
try:
    print(board.request("get")["live"])
    print(board.request("diag")["diag"])
    board.request("sub")
    for _ in range(100):
        frame = board.next_telemetry(timeout=2)
        imu = None
        if frame.get("imu_ok") == 1:
            filtered = (frame.get("imu_kf_en") == 1 and frame.get("imu_kf") == 1
                        and "acc_f" in frame and "gyro_f" in frame)
            imu = (frame["acc_f"], frame["gyro_f"]) if filtered else (
                frame["acc"], frame["gyro"])
        quat = frame.get("quat") if frame.get("imu_ok") == 1 and frame.get("quat_ok") == 1 else None
        print(frame["seq"], imu, quat, frame["theta"], frame["omega"], frame["i_meas"])
    board.request("unsub")
finally:
    board.close()
```

### 控制调用映射

在已建立的 `board` 客户端上，每一行分别对应一个独立操作。启用控制前完成编码器、电流和限幅校验：

```python
board.request("imu_filter", v=1)
board.request("duty", v=0)  # Confirmed stop before changing calibration/reference.
board.request("zero")
board.request("set", enc_cpr=8344, enc_dir=0)
board.request("set", ctrl_mode=2, theta_des=0.3,
              pid_kp=2, pid_ki=0.5, pid_kd=0.05,
              duty_max=0.2, theta_min=-1.5, theta_max=1.5)
board.request("duty", v=0)
```

参数范围检查仍由调用方负责。正常通信时用 `request("duty", v=0)` 等待确认；命令超时后可调用 `stop_unconfirmed()` 尽力停机，并明确记录未确认结果，断线时无法保证命令送达。不要用 `close()` 代替停机。

## 6. 浏览器／其他语言接入

网页已有可复用 [protocol.mjs](../tools/web/protocol.mjs)：`encodeCommand` 检查长度与有限数值，`LineDecoder` 处理 UTF-8 分包，`SerialConnection` 分发回复及遥测。浏览器需从 localhost／HTTPS 加载模块，连接动作必须由用户点击触发。以下代码放在网页按钮处理逻辑中，`telem` 是采集逻辑的示例回调：

```javascript
import { SerialConnection } from './protocol.mjs';

const board = new SerialConnection(navigator.serial);
board.addEventListener('message', event => {
  const frame = event.detail;
  if (frame.t === 'telem') telem(frame);
});
// In a user-triggered button handler:
await board.connect(115200);
const reply = await board.request({ cmd: 'get' });
console.log(reply.live);
await board.request({ cmd: 'sub' });
```

路径按宿主页面位置调整；此模块允许不同名称命令同时等待回复，二次开发仍建议统一串行队列。页面原有停止优先发送逻辑不能消除同名回复歧义。C#／C++／ROS 节点可沿用相同帧解析、单接收线程和命令串行化策略，无需重新定义板端协议。

## 7. 调试与兼容检查

| 情况 | 处理 |
|------|------|
| 没有 `ok` 回复 | 检查换行、255 字节限制、实际端口、固件是否进入 `USB JSON ready` |
| `unknown` | 命令拼写或固件不支持；读取 `diag.firmware_sha` 辅助定位构建版本 |
| `imu_ok=0` | IMU 不可用，保留电机／编码器数据但停止使用六轴；查询 `diag` |
| 滤波字段缺失 | 旧固件：使用有效原始六轴，不自行假装已有板端滤波数据 |
| 四元数字段缺失 | 旧固件：保留六轴读取，但板端姿态不可用；更新固件后使用 `quat_ok` 校验 |
| 只收到日志 | 固件未运行到命令服务、端口选错或未订阅，按启动日志排查 |
| 配置频率高于实际接收速率 | 统计完整 `telem` 帧的到达时间；读取 `diag.telem_work_us` 和四段耗时，观察 `telem_skipped` 增量。频率修改仅当前生效，持久化需 `save` 确认 |
| 无遥测／数据过期 | 确认 `sub`、新帧接收时间及 `seq`，不要继续用历史帧作控制反馈 |
| 复位／拔线 | 清除主机队列及解析缓冲，重新连接、查询状态和订阅；不自动恢复运动指令 |

实现依据为 [app_usb.c](../software/main/app_usb.c)、[bsp_usb.c](../software/components/bsp/src/bsp_usb.c)、[params.h](../software/components/svc/include/svc/params.h) 及 [imu_ff.c](../software/components/svc/src/imu_ff.c)。硬件与标定说明见 [引脚表](../hardware/pinmap.md)、[力矩估计](torque_estimation.md)，首次连接见 [快速上手](getting_started.md)。
