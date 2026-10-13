# Wi-Fi 配网、连接与接口协议

板子以 STA 模式连接 **2.4 GHz** 路由器或热点，通过 WebSocket 提供与 USB 相同的 IMU、电机、编码器、卡尔曼和姿态四元数数据，以及四种电机控制模式。网页仍由电脑上的 `tools/serve.py` 提供，板子仅提供通信接口，不托管网页。

## 首次使用

1. 烧录当前固件，在 Chrome / Edge 打开 <http://127.0.0.1:8776/>，连接方式选择「USB 串口」，连接板子。
2. 展开「通过 USB 配置板子 Wi-Fi」，自动读取开关状态。填写 SSID 和密码，点击「保存 Wi-Fi 配置」，再打开「启用板端 Wi-Fi」开关。保存配置本身不会打开开关；若已经启用则会按新配置重新连接。配置和开关立即写入独立 NVS，不需要点击电机参数的「保存」。
3. 等待状态显示已连接和 IP。网页会在开启后查询约 30 秒，也可以点击「读取 Wi-Fi 状态 / 地址」。启用时板子持续每约 2 秒尝试重连；网页结束自动查询不代表板子停止尝试。
4. 点击「停机并断开」，将连接方式改为「Wi-Fi」。IP 和接入密钥已自动填入，点击「连接 Wi-Fi」。电脑可使用同一路由器的 Wi-Fi 或有线网络；访客网络隔离会阻止连接。
5. 连接成功后自动 `get` / `sub`，使用原有控制面板。结束时点击「停机并断开」。不需要无线时，重新连接 USB 并关闭「启用板端 Wi-Fi」，确认状态显示资源已释放。关闭会保留 SSID、密码和接入密钥，再次开启无需填写密码。

首次无配置时默认关闭。开关状态在重启后保留：保存为关闭时不上电启动无线驱动或 HTTP 服务，保存为开启时自动联网；升级已有 Wi-Fi 固件会保留原来的 `enabled` 值，可通过 USB 关闭。

SSID 按 UTF-8 字节计算，允许 1～32 字节；密码允许 8～63 字节或 64 位十六进制 PSK，开放网络留空。使用 WPA2 个人网络或开放网络，不支持企业 EAP 配网。名称和密码中的引号、反斜杠由 JSON 编码处理；不接受嵌入空字符。单条命令仍需满足 255 字节正文上限。

Wi-Fi 地址和 32 位接入密钥仅保存在当前网页内存中；刷新后先通过 USB 点击「读取 Wi-Fi 状态 / 地址」重新获取，也可手动输入之前记录的 IP 与密钥。DHCP 地址可能变化。切换网页连接方式前必须断开当前连接，网页不会同时占用 USB 和 Wi-Fi。

## USB 配置命令

以下请求按 [USB JSON 行协议](../protocol/usb_protocol.md) 发送，每条以 LF 结束：

| 命令 | 请求示例 | 行为 |
|------|----------|------|
| 查询 | `{"cmd":"wifi_get"}` | 读取网络状态；USB 回复含接入密钥；Wi-Fi 回复省略密钥 |
| 保存配置 | `{"cmd":"wifi_set","ssid":"lab-2.4G","password":"example-password"}` | 仅 USB 允许；保留原开关状态，开启时重新连接，不修改电机参数 |
| 开启 | `{"cmd":"wifi_enable","v":1}` | 仅 USB 允许；需已保存 SSID，保存开关并异步启动驱动、服务及联网 |
| 关闭 | `{"cmd":"wifi_enable","v":0}` | 仅 USB 允许；保存开关并异步关闭无线和网络服务，保留配置及接入密钥 |
| 清除 | `{"cmd":"wifi_forget"}` | 仅 USB 允许；清除网络配置，关闭并释放无线资源，更换接入密钥 |

已连接时的 USB 回复示例（密钥为示例值）：

```json
{"t":"ok","cmd":"wifi_get","wifi":{"ready":1,"enabled":1,"running":1,"busy":0,"error":"","connected":1,"ssid":"lab-2.4G","ip":"192.168.1.100","reason":0,"ws_url":"ws://192.168.1.100/ws","token":"0123456789abcdef0123456789abcdef"}}
```

| 字段 | 含义 |
|------|------|
| `ready` | USB Wi-Fi 配置与管理模块就绪；关闭无线时也为 1；管理模块初始化失败时为 0 |
| `enabled` | 保存的目标开关状态；重启后保留 |
| `running` | 无线运行状态；关闭完成且无错误时为 0 |
| `busy` | 正在异步应用配置或启动/关闭资源；为 1 时操作尚未完成 |
| `error` | 最近一次资源启动/关闭错误名称；成功时为空字符串 |
| `connected` | 已获得 IPv4 地址；保存或开启的回复 ok 不等于已经联网 |
| `ssid` | 保存的网络名称；不返回路由器密码 |
| `ip` / `ws_url` | 当前地址；未连接或正在重配时为空字符串 |
| `reason` | 最近一次 Wi-Fi 断开原因码，来自 ESP-IDF；获得 IP 后清为 0 |
| `token` | 128 位随机接入密钥的 32 位十六进制表示；管理模块就绪的 USB 回复中出现，关闭无线时也可读取 |

配置验证或 NVS 保存失败回复 `t=err`、原 `cmd` 和 ESP-IDF 错误名称；`wifi_enable.v` 必须是数字 0/1，未保存网络时开启返回 `ESP_ERR_INVALID_STATE`。Wi-Fi 链路上的 `wifi_set` / `wifi_enable` / `wifi_forget` 返回 `USB provisioning only`。NVS 使用 `ffh_wifi` 命名空间保存 SSID、密码、enabled、token，与电机参数独立；当前没有启用 NVS 加密。

写操作的 ok 确认配置和开关已保存、操作已入队。之后查询 `wifi_get`：开启需 `busy=0,error="",connected=1`；关闭需 `enabled=0,busy=0,error="",running=0` 才能确认资源释放。网页自动轮询约 30 秒，关闭后清空失效地址但保留密钥。驱动或服务启动失败会清理已分配资源，通过 `error` 提示，USB 可再次开关重试。

## WebSocket 帧与 SDK

连接地址为 `ws://<板子IP>/ws?token=<接入密钥>`，端口默认 80。握手校验接入密钥，同一时刻只接受一个 Wi-Fi 客户端；错误密钥拒绝握手，已有客户端时拒绝新连接。

每个请求是**一个完整文本帧**，内容是 JSON 对象，无需 `>>` 前缀或 LF；JSON 最多 255 UTF-8 字节。二进制、分片、嵌入 NUL 或超长请求关闭会话。回复和遥测也是单个 JSON 文本帧，无普通 ESP_LOG 日志。Ping/Pong/Close 由 ESP-IDF HTTP Server 处理。

`get`、`set`、`duty`、`heater`、`sub`、`unsub`、`imu_filter`、`diag`、`imu_probe`、`zero`、`save` 与 USB 语义相同。IMU 与电机字段详见 [SDK 说明](interface_sdk.md#3-imu-与电机遥测)，姿态为 `quat=[w,x,y,z]`。

在本项目网页的模块环境中可复用 `WifiConnection`，它处理回复匹配、超时和掉线。以下示例只查询和订阅，结束时明确停机：

```javascript
import { WifiConnection } from './protocol.mjs';

const board = new WifiConnection();
board.addEventListener('message', ({ detail }) => {
  if (detail.t === 'telem') {
    console.log(detail.theta, detail.counts, detail.quat);
  }
});
await board.connect({ address: '192.168.1.100', token: '<从USB读取的32位密钥>' });
const params = await board.request({ cmd: 'get' });
await board.request({ cmd: 'sub' });

// 结束采集时执行；若任何请求失败，不宣称停机成功。
await board.request({ cmd: 'duty', v: 0 });
await board.request({ cmd: 'heater', v: 0 });
await board.request({ cmd: 'unsub' });
await board.close();
```

现有 Python `HandleSDK` 使用 USB 串口；开发 Wi-Fi 客户端时使用 WebSocket 库，将相同 JSON 命令放入文本帧，持续解析 `t=telem` 与 `t=ok/err`。协议没有请求 ID，保持每个同名命令最多一个待确认请求；遥测可能与回复交错。超时与断线表示执行结果未知，不能据此自动重复闭环启动、清零或保存。

## 调度与连接限制

USB 和 Wi-Fi 的订阅状态分别维护，`unsub` 只取消发出命令的链路。`telem_hz` 是共享参数。两条链路复用同一遥测快照和命令处理器，命令通过互斥锁串行处理；两条链路都控制电机时，以实际处理的最后一个命令为准。

网络发包进入 HTTP 任务队列，最多保留一个待发送遥测帧；网络拥堵时跳过新帧，避免队列无限增长。断开会清除该 Wi-Fi 会话的订阅，重连需再次 `sub`；不会自动重放控制命令。TCP keepalive 在空闲 10 秒后每 5 秒探测，最多 3 次，用于清理暂停订阅时失联的客户端；实际掉线检测时间受网络影响。启用 Wi-Fi 会增加 Flash / RAM 占用与单核调度负载，实际采样和遥测速率需要实板确认。

关闭开关时终止 WebSocket 会话及 HTTP 任务，调用 `esp_wifi_stop` / `esp_wifi_deinit` 释放无线驱动、收发缓冲和无线任务，并销毁 STA 网口。Wi-Fi 管理任务保留 4 KB 栈、队列和互斥量，关闭时阻塞等待 USB 请求，不再每 2 秒唤醒。首次开启后 lwIP 和系统事件循环作为共享基础设施保留，ESP-IDF 不支持 `esp_netif_deinit`；编译进固件的 Flash 占用不随开关变化。IMU、USB 和电机任务继续运行。

WebSocket 使用 `ws://`，当前未启用 TLS，仅适用于可信局域网。密码不通过网络查询返回，网页通信日志隐藏 password/token，接入密钥不会写入浏览器持久存储。浏览器询问本地网络访问权限时需允许；HTTPS 页面可能限制连接 `ws://`，本工具应从本机 HTTP 服务打开。

当前没有连接丢失自动停机或控制租约。拔线、网络中断、关网页都不能保证板端停机；先使用「停机并断开」。Wi-Fi 配置不会修改保存的控制模式，原有上电恢复行为保持不变。

实现采用 ESP-IDF [Wi-Fi 驱动](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32c3/api-reference/network/esp_wifi.html)和 [HTTP Server WebSocket](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32c3/api-reference/protocols/esp_http_server.html)。ESP-IDF 6 的 cJSON 使用[官方组件](https://components.espressif.com/components/espressif/cjson/versions/1.7.19~2/readme)，首次构建需下载，后续可使用组件缓存。
