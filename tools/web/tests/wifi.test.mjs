import test from 'node:test';
import assert from 'node:assert/strict';
import { WifiConnection, BoardConnection, wifiConfig, wifiAddress, redactSecrets } from '../protocol.mjs';

const token = '0123456789abcdef0123456789abcdef';
class FakeSocket extends EventTarget {
  static sockets = [];
  constructor(url) {
    super(); this.url = url; this.readyState = 0; this.bufferedAmount = 0; this.sent = [];
    FakeSocket.sockets.push(this);
    queueMicrotask(() => { this.readyState = 1; this.dispatchEvent(new Event('open')); });
  }
  send(text) { this.sent.push(JSON.parse(text)); }
  message(value) { this.dispatchEvent(new MessageEvent('message', { data: JSON.stringify(value) })); }
  close() { this.readyState = 3; this.dispatchEvent(new Event('close')); }
}

test('USB 配网支持中文和转义字符，按字节限制名称、密码和总命令长度', () => {
  const config = wifiConfig('实验室"WiFi', 'my\\password"');
  assert.equal(config.ssid, '实验室"WiFi');
  assert.equal(wifiConfig('open', '').password, '');
  assert.equal(wifiConfig('network', 'a'.repeat(64)).password.length, 64);
  for (const [ssid, password] of [['', '12345678'], ['中'.repeat(11), '12345678'],
    ['x', 'short'], ['x', 'z'.repeat(64)], ['x\0y', '12345678']]) assert.throws(() => wifiConfig(ssid, password));
});

test('连接地址和密钥分开输入；日志深层隐藏密码和密钥', () => {
  const address = wifiAddress('192.168.1.10', token);
  assert.equal(address.endpoint, 'ws://192.168.1.10/ws');
  assert.equal(new URL(address.url).searchParams.get('token'), token);
  for (const value of ['https://192.168.1.10', 'ws://u:p@localhost/ws', 'ws://localhost/other', 'ws://localhost/ws?token=x']) {
    assert.throws(() => wifiAddress(value, token));
  }
  assert.throws(() => wifiAddress('192.168.1.10', 'wrong'));
  const value = { password: 'secret', wifi: { ssid: 'lab', token }, other: [{ password: 'hidden' }] };
  assert.deepEqual(redactSecrets(value), { password: '***', wifi: { ssid: 'lab', token: '***' }, other: [{ password: '***' }] });
  assert.equal(value.password, 'secret');
});

test('Wi-Fi 命令以 ACK 确认，遥测不吞掉回复，停止可越过未确认 duty；断线拒绝请求且可重连', async () => {
  const connection = new WifiConnection(FakeSocket);
  let closed = 0, telemetry = 0;
  connection.addEventListener('close', () => closed++);
  connection.addEventListener('message', event => { if (event.detail.t === 'telem') telemetry++; });
  await connection.connect({ address: '192.168.1.10', token });
  const socket = FakeSocket.sockets.at(-1);
  const get = connection.request({ cmd: 'get' });
  await connection.writeTail;
  socket.message({ t: 'telem', seq: 1 });
  socket.message({ t: 'ok', cmd: 'get', live: { enc_cpr: 8344 } });
  assert.equal((await get).live.enc_cpr, 8344); assert.equal(telemetry, 1);
  const duty = connection.request({ cmd: 'duty', v: .1 });
  const rejectedDuty = assert.rejects(duty, /中断/);
  await connection.writeTail;
  await assert.rejects(connection.request({ cmd: 'duty', v: 0 }), /停止命令已发送/);
  assert.deepEqual(socket.sent.at(-1), { cmd: 'duty', v: 0 });
  socket.close();
  await rejectedDuty; await connection.closeTask;
  assert.equal(connection.connected, false); assert.equal(closed, 1); assert.equal(connection.pending.size, 0);
  await connection.connect({ address: '192.168.1.10', token });
  assert.equal(connection.connected, true); await connection.close();
});

test('Wi-Fi 超时、板端错误与发送拥堵不会被误报为成功', async () => {
  const connection = new WifiConnection(FakeSocket);
  await connection.connect({ address: '192.168.1.10', token });
  const socket = FakeSocket.sockets.at(-1);
  await assert.rejects(connection.request({ cmd: 'get' }, 10), /回复超时/);
  const denied = connection.request({ cmd: 'wifi_set', ssid: 'lab', password: '12345678' });
  const rejection = assert.rejects(denied, /USB provisioning only/);
  await connection.writeTail; socket.message({ t: 'err', cmd: 'wifi_set', msg: 'USB provisioning only' });
  await rejection;
  socket.bufferedAmount = 65537;
  await assert.rejects(connection.request({ cmd: 'get' }), /未发送/);
  await connection.close();
});

test('连接选择拒绝同时连接两种链路，断开后可切换；失败不会锁住下一次连接', async () => {
  const connection = new BoardConnection({ requestPort: async () => { throw new Error('cancel'); } }, FakeSocket);
  await assert.rejects(connection.connect('usb', 115200), /cancel/);
  await connection.connect('wifi', { address: '192.168.1.10', token });
  assert.equal(connection.kind, 'wifi');
  await assert.rejects(connection.connect('usb', 115200), /先停机/);
  await connection.close(); assert.equal(connection.connected, false);
  await assert.rejects(connection.connect('usb', 115200), /cancel/);
});
