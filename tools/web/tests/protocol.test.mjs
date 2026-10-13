import test from 'node:test';
import assert from 'node:assert/strict';
import { LineDecoder, parseLine, encodeCommand, isTelemetry, csvRow, CSV_FIELDS, SerialConnection } from '../protocol.mjs';
import { hasFilteredImu } from '../imu-filter.mjs';

test('CSV 导出板端四元数，无效或旧固件缺少姿态时分量留空', () => {
  const data = { imu_ok: 1, quat_ok: 1, quat_kf: 1, quat: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] };
  const column = CSV_FIELDS.indexOf('quat_w') + 2;
  const values = value => csvRow({ at: 0, source: 'usb', data: value }).split(',');
  assert.equal(Number(values(data)[column]), Math.SQRT1_2);
  for (const patch of [{ quat_ok: 0 }, { imu_ok: 0 }, { quat: undefined }, { quat: [2, 0, 0, 0] }]) {
    assert.equal(values({ ...data, ...patch })[column], '');
  }
});

test('滤波对比与 CSV 只使用板端有效的同帧结果，旧固件仍可读取', () => {
  const data = { t: 'telem', seq: 1, imu_ok: 1, acc: [0, 0, 1], gyro: [1, 2, 3],
    imu_kf_en: 1, imu_kf: 1, acc_f: [.01, .02, .98], gyro_f: [.8, 1.8, 2.8] };
  assert.equal(hasFilteredImu(data), true);
  const csv = value => csvRow({ at: 0, source: 'usb', data: value }).split(',');
  const column = CSV_FIELDS.indexOf('acc_f_z') + 2;
  assert.equal(csv(data)[column], '0.98');
  for (const patch of [{ imu_kf_en: 0 }, { imu_kf: 0 }, { imu_ok: 0 },
    { acc_f: [null, 0, 1] }, { gyro_f: [1, 2, NaN] }, { imu_kf_en: undefined }]) {
    const value = { ...data, ...patch };
    assert.equal(hasFilteredImu(value), false);
    assert.equal(csv(value)[column], '');
    assert.equal(isTelemetry(value), true);
  }
});

test('USB 分包、粘包、CRLF 和 UTF-8 中文日志不会破坏 JSON 帧', () => {
  const lines = [];
  const decoder = new LineDecoder(line => lines.push(parseLine(line)));
  const bytes = new TextEncoder().encode('I (1) app: 手柄启动\r\n>>{"t":"ok","cmd":"sub"}\n>>{"t":"telem","seq":1}\n');
  for (let i = 0; i < bytes.length; i += 2) decoder.feed(bytes.slice(i, i + 2));
  assert.equal(lines.length, 3);
  assert.equal(lines[0].text, 'I (1) app: 手柄启动');
  assert.equal(lines[1].value.cmd, 'sub');
  assert.equal(lines[2].value.seq, 1);
});

test('日志、损坏 JSON、ANSI 日志和混入日志前缀的回复可区分', () => {
  assert.equal(parseLine('I (20) bmi088: ok').kind, 'log');
  assert.equal(parseLine('>>{"t":nan}').kind, 'invalid');
  assert.equal(parseLine('\x1b[32m>>{"t":"ok","cmd":"get"}\x1b[0m').value.cmd, 'get');
  assert.equal(parseLine('I (2) log: >>{"t":"ok","cmd":"sub"}').value.cmd, 'sub');
});

test('超长行整行丢弃，下一帧恢复', () => {
  const lines = []; let overflows = 0;
  const decoder = new LineDecoder(line => lines.push(line), () => overflows++, 30);
  decoder.feedText('x'.repeat(31) + '>>{"cmd":"duty"}\n>>{"t":"ok"}\n');
  assert.equal(overflows, 1);
  assert.deepEqual(lines, ['>>{"t":"ok"}']);
});

test('命令按固件字节限制编码，拒绝 NaN 和超长批次', () => {
  assert.equal(new TextDecoder().decode(encodeCommand({ cmd: 'duty', v: -.2 })), '{"cmd":"duty","v":-0.2}\n');
  assert.throws(() => encodeCommand({ cmd: 'set', k: NaN }), /有限数值/);
  assert.throws(() => encodeCommand({ cmd: 'get', note: '中'.repeat(100) }), /255/);
  assert.throws(() => encodeCommand([]), /cmd/);
});

test('遥测校验和 CSV 字段、单位数据展开，缺失字段留空', () => {
  const data = { t: 'telem', seq: 3, acc: [0, 0, 1], gyro: [1, 2, 3], theta: .5 };
  assert.ok(isTelemetry(data));
  assert.equal(isTelemetry({ ...data, acc: [0, null, 1] }), false);
  const columns = csvRow({ at: 0, source: 'usb', data }).split(',');
  assert.equal(columns[0], '1970-01-01T00:00:00.000Z');
  assert.equal(columns[1], 'usb');
  assert.equal(columns[7], '1'); // acc_z
  assert.equal(columns[11], '0.5'); // theta
});

function fakePort() {
  let controller;
  const sent = [];
  const port = {
    readable: new ReadableStream({ start(value) { controller = value; } }),
    writable: new WritableStream({ write(bytes) { sent.push(JSON.parse(new TextDecoder().decode(bytes))); } }),
    open: async () => {}, close: async () => { assert.equal(port.readable.locked, false); assert.equal(port.writable.locked, false); },
    getInfo: () => ({ usbVendorId: 0x303a, usbProductId: 0x1001 })
  };
  return { port, sent, reply: value => controller.enqueue(new TextEncoder().encode(`>>${JSON.stringify(value)}\n`)), unplug: () => controller.error(new Error('拔线')) };
}

test('串口请求以板端 ACK 为准，错误回复、超时和断开释放锁', async () => {
  const fake = fakePort();
  const client = new SerialConnection({ requestPort: async () => fake.port });
  await client.connect(115200);
  const get = client.request({ cmd: 'get' });
  fake.reply({ t: 'ok', cmd: 'get', live: { k: .8 } });
  assert.equal((await get).live.k, .8);
  const heater = client.request({ cmd: 'heater', v: 0 });
  fake.reply({ t: 'err', cmd: 'heater', msg: 'nvs' });
  await assert.rejects(heater, /板端拒绝/);
  await assert.rejects(client.request({ cmd: 'sub' }, 20), /超时/);
  const pending = client.request({ cmd: 'get' });
  const rejected = assert.rejects(pending, /已断开/);
  await client.close(); await rejected;
  assert.equal(client.connected, false);
  assert.ok(fake.sent.some(value => value.cmd === 'sub'));
});

test('等待 duty 回复时点击停止仍发送 duty=0，同时不伪造确认', async () => {
  const fake = fakePort();
  const client = new SerialConnection({ requestPort: async () => fake.port });
  await client.connect(115200);
  const first = client.request({ cmd: 'duty', v: .2 });
  const firstRejected = assert.rejects(first, /已断开/);
  await assert.rejects(client.request({ cmd: 'duty', v: 0 }), /停止命令已发送/);
  assert.deepEqual(fake.sent, [{ cmd: 'duty', v: .2 }, { cmd: 'duty', v: 0 }]);
  await client.close(); await firstRejected;
});

test('拔线自动清理，拒绝待确认请求', async () => {
  const fake = fakePort();
  const client = new SerialConnection({ requestPort: async () => fake.port });
  await client.connect(115200);
  const close = new Promise(resolve => client.addEventListener('close', resolve, { once: true }));
  const pending = assert.rejects(client.request({ cmd: 'get' }), /读取中断/);
  fake.unplug(); await pending; await close;
  assert.equal(client.port, null);
});
