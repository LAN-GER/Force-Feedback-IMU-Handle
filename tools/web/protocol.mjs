import { hasFilteredImu } from './imu-filter.mjs';
import { hasBoardAttitude } from './imu-attitude.mjs';
// The firmware uses newline-delimited JSON with a >> response prefix.
export const MAX_COMMAND_BYTES = 255; // bsp_usb.c: USB_LINE_MAX = 256, including NUL.

export function redactSecrets(value) {
  if (!value || typeof value !== 'object') return value;
  if (Array.isArray(value)) return value.map(redactSecrets);
  return Object.fromEntries(Object.entries(value).map(([key, item]) =>
    [key, ['password', 'token'].includes(key) ? '***' : redactSecrets(item)]));
}

export function encodeCommand(command) {
  if (!command || typeof command !== 'object' || Array.isArray(command) ||
      typeof command.cmd !== 'string' || !command.cmd.length) {
    throw new Error('命令必须是包含 cmd 字符串的 JSON 对象');
  }
  const inspect = value => {
    if (typeof value === 'number' && !Number.isFinite(value)) throw new Error('参数必须是有限数值');
    if (value && typeof value === 'object') Object.values(value).forEach(inspect);
  };
  inspect(command);
  const json = JSON.stringify(command);
  const encoded = new TextEncoder().encode(json);
  if (encoded.length > MAX_COMMAND_BYTES) {
    throw new Error(`命令超过固件 ${MAX_COMMAND_BYTES} 字节上限，请分批发送参数`);
  }
  return new TextEncoder().encode(`${json}\n`);
}

export function parseLine(line) {
  const clean = line.replace(/\x1b\[[0-9;]*[A-Za-z]/g, '').trim();
  const prefix = clean.indexOf('>>');
  const candidate = prefix >= 0 ? clean.slice(prefix + 2).trim() : clean;
  if (!candidate.startsWith('{')) return { kind: 'log', text: clean };
  try {
    const value = JSON.parse(candidate);
    if (!value || typeof value !== 'object' || Array.isArray(value)) throw new Error('非 JSON 对象');
    return { kind: 'json', value, text: clean };
  } catch {
    return { kind: 'invalid', text: clean };
  }
}

export class LineDecoder {
  constructor(onLine, onOverflow = () => {}, maxLength = 8192) {
    this.onLine = onLine;
    this.onOverflow = onOverflow;
    this.maxLength = maxLength;
    this.decoder = new TextDecoder();
    this.buffer = '';
    this.discarding = false;
  }
  feed(bytes) { this.feedText(this.decoder.decode(bytes, { stream: true })); }
  feedText(text) {
    for (const character of text) {
      if (character === '\n') {
        if (!this.discarding && this.buffer.trim()) this.onLine(this.buffer.replace(/\r$/, ''));
        this.buffer = '';
        this.discarding = false;
      } else if (!this.discarding) {
        this.buffer += character;
        if (this.buffer.length > this.maxLength) {
          this.buffer = '';
          this.discarding = true;
          this.onOverflow();
        }
      }
    }
  }
}

export function isTelemetry(value) {
  return value.t === 'telem' && Number.isFinite(value.seq) &&
    ['acc', 'gyro'].every(key => Array.isArray(value[key]) && value[key].length === 3 &&
      value[key].every(Number.isFinite));
}

export const CSV_FIELDS = ['seq', 'imu_ok', 'mode', 'acc_x', 'acc_y', 'acc_z',
  'gyro_x', 'gyro_y', 'gyro_z', 'theta', 'omega', 'counts', 'duty', 'tau', 'ff_en',
  'lim', 'pid_err', 'i_meas', 'i_des', 'tau_des', 'tau_meas', 'i_hat', 'tau_m_hat',
  'tau_ext_hat', 'alpha_hat', 'imu_acc_id', 'imu_gyro_id', 'imu_kf_en', 'imu_kf',
  'acc_f_x', 'acc_f_y', 'acc_f_z', 'gyro_f_x', 'gyro_f_y', 'gyro_f_z',
  'quat_ok', 'quat_kf', 'quat_w', 'quat_x', 'quat_y', 'quat_z'];

export function csvRow(record) {
  const values = { ...record.data };
  ['w', 'x', 'y', 'z'].forEach((axis, i) => {
    values[`quat_${axis}`] = hasBoardAttitude(record.data) ? record.data.quat[i] : undefined;
  });
  ['acc', 'gyro'].forEach(key => ['x', 'y', 'z'].forEach((axis, i) => {
    values[`${key}_${axis}`] = record.data[key]?.[i];
    values[`${key}_f_${axis}`] = hasFilteredImu(record.data) ? record.data[`${key}_f`][i] : undefined;
  }));
  return [new Date(record.at).toISOString(), record.source,
    ...CSV_FIELDS.map(key => Number.isFinite(values[key]) ? values[key] : '')].join(',');
}

export class SerialConnection extends EventTarget {
  constructor(serial) {
    super();
    this.serial = serial;
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.pending = new Map();
    this.writeTail = Promise.resolve();
    this.closing = false;
    this.closeTask = null;
  }
  emit(type, detail) { this.dispatchEvent(new CustomEvent(type, { detail })); }
  get connected() { return !!this.port && !this.closing; }
  async connect(baudRate) {
    if (this.port) throw new Error('串口已连接');
    const port = await this.serial.requestPort();
    await port.open({ baudRate, bufferSize: 16384 });
    if (!port.readable || !port.writable) {
      await port.close();
      throw new Error('串口读写流不可用');
    }
    this.port = port;
    this.closing = false;
    this.closeTask = null;
    this.writeTail = Promise.resolve();
    this.writer = port.writable.getWriter();
    this.reader = port.readable.getReader();
    this.framer = new LineDecoder(line => this.onLine(line), () => this.emit('log', '丢弃过长串口行'));
    this.readTask = this.readLoop();
    this.emit('open', port.getInfo());
  }
  onLine(line) {
    const parsed = parseLine(line);
    this.emit('line', parsed);
    if (parsed.kind !== 'json') return;
    const message = parsed.value;
    if (message.t === 'ok' || message.t === 'err') {
      const pending = this.pending.get(message.cmd);
      if (pending) {
        clearTimeout(pending.timer);
        this.pending.delete(message.cmd);
        if (message.t === 'err') pending.reject(new Error(`板端拒绝 ${message.cmd}: ${message.msg || '未知错误'}`));
        else pending.resolve(message);
      }
    }
    this.emit('message', message);
  }
  async readLoop() {
    let reason = '串口数据流已结束';
    try {
      while (!this.closing) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (value) this.framer.feed(value);
      }
    } catch (error) {
      reason = `串口读取中断：${error.message}`;
    } finally {
      this.reader.releaseLock();
      this.reader = null;
      if (!this.closing) queueMicrotask(() => this.close(reason));
    }
  }
  request(command, timeoutMs = 3500) {
    if (!this.connected) return Promise.reject(new Error('请先连接板子'));
    let bytes;
    try { bytes = encodeCommand(command); } catch (error) { return Promise.reject(error); }
    if (this.pending.has(command.cmd)) {
      // A stop must still reach the wire when an earlier duty reply is missing.
      // This protocol has no request IDs, so do not claim a separate stop ACK.
      if (command.cmd === 'duty' && command.v === 0) {
        return this.enqueueWrite(command, bytes).then(() => {
          throw new Error('停止命令已发送；此前 duty 尚未回复，无法单独确认，请检查占空比遥测');
        });
      }
      return Promise.reject(new Error(`${command.cmd} 命令仍在等待回复`));
    }
    return new Promise((resolve, reject) => {
      const pending = { resolve, reject, timer: null };
      this.pending.set(command.cmd, pending);
      const fail = error => {
        if (this.pending.get(command.cmd) !== pending) return;
        clearTimeout(pending.timer);
        this.pending.delete(command.cmd);
        reject(error);
      };
      pending.timer = setTimeout(() => fail(new Error(`${command.cmd} 回复超时；不能确认板端执行结果`)), timeoutMs);
      this.enqueueWrite(command, bytes).catch(fail);
    });
  }
  enqueueWrite(command, bytes) {
    const writer = this.writer;
    const write = this.writeTail.then(async () => {
      if (!this.connected || writer !== this.writer) throw new Error('串口已断开');
      await writer.write(bytes);
      this.emit('tx', JSON.stringify(redactSecrets(command)));
    });
    this.writeTail = write.catch(() => {});
    return write;
  }
  close(reason = '已断开') {
    if (this.closeTask) return this.closeTask;
    if (!this.port) return Promise.resolve();
    this.closing = true;
    this.closeTask = (async () => {
      for (const pending of this.pending.values()) {
        clearTimeout(pending.timer);
        pending.reject(new Error(reason));
      }
      this.pending.clear();
      await Promise.allSettled([this.reader?.cancel(), this.writer?.abort()]);
      await this.readTask;
      await this.writeTail;
      this.writer?.releaseLock();
      this.writer = null;
      try { await this.port.close(); } catch (error) { this.emit('log', `关闭串口：${error.message}`); }
      this.port = null;
      this.emit('close', reason);
    })();
    return this.closeTask;
  }
}

export function wifiAddress(address, token) {
  const value = address.trim();
  if (!value) throw new Error('请填写板子的 Wi-Fi IP 地址');
  let url;
  try { url = new URL(value.includes('://') ? value : `ws://${value}/ws`); }
  catch { throw new Error('Wi-Fi 地址无效'); }
  if (!['ws:', 'wss:'].includes(url.protocol) || !url.hostname || url.username || url.password || url.hash) {
    throw new Error('请使用板子的 ws://IP/ws 地址');
  }
  if (url.pathname === '/') url.pathname = '/ws';
  if (url.pathname !== '/ws' || url.search) throw new Error('Wi-Fi 地址应为 ws://IP/ws，不包含密钥或查询参数');
  const key = token.trim();
  if (!/^[0-9a-f]{32}$/i.test(key)) throw new Error('请通过 USB 读取板子的 32 位 Wi-Fi 接入密钥');
  const endpoint = url.href;
  url.searchParams.set('token', key);
  return { endpoint, url: url.href };
}

export function wifiConfig(ssid, password) {
  const encoder = new TextEncoder();
  const nameBytes = encoder.encode(ssid).length;
  const passwordBytes = encoder.encode(password).length;
  if (!nameBytes || nameBytes > 32 || ssid.includes('\0') || password.includes('\0')) {
    throw new Error('Wi-Fi 名称需为 1～32 字节，且不能包含空字符');
  }
  if (!(passwordBytes === 0 || (passwordBytes >= 8 && passwordBytes <= 63) || /^[0-9a-f]{64}$/i.test(password))) {
    throw new Error('Wi-Fi 密码需为 8～63 字节、64 位十六进制密钥，或留空连接开放网络');
  }
  const command = { cmd: 'wifi_set', ssid, password };
  encodeCommand(command);
  return command;
}

/** Reuses command matching, frame parsing and write ordering from the USB transport. */
export class WifiConnection extends SerialConnection {
  constructor(WebSocketClass = globalThis.WebSocket) {
    super(null);
    this.WebSocketClass = WebSocketClass;
    this.socket = null;
    this.endpoint = '';
  }
  get connected() { return this.socket?.readyState === 1 && !this.closing; }
  async connect({ address, token }, timeoutMs = 6000) {
    if (this.socket) throw new Error('Wi-Fi 已连接或正在连接');
    const target = wifiAddress(address, token);
    const socket = new this.WebSocketClass(target.url);
    this.socket = socket;
    this.endpoint = target.endpoint;
    this.closing = false;
    this.closeTask = null;
    this.writeTail = Promise.resolve();
    socket.addEventListener('message', event => {
      if (socket !== this.socket || !this.connected) return;
      if (typeof event.data !== 'string' || event.data.length > 8192) {
        this.close('Wi-Fi 收到无效或过长的数据帧'); return;
      }
      this.onLine(event.data);
    });
    socket.addEventListener('close', () => {
      if (socket === this.socket && this.writer && !this.closing) this.close('Wi-Fi 连接已中断');
    });
    try {
      await new Promise((resolve, reject) => {
        const fail = () => { cleanup(); reject(new Error('Wi-Fi 连接失败，请检查 IP、接入密钥、同一局域网及浏览器本地网络权限')); };
        const open = () => { cleanup(); resolve(); };
        const cleanup = () => {
          clearTimeout(timer);
          socket.removeEventListener('open', open);
          socket.removeEventListener('error', fail);
          socket.removeEventListener('close', fail);
        };
        const timer = setTimeout(fail, timeoutMs);
        socket.addEventListener('open', open);
        socket.addEventListener('error', fail);
        socket.addEventListener('close', fail);
      });
      this.writer = {
        write: async bytes => {
          if (!this.connected || socket !== this.socket) throw new Error('Wi-Fi 已断开');
          if (socket.bufferedAmount > 65536) throw new Error('Wi-Fi 发送拥堵，命令未发送');
          socket.send(new TextDecoder().decode(bytes).trimEnd());
        }
      };
      this.emit('open', { wifi: true, endpoint: target.endpoint });
    } catch (error) {
      this.socket = null;
      socket.close();
      throw error;
    }
  }
  close(reason = 'Wi-Fi 已断开') {
    if (this.closeTask) return this.closeTask;
    const socket = this.socket;
    if (!socket) return Promise.resolve();
    this.closing = true;
    this.closeTask = (async () => {
      for (const pending of this.pending.values()) {
        clearTimeout(pending.timer);
        pending.reject(new Error(reason));
      }
      this.pending.clear();
      await this.writeTail;
      if (socket.readyState !== 3) {
        await new Promise(resolve => {
          const finish = () => { clearTimeout(timer); socket.removeEventListener('close', finish); resolve(); };
          const timer = setTimeout(finish, 500);
          socket.addEventListener('close', finish);
          socket.close();
        });
      }
      this.socket = null;
      this.writer = null;
      this.emit('close', reason);
    })();
    return this.closeTask;
  }
}

export class BoardConnection extends EventTarget {
  constructor(serial, WebSocketClass = globalThis.WebSocket) {
    super();
    this.usb = new SerialConnection(serial);
    this.wifi = new WifiConnection(WebSocketClass);
    this.active = null;
    for (const [kind, connection] of [['usb', this.usb], ['wifi', this.wifi]]) {
      connection.kind = kind;
      for (const type of ['open', 'close', 'line', 'message', 'log', 'tx']) {
        connection.addEventListener(type, event => {
          if (this.active !== connection) return;
          if (type === 'close') this.active = null;
          this.dispatchEvent(new CustomEvent(type, { detail: event.detail }));
        });
      }
    }
  }
  get connected() { return !!this.active?.connected; }
  get kind() { return this.active?.kind || ''; }
  async connect(kind, options) {
    if (this.active) throw new Error('请先停机并断开当前连接');
    const connection = kind === 'wifi' ? this.wifi : this.usb;
    this.active = connection;
    try { await connection.connect(options); }
    catch (error) { if (this.active === connection) this.active = null; throw error; }
  }
  request(command, timeoutMs) {
    return this.active ? this.active.request(command, timeoutMs) : Promise.reject(new Error('请先连接板子'));
  }
  close(reason) { return this.active?.close(reason) || Promise.resolve(); }
}
