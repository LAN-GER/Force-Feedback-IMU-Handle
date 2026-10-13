import { BoardConnection, isTelemetry, CSV_FIELDS, csvRow, encodeCommand, wifiConfig, redactSecrets } from './protocol.mjs';
import { hasFilteredImu, DemoImuKalman } from './imu-filter.mjs';
import { ImuAttitude, DemoImuAttitude, hasBoardAttitude, quaternionCssMatrix } from './imu-attitude.mjs';
import { MODE_NAMES, MODE_KEYS, ANGLE_KEYS, MODE_DESCRIPTIONS, fromRadians, toRadians,
  motorCommand, validateMotorParams, torqueFromCurrent, currentTarget } from './motor-controls.mjs';

const $ = id => document.getElementById(id);
const modeNames = MODE_NAMES;
const colors = ['#df5a30', '#19867f', '#5e6ec7'];
const session = new BoardConnection(navigator.serial);
const demoKalman = new DemoImuKalman();
const attitude = new ImuAttitude();
const demoAttitude = new DemoImuAttitude();
const state = { demo: false, demoTimer: null, demoTime: 0, demoSeq: 0, demoDuty: 0,
  demoTheta: 0, demoOmega: 0, demoIntegral: 0, demoApplied: 0,
  demoMode: 0, demoSubscribed: false, subscribed: false, lastAt: 0, frames: 0,
  rateAt: performance.now(), busy: false, latest: null, history: [], records: [],
  recordStart: 0, params: {}, dirty: new Set(), logs: [], selectedMode: 0,
  selectionDirty: false, angleUnit: 'deg', motorPending: false, controlEpoch: 0,
  filterPending: false, filterEpoch: 0, pose: null, poseValid: false, poseSource: '',
  wifiTimer: null, wifiEpoch: 0, wifiPending: false, wifi: null, savePending: false, saveEpoch: 0 };
const paramDefs = [
  ['theta_des', '目标角度 / rad'], ['duty_max', '闭环 PWM 上限', 0.001, 1],
  ['k', '阻抗刚度 K', 0], ['b', '阻抗阻尼 B', 0],
  ['theta_min', '软限位下限 / rad'], ['theta_max', '软限位上限 / rad'],
  ['enc_cpr', '输出轴每转计数', 1, 2147483647, 1], ['enc_dir', '编码器方向', 0, 1, 1, ['正向', '反向']],
  ['pid_kp', '位置 Kp', 0], ['pid_ki', '位置 Ki', 0], ['pid_kd', '位置 Kd', 0],
  ['tau_des', '目标力矩 / N·m'], ['tq_kp', '电流 Kp', 0], ['tq_ki', '电流 Ki', 0],
  ['i_max', '目标电流限幅 / A', 0.051], ['kt', '力矩系数 Kt / N·m/A', 0.000002],
  ['heater_duty', '加热占空比', 0, 1], ['telem_hz', '遥测频率 / Hz', 1, 100, 1],
  ['dob_enable', 'DOB 使能', 0, 1, 1, ['关闭', '启用']], ['dob_vm', 'DOB 电压 / V', 0.501, 19.999],
  ['dob_r', 'DOB 电阻 / Ω', 0.051, 99.999], ['dob_ke', 'DOB 反电动势系数', 0],
  ['dob_j', 'DOB 惯量 / kg·m²', 0.000000001], ['dob_b_visc', 'DOB 粘滞系数', 0],
  ['dob_tau_c', 'DOB 库仑摩擦 / N·m', 0], ['dob_fc_hz', 'DOB 滤波截止 / Hz', 1, 200]
];

function paramContainer(key) {
  if (ANGLE_KEYS.includes(key)) return 'mode-angle-fields';
  if (key === 'duty_max') return 'mode-pwm-fields';
  if (key === 'k' || key === 'b') return 'impedance-fields';
  if (key.startsWith('pid_')) return 'position-fields';
  if (['tau_des', 'tq_kp', 'tq_ki', 'i_max', 'kt'].includes(key)) return 'torque-fields';
  return key.startsWith('dob_') ? 'dob-fields' : 'param-fields';
}
function readParam(key) {
  const value = numberInput(`p-${key}`);
  return ANGLE_KEYS.includes(key) ? toRadians(value, state.angleUnit) : value;
}
function writeParam(key, value) {
  const display = ANGLE_KEYS.includes(key) ? fromRadians(value, state.angleUnit) : value;
  $(`p-${key}`).value = ANGLE_KEYS.includes(key) ? Number(display.toFixed(10)) : display;
}
function markDirty(key) {
  state.dirty.add(key); $(`p-${key}`).classList.add('dirty');
  $('param-state').textContent = `待发送 ${state.dirty.size} 项修改`;
  updateMotorDraft();
}

for (const [key, title, min, max, step, options] of paramDefs) {
  const label = document.createElement('label');
  const caption = document.createElement('span'); caption.id = `label-${key}`; caption.textContent = title;
  label.append(caption);
  const input = document.createElement(options ? 'select' : 'input');
  input.id = `p-${key}`;
  if (options) {
    const blank = document.createElement('option'); blank.value = ''; blank.textContent = '未读取'; input.append(blank);
    options.forEach((text, i) => { const option = document.createElement('option'); option.value = i; option.textContent = text; input.append(option); });
  } else {
    input.type = 'number'; input.step = step || 'any'; input.placeholder = '未读取';
    if (min !== undefined) input.min = min;
    if (max !== undefined) input.max = max;
  }
  input.addEventListener('input', () => markDirty(key));
  label.append(input); $(paramContainer(key)).append(label);
}

function selectMode(mode, userSelected = false) {
  state.selectedMode = mode;
  if (userSelected) state.selectionDirty = true;
  document.querySelectorAll('[data-mode]').forEach(button => button.setAttribute('aria-pressed', String(Number(button.dataset.mode) === mode)));
  $('mode-description').textContent = MODE_DESCRIPTIONS[mode];
  $('openloop-controls').hidden = mode !== 0; $('mode-form').hidden = mode === 0;
  $('angle-tools').hidden = mode === 3; $('mode-angle-fields').hidden = mode === 3;
  $('impedance-fields').hidden = mode !== 1; $('position-fields').hidden = mode !== 2;
  $('torque-fields').hidden = mode !== 3; $('current-tools').hidden = mode !== 3;
  $('apply-mode').textContent = `启动 / 更新${modeNames[mode]}`;
  updateAngleLabels(); updateMotorDraft(); updateConnection();
}
function updateAngleLabels() {
  const unit = state.angleUnit === 'deg' ? '°' : 'rad';
  $('label-theta_des').textContent = `${state.selectedMode === 1 ? '弹簧中心角' : '目标角度'} / ${unit}`;
  $('label-theta_min').textContent = `软限位下限 / ${unit}`;
  $('label-theta_max').textContent = `软限位上限 / ${unit}`;
}
function updateMotorDraft() {
  const keys = MODE_KEYS[state.selectedMode];
  const missing = keys.some(key => !Number.isFinite(state.params[key]));
  $('mode-draft-state').textContent = missing ? '请先读取板端参数，填写完整后启动。'
    : '当前表单为待发送值；点击启动 / 更新后，以板端回复和遥测确认。';
  if (state.selectedMode === 3) {
    try {
      const target = currentTarget(readParam('tau_des'), readParam('kt'), readParam('i_max'));
      $('torque-preview').textContent = `请求电流 ${target.requested.toFixed(4)} A；限幅后 ${target.applied.toFixed(4)} A${target.limited ? '（已超出电流限幅，板端会截断目标）' : ''}。`;
    } catch { $('torque-preview').textContent = '填写目标力矩、Kt 和电流限幅后可预览目标电流。'; }
  }
}

function syncDuty(value) {
  $('duty-input').value = value; $('duty-slider').value = value * 100;
  $('duty-preview').textContent = `${(value * 100).toFixed(0)}%`;
}
for (const group of ['acc', 'gyro']) {
  const heading = document.createElement('div'); heading.className = 'axis-row axis-heading';
  heading.innerHTML = '<span>轴</span><span>原始值</span><span class="filtered-value">卡尔曼值</span>';
  $(`${group}-values`).append(heading);
  ['X', 'Y', 'Z'].forEach((axis, i) => {
    const row = document.createElement('div'); row.className = 'axis-row';
    const label = document.createElement('span'); label.className = axis.toLowerCase(); label.textContent = axis;
    const value = document.createElement('strong'); value.id = `${group}-${i}`; value.textContent = '—';
    const filtered = document.createElement('strong'); filtered.id = `${group}-f-${i}`;
    filtered.className = 'filtered-value'; filtered.textContent = '—';
    row.append(label, value, filtered); $(`${group}-values`).append(row);
  });
}

function notice(message, type = '') { $('notice').textContent = message; $('notice').className = `notice ${type}`; }
function nvsStatus(message, status = 'idle') {
  $('nvs-status').textContent = message; $('nvs-status').dataset.state = status;
}
function log(message) {
  state.logs.push(`${new Date().toLocaleTimeString('zh-CN', { hour12: false })}  ${message}`);
  if (state.logs.length > 150) state.logs.splice(0, state.logs.length - 150);
  const nearBottom = $('log').scrollHeight - $('log').scrollTop - $('log').clientHeight < 35;
  $('log').textContent = state.logs.join('\n');
  if (nearBottom) $('log').scrollTop = $('log').scrollHeight;
}
function updateConnection() {
  const active = session.connected || state.demo;
  const wifi = $('transport').value === 'wifi';
  const canConnect = wifi ? !!globalThis.WebSocket : !!navigator.serial && isSecureContext;
  document.querySelectorAll('[data-device]').forEach(button => { button.disabled = !active || state.busy; });
  $('save').disabled ||= state.savePending;
  $('save').textContent = state.savePending ? '正在保存…' : '保存参数至 NVS';
  $('apply-mode').disabled ||= state.motorPending || MODE_KEYS[state.selectedMode].some(key => !Number.isFinite(state.params[key]));
  $('imu-filter-enable').disabled ||= state.filterPending || ![0, 1].includes(state.params.imu_kf_en);
  $('connect').hidden = active;
  $('connect').disabled = state.busy || !canConnect;
  $('connect').textContent = wifi ? '连接 Wi-Fi' : '选择串口并连接';
  $('choose-port').disabled = active || state.busy || !canConnect;
  $('choose-port').textContent = state.demo ? '演示中，请先退出演示' : session.connected ? (wifi ? 'Wi-Fi 已连接' : '串口已连接') : state.busy ? '正在选择 / 连接…' : wifi ? '连接 Wi-Fi' : '选择串口并连接';
  $('selected-port').textContent = state.demo ? '模拟数据 · 未连接板子' : session.connected ? $('port-info').textContent : wifi ? '尚未连接 Wi-Fi' : '尚未选择端口';
  $('connection-title').textContent = wifi ? 'Wi-Fi 连接' : 'USB 串口';
  $('wifi-connect-fields').hidden = !wifi;
  $('transport').disabled = active || state.busy;
  $('wifi-address').disabled = active || state.busy;
  $('wifi-token').disabled = active || state.busy;
  $('port-selection-hint').textContent = state.demo ? '退出演示模式后，可连接真实板子。' : session.connected ? '已连接所选设备；切换连接方式前请先点击「停机并断开」。' : wifi ? '通过 USB 获取 IP 和接入密钥后，可在同一局域网无线连接。' : !isSecureContext ? '请通过 http://127.0.0.1:8776 打开页面后选择端口。' : !navigator.serial ? '当前浏览器不支持串口，请使用桌面 Chrome / Edge。' : '点击按钮，在浏览器弹窗中选择板子的 COM 端口，然后确认连接。';
  const usb = session.connected && session.kind === 'usb' && !state.busy && !state.wifiPending;
  $('configure-wifi').disabled = !usb;
  $('wifi-enable').disabled = !usb || !state.wifi?.ready || !Number.isInteger(state.wifi?.running) || !!state.wifi?.busy;
  $('forget-wifi').disabled = !usb;
  $('wifi-ssid').disabled = !usb;
  $('wifi-password').disabled = !usb;
  $('read-wifi').disabled = !session.connected || state.busy || state.wifiPending;
  $('disconnect').hidden = !active;
  $('disconnect').disabled = state.busy;
  $('demo').disabled = session.connected || state.busy;
  $('demo').textContent = state.demo ? '退出演示' : '演示模式';
  $('baud').disabled = wifi || active || state.busy;
  $('connection-state').textContent = state.demo ? '模拟数据' : session.connected ? (wifi ? 'Wi-Fi 已连接' : 'USB 已连接') : '未连接';
  $('connection-state').className = `badge ${state.demo ? 'warn' : session.connected ? 'good' : ''}`;
  $('export').disabled = state.records.length === state.recordStart;
}
function setSubscribed(value) {
  state.subscribed = value;
  if (!value) { attitude.pause(); state.poseValid = false; }
  $('subscribe').textContent = value ? '已订阅' : '开始订阅';
}
function resetData() {
  state.saveEpoch++; state.savePending = false;
  nvsStatus('本次连接尚未执行 NVS 保存。');
  attitude.reset(); demoAttitude.reset(); state.pose = null; state.poseValid = false; state.poseSource = '';
  $('imu-cube').style.transform = quaternionCssMatrix([1, 0, 0, 0]);
  $('attitude-reference').textContent = '板端四元数：倾斜参考重力，航向从板端姿态初始化起累计。';
  renderAttitude();
  state.filterEpoch++; state.filterPending = false; demoKalman.reset();
  $('imu-filter-enable').checked = false;
  $('imu-filter-command').textContent = '连接并读取参数后可切换滤波。';
  $('imu-filter-status').textContent = '等待滤波数据';
  $('imu-kalman-status').textContent = '等待数据'; $('imu-kalman-status').className = 'badge';
  state.controlEpoch++; state.motorPending = false; state.selectionDirty = false;
  state.latest = null; state.lastAt = 0; state.frames = 0; state.rateAt = performance.now();
  state.history = []; state.records = []; state.recordStart = 0; state.params = {}; state.dirty.clear();
  for (const [key] of paramDefs) { $(`p-${key}`).value = ''; $(`p-${key}`).classList.remove('dirty'); }
  $('param-state').textContent = '尚未读取参数'; $('record-count').textContent = '已缓存 0 帧';
  ['seq', 'rate', 'mode', 'theta-deg', 'omega', 'counts', 'duty', 'i_meas', 'i_des', 'pid_err',
    'tau_des', 'tau_meas', 'tau_m_hat', 'tau_ext_hat'].forEach(id => { $(id).textContent = '—'; });
  ['acc', 'gyro'].forEach(key => [0, 1, 2].forEach(i => { $(`${key}-${i}`).textContent = '—'; }));
  ['acc', 'gyro'].forEach(key => [0, 1, 2].forEach(i => { $(`${key}-f-${i}`).textContent = '—'; }));
  $('theta-rad').textContent = '— rad'; $('last-frame').textContent = '尚无遥测';
  $('limit').textContent = '限幅状态 —'; $('imu-status').textContent = '等待数据'; $('imu-status').className = 'badge';
  $('imu-ids').textContent = '芯片 ID 尚未收到';
  $('diagnostics').textContent = '读取诊断可查询初始化结果，无需抓取开机日志。';
  $('spi-probe').textContent = 'IMU 初始化失败时，可探测不同 SPI 速率、模式与 MISO 弱上/下拉。';
  $('dial-needle').removeAttribute('transform');
  $('motor-control-state').textContent = '等待板端状态'; $('motor-control-state').className = 'badge';
  $('motor-command-state').textContent = '选择模式只切换表单，点击发送才生效。';
  $('current-target').value = 0; syncDuty(0); selectMode(0);
  setSubscribed(false); updateConnection(); drawCharts();
}
function renderParams(values) {
  state.params = { ...state.params, ...values };
  if ([0, 1].includes(values.imu_kf_en)) {
    if (!state.filterPending) $('imu-filter-enable').checked = values.imu_kf_en === 1;
    $('imu-filter-command').textContent = `板端参数：滤波${values.imu_kf_en ? '开启' : '关闭'}；保存至 NVS 后重启保留。`;
  } else if (![0, 1].includes(state.params.imu_kf_en)) {
    $('imu-filter-command').textContent = '当前固件未提供滤波开关，请烧录新版固件。';
  }
  for (const [key] of paramDefs) {
    if (Number.isFinite(values[key]) && !state.dirty.has(key)) writeParam(key, values[key]);
  }
  if (Number.isFinite(values.telem_hz)) $('hz').value = values.telem_hz;
  if (!state.dirty.size) $('param-state').textContent = '已收到板端参数';
  if (Number.isInteger(values.ctrl_mode) && values.ctrl_mode >= 0 && values.ctrl_mode <= 3 && !state.selectionDirty) selectMode(values.ctrl_mode);
  updateMotorDraft(); updateConnection();
}
function acceptTelemetry(data) {
  if (!isTelemetry(data)) { log('RX 遥测字段不完整，已忽略'); return; }
  const at = Date.now();
  state.latest = data; state.lastAt = at; state.frames++;
  const attitudeRevision = attitude.revision;
  const pose = attitude.update(data.quat, hasBoardAttitude(data) && state.subscribed, data.seq);
  if (attitude.revision !== attitudeRevision) $('attitude-reference').textContent = '板端采样已重启，姿态预览参考已重置。';
  state.poseValid = !!pose;
  if (pose) {
    state.pose = pose; state.poseSource = `${state.demo ? '模拟' : '板端'}四元数 · ${data.quat_kf ? '卡尔曼六轴' : '原始六轴'}`;
    $('imu-cube').style.transform = quaternionCssMatrix(pose.quaternion);
  }
  const record = { at, source: state.demo ? 'demo' : session.kind, data };
  state.history.push(record);
  while (state.history.length && (state.history[0].at < at - 10000 || state.history.length > 2200)) state.history.shift();
  state.records.push(record);
  if (state.records.length - state.recordStart > 30000) state.recordStart++;
  if (state.recordStart > 5000) { state.records = state.records.slice(state.recordStart); state.recordStart = 0; }
  $('export').disabled = false;
}
function displayNumber(id, value, precision = 4) { $(id).textContent = Number.isFinite(value) ? value.toFixed(precision) : '—'; }
function renderAttitude() {
  const fresh = (session.connected || state.demo) && state.subscribed && Date.now() - state.lastAt < 2000;
  const live = fresh && state.poseValid && !!state.pose;
  $('attitude-status').textContent = fresh && !Array.isArray(state.latest?.quat) ? '固件未提供四元数 · 请更新'
    : !state.pose ? (fresh ? '等待有效姿态' : '等待数据')
    : !fresh ? '历史姿态 · 已暂停' : !state.poseValid ? 'IMU 无效 · 已暂停'
    : state.demo ? '模拟姿态' : '实时姿态';
  $('attitude-status').className = `badge ${live ? 'good' : state.pose ? 'warn' : ''}`;
  $('attitude-source').textContent = state.pose ? state.poseSource : '等待板端四元数 [w,x,y,z]';
  $('attitude-zero').disabled = !live;
  document.querySelector('.cube-scene').classList.toggle('paused', !live);
  for (const key of ['roll', 'pitch', 'yaw']) displayNumber(`attitude-${key}`, state.pose?.[key], 1);
  if (state.pose) document.querySelector('.cube-scene').setAttribute('aria-label',
    `${live ? '实时' : '已暂停'} IMU 方块，横滚 ${state.pose.roll.toFixed(1)} 度，俯仰 ${state.pose.pitch.toFixed(1)} 度，相对航向 ${state.pose.yaw.toFixed(1)} 度`);
  else document.querySelector('.cube-scene').setAttribute('aria-label', '等待 IMU 姿态数据的三维方块');
}
function chipId(value) { return Number.isInteger(value) && value >= 0 && value <= 255 ? `0x${value.toString(16).toUpperCase().padStart(2, '0')}` : '—'; }
function renderChipIds(acc, gyro) {
  $('imu-ids').textContent = `Acc ${chipId(acc)} / Gyro ${chipId(gyro)}（正常 0x1E / 0x0F）`;
}
function renderDiagnostics(data) {
  renderChipIds(data.acc_id, data.gyro_id);
  const resetNames = ['未知', '上电', '外部复位', '软件复位', '异常 / panic', '中断看门狗',
    '任务看门狗', '其他看门狗', '深睡眠唤醒', '欠压', 'SDIO', 'USB', 'JTAG', 'eFuse', '电源毛刺', 'CPU 锁死'];
  const uptime = Number.isFinite(data.uptime_ms) ? `${(data.uptime_ms / 1000).toFixed(1)} s` : '—';
  $('diagnostics').textContent = `${state.demo ? '模拟诊断；' : ''}初始化：${data.imu_init ?? '—'}；Acc SPI：${data.acc_spi ?? '—'}；Gyro SPI：${data.gyro_spi ?? '—'}；运行 ${uptime}；复位原因：${resetNames[data.reset_reason] ?? data.reset_reason ?? '—'}；固件 ${data.firmware_sha ?? '—'}`;
  if (Number.isFinite(data.telem_hz) && Number.isFinite(data.telem_work_us)) {
    const max = Number.isFinite(data.telem_work_max_us) ? `${(data.telem_work_max_us / 1000).toFixed(2)} ms` : '—';
    $('diagnostics').textContent += `\n遥测：配置 ${data.telem_hz} Hz；最近组包/发送 ${(data.telem_work_us / 1000).toFixed(2)} ms；最高 ${max}；累计跳过 ${data.telem_skipped ?? '—'} 个发送通知。`;
    const stages = [['快照', 'telem_snapshot_us'], ['组包', 'telem_format_us'],
      ['USB 写入', 'telem_usb_us'], ['Wi-Fi 入队', 'telem_wifi_us']];
    if (stages.every(([, key]) => Number.isFinite(data[key]))) {
      $('diagnostics').textContent += `\n分段耗时：${stages.map(([label, key]) => `${label} ${(data[key] / 1000).toFixed(2)} ms`).join('；')}；组包失败 ${data.telem_format_errors ?? '—'} 次。`;
    }
    if (data.telem_hz > 0 && data.telem_work_us > 1000000 / data.telem_hz) $('diagnostics').textContent += ' 最近一次测量超过配置周期，可能跳帧。';
  }
}
function renderProbe(data) {
  if (!Array.isArray(data.cases)) {
    $('spi-probe').textContent = '探测回复缺少结果，无法判断。';
    return;
  }
  const raw = values => Array.isArray(values) ? values.map(chipId).join(' ') : '—';
  const lines = [`${state.demo ? '模拟探测；' : ''}空闲 MISO：弱上拉 ${data.miso_up ?? '—'} / 弱下拉 ${data.miso_down ?? '—'}（测试错误 ${data.miso_err ?? '—'}）`];
  if (Number.isInteger(data.miso_input)) {
    lines.push(`GPIO6 配置：输入使能 ${data.miso_input} / OE 寄存器 ${data.miso_oe ?? '—'} / 外设控制输出使能 ${data.miso_periph ?? '—'}；OE 寄存器为 1 不能单独说明 MISO 正在输出。`);
  }
  for (const c of data.cases) {
    const rate = Number.isFinite(c.hz) ? `${c.hz / 1000} kHz` : '—';
    lines.push(`Mode${c.mode} / ${rate}：Acc [${raw(c.acc)}] / Gyro [${raw(c.gyro)}]；SPI 错误 ${c.ae ?? '—'} / ${c.ge ?? '—'}`);
  }
  lines.push('原始字节：Acc 第 3 字节应为 0x1E，Gyro 第 2 字节应为 0x0F；SPI 错误 0 表示传输成功。');
  if (data.miso_err !== 0) {
    lines.push('空闲电平测试未完成，不能用 −1 判断线路电平或短路。259 = ESP_ERR_INVALID_STATE；旧版探测可能误拒绝 SPI 外设控制的 OE 配置。');
  } else if (data.miso_up === 0 && data.miso_down === 0) {
    lines.push('空闲 MISO 疑似被固定拉低，需检查 SDO/MISO 对地阻值和焊接。');
  } else if (data.miso_up === 1 && data.miso_down === 0 && data.miso_err === 0) {
    lines.push('空闲 MISO 能跟随弱上/下拉；这不能单独证明芯片到 MCU 的连接正常。');
  }
  $('spi-probe').textContent = lines.join('\n');
}
function renderTelemetry() {
  const data = state.latest;
  if (!data) return;
  if (Number.isInteger(data.imu_acc_id) || Number.isInteger(data.imu_gyro_id)) renderChipIds(data.imu_acc_id, data.imu_gyro_id);
  ['acc', 'gyro'].forEach(key => [0, 1, 2].forEach(i => displayNumber(`${key}-${i}`, data.imu_ok ? data[key][i] : undefined, key === 'acc' ? 4 : 2)));
  ['acc', 'gyro'].forEach(key => [0, 1, 2].forEach(i => displayNumber(`${key}-f-${i}`, hasFilteredImu(data) ? data[`${key}_f`][i] : undefined, key === 'acc' ? 4 : 2)));
  const imuFresh = (session.connected || state.demo) && state.subscribed && Date.now() - state.lastAt < 2000;
  $('imu-kalman-status').textContent = !imuFresh ? '历史数据'
    : !data.imu_ok ? 'IMU 数据无效'
    : ![0, 1].includes(data.imu_kf_en) ? '未提供滤波数据'
    : !data.imu_kf_en ? '滤波已关闭'
    : hasFilteredImu(data) ? (state.demo ? '模拟滤波波形' : '正在接收') : '等待有效样本';
  $('imu-kalman-status').className = `badge ${imuFresh && hasFilteredImu(data) ? 'good' : 'warn'}`;
  $('imu-filter-status').textContent = !imuFresh ? '历史数据 · 滤波状态以恢复遥测后的新帧为准'
    : !data.imu_ok ? 'IMU 读取失败 · 滤波数据不可用'
    : ![0, 1].includes(data.imu_kf_en) ? '当前遥测未提供滤波数据'
    : !data.imu_kf_en ? '滤波已关闭 · 当前只显示原始数据'
    : hasFilteredImu(data) ? `${state.demo ? '模拟' : '板端'}卡尔曼滤波运行中 · 原始值与滤波值来自同一帧`
    : '滤波已开启 · 等待有效的新样本';
  displayNumber('theta-deg', data.theta * 180 / Math.PI, 2);
  $('theta-rad').textContent = Number.isFinite(data.theta) ? `${data.theta.toFixed(5)} rad` : '— rad';
  if (Number.isFinite(data.theta)) $('dial-needle').setAttribute('transform', `rotate(${data.theta * 180 / Math.PI % 360} 80 80)`);
  displayNumber('omega', data.omega, 3); displayNumber('counts', data.counts, 0);
  $('duty').textContent = Number.isFinite(data.duty) ? `${(data.duty * 100).toFixed(1)}%` : '—';
  ['i_meas', 'i_des', 'pid_err', 'tau_des', 'tau_meas', 'tau_m_hat', 'tau_ext_hat'].forEach(key => displayNumber(key, data[key], key.startsWith('tau') ? 6 : 4));
  $('seq').textContent = data.seq;
  $('mode').textContent = modeNames[data.mode] || '未知模式';
  $('limit').textContent = data.lim ? '限幅已触发' : data.lim === 0 ? '未触发限幅' : '限幅状态 —';
  $('record-count').textContent = `已缓存 ${state.records.length - state.recordStart} 帧`;
  const fresh = (session.connected || state.demo) && state.subscribed && Date.now() - state.lastAt < 2000;
  const stopped = data.mode === 0 && Math.abs(data.duty) < .0001;
  $('motor-control-state').textContent = !fresh ? `历史状态：${modeNames[data.mode] || '未知'}`
    : stopped ? '板端待机 · PWM 0' : `板端：${modeNames[data.mode] || '未知'}`;
  $('motor-control-state').className = `badge ${!fresh ? 'warn' : stopped ? '' : 'good'}`;
}

function plot(canvas, series, minRange, emptyText = '等待遥测数据') {
  series = series.map((item, i) => typeof item === 'function' ? { get: item, color: colors[i] } : item);
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = devicePixelRatio || 1;
  const width = rect.width, height = rect.height;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height);
  const points = state.history;
  const at = state.lastAt || Date.now();
  let range = minRange;
  let hasValues = false;
  for (const record of points) for (const { get } of series) {
    const value = get(record.data);
    if (Number.isFinite(value)) { hasValues = true; range = Math.max(range, Math.abs(value) * 1.12); }
  }
  const left = 47, right = width - 12, top = 15, bottom = height - 24;
  const y = value => top + (range - value) / (range * 2) * (bottom - top);
  ctx.font = '9px Consolas, monospace'; ctx.lineWidth = 1;
  for (const value of [-range, 0, range]) {
    ctx.strokeStyle = '#e4e9e2'; ctx.beginPath(); ctx.moveTo(left, y(value)); ctx.lineTo(right, y(value)); ctx.stroke();
    ctx.fillStyle = '#7d8d84'; ctx.fillText(Math.abs(value) < .01 ? value.toExponential(1) : value.toFixed(range >= 10 ? 0 : 2), 4, y(value) + 3);
  }
  ctx.fillStyle = '#8b978f'; ctx.fillText('−10 s', left, height - 7); ctx.fillText('现在', right - 23, height - 7);
  series.forEach(({ get, color, dash = [] }) => {
    ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.lineWidth = 1.6; ctx.beginPath(); let previousAt = 0;
    for (const record of points) {
      const value = get(record.data); if (!Number.isFinite(value)) { previousAt = 0; continue; }
      const x = left + (record.at - at + 10000) / 10000 * (right - left);
      if (!previousAt || record.at - previousAt > 700) ctx.moveTo(x, y(value)); else ctx.lineTo(x, y(value));
      previousAt = record.at;
    }
    ctx.stroke();
  });
  ctx.setLineDash([]);
  if (!hasValues) { ctx.fillStyle = '#9aa69d'; ctx.textAlign = 'center'; ctx.fillText(emptyText, width / 2, height / 2); ctx.textAlign = 'left'; }
}
function drawCharts() {
  for (const group of ['acc', 'gyro']) {
    const compare = $('imu-filter-compare').checked;
    const raw = [0, 1, 2].map(i => ({ get: data => data.imu_ok ? data[group][i] : undefined,
      color: colors[i], dash: compare ? [4, 3] : [] }));
    const filtered = compare ? [0, 1, 2].map(i => ({ get: data => hasFilteredImu(data) ? data[`${group}_f`][i] : undefined,
      color: colors[i] })) : [];
    plot($(`${group}-chart`), [...raw, ...filtered], group === 'acc' ? 1.2 : 10);
    const onlyFiltered = [0, 1, 2].map(i => ({
      get: data => hasFilteredImu(data) ? data[`${group}_f`][i] : undefined,
      color: colors[i]
    }));
    plot($(`${group}-filtered-chart`), onlyFiltered, group === 'acc' ? 1.2 : 10, '等待有效的卡尔曼滤波数据');
  }
  plot($('current-chart'), [data => data.i_meas, data => data.i_des], .1);
  plot($('position-chart'), [data => fromRadians(data.theta, 'deg'),
    data => data.mode === 2 && Number.isFinite(data.pid_err) ? fromRadians(data.theta + data.pid_err, 'deg') : undefined], 5);
}

async function command(value) {
  encodeCommand(value);
  $('command-state').textContent = `${value.cmd} 等待板端回复…`;
  try {
    const reply = state.demo ? await demoCommand(value) : await session.request(value);
    $('command-state').textContent = `${value.cmd} ${state.demo ? '模拟' : '板端'}已确认`;
    if (reply.live) renderParams(reply.live);
    if (value.cmd === 'sub') setSubscribed(true);
    if (value.cmd === 'unsub') setSubscribed(false);
    return reply;
  } catch (error) {
    $('command-state').textContent = error.message;
    throw error;
  }
}
function action(id, callback) {
  $(id).addEventListener('click', async () => {
    try { await callback(); } catch (error) { log(`ERROR ${error.message}`); notice(error.message, 'error'); }
  });
}
function form(id, callback) {
  $(id).addEventListener('submit', async event => {
    event.preventDefault();
    try { await callback(); } catch (error) { log(`ERROR ${error.message}`); notice(error.message, 'error'); }
  });
}
function numberInput(id) {
  const input = $(id);
  if (!input.value.trim() || !input.checkValidity() || !Number.isFinite(Number(input.value))) throw new Error('请填写有效且在范围内的数值');
  return Number(input.value);
}

function clearSentEdits(patch, draft) {
  for (const key of Object.keys(patch)) {
    if (!$(`p-${key}`)) continue;
    // Compare displayed snapshots, so in-flight edits and unit conversions survive.
    if ($(`p-${key}`).value === draft[key] && draft.unit === state.angleUnit) {
      state.dirty.delete(key); $(`p-${key}`).classList.remove('dirty');
    }
  }
}
function draftSnapshot(keys) {
  return Object.fromEntries([['unit', state.angleUnit], ...keys.map(key => [key, $(`p-${key}`).value])]);
}

async function sendMotor(mode) {
  if (state.motorPending) throw new Error('电机控制命令仍在等待回复');
  const patch = Object.fromEntries(MODE_KEYS[mode].map(key => [key, readParam(key)]));
  const value = motorCommand(mode, patch, mode === 0 ? numberInput('duty-input') : 0);
  encodeCommand(value);
  const draft = draftSnapshot(MODE_KEYS[mode]);
  const epoch = state.controlEpoch;
  state.motorPending = true; updateConnection();
  $('motor-command-state').textContent = `${modeNames[mode]}命令已提交，等待板端回复…`;
  try {
    const reply = await command(value);
    if (epoch !== state.controlEpoch) return;
    if (reply.live && reply.live.ctrl_mode !== mode) throw new Error('板端回复的控制模式与请求不一致，请读取参数');
    clearSentEdits(patch, draft);
    if (mode === 0) state.params.ctrl_mode = 0;
    if (state.selectedMode === mode) state.selectionDirty = false;
    if (reply.live) renderParams(reply.live);
    $('motor-command-state').textContent = `${modeNames[mode]}${state.demo ? '模拟' : '板端'}已确认；运行状态以遥测为准。`;
    $('param-state').textContent = state.dirty.size ? `待发送 ${state.dirty.size} 项修改` : '板端已确认修改';
  } catch (error) {
    if (epoch === state.controlEpoch) $('motor-command-state').textContent = `控制未确认：${error.message}`;
    throw error;
  } finally {
    if (epoch === state.controlEpoch) state.motorPending = false;
    updateConnection();
  }
}

session.addEventListener('open', event => {
  resetData();
  const info = event.detail;
  if (!info.wifi) {
    state.wifi = null; $('wifi-enable').checked = false;
    $('wifi-switch-state').textContent = '通过 USB 读取状态后可切换';
    for (const id of ['wifi-address', 'wifi-token', 'wifi-ssid', 'wifi-password']) $(id).value = '';
    $('wifi-status').textContent = 'USB 已连接，可配置 Wi-Fi 或读取这块板子的网络地址与接入密钥。';
  }
  $('port-info').textContent = info.wifi ? info.endpoint : info.usbVendorId !== undefined ? `USB ${info.usbVendorId.toString(16).padStart(4, '0')}:${(info.usbProductId || 0).toString(16).padStart(4, '0')}` : '已选择串口';
  log(info.wifi ? 'Wi-Fi 连接已打开' : 'USB 串口已打开'); notice('已连接，正在读取参数并订阅遥测。'); updateConnection();
});
session.addEventListener('close', event => {
  if (state.savePending || $('nvs-status').dataset.state === 'unknown') nvsStatus('保存结果未确认：连接已断开，无法确认板端是否已写入 NVS。', 'unknown');
  state.saveEpoch++; state.savePending = false;
  clearTimeout(state.wifiTimer); state.wifiEpoch++; state.wifiPending = false;
  state.filterEpoch++; state.filterPending = false;
  state.controlEpoch++; state.motorPending = false;
  $('motor-command-state').textContent = '连接已断开，无法确认电机停机。';
  state.busy = false; setSubscribed(false); updateConnection();
  $('port-info').textContent = $('transport').value === 'wifi' ? 'Wi-Fi / WebSocket' : 'USB-Serial/JTAG';
  notice(`${event.detail}。页面已停止收发，拔线时无法确认电机是否停止。`, 'error'); log(event.detail);
});
session.addEventListener('tx', event => log(`TX ${event.detail}`));
session.addEventListener('log', event => log(event.detail));
session.addEventListener('line', event => {
  const item = event.detail;
  if (item.kind === 'invalid') log(`RX JSON 解析失败：${item.text}`);
  else if (item.kind === 'log' || item.value.t !== 'telem' || $('show-telem').checked) log(`RX ${item.kind === 'json' ? JSON.stringify(redactSecrets(item.value)) : item.text}`);
});
session.addEventListener('message', event => {
  const data = event.detail;
  if (data.t === 'telem') acceptTelemetry(data);
  if (data.live) renderParams(data.live);
  if (data.diag && typeof data.diag === 'object') renderDiagnostics(data.diag);
  if (data.probe && typeof data.probe === 'object') renderProbe(data.probe);
  if (data.wifi && typeof data.wifi === 'object') renderWifi(data.wifi);
  if (data.t === 'err') notice(`板端错误：${data.cmd} / ${data.msg || '未知错误'}`, 'error');
});

async function connectBoard() {
  state.busy = true; updateConnection();
  try {
    const transport = $('transport').value;
    await session.connect(transport, transport === 'wifi'
      ? { address: $('wifi-address').value, token: $('wifi-token').value } : Number($('baud').value));
    state.busy = false; updateConnection();
    // Launch independently: a missing get response must not delay telemetry.
    const results = await Promise.allSettled([command({ cmd: 'get' }), command({ cmd: 'sub' })]);
    const errors = results.filter(result => result.status === 'rejected');
    if (errors.length) notice(errors.map(result => result.reason.message).join('；'), 'error');
    else notice('板端已确认订阅。电机控制保持板端当前状态。');
  } catch (error) {
    if (error.name === 'NotFoundError') notice('已取消选择串口。');
    else throw new Error(`连接失败：${error.message}${$('transport').value === 'usb' ? '。请关闭串口 monitor 后重试。' : ''}`);
  } finally { state.busy = false; updateConnection(); }
}
action('connect', connectBoard);
action('choose-port', connectBoard);
$('transport').addEventListener('change', updateConnection);
action('disconnect', async () => {
  if (state.demo) { stopDemo(); return; }
  state.busy = true; updateConnection();
  const errors = [];
  try {
    for (const value of [{ cmd: 'duty', v: 0 }, { cmd: 'heater', v: 0 }, { cmd: 'unsub' }]) {
      try { await command(value); } catch (error) { errors.push(error.message); }
    }
  } finally {
    await session.close('已主动断开'); state.busy = false; updateConnection();
    notice(errors.length ? `已断开，但部分命令未确认：${errors.join('；')}` : `电机停止、加热关闭和取消订阅均已收到板端确认，${$('transport').value === 'wifi' ? 'Wi-Fi 连接' : '串口'}已释放。`, errors.length ? 'error' : '');
  }
});
action('read-params', () => command({ cmd: 'get' }));

function renderWifi(wifi) {
  state.wifi = wifi;
  $('wifi-enable').checked = wifi.enabled === 1;
  if (wifi.ssid && !$('wifi-ssid').value) $('wifi-ssid').value = wifi.ssid;
  if (typeof wifi.token === 'string') $('wifi-token').value = wifi.token;
  if (wifi.ws_url) $('wifi-address').value = wifi.ws_url;
  else if (!wifi.enabled) $('wifi-address').value = '';
  const supported = Number.isInteger(wifi.running);
  $('wifi-switch-state').textContent = !supported ? '当前固件未提供 Wi-Fi 开关，请更新固件'
    : wifi.error ? `板端操作失败：${wifi.error}`
    : wifi.busy ? (wifi.enabled ? '正在启动无线服务…' : '正在关闭并释放资源…')
    : wifi.enabled ? 'Wi-Fi 已启用 · 重启后保留' : wifi.running ? '无线资源尚未释放，请再次关闭'
    : 'Wi-Fi 已关闭 · 无线驱动和网络服务已释放';
  $('wifi-status').textContent = !wifi.ready ? '当前固件的 Wi-Fi 服务不可用，请查看板端启动日志或烧录当前固件。'
    : wifi.error ? `Wi-Fi 操作失败：${wifi.error}。可通过 USB 再次切换开关重试。`
    : wifi.busy && !wifi.enabled ? '正在关闭 Wi-Fi，等待板端释放资源。'
    : wifi.connected ? `已连接 ${wifi.ssid} · IP ${wifi.ip} · ${wifi.ws_url}。停机并断开后，可切换到 Wi-Fi。`
    : wifi.enabled ? `正在连接 ${wifi.ssid}，等待获取 IP${wifi.reason ? `；最近断开原因 ${wifi.reason}` : ''}。`
    : wifi.ssid ? `Wi-Fi 已关闭，已保留 ${wifi.ssid} 的配置；打开开关即可连接。`
    : 'Wi-Fi 已关闭。请先通过 USB 保存 2.4 GHz 网络配置，再打开开关。';
  updateConnection();
}

function pollWifi(attempt = 0, epoch = ++state.wifiEpoch) {
  clearTimeout(state.wifiTimer);
  state.wifiTimer = setTimeout(async () => {
    if (epoch !== state.wifiEpoch || !session.connected || session.kind !== 'usb') return;
    if (state.wifiPending) { pollWifi(attempt, epoch); return; }
    state.wifiPending = true; updateConnection();
    try {
      const reply = await command({ cmd: 'wifi_get' });
      if (epoch !== state.wifiEpoch) return;
      const waiting = reply.wifi?.busy || (reply.wifi?.enabled && !reply.wifi.connected && !reply.wifi.error);
      if (waiting && attempt < 19) pollWifi(attempt + 1, epoch);
      else if (waiting) $('wifi-status').textContent += ' 操作尚未完成，请再次读取状态确认。';
    } catch (error) {
      if (epoch === state.wifiEpoch) $('wifi-status').textContent = `Wi-Fi 状态读取失败：${error.message}`;
    } finally {
      if (epoch === state.wifiEpoch) { state.wifiPending = false; updateConnection(); }
    }
  }, 1500);
}

form('wifi-config-form', async () => {
  if (!session.connected || session.kind !== 'usb' || state.demo) throw new Error('请先通过 USB 连接板子进行配网');
  const value = wifiConfig($('wifi-ssid').value, $('wifi-password').value);
  state.wifiPending = true; updateConnection();
  clearTimeout(state.wifiTimer); state.wifiEpoch++;
  try {
    const reply = await command(value);
    if (!reply.wifi) throw new Error('固件未提供 Wi-Fi 配网回复，请更新固件');
    $('wifi-password').value = '';
    renderWifi(reply.wifi);
    if (reply.wifi.busy || (reply.wifi.enabled && !reply.wifi.connected)) pollWifi();
    notice(reply.wifi.enabled ? 'Wi-Fi 配置已保存，正在重新连接。' : 'Wi-Fi 配置已保存；打开「启用板端 Wi-Fi」开关后连接。');
  } finally { state.wifiPending = false; updateConnection(); }
});

action('read-wifi', async () => {
  state.wifiPending = true; updateConnection();
  try {
    const reply = await command({ cmd: 'wifi_get' });
    if (!reply.wifi) throw new Error('固件未提供 Wi-Fi 状态，请更新固件');
    renderWifi(reply.wifi);
    if (session.kind === 'usb' && (reply.wifi.busy || (reply.wifi.enabled && !reply.wifi.connected && !reply.wifi.error))) pollWifi();
  } finally { state.wifiPending = false; updateConnection(); }
});

document.querySelector('.wifi-provision').addEventListener('toggle', event => {
  if (event.target.open && session.connected && session.kind === 'usb' && !state.wifi && !state.wifiPending) $('read-wifi').click();
});
$('wifi-enable').addEventListener('change', async () => {
  const requested = $('wifi-enable').checked ? 1 : 0;
  if (!session.connected || session.kind !== 'usb' || state.demo) {
    $('wifi-enable').checked = state.wifi?.enabled === 1;
    notice('请通过 USB 连接板子后切换 Wi-Fi', 'error'); return;
  }
  clearTimeout(state.wifiTimer); state.wifiEpoch++;
  state.wifiPending = true; updateConnection();
  try {
    const reply = await command({ cmd: 'wifi_enable', v: requested });
    if (reply.wifi?.enabled !== requested) throw new Error('板端未确认 Wi-Fi 开关，请读取状态确认');
    renderWifi(reply.wifi);
    if (reply.wifi.busy || (requested && !reply.wifi.connected)) pollWifi();
    notice(requested ? 'Wi-Fi 开启请求已保存，等待联网。' : 'Wi-Fi 关闭请求已保存，等待释放资源；账号密码保留。');
  } catch (error) {
    $('wifi-enable').checked = state.wifi?.enabled === 1;
    $('wifi-switch-state').textContent = '开关操作未确认；请读取 Wi-Fi 状态确认当前状态';
    notice(`Wi-Fi 开关未确认：${error.message}`, 'error');
  } finally { state.wifiPending = false; updateConnection(); }
});

action('forget-wifi', async () => {
  if (!session.connected || session.kind !== 'usb') throw new Error('只能通过 USB 清除 Wi-Fi 配置');
  state.wifiPending = true; updateConnection();
  clearTimeout(state.wifiTimer); state.wifiEpoch++;
  try {
    const reply = await command({ cmd: 'wifi_forget' });
    if (!reply.wifi || reply.wifi.enabled) throw new Error('板端未确认清除 Wi-Fi 配置');
    for (const id of ['wifi-ssid', 'wifi-password', 'wifi-address', 'wifi-token']) $(id).value = '';
    renderWifi(reply.wifi);
    if (reply.wifi.busy) pollWifi();
    notice('Wi-Fi 配置已清除，接入密钥已更换。');
  } finally { state.wifiPending = false; updateConnection(); }
});
action('attitude-zero', () => {
  if (!state.poseValid || !state.subscribed || Date.now() - state.lastAt >= 2000 || !(session.connected || state.demo)) throw new Error('需要有效的实时 IMU 姿态');
  if (!attitude.zero()) return;
  state.pose = attitude.pose();
  $('imu-cube').style.transform = quaternionCssMatrix(state.pose.quaternion);
  $('attitude-reference').textContent = '已以当前姿态归零；后续角度为相对此姿态的转动。';
  renderAttitude();
});
$('imu-filter-enable').addEventListener('change', async () => {
  const requested = $('imu-filter-enable').checked ? 1 : 0, epoch = state.filterEpoch;
  state.filterPending = true; updateConnection();
  $('imu-filter-command').textContent = '正在切换板端滤波，等待回复…';
  try {
    const reply = await command({ cmd: 'imu_filter', v: requested });
    if (epoch !== state.filterEpoch) return;
    if (reply.live?.imu_kf_en !== requested) throw new Error('板端未确认请求的滤波状态');
    $('imu-filter-enable').checked = requested === 1;
    $('imu-filter-command').textContent = `板端已确认滤波${requested ? '开启' : '关闭'}；保存至 NVS 后重启保留。`;
  } catch (error) {
    if (epoch === state.filterEpoch) {
      $('imu-filter-enable').checked = state.params.imu_kf_en === 1;
      $('imu-filter-command').textContent = `滤波切换未确认：${error.message}；请读取参数确认当前状态。`;
      notice(error.message, 'error'); log(error.message);
    }
  } finally {
    if (epoch === state.filterEpoch) state.filterPending = false;
    updateConnection();
  }
});
function updateImuComparison() {
  const compare = $('imu-filter-compare').checked;
  document.querySelector('.imu-panel').classList.toggle('raw-only', !compare);
  $('imu-filter-legend').textContent = compare
    ? '同色为同一轴：虚线是原始值，实线是卡尔曼值。滤波在板端运行，姿态预览由网页计算。'
    : '当前显示原始数据。对比开关只改变显示，板端滤波由上方开关控制。';
  drawCharts();
}
$('imu-filter-compare').addEventListener('change', updateImuComparison);
action('read-diag', async () => {
  try {
    const reply = await command({ cmd: 'diag' });
    if (state.demo) renderDiagnostics(reply.diag);
  } catch (error) {
    $('diagnostics').textContent = '诊断未确认；旧固件可能不支持 diag，需烧录当前固件。';
    throw error;
  }
});
action('probe-spi', async () => {
  $('spi-probe').textContent = '正在探测 SPI…';
  try {
    const reply = await command({ cmd: 'imu_probe' });
    if (state.demo) renderProbe(reply.probe);
  } catch (error) {
    $('spi-probe').textContent = `探测未确认：${error.message}。仅支持当前固件且 IMU 初始化失败的状态。`;
    throw error;
  }
});
action('subscribe', () => command({ cmd: 'sub' }));
action('unsubscribe', () => command({ cmd: 'unsub' }));
action('set-hz', async () => {
  const hz = Number($('hz').value);
  const reply = await command({ cmd: 'set', telem_hz: hz });
  if (reply.live?.telem_hz !== hz) throw new Error('板端未确认请求的遥测频率，请读取参数确认');
  notice(state.demo ? `模拟遥测频率已设为 ${hz} Hz。`
    : `当前遥测频率已设为 ${hz} Hz；重启保留需点击「保存参数至 NVS」。保存会包含当前控制模式、目标和加热参数。`);
  if (!state.savePending) nvsStatus(state.demo ? '模拟频率已修改，未写入真实板子。'
    : `遥测频率 ${hz} Hz 已临时生效；要重启保留，请点击上方保存按钮。`, 'idle');
});
action('stop', async () => {
  state.controlEpoch++; state.motorPending = false; state.selectionDirty = true;
  syncDuty(0); updateConnection();
  $('motor-command-state').textContent = '正在发送停机命令…';
  try {
    await command({ cmd: 'duty', v: 0 });
    state.params.ctrl_mode = 0;
    $('motor-command-state').textContent = '板端已确认停止命令；检查遥测 PWM 是否为 0。';
  } catch (error) {
    $('motor-command-state').textContent = `停机未确认：${error.message}`;
    throw error;
  }
});
action('zero', () => command({ cmd: 'zero' }));
action('save', async () => {
  if (state.savePending) return;
  const epoch = state.saveEpoch, demo = state.demo;
  state.savePending = true; updateConnection();
  nvsStatus(demo ? '正在模拟保存…' : '正在保存到板端 NVS，等待板端确认…', 'pending');
  try {
    await command({ cmd: 'save' });
    if (epoch !== state.saveEpoch) return;
    const time = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    const message = demo ? '模拟保存完成，未写入真实板子。'
      : `上次保存成功 · ${time}：板端已确认写入 NVS，保存的参数在重启后恢复。`;
    nvsStatus(message, demo ? 'idle' : 'success'); notice(message);
  } catch (error) {
    if (epoch !== state.saveEpoch) return;
    const rejected = error.message.includes('板端拒绝');
    nvsStatus(`${rejected ? '保存失败' : '保存结果未确认'}：${error.message}`, rejected ? 'error' : 'unknown');
    throw error;
  } finally {
    if (epoch === state.saveEpoch) { state.savePending = false; updateConnection(); }
  }
});
action('clear-log', () => { state.logs = []; $('log').textContent = ''; });
form('duty-form', () => sendMotor(0));
form('mode-form', () => sendMotor(state.selectedMode));
document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => selectMode(Number(button.dataset.mode), true)));
$('duty-slider').addEventListener('input', () => syncDuty(Number($('duty-slider').value) / 100));
$('duty-input').addEventListener('input', () => {
  const value = Number($('duty-input').value);
  if (Number.isFinite(value) && Math.abs(value) <= 1) { $('duty-slider').value = value * 100; $('duty-preview').textContent = `${(value * 100).toFixed(0)}%`; }
});
action('reverse-duty', () => syncDuty(-numberInput('duty-input')));
$('angle-unit').addEventListener('change', () => {
  try {
    const angles = ANGLE_KEYS.filter(key => $(`p-${key}`).value.trim()).map(key => [key, readParam(key)]);
    state.angleUnit = $('angle-unit').value;
    for (const [key, value] of angles) writeParam(key, value);
    updateAngleLabels(); updateMotorDraft();
  } catch (error) { $('angle-unit').value = state.angleUnit; notice(error.message, 'error'); }
});
action('use-current-angle', () => {
  if (!state.subscribed || Date.now() - state.lastAt >= 2000 || !Number.isFinite(state.latest?.theta)) throw new Error('需要正在接收的编码器遥测，请先恢复订阅');
  writeParam('theta_des', state.latest.theta); markDirty('theta_des');
});
action('convert-current', () => {
  writeParam('tau_des', torqueFromCurrent(numberInput('current-target'), readParam('kt'))); markDirty('tau_des');
});
form('heater-form', () => command({ cmd: 'heater', v: numberInput('heater-input') }));
form('raw-form', () => command(JSON.parse($('raw-input').value)));
form('params-form', async () => {
  const patch = {};
  for (const key of state.dirty) patch[key] = readParam(key);
  if (!Object.keys(patch).length) throw new Error('请先修改需要发送的参数');
  const next = { ...state.params, ...patch };
  validateMotorParams(next);
  const commandValue = { cmd: 'set', ...patch };
  encodeCommand(commandValue); // Reject oversized batches before any command is sent.
  const draft = draftSnapshot(Object.keys(patch));
  const reply = await command(commandValue);
  // Keep edits typed while the request was in flight.
  clearSentEdits(patch, draft);
  if (reply.live) renderParams(reply.live);
  $('param-state').textContent = state.dirty.size ? `待发送 ${state.dirty.size} 项修改` : '板端已确认修改';
});
action('export', () => {
  const records = state.records.slice(state.recordStart);
  if (!records.length) return;
  const text = '\uFEFF' + ['received_at,source,' + CSV_FIELDS.join(','), ...records.map(csvRow)].join('\r\n');
  const url = URL.createObjectURL(new Blob([text], { type: 'text/csv;charset=utf-8' }));
  const anchor = document.createElement('a'); anchor.href = url;
  anchor.download = `ff-handle-${state.demo ? 'demo-' : ''}${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});

const demoDefaults = { ctrl_mode: 0, theta_des: 0, duty_max: .6, k: .8, b: .05,
  theta_min: -1.5, theta_max: 1.5, enc_cpr: 8344, enc_dir: 0, pid_kp: 2, pid_ki: .5,
  pid_kd: .05, tau_des: 0, tq_kp: 1.5, tq_ki: 8, i_max: 1, kt: .02, heater_duty: 0,
  telem_hz: 50, imu_kf_en: 1, dob_enable: 1, dob_vm: 5, dob_r: 5, dob_ke: .02, dob_j: .00005,
  dob_b_visc: .0001, dob_tau_c: 0, dob_fc_hz: 30 };
async function demoCommand(value) {
  log(`DEMO TX ${JSON.stringify(value)}`);
  const reply = { t: 'ok', cmd: value.cmd };
  switch (value.cmd) {
    case 'get': reply.live = { ...state.params }; break;
    case 'imu_filter':
      if (![0, 1].includes(value.v)) throw new Error('v 必须为 0 或 1');
      state.params.imu_kf_en = value.v; demoKalman.reset(); reply.live = { ...state.params }; break;
    case 'diag': reply.diag = { acc_id: 30, gyro_id: 15, imu_init: 'ESP_OK', acc_spi: 'ESP_OK',
      gyro_spi: 'ESP_OK', uptime_ms: state.demoTime * 1000, reset_reason: 1, firmware_sha: 'demo' }; break;
    case 'imu_probe': reply.probe = { miso_up: 1, miso_down: 0, miso_err: 0,
      cases: [0, 3].flatMap(mode => [8000000, 1000000, 250000].map(hz => ({ mode, hz,
        acc: [255, 255, 30], gyro: [255, 15], ae: 0, ge: 0 }))) }; break;
    case 'sub': state.demoSubscribed = true; break;
    case 'unsub': state.demoSubscribed = false; break;
    case 'duty': state.demoDuty = Math.max(-1, Math.min(1, Number(value.v) || 0)); state.demoMode = 0; state.params.ctrl_mode = 0; state.demoIntegral = 0; break;
    case 'heater': state.params.heater_duty = Number(value.v); reply.live = { ...state.params }; break;
    case 'set': {
      const next = { ...state.params, ...value }; delete next.cmd;
      validateMotorParams(next);
      if (!Number.isInteger(next.ctrl_mode) || next.ctrl_mode < 0 || next.ctrl_mode > 3) throw new Error('请选择支持的电机模式');
      state.params = next; state.demoMode = next.ctrl_mode; state.demoIntegral = 0;
      reply.live = { ...next }; break;
    }
    case 'zero': state.demoTheta = 0; state.demoIntegral = 0; break;
    case 'save': break;
    default: throw new Error('演示模式不支持此命令');
  }
  log(`DEMO RX ${JSON.stringify(reply)}`);
  return reply;
}
function stopDemo() {
  clearInterval(state.demoTimer); state.demo = false; state.demoSubscribed = false;
  resetData(); updateConnection(); notice('已退出演示，模拟记录已清除。');
}
action('demo', () => {
  if (state.demo) { stopDemo(); return; }
  resetData(); state.demo = true; state.demoSubscribed = true; state.demoSeq = 0;
  state.demoTime = 0; state.demoMode = 0; state.demoDuty = 0;
  state.demoTheta = 0; state.demoOmega = 0; state.demoIntegral = 0; state.demoApplied = 0;
  renderParams(demoDefaults); setSubscribed(true); updateConnection();
  notice('演示模式：全部数值为模拟数据，未连接板子；控制按钮只作用于模拟器。', 'demo');
  let last = performance.now(), emittedAt = 0;
  state.demoTimer = setInterval(() => {
    const now = performance.now(), dt = Math.min(.05, (now - last) / 1000);
    state.demoTime += (now - last) / 1000; last = now;
    const p = state.params, clamp = (value, limit) => Math.max(-limit, Math.min(limit, value));
    const targetAngle = Math.max(p.theta_min, Math.min(p.theta_max, p.theta_des));
    const error = targetAngle - state.demoTheta;
    const desiredCurrent = currentTarget(p.tau_des, p.kt, p.i_max).applied;
    let applied = state.demoDuty;
    if (state.demoMode === 1) applied = p.k * error - p.b * state.demoOmega;
    if (state.demoMode === 2) {
      state.demoIntegral = clamp(state.demoIntegral + error * dt, 2);
      applied = p.pid_kp * error + p.pid_ki * state.demoIntegral - p.pid_kd * state.demoOmega;
    }
    if (state.demoMode === 3) {
      const iError = desiredCurrent - state.demoApplied * .9;
      state.demoIntegral = clamp(state.demoIntegral + iError * dt, 1);
      applied = p.tq_kp * iError + p.tq_ki * state.demoIntegral;
    }
    state.demoApplied = clamp(applied, state.demoMode ? p.duty_max : 1);
    state.demoOmega += (state.demoApplied * 4 - state.demoOmega * 1.2) * dt;
    state.demoTheta += state.demoOmega * dt;
    const t = state.demoTime, theta = state.demoTheta, omega = state.demoOmega;
    // Coherent simulated body rotation: gravity and body angular rates agree.
    const rad = Math.PI / 180, roll = 30 * rad * Math.sin(t * .6), pitch = 20 * rad * Math.sin(t * .45);
    const rollRate = 18 * Math.cos(t * .6), pitchRate = 9 * Math.cos(t * .45), yawRate = 16 * Math.cos(t * .35);
    const acc = [-Math.sin(pitch), Math.sin(roll) * Math.cos(pitch), Math.cos(roll) * Math.cos(pitch)]
      .map((v, axis) => v + .04 * Math.sin(t * 193 + axis * 2));
    const gyro = [rollRate - yawRate * Math.sin(pitch),
      pitchRate * Math.cos(roll) + yawRate * Math.sin(roll) * Math.cos(pitch),
      -pitchRate * Math.sin(roll) + yawRate * Math.cos(roll) * Math.cos(pitch)]
      .map((v, axis) => v + 1.5 * Math.sin(t * 227 + axis * 2));
    const filtered = demoKalman.step(acc, gyro, p.imu_kf_en === 1, t * 1000);
    const demoPose = demoAttitude.update(filtered?.acc_f || acc, filtered?.gyro_f || gyro,
      true, t * 1000);
    if (!state.demoSubscribed || now - emittedAt < 1000 / state.params.telem_hz) return;
    emittedAt = now;
    const duty = state.demoApplied;
    const i = duty * .9;
    acceptTelemetry({ t: 'telem', seq: ++state.demoSeq, imu_ok: 1, mode: state.demoMode,
      acc, gyro, imu_kf_en: p.imu_kf_en, imu_kf: filtered ? 1 : 0,
      acc_f: filtered?.acc_f || [0, 0, 0], gyro_f: filtered?.gyro_f || [0, 0, 0],
      quat: demoPose?.quaternion || [1, 0, 0, 0], quat_ok: demoPose ? 1 : 0,
      quat_kf: demoPose && filtered ? 1 : 0,
      theta, omega, counts: Math.round(theta / (2 * Math.PI) * state.params.enc_cpr),
      duty, i_meas: i, i_des: state.demoMode === 3 ? desiredCurrent : 0,
      pid_err: state.demoMode === 2 ? targetAngle - theta : 0,
      tau: state.demoMode === 1 ? duty : 0, ff_en: state.demoMode === 1 ? 1 : 0,
      lim: Number(state.demoMode !== 0 && Math.abs(applied) > p.duty_max), tau_des: state.params.tau_des, tau_meas: i * state.params.kt,
      i_hat: i, tau_m_hat: i * state.params.kt, tau_ext_hat: .002 * Math.sin(t * .8), alpha_hat: -.4 * Math.sin(t) });
  }, 10);
});

setInterval(() => {
  renderTelemetry(); renderAttitude(); drawCharts();
  const active = state.demo || session.connected;
  const age = state.lastAt ? Date.now() - state.lastAt : Infinity;
  const fresh = active && state.subscribed && age < 2000;
  $('stream-state').textContent = !active ? (state.lastAt ? '已断开 · 历史数据' : '等待连接') : !state.subscribed ? '订阅已暂停' : fresh ? (state.demo ? '模拟遥测' : '正在接收') : '等待遥测';
  if (state.lastAt) $('last-frame').textContent = `距上一帧 ${(age / 1000).toFixed(1)} s`;
  $('imu-status').textContent = !state.latest ? '等待数据' : !fresh ? '历史数据' : state.latest.imu_ok ? 'IMU 正常' : 'IMU 读取失败';
  $('imu-status').className = `badge ${fresh && state.latest?.imu_ok ? 'good' : state.latest ? 'warn' : ''}`;
  const elapsed = (performance.now() - state.rateAt) / 1000;
  if (elapsed >= 1) { $('rate').textContent = (state.frames / elapsed).toFixed(1); state.frames = 0; state.rateAt = performance.now(); }
}, 100);

if (!isSecureContext) notice('串口访问需要安全上下文，请通过 http://localhost:8776 打开页面。', 'error');
else if (!navigator.serial) notice('当前浏览器不支持 USB 串口，可选择 Wi-Fi 或演示模式；首次 USB 配网请使用桌面 Chrome / Edge。', 'error');
selectMode(0); updateConnection(); renderAttitude(); drawCharts();
