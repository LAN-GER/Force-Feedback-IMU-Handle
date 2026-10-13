// Real loopback WebSocket + mock USB provisioning; same arguments as browser-smoke.cjs.
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const path = require('node:path');
const playwrightPath = process.argv[2] || 'playwright';
const { chromium } = require(playwrightPath);
const { wsServer: WebSocketServer } = require(path.join(path.dirname(require.resolve(playwrightPath)), '../playwright-core/lib/utilsBundle.js'));
const token = '0123456789abcdef0123456789abcdef';
const params = { ctrl_mode: 0, ff_enable: 0, k: .8, b: .05, theta_des: 0, duty_max: .6,
  theta_min: -1.5, theta_max: 1.5, enc_cpr: 8344, enc_dir: 0, telem_hz: 50,
  heater_duty: 0, imu_kf_en: 1, pid_kp: 2, pid_ki: .5, pid_kd: .05, tau_des: .01,
  tq_kp: 1.5, tq_ki: 8, i_max: 1, kt: .02, dob_enable: 1, dob_vm: 5, dob_r: 5,
  dob_ke: .02, dob_kt: .02, dob_j: .00005, dob_b_visc: .0001, dob_tau_c: 0, dob_fc_hz: 30 };

(async () => {
  const checks = [], errors = [], commands = [];
  const server = new WebSocketServer({ host: '127.0.0.1', port: 0,
    verifyClient: info => new URL(info.req.url, 'http://localhost').searchParams.get('token') === token });
  await new Promise(resolve => server.once('listening', resolve));
  const endpoint = `ws://127.0.0.1:${server.address().port}/ws`;
  let seq = 0, duty = 0;
  server.on('connection', socket => {
    let timer;
    const send = value => { if (socket.readyState === 1) socket.send(JSON.stringify(value)); };
    const telemetry = () => send({ t: 'telem', seq: ++seq, imu_ok: 1, mode: params.ctrl_mode,
      acc: [0, 0, 1], gyro: [0, 0, 0], acc_f: [0, 0, 1], gyro_f: [0, 0, 0],
      imu_kf_en: params.imu_kf_en, imu_kf: params.imu_kf_en, imu_acc_id: 30, imu_gyro_id: 15,
      quat_ok: 1, quat_kf: params.imu_kf_en, quat: [1, 0, 0, 0],
      theta: .5, omega: .8, counts: 664, duty, tau: 0, ff_en: 0, lim: 0, pid_err: .1,
      i_meas: .1, i_des: .2, tau_des: .01, tau_meas: .002, i_hat: .1,
      tau_m_hat: .002, tau_ext_hat: .001, alpha_hat: .1 });
    socket.on('message', bytes => {
      const value = JSON.parse(bytes.toString()); commands.push(value);
      if (value.cmd === 'wifi_set' || value.cmd === 'wifi_enable' || value.cmd === 'wifi_forget') {
        send({ t: 'err', cmd: value.cmd, msg: 'USB provisioning only' }); return;
      }
      if (value.cmd === 'sub') { clearInterval(timer); timer = setInterval(telemetry, 20); }
      if (value.cmd === 'unsub') clearInterval(timer);
      if (value.cmd === 'set') { const { cmd, ...patch } = value; Object.assign(params, patch); }
      if (value.cmd === 'imu_filter') params.imu_kf_en = value.v;
      if (value.cmd === 'heater') params.heater_duty = value.v;
      if (value.cmd === 'duty') { duty = value.v; params.ctrl_mode = 0; }
      send({ t: 'ok', cmd: value.cmd, ...(['get', 'set', 'imu_filter', 'heater'].includes(value.cmd) ? { live: params } : {}) });
    });
    socket.on('close', () => clearInterval(timer));
  });
  const browser = await chromium.launch({ executablePath: process.argv[3], headless: true });
  try {
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    page.on('pageerror', error => errors.push(error.message));
    await page.addInitScript(({ params, token, endpoint }) => {
      const mock = window.__usb = { commands: [], ssid: '', enabled: false, running: false, busy: false,
        deferApply: false, dropEnable: false, failEnable: false };
      let controller;
      const reply = value => controller.enqueue(new TextEncoder().encode(`>>${JSON.stringify(value)}\n`));
      Object.defineProperty(navigator, 'serial', { configurable: true, value: { requestPort: async () => ({
        getInfo: () => ({ usbVendorId: 0x303a, usbProductId: 0x1001 }), open: async () => {}, close: async () => {},
        readable: new ReadableStream({ start(c) { controller = c; } }),
        writable: new WritableStream({ write(bytes) {
          const value = JSON.parse(new TextDecoder().decode(bytes)); mock.commands.push(value);
          if (value.cmd === 'wifi_set') mock.ssid = value.ssid;
          if (value.cmd === 'wifi_enable') {
            if (mock.dropEnable) return;
            if (mock.failEnable || (value.v && !mock.ssid)) {
              reply({ t: 'err', cmd: value.cmd, msg: 'ESP_ERR_INVALID_STATE' }); return;
            }
            mock.enabled = !!value.v;
            mock.busy = mock.deferApply;
            if (!mock.busy) mock.running = mock.enabled;
          }
          if (value.cmd === 'wifi_forget') { mock.ssid = ''; mock.enabled = false; mock.running = false; mock.busy = false; }
          reply({ t: 'ok', cmd: value.cmd,
            ...(value.cmd === 'get' ? { live: params } : {}),
            ...(value.cmd.startsWith('wifi_') ? { wifi: { ready: 1, enabled: +mock.enabled, running: +mock.running, busy: +mock.busy, error: '',
              connected: +(mock.enabled && mock.running && !mock.busy), ssid: mock.ssid, ip: mock.enabled && !mock.busy ? '127.0.0.1' : '',
              reason: 0, ws_url: mock.enabled && !mock.busy ? endpoint : '', token } } : {}) });
        } })
      }) } });
    }, { params, token, endpoint });
    await page.goto('http://127.0.0.1:8776'); await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('板端已确认订阅'));
    await page.locator('.wifi-provision summary').click();
    await page.waitForFunction(() => !document.querySelector('#wifi-enable').disabled);
    assert.equal(await page.locator('#wifi-enable').isChecked(), false);
    await page.click('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('ESP_ERR_INVALID_STATE'));
    assert.equal(await page.locator('#wifi-enable').isChecked(), false);
    checks.push('USB 展开配置自动查询开关，未配置网络时开启失败会恢复显示');
    await page.fill('#wifi-ssid', '实验室"WiFi'); await page.fill('#wifi-password', 'secret\\pass"');
    await page.click('#configure-wifi');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('配置已保存'));
    assert.equal(await page.locator('#wifi-enable').isChecked(), false);
    assert.equal(await page.locator('#wifi-address').inputValue(), '');
    await page.check('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#wifi-status').textContent.includes('已连接'));
    assert.equal(await page.locator('#wifi-address').inputValue(), endpoint);
    assert.equal(await page.locator('#wifi-token').inputValue(), token);
    assert.equal(await page.locator('#wifi-password').inputValue(), '');
    const log = await page.locator('#log').textContent();
    assert.ok(!log.includes('secret') && !log.includes(token));
    assert.ok(log.includes('***'));
    checks.push('USB 配网支持中文和转义字符，自动填入地址与密钥，清空密码并隐藏日志中的秘密');
    await page.evaluate(() => { __usb.deferApply = true; });
    await page.uncheck('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#wifi-switch-state').textContent.includes('正在关闭'));
    assert.equal(await page.locator('#wifi-enable').isDisabled(), true);
    assert.ok(!(await page.locator('#wifi-switch-state').textContent()).includes('已释放'));
    await page.evaluate(() => { __usb.deferApply = false; __usb.busy = false; __usb.running = false; });
    await page.waitForFunction(() => document.querySelector('#wifi-switch-state').textContent.includes('已释放'));
    checks.push('关闭 ACK 标记为异步处理中，查询确认后才显示资源已释放');
    assert.equal(await page.locator('#wifi-ssid').inputValue(), '实验室"WiFi');
    assert.equal(await page.locator('#wifi-token').inputValue(), token);
    assert.equal(await page.locator('#wifi-address').inputValue(), '');
    const configs = await page.evaluate(() => __usb.commands.filter(v => v.cmd === 'wifi_set').length);
    await page.check('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#wifi-status').textContent.includes('已连接'));
    assert.equal(await page.evaluate(() => __usb.commands.filter(v => v.cmd === 'wifi_set').length), configs);
    await page.evaluate(() => { __usb.failEnable = true; });
    await page.click('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('开关未确认'));
    assert.equal(await page.locator('#wifi-enable').isChecked(), true);
    await page.evaluate(() => { __usb.failEnable = false; });
    checks.push('USB 关闭保留 SSID 和密钥、清空失效地址；重新开启无需填写密码，拒绝请求恢复已确认状态');
    await page.evaluate(() => { __usb.dropEnable = true; });
    await page.click('#wifi-enable');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('回复超时'));
    assert.equal(await page.locator('#wifi-enable').isChecked(), true);
    assert.ok((await page.locator('#wifi-switch-state').textContent()).includes('操作未确认'));
    assert.equal(await page.locator('#stop').isEnabled(), true);
    await page.evaluate(() => { __usb.dropEnable = false; });
    checks.push('开关回复超时明确标记未确认，恢复历史开关显示且仍可停止电机');
    if (process.argv[4]) {
      await fs.mkdir(process.argv[4], { recursive: true });
      await page.screenshot({ path: path.join(process.argv[4], 'wifi-switch-desktop.png') });
      await page.setViewportSize({ width: 390, height: 844 });
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
      await page.screenshot({ path: path.join(process.argv[4], 'wifi-switch-mobile.png') });
      await page.setViewportSize({ width: 1440, height: 1100 });
    }
    await page.click('#disconnect');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('串口已释放'));
    assert.deepEqual(await page.evaluate(() => __usb.commands.slice(-3)), [{ cmd: 'duty', v: 0 }, { cmd: 'heater', v: 0 }, { cmd: 'unsub' }]);
    const usbCount = await page.evaluate(() => __usb.commands.length);
    await page.selectOption('#transport', 'wifi');
    await page.fill('#wifi-token', 'f'.repeat(32)); await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('连接失败'));
    assert.equal(await page.locator('#choose-port').isEnabled(), true);
    await page.fill('#wifi-token', token); await page.click('#connect');
    await page.waitForFunction(() => document.querySelector('#theta-deg').textContent === '28.65');
    assert.deepEqual(commands.slice(0, 2).map(v => v.cmd), ['get', 'sub']);
    assert.equal(await page.locator('#connection-state').textContent(), 'Wi-Fi 已连接');
    assert.equal(await page.locator('#configure-wifi').isDisabled(), true);
    assert.equal(await page.locator('#wifi-enable').isDisabled(), true);
    assert.equal(await page.evaluate(() => __usb.commands.length), usbCount);
    checks.push('USB 停机断开后连接真实 WebSocket；错误密钥可重试，Wi-Fi 仅自动 get/sub 且不占用串口');
    await page.fill('#duty-input', '.1'); await page.click('#duty-form button');
    await page.waitForFunction(() => document.querySelector('#duty').textContent === '10.0%');
    await page.click('#stop');
    await page.waitForFunction(() => document.querySelector('#duty').textContent === '0.0%');
    for (const mode of [1, 2, 3]) {
      await page.click(`[data-mode="${mode}"]`); await page.click('#apply-mode');
      await page.waitForFunction(mode => document.querySelector('#mode').textContent === ['', '阻抗力反馈', '位置 PID', '力矩 / 电流 PI'][mode], mode);
      assert.equal(commands.at(-1).ctrl_mode, mode);
    }
    await page.uncheck('#imu-filter-enable');
    await page.waitForFunction(() => document.querySelector('#attitude-source').textContent === '板端四元数 · 原始六轴');
    checks.push('Wi-Fi 下 PWM、停止、三种闭环模式及板端滤波均通过 ACK 和遥测确认');
    await page.fill('#raw-input', JSON.stringify({ cmd: 'wifi_set', ssid: 'other', password: '12345678' }));
    await page.click('#raw-form button');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('USB provisioning only'));
    assert.ok(!(await page.locator('#log').textContent()).includes('12345678'));
    checks.push('Wi-Fi 下的配网指令明确拒绝，错误反馈仍隐藏密码');
    await page.fill('#raw-input', JSON.stringify({ cmd: 'wifi_enable', v: 0 }));
    await page.click('#raw-form button');
    await page.waitForFunction(() => document.querySelector('#command-state').textContent.includes('wifi_enable') && document.querySelector('#notice').textContent.includes('USB provisioning only'));
    assert.deepEqual(commands.at(-1), { cmd: 'wifi_enable', v: 0 });
    checks.push('Wi-Fi 链路不允许修改板端无线开关');
    const download = page.waitForEvent('download'); await page.click('#export');
    const csv = await fs.readFile(await (await download).path(), 'utf8');
    assert.ok(csv.split('\n').some(line => line.includes(',wifi,')));
    await page.setViewportSize({ width: 390, height: 844 });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
    checks.push('Wi-Fi CSV 标记来源，连接字段及配网表单在 390px 宽度无横向溢出');
    for (const socket of server.clients) socket.terminate();
    await page.waitForFunction(() => document.querySelector('#connection-state').textContent === '未连接');
    assert.equal(await page.locator('#stop').isDisabled(), true);
    await page.waitForFunction(() => document.querySelector('#stream-state').textContent.includes('历史数据'));
    await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#connection-state').textContent === 'Wi-Fi 已连接');
    await page.click('#disconnect');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('Wi-Fi 连接已释放'));
    assert.deepEqual(commands.slice(-3), [{ cmd: 'duty', v: 0 }, { cmd: 'heater', v: 0 }, { cmd: 'unsub' }]);
    checks.push('网络掉线冻结为历史数据并清理连接，可重连；主动断开先停机、关闭加热、取消订阅');
    await page.selectOption('#transport', 'usb'); await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#connection-state').textContent === 'USB 已连接');
    assert.equal(await page.locator('#wifi-token').inputValue(), '');
    await page.click('#forget-wifi');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('配置已清除'));
    assert.equal(await page.locator('#wifi-address').inputValue(), '');
    await page.click('#disconnect');
    checks.push('Wi-Fi 断开后可切回 USB，清除配置会清空旧连接地址');
    const noSerial = await browser.newPage();
    noSerial.on('pageerror', error => errors.push(error.message));
    await noSerial.addInitScript(() => Object.defineProperty(navigator, 'serial', { configurable: true, value: undefined }));
    await noSerial.goto('http://127.0.0.1:8776'); await noSerial.selectOption('#transport', 'wifi');
    await noSerial.fill('#wifi-address', endpoint); await noSerial.fill('#wifi-token', token);
    await noSerial.click('#choose-port');
    await noSerial.waitForFunction(() => document.querySelector('#theta-deg').textContent === '28.65');
    await noSerial.click('#disconnect');
    checks.push('不支持 Web Serial 的浏览器仍可独立使用 Wi-Fi');
    assert.deepEqual(errors, []);
    console.log(JSON.stringify({ passed: checks.length, checks, pageErrors: errors }, null, 2));
  } finally {
    await browser.close();
    for (const socket of server.clients) socket.terminate();
    await new Promise(resolve => server.close(resolve));
  }
})().catch(error => { console.error(error); process.exitCode = 1; });
