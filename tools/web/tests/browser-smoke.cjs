// Optional browser QA: node browser-smoke.cjs <playwright module path> <browser executable> [screenshot directory]
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const path = require('node:path');
const { chromium } = require(process.argv[2] || 'playwright');

async function injectMock(page) {
  await page.addInitScript(() => {
    const mock = window.__mock = { commands: [], closes: 0, seq: 0, imuOk: true, cancelNext: false,
      kfSupported: true, quatSupported: true, quat: [1, 0, 0, 0], rejectFilter: false,
      drop: new Set(), params: { imu_kf_en: 1, ctrl_mode: 0, k: .8, b: .05, theta_des: 0, duty_max: .6,
        theta_min: -1.5, theta_max: 1.5, enc_cpr: 600, enc_dir: 0, telem_hz: 50,
        heater_duty: 0, pid_kp: 2, pid_ki: .5, pid_kd: .05, tau_des: .01,
        tq_kp: 1.5, tq_ki: 8, i_max: 1, kt: .02, dob_enable: 1, dob_vm: 5,
        dob_r: 5, dob_ke: .02, dob_j: .00005, dob_b_visc: .0001, dob_tau_c: 0, dob_fc_hz: 30 } };
    const encoder = new TextEncoder();
    let controller, timer, port, duty = 0, counts = 30;
    function reply(value) {
      const bytes = encoder.encode(`>>${JSON.stringify(value)}\r\n`);
      const split = Math.floor(bytes.length / 2);
      controller.enqueue(bytes.slice(0, split)); controller.enqueue(bytes.slice(split));
    }
    function telemetry() {
      mock.seq += 20; // Each mock frame spans twenty 1kHz firmware sampler ticks.
      reply({ t: 'telem', seq: mock.seq, imu_ok: mock.imuOk ? 1 : 0, mode: mock.params.ctrl_mode,
        imu_acc_id: mock.imuOk ? 30 : 0, imu_gyro_id: mock.imuOk ? 15 : 0,
        acc: mock.acc || [.125, -.25, 1], gyro: mock.gyro || [1.5, -2.5, 3.5], theta: counts * Math.PI / 300,
        ...(mock.quatSupported ? { quat: mock.quat, quat_ok: mock.imuOk ? 1 : 0,
          quat_kf: mock.kfSupported && mock.params.imu_kf_en ? 1 : 0 } : {}),
        ...(mock.kfSupported ? { imu_kf_en: mock.params.imu_kf_en,
          imu_kf: mock.imuOk && mock.params.imu_kf_en ? 1 : 0,
          acc_f: mock.accFiltered || [.1, -.2, .98], gyro_f: mock.gyroFiltered || [1.2, -2.2, 3.2] } : {}),
        omega: .8, counts, duty, tau: 0, ff_en: 0, lim: 0, pid_err: .1,
        i_meas: duty * .9, i_des: .5, tau_des: .01, tau_meas: .009,
        i_hat: .45, tau_m_hat: .009, tau_ext_hat: .004, alpha_hat: .2 });
    }
    mock.setFramePeriod = milliseconds => { clearInterval(timer); timer = setInterval(telemetry, milliseconds); };
    function createPort() {
      port = {
        readable: new ReadableStream({ start(value) { controller = value; } }),
        writable: new WritableStream({ write(bytes) {
          const value = JSON.parse(new TextDecoder().decode(bytes));
          mock.commands.push(value);
          if (value.cmd === 'imu_filter') {
            if (!mock.kfSupported || mock.rejectFilter) {
              reply({ t: 'err', cmd: value.cmd, msg: 'unknown' }); return;
            }
            mock.params.imu_kf_en = value.v;
          }
          if (value.cmd === 'sub') { clearInterval(timer); timer = setInterval(telemetry, 20); }
          if (value.cmd === 'unsub') { clearInterval(timer); timer = null; }
          if (value.cmd === 'duty') { duty = value.v; mock.params.ctrl_mode = 0; }
          if (value.cmd === 'heater') mock.params.heater_duty = value.v;
          if (value.cmd === 'zero') counts = 0;
          if (value.cmd === 'set') { Object.assign(mock.params, value); delete mock.params.cmd; }
          if (mock.drop.has(value.cmd)) return;
          if (value.cmd === 'save' && mock.rejectSave) {
            reply({ t: 'err', cmd: 'save', msg: 'nvs' }); return;
          }
          if (value.cmd === 'save' && mock.holdSave) {
            mock.releaseSave = () => { reply({ t: 'ok', cmd: 'save' }); mock.holdSave = false; };
            return;
          }
          if (value.cmd === 'set' && mock.holdSet) {
            const held = { t: 'ok', cmd: 'set', live: { ...mock.params } };
            mock.releaseSet = () => { reply(held); mock.holdSet = false; };
            return;
          }
          if (value.cmd === 'imu_probe') {
            if (mock.imuOk) { reply({ t: 'err', cmd: value.cmd, msg: 'ESP_ERR_INVALID_STATE' }); return; }
            reply({ t: 'ok', cmd: value.cmd, probe: {
              miso_up: mock.probeError ? -1 : 1, miso_down: mock.probeError ? -1 : 0,
              miso_err: mock.probeError || 0, miso_input: 1, miso_oe: 1, miso_periph: 1,
              cases: [0, 3].flatMap(mode => [8000000, 1000000, 250000].map(hz => ({ mode, hz,
                acc: [255, 255, 255], gyro: [255, 255], ae: 0, ge: 0 }))) } });
            return;
          }
          if (value.cmd === 'diag') {
            reply({ t: 'ok', cmd: 'diag', diag: { acc_id: mock.imuOk ? 30 : 0,
              gyro_id: mock.imuOk ? 15 : 0, imu_init: mock.imuOk ? 'ESP_OK' : 'ESP_ERR_INVALID_RESPONSE',
              telem_hz: mock.params.telem_hz, telem_work_us: 14000, telem_work_max_us: 17000, telem_skipped: 12,
              telem_snapshot_us: 100, telem_format_us: 10500, telem_usb_us: 3350, telem_wifi_us: 50, telem_format_errors: 0,
              acc_spi: 'ESP_OK', gyro_spi: 'ESP_OK', uptime_ms: 12500, reset_reason: 1, firmware_sha: '0123456789abcdef' } });
            return;
          }
          if (value.cmd === 'unknown') reply({ t: 'err', cmd: value.cmd, msg: 'unknown' });
          else reply({ t: 'ok', cmd: value.cmd, ...(['get', 'set', 'heater', 'imu_filter'].includes(value.cmd) ? { live: { ...mock.params } } : {}) });
        } }),
        open: async () => { controller.enqueue(encoder.encode('I (2) app: 手柄启动 <img src=x onerror=alert(1)>\r\n')); },
        close: async () => {
          if (port.readable.locked || port.writable.locked) throw new Error('串口锁未释放');
          clearInterval(timer); mock.closes++;
        },
        getInfo: () => ({ usbVendorId: 0x303a, usbProductId: 0x1001 })
      };
      mock.unplug = () => { clearInterval(timer); controller.error(new Error('模拟 USB 拔线')); };
      return port;
    }
    Object.defineProperty(navigator, 'serial', { configurable: true, value: {
      requestPort: async () => {
        if (mock.cancelNext) { mock.cancelNext = false; throw new DOMException('取消', 'NotFoundError'); }
        return createPort();
      }
    } });
  });
}

(async () => {
  const browser = await chromium.launch({ executablePath: process.argv[3], headless: true });
  const errors = [];
  const checks = [];
  try {
    const context = await browser.newContext({ viewport: { width: 1440, height: 1250 } });
    const page = await context.newPage(); page.on('pageerror', error => { errors.push(error.message); console.error('PAGE ERROR:', error.message); });
    await injectMock(page);
    await page.goto('http://127.0.0.1:8776');
    await page.waitForFunction(() => document.querySelector('#acc-0'));
    assert.equal(await page.locator('#stop').isDisabled(), true);
    assert.equal(await page.locator('#choose-port').isVisible(), true);
    assert.ok((await page.locator('#port-selection-hint').textContent()).includes('浏览器弹窗'));
    await page.evaluate(() => { __mock.cancelNext = true; });
    await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('取消'));
    checks.push('串口选择取消恢复');

    await page.click('#choose-port');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('板端已确认订阅'));
    await page.waitForFunction(() => document.querySelector('#acc-0').textContent === '0.1250');
    assert.deepEqual(await page.evaluate(() => __mock.commands.slice(0, 2).map(value => value.cmd)), ['get', 'sub']);
    assert.equal(await page.inputValue('#p-k'), '0.8');
    assert.equal(await page.locator('#theta-deg').textContent(), '18.00');
    assert.equal(await page.locator('#counts').textContent(), '30');
    assert.equal(await page.locator('#tau_ext_hat').textContent(), '0.004000');
    assert.equal(await page.locator('#log img').count(), 0);
    checks.push('仅 get/sub 自动发送；分包遥测、参数回填、编码器及 DOB 显示');

    await page.waitForFunction(() => document.querySelector('#attitude-status').textContent === '实时姿态');
    assert.equal(await page.locator('#attitude-source').textContent(), '板端四元数 · 卡尔曼六轴');
    const attitudeCommandCount = await page.evaluate(() => __mock.commands.length);
    await page.click('#attitude-zero');
    await page.waitForFunction(() => Math.abs(Number(document.querySelector('#attitude-roll').textContent)) < 1);
    assert.equal(await page.evaluate(() => __mock.commands.length), attitudeCommandCount);
    const cubeBefore = await page.locator('#imu-cube').getAttribute('style');
    await page.evaluate(() => {
      __mock.acc = __mock.accFiltered = [0, 0, 2];
      __mock.gyro = __mock.gyroFiltered = [0, 0, 0];
      __mock.quat = [Math.cos(Math.PI / 6), Math.sin(Math.PI / 6), 0, 0];
    });
    await page.waitForFunction(() => Number(document.querySelector('#attitude-roll').textContent) > 15);
    assert.notEqual(await page.locator('#imu-cube').getAttribute('style'), cubeBefore);
    await page.evaluate(() => { delete __mock.acc; delete __mock.accFiltered; delete __mock.gyro; delete __mock.gyroFiltered; });
    checks.push('IMU 3D 方块直接使用板端四元数，在六轴静止时仍跟随板端姿态；显示归零不发送命令');

    await page.waitForFunction(() => document.querySelector('#acc-f-0').textContent === '0.1000');
    assert.equal(await page.locator('#imu-filter-enable').isChecked(), true);
    assert.equal(await page.locator('#gyro-f-0').textContent(), '1.20');
    const beforeCompare = await page.evaluate(() => __mock.commands.length);
    await page.uncheck('#imu-filter-compare');
    assert.equal(await page.locator('#acc-f-0').isVisible(), false);
    assert.equal(await page.evaluate(() => __mock.commands.length), beforeCompare);
    await page.check('#imu-filter-compare');
    assert.equal(await page.locator('#acc-f-0').isVisible(), true);
    await page.uncheck('#imu-filter-enable');
    await page.waitForFunction(() => document.querySelector('#imu-filter-command').textContent.includes('已确认滤波关闭'));
    await page.waitForFunction(() => document.querySelector('#acc-f-0').textContent === '—');
    await page.waitForFunction(() => document.querySelector('#attitude-source').textContent === '板端四元数 · 原始六轴');
    assert.deepEqual(await page.evaluate(() => __mock.commands.at(-1)), { cmd: 'imu_filter', v: 0 });
    assert.equal(await page.locator('#acc-0').textContent(), '0.1250');
    await page.check('#imu-filter-enable');
    await page.waitForFunction(() => document.querySelector('#imu-filter-command').textContent.includes('已确认滤波开启'));
    await page.evaluate(() => { __mock.rejectFilter = true; });
    await page.click('#imu-filter-enable'); // A rejected request immediately restores the checkbox.
    await page.waitForFunction(() => document.querySelector('#imu-filter-command').textContent.includes('未确认'));
    assert.equal(await page.locator('#imu-filter-enable').isChecked(), true);
    assert.equal(await page.locator('#stop').isDisabled(), false);
    await page.evaluate(() => { __mock.rejectFilter = false; __mock.drop.add('imu_filter'); });
    await page.uncheck('#imu-filter-enable');
    assert.equal(await page.locator('#imu-filter-enable').isDisabled(), true);
    assert.equal(await page.locator('#stop').isDisabled(), false);
    await page.waitForFunction(() => document.querySelector('#imu-filter-command').textContent.includes('回复超时'));
    assert.equal(await page.locator('#imu-filter-enable').isChecked(), true);
    await page.evaluate(() => { __mock.drop.delete('imu_filter'); });
    await page.click('#read-params');
    await page.waitForFunction(() => !document.querySelector('#imu-filter-enable').checked);
    await page.check('#imu-filter-enable');
    await page.waitForFunction(() => document.querySelector('#acc-f-0').textContent === '0.1000');
    checks.push('六轴原始/卡尔曼值对比；显示开关不发命令；板端开关 ACK、拒绝及超时恢复，不影响停止按钮');

    await page.evaluate(() => { __mock.imuOk = false; });
    await page.waitForFunction(() => document.querySelector('#imu-status').textContent === 'IMU 读取失败');
    await page.waitForFunction(() => document.querySelector('#attitude-status').textContent.includes('已暂停'));
    assert.equal(await page.locator('#attitude-zero').isDisabled(), true);
    const pausedCube = await page.locator('#imu-cube').getAttribute('style');
    await page.waitForTimeout(100);
    assert.equal(await page.locator('#imu-cube').getAttribute('style'), pausedCube);
    await page.click('#read-diag');
    await page.waitForFunction(() => document.querySelector('#diagnostics').textContent.includes('ESP_ERR_INVALID_RESPONSE'));
    assert.ok((await page.locator('#imu-ids').textContent()).includes('Acc 0x00 / Gyro 0x00'));
    assert.ok((await page.locator('#diagnostics').textContent()).includes('Acc SPI：ESP_OK'));
    assert.ok((await page.locator('#diagnostics').textContent()).includes('复位原因：上电'));
    assert.ok((await page.locator('#diagnostics').textContent()).includes('0123456789abcdef'));
    await page.click('#probe-spi');
    await page.waitForFunction(() => document.querySelector('#spi-probe').textContent.includes('Mode3 / 250 kHz'));
    assert.ok((await page.locator('#spi-probe').textContent()).includes('0xFF 0xFF 0xFF'));
    assert.ok((await page.locator('#spi-probe').textContent()).includes('弱上拉 1 / 弱下拉 0'));
    assert.ok((await page.locator('#spi-probe').textContent()).includes('外设控制输出使能 1'));
    await page.evaluate(() => { __mock.probeError = 259; });
    await page.click('#probe-spi');
    await page.waitForFunction(() => document.querySelector('#spi-probe').textContent.includes('测试错误 259'));
    const invalidProbe = await page.locator('#spi-probe').textContent();
    assert.ok(invalidProbe.includes('空闲电平测试未完成'));
    assert.ok(!invalidProbe.includes('疑似被固定拉低'));
    assert.ok(!invalidProbe.includes('空闲 MISO 能跟随'));
    await page.evaluate(() => { __mock.probeError = 0; });
    checks.push('SPI 电平测试错误 259 不被解释为短路；显示 GPIO 外设控制 OE 配置');
    assert.equal(await page.locator('#acc-0').textContent(), '—');
    assert.equal(await page.locator('#gyro-0').textContent(), '—');
    assert.equal(await page.locator('#acc-f-0').textContent(), '—');
    assert.equal(await page.locator('#gyro-f-0').textContent(), '—');
    assert.equal(await page.locator('#counts').textContent(), '30');
    const failedSeq = Number(await page.locator('#seq').textContent());
    await page.waitForFunction(seq => Number(document.querySelector('#seq').textContent) > seq, failedSeq);
    await page.click('#read-params');
    await page.waitForFunction(() => document.querySelector('#command-state').textContent === 'get 板端已确认');
    await page.evaluate(() => { __mock.imuOk = true; });
    await page.waitForFunction(() => document.querySelector('#acc-0').textContent === '0.1250');
    await page.click('#probe-spi');
    await page.waitForFunction(() => document.querySelector('#spi-probe').textContent.includes('探测未确认'));
    checks.push('IMU 故障不显示无效读数，编码器遥测及 get 继续工作');
    checks.push('SPI 探测显示六组原始字节及弱上/下拉；正常 IMU 拒绝探测并显示错误');

    await page.evaluate(() => { __mock.drop.add('diag'); });
    await page.click('#read-diag');
    await page.waitForFunction(() => document.querySelector('#diagnostics').textContent.includes('诊断未确认'));
    await page.evaluate(() => { __mock.drop.delete('diag'); });
    await page.click('#read-diag');
    await page.waitForFunction(() => document.querySelector('#diagnostics').textContent.includes('初始化：ESP_OK'));
    checks.push('只读诊断显示 ID、SPI、复位原因；超时后明确标记未确认并可重试');

    await page.click('#unsubscribe');
    await page.waitForFunction(() => document.querySelector('#subscribe').textContent === '开始订阅');
    await page.waitForFunction(() => document.querySelector('#attitude-status').textContent.includes('历史姿态'));
    const pausedSeq = await page.evaluate(() => __mock.seq);
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(() => __mock.seq), pausedSeq);
    await page.click('#subscribe');
    await page.waitForFunction(() => document.querySelector('#subscribe').textContent === '已订阅');
    checks.push('暂停与恢复订阅');

    await page.fill('#duty-input', '0.2'); await page.click('#duty-form button');
    await page.waitForFunction(() => document.querySelector('#duty').textContent === '20.0%');
    await page.click('#stop');
    await page.waitForFunction(() => document.querySelector('#duty').textContent === '0.0%');
    await page.fill('#heater-input', '0.1'); await page.click('#heater-form button');
    await page.waitForFunction(() => __mock.params.heater_duty === .1);
    await page.click('#zero'); await page.waitForFunction(() => document.querySelector('#counts').textContent === '0');
    checks.push('PWM、停止、加热、编码器清零');

    const modePreviewCount = await page.evaluate(() => __mock.commands.length);
    await page.click('[data-mode="2"]');
    await page.selectOption('#angle-unit', 'rad'); await page.fill('#p-theta_des', '0.3');
    assert.equal(await page.evaluate(() => __mock.commands.length), modePreviewCount);
    await page.click('#apply-mode');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('位置 PID板端已确认'));
    const changed = await page.evaluate(() => __mock.commands.filter(value => value.cmd === 'set').at(-1));
    assert.deepEqual(changed, { cmd: 'set', ctrl_mode: 2, theta_des: .3, pid_kp: 2, pid_ki: .5,
      pid_kd: .05, duty_max: .6, theta_min: -1.5, theta_max: 1.5 });
    await page.waitForFunction(() => document.querySelector('#mode').textContent === '位置 PID');
    await page.selectOption('#angle-unit', 'deg');
    assert.ok(Math.abs(Number(await page.inputValue('#p-theta_des')) - .3 * 180 / Math.PI) < 1e-8);
    const captureCount = await page.evaluate(() => __mock.commands.length);
    await page.click('#use-current-angle');
    assert.ok(Math.abs(Number(await page.inputValue('#p-theta_des')) - 0) < 1e-8); // zeroed above
    assert.equal(await page.evaluate(() => __mock.commands.length), captureCount);
    await page.fill('#p-theta_des', '45');
    await page.click('#apply-mode');
    await page.waitForFunction(() => !document.querySelector('#p-theta_des').classList.contains('dirty'));
    assert.ok(Math.abs((await page.evaluate(() => __mock.commands.at(-1))).theta_des - Math.PI / 4) < 1e-12);
    checks.push('模式预览和当前位置目标不发送命令；位置 PID 完整参数及度/弧度转换');

    await page.click('[data-mode="1"]'); await page.fill('#p-k', '0.9');
    await page.click('#apply-mode');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('阻抗力反馈板端已确认'));
    const impedance = await page.evaluate(() => __mock.commands.at(-1));
    assert.equal(impedance.ctrl_mode, 1); assert.equal(impedance.k, .9);
    assert.equal('pid_kp' in impedance, false);
    checks.push('阻抗模式发送中心角、刚度、阻尼与限位，不夹带位置 PID 参数');

    await page.click('[data-mode="3"]'); await page.fill('#current-target', '-0.5');
    const convertCount = await page.evaluate(() => __mock.commands.length);
    await page.click('#convert-current');
    assert.equal(Number(await page.inputValue('#p-tau_des')), -.01);
    assert.equal(await page.evaluate(() => __mock.commands.length), convertCount);
    await page.click('#apply-mode');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('力矩 / 电流 PI板端已确认'));
    const torque = await page.evaluate(() => __mock.commands.at(-1));
    assert.equal(torque.ctrl_mode, 3); assert.equal(torque.tau_des, -.01);
    assert.equal('theta_min' in torque, false); assert.equal('ff_enable' in torque, false);
    assert.ok((await page.locator('#torque-preview').textContent()).includes('-0.5000 A'));
    checks.push('力矩 PI 目标电流换算、完整参数、目标电流预览及模式互斥');

    await page.click('[data-mode="2"]'); await page.fill('#p-theta_min', '90');
    const invalidLimitCount = await page.evaluate(() => __mock.commands.length);
    await page.click('#apply-mode');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('下限必须小于上限'));
    assert.equal(await page.evaluate(() => __mock.commands.length), invalidLimitCount);
    await page.fill('#p-theta_min', '-90');
    await page.evaluate(() => { __mock.holdSet = true; });
    await page.click('#apply-mode');
    await page.waitForFunction(() => typeof __mock.releaseSet === 'function');
    await page.fill('#p-theta_des', '60');
    await page.evaluate(() => __mock.releaseSet());
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('位置 PID板端已确认'));
    assert.equal(await page.inputValue('#p-theta_des'), '60');
    assert.ok(await page.locator('#p-theta_des').evaluate(input => input.classList.contains('dirty')));
    checks.push('非法限位阻止发送；控制回复不覆盖等待期间的新目标');

    await page.evaluate(() => { __mock.holdSet = true; delete __mock.releaseSet; });
    await page.click('#apply-mode');
    await page.waitForFunction(() => typeof __mock.releaseSet === 'function');
    await page.click('#stop');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('板端已确认停止'));
    await page.evaluate(() => __mock.releaseSet());
    await page.waitForFunction(() => document.querySelector('#motor-control-state').textContent.includes('板端待机'));
    assert.ok((await page.locator('#motor-command-state').textContent()).includes('板端已确认停止'));
    checks.push('闭环命令等待回复期间仍可停机；延迟启动回复不覆盖停止状态');

    await page.click('[data-mode="3"]');
    await page.evaluate(() => __mock.drop.add('set'));
    await page.click('#apply-mode');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('控制未确认'));
    assert.equal(await page.locator('#apply-mode').isDisabled(), false);
    assert.equal(await page.locator('#stop').isDisabled(), false);
    await page.evaluate(() => __mock.drop.delete('set'));
    await page.click('#stop');
    await page.waitForFunction(() => document.querySelector('#motor-command-state').textContent.includes('板端已确认停止'));
    checks.push('闭环启动超时不声称成功、释放重试按钮并保持停止可用');

    await page.locator('.controls-panel summary').click();
    await page.fill('#p-enc_cpr', '800');
    await page.click('#params-form button');
    await page.waitForFunction(() => document.querySelector('#param-state').textContent === '板端已确认修改');
    assert.equal((await page.evaluate(() => __mock.commands.at(-1))).ctrl_mode, undefined);
    await page.selectOption('#hz', '100'); await page.click('#set-hz');
    await page.waitForFunction(() => document.querySelector('#nvs-status').textContent.includes('100 Hz 已临时生效'));
    assert.equal(await page.evaluate(() => __mock.params.telem_hz), 100);
    await page.click('#read-diag');
    await page.waitForFunction(() => document.querySelector('#diagnostics').textContent.includes('配置 100 Hz'));
    const telemetryDiag = await page.locator('#diagnostics').textContent();
    assert.ok(telemetryDiag.includes('14.00 ms') && telemetryDiag.includes('17.00 ms') && telemetryDiag.includes('12 个发送通知'));
    assert.ok(telemetryDiag.includes('超过配置周期'));
    assert.ok(telemetryDiag.includes('快照 0.10 ms') && telemetryDiag.includes('组包 10.50 ms'));
    assert.ok(telemetryDiag.includes('USB 写入 3.35 ms') && telemetryDiag.includes('Wi-Fi 入队 0.05 ms'));
    assert.ok(telemetryDiag.includes('组包失败 0 次'));
    checks.push('遥测诊断显示发送耗时和跳过通知，工作耗时超过配置周期时提示可能跳帧');
    // Configured frequency must never substitute for measured arrival count.
    await page.evaluate(() => __mock.setFramePeriod(20));
    await page.waitForFunction(() => {
      const rate = Number(document.querySelector('#rate').textContent);
      return rate >= 40 && rate <= 60;
    });
    await page.evaluate(() => __mock.setFramePeriod(10));
    await page.waitForFunction(() => {
      const rate = Number(document.querySelector('#rate').textContent);
      return rate >= 85 && rate <= 115;
    });
    await page.evaluate(() => __mock.setFramePeriod(20));
    checks.push('配置 100 Hz 时接收速率仍按实际帧计数：输入 50/100 帧每秒分别显示约 50/100');
    await page.evaluate(() => { __mock.holdSave = true; });
    await page.click('#save');
    await page.waitForFunction(() => !!__mock.releaseSave);
    assert.equal(await page.locator('#nvs-status').getAttribute('data-state'), 'pending');
    assert.equal(await page.locator('#save').isDisabled(), true);
    assert.equal(await page.locator('#stop').isEnabled(), true);
    assert.ok(!(await page.locator('#nvs-status').textContent()).includes('保存成功'));
    await page.evaluate(() => __mock.releaseSave());
    await page.waitForFunction(() => document.querySelector('#nvs-status').dataset.state === 'success');
    assert.ok(/\d{2}:\d{2}:\d{2}/.test(await page.locator('#nvs-status').textContent()));
    assert.equal(await page.locator('#save').isEnabled(), true);
    checks.push('遥测频率提示需保存；NVS 等待 ACK 时禁用重复保存但仍可停机，成功后显示时间');
    checks.push('只发送修改参数、模式切换和 NVS 保存');
    await page.evaluate(() => { __mock.rejectSave = true; });
    await page.click('#save');
    await page.waitForFunction(() => document.querySelector('#nvs-status').dataset.state === 'error');
    assert.ok((await page.locator('#nvs-status').textContent()).includes('保存失败'));
    await page.evaluate(() => { __mock.rejectSave = false; __mock.drop.add('save'); });
    await page.click('#save');
    await page.waitForFunction(() => document.querySelector('#nvs-status').dataset.state === 'unknown');
    assert.ok((await page.locator('#nvs-status').textContent()).includes('回复超时'));
    assert.equal(await page.locator('#save').isEnabled(), true);
    await page.evaluate(() => __mock.drop.delete('save'));
    checks.push('NVS 板端拒绝显示保存失败，超时显示结果未确认，可重试且不会误报成功');

    await page.fill('#raw-input', '{"cmd":"unknown"}'); await page.click('#raw-form button');
    await page.waitForFunction(() => document.querySelector('#command-state').textContent.includes('板端拒绝'));
    await page.fill('#raw-input', '{"cmd":"set","note":"' + '中'.repeat(100) + '"}');
    const countBefore = await page.evaluate(() => __mock.commands.length);
    await page.click('#raw-form button');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('255'));
    assert.equal(await page.evaluate(() => __mock.commands.length), countBefore);
    checks.push('板端错误及超长命令不发送');

    const downloaded = page.waitForEvent('download'); await page.click('#export');
    const download = await downloaded;
    const csv = await fs.readFile(await download.path(), 'utf8');
    assert.ok(csv.includes('received_at,source,seq,imu_ok,mode,acc_x'));
    assert.ok(csv.split('\n')[0].includes('imu_acc_id,imu_gyro_id'));
    assert.ok(csv.includes(',usb,')); assert.ok(csv.includes(',0.125,-0.25,1,1.5,-2.5,3.5,'));
    assert.ok(csv.split('\n')[0].includes('imu_kf_en,imu_kf,acc_f_x,acc_f_y,acc_f_z,gyro_f_x,gyro_f_y,gyro_f_z'));
    assert.ok(csv.includes(',0.1,-0.2,0.98,1.2,-2.2,3.2'));
    checks.push('CSV 实际下载、来源及传感器字段');

    // Earlier unacknowledged duty must not suppress a subsequent stop.
    await page.click('[data-mode="0"]');
    await page.evaluate(() => __mock.drop.add('duty'));
    await page.fill('#duty-input', '0.15'); await page.click('#duty-form button');
    await page.waitForFunction(() => __mock.commands.at(-1).v === .15);
    await page.click('#stop');
    await page.waitForFunction(() => __mock.commands.at(-1).cmd === 'duty' && __mock.commands.at(-1).v === 0);
    assert.ok((await page.locator('#notice').textContent()).includes('停止命令已发送'));
    await page.evaluate(() => { __mock.drop.delete('duty'); __mock.unplug(); });
    await page.waitForFunction(() => document.querySelector('#connection-state').textContent === '未连接');
    assert.equal(await page.locator('#stop').isDisabled(), true);
    await page.waitForFunction(() => document.querySelector('#stream-state').textContent.includes('历史数据'));
    checks.push('无 ACK 时仍可发送停止；拔线清理、历史数据标记');

    await page.click('#connect');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('板端已确认订阅'));
    await page.click('#disconnect');
    await page.waitForFunction(() => document.querySelector('#notice').textContent.includes('串口已释放'));
    const tail = await page.evaluate(() => __mock.commands.slice(-3));
    assert.deepEqual(tail, [{ cmd: 'duty', v: 0 }, { cmd: 'heater', v: 0 }, { cmd: 'unsub' }]);
    assert.equal(await page.evaluate(() => __mock.closes), 2);
    checks.push('重新连接、停机断开顺序与读写锁释放');

    const legacy = await context.newPage(); legacy.on('pageerror', error => errors.push(error.message));
    await injectMock(legacy); await legacy.goto('http://127.0.0.1:8776');
    await legacy.evaluate(() => { __mock.kfSupported = false; __mock.quatSupported = false; delete __mock.params.imu_kf_en; });
    await legacy.click('#choose-port');
    await legacy.waitForFunction(() => document.querySelector('#acc-0').textContent === '0.1250');
    assert.equal(await legacy.locator('#imu-filter-enable').isDisabled(), true);
    assert.equal(await legacy.locator('#acc-f-0').textContent(), '—');
    assert.ok((await legacy.locator('#imu-filter-command').textContent()).includes('新版固件'));
    await legacy.waitForFunction(() => document.querySelector('#attitude-status').textContent.includes('固件未提供四元数'));
    assert.equal(await legacy.locator('#attitude-zero').isDisabled(), true);
    assert.deepEqual(await legacy.evaluate(() => __mock.commands.map(v => v.cmd)), ['get', 'sub']);
    await legacy.click('#disconnect');
    const saving = await context.newPage(); saving.on('pageerror', error => errors.push(error.message));
    await injectMock(saving); await saving.goto('http://127.0.0.1:8776'); await saving.click('#choose-port');
    await saving.waitForFunction(() => document.querySelector('#notice').textContent.includes('板端已确认订阅'));
    await saving.evaluate(() => { __mock.holdSave = true; }); await saving.click('#save');
    await saving.waitForFunction(() => !!__mock.releaseSave);
    await saving.evaluate(() => __mock.unplug());
    await saving.waitForFunction(() => document.querySelector('#connection-state').textContent === '未连接' && document.querySelector('#nvs-status').dataset.state === 'unknown');
    assert.ok((await saving.locator('#nvs-status').textContent()).includes('连接已断开'));
    await saving.click('#connect');
    await saving.waitForFunction(() => document.querySelector('#notice').textContent.includes('板端已确认订阅'));
    await saving.evaluate(() => __mock.releaseSave());
    assert.equal(await saving.locator('#nvs-status').getAttribute('data-state'), 'idle');
    assert.ok((await saving.locator('#nvs-status').textContent()).includes('尚未执行'));
    await saving.click('#disconnect'); await saving.close();
    checks.push('保存时断线标记结果未知，重连重置状态且旧 save 回复不误报成功');
    await legacy.waitForFunction(() => document.querySelector('#notice').textContent.includes('串口已释放'));
    await legacy.close();
    checks.push('旧固件保留原始数据，禁用滤波开关和姿态归零，提示缺少板端四元数');

    await page.click('#demo'); await page.waitForFunction(() => document.querySelector('#acc-0').textContent !== '—');
    await page.click('#read-diag');
    await page.waitForFunction(() => document.querySelector('#diagnostics').textContent.includes('模拟诊断'));
    assert.equal(await page.locator('#connection-state').textContent(), '模拟数据');
    await page.click('#save');
    await page.waitForFunction(() => document.querySelector('#nvs-status').textContent.includes('模拟保存完成'));
    assert.ok((await page.locator('#nvs-status').textContent()).includes('未写入真实板子'));
    checks.push('演示模式保存明确标记模拟，避免误认为写入板端 NVS');
    await page.waitForFunction(() => document.querySelector('#acc-f-0').textContent !== '—');
    await page.waitForTimeout(500);
    await page.locator('.controls-panel summary').click(); // collapse before screenshot
    if (process.argv[4]) {
      await page.locator('.attitude-panel').screenshot({ path: path.join(process.argv[4], 'imu-cube-desktop.png') });
      await page.screenshot({ path: path.join(process.argv[4], 'ff-handle-web-desktop.png'), fullPage: true });
    }
    await page.setViewportSize({ width: 390, height: 844 });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
    if (process.argv[4]) await page.screenshot({ path: path.join(process.argv[4], 'ff-handle-web-mobile.png'), fullPage: true });
    if (process.argv[4]) await page.locator('.attitude-panel').screenshot({ path: path.join(process.argv[4], 'imu-cube-mobile.png') });
    for (const mode of [1, 2, 3]) {
      await page.click(`[data-mode="${mode}"]`);
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
      assert.equal(await page.locator('#apply-mode').isVisible(), true);
      if (process.argv[4]) await page.locator('.controls-panel').screenshot({ path: path.join(process.argv[4], `ff-handle-mode-${mode}-mobile.png`) });
    }
    checks.push('阻抗、位置 PID 和力矩 PI 表单在 390px 手机宽度无溢出');
    await page.click('#demo');
    assert.equal(await page.locator('#acc-0').textContent(), '—');
    assert.equal(await page.locator('#export').isDisabled(), true);
    checks.push('演示数据与 USB 隔离、退出清空、390px 移动布局');

    const unsupported = await context.newPage(); unsupported.on('pageerror', error => errors.push(error.message));
    await unsupported.addInitScript(() => Object.defineProperty(navigator, 'serial', { configurable: true, value: undefined }));
    await unsupported.goto('http://127.0.0.1:8776');
    await unsupported.waitForFunction(() => document.querySelector('#notice').textContent.includes('不支持'));
    assert.equal(await unsupported.locator('#connect').isDisabled(), true);
    await unsupported.click('#demo'); await unsupported.waitForFunction(() => document.querySelector('#acc-0').textContent !== '—');
    checks.push('不支持串口的浏览器仍可演示');

    assert.deepEqual(errors, []);
    console.log(JSON.stringify({ passed: checks.length, checks, pageErrors: errors }, null, 2));
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
