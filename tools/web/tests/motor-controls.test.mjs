import test from 'node:test';
import assert from 'node:assert/strict';
import { motorCommand, fromRadians, toRadians, currentTarget, torqueFromCurrent } from '../motor-controls.mjs';
import { encodeCommand } from '../protocol.mjs';

const params = { theta_des: .5, theta_min: -1.5, theta_max: 1.5, duty_max: .6,
  k: .8, b: .05, pid_kp: 2, pid_ki: .5, pid_kd: .05,
  tau_des: .01, tq_kp: 1.5, tq_ki: 8, i_max: 1, kt: .02 };

test('四种模式只发送对应参数；PWM 指令进入开环，闭环模式与配置原子发送', () => {
  assert.deepEqual(motorCommand(0, params, -.2), { cmd: 'duty', v: -.2 });
  const spring = motorCommand(1, params);
  assert.deepEqual(spring, { cmd: 'set', ctrl_mode: 1, theta_des: .5, k: .8, b: .05,
    duty_max: .6, theta_min: -1.5, theta_max: 1.5 });
  const position = motorCommand(2, params);
  assert.equal(position.pid_kd, .05); assert.equal('k' in position, false);
  const torque = motorCommand(3, params);
  assert.equal(torque.tau_des, .01); assert.equal('theta_min' in torque, false);
  assert.equal('ff_enable' in torque, false); // Legacy flag would override ctrl_mode.
  for (let mode = 0; mode < 4; mode++) assert.doesNotThrow(() => encodeCommand(motorCommand(mode, params)));
});
test('度与弧度互换不改变实际目标角度', () => {
  assert.ok(Math.abs(toRadians(90, 'deg') - Math.PI / 2) < 1e-12);
  assert.equal(toRadians(.3, 'rad'), .3);
  assert.ok(Math.abs(fromRadians(toRadians(45, 'deg'), 'deg') - 45) < 1e-12);
});
test('非法闭环参数、缺失参数、错误限位与越界目标在发送前拒绝', () => {
  for (const patch of [{ duty_max: 0 }, { duty_max: 1.1 }, { pid_ki: -1 },
    { theta_min: 2 }, { theta_des: 2 }, { pid_kp: NaN }, { pid_kd: undefined }]) {
    assert.throws(() => motorCommand(2, { ...params, ...patch }));
  }
  assert.throws(() => motorCommand(3, { ...params, kt: 0 }));
  assert.throws(() => motorCommand(3, { ...params, i_max: .05 }));
  assert.throws(() => motorCommand(0, params, 1.01));
  assert.throws(() => motorCommand(4, params));
});
test('电流和力矩可双向换算，预览与固件电流限幅一致', () => {
  assert.equal(torqueFromCurrent(-.5, .02), -.01);
  assert.deepEqual(currentTarget(-.05, .02, 1), { requested: -2.5, applied: -1, limited: true });
  assert.deepEqual(currentTarget(.01, .02, 1), { requested: .5, applied: .5, limited: false });
  assert.throws(() => torqueFromCurrent(.5, 0));
});
