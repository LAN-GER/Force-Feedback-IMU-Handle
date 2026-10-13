import test from 'node:test';
import assert from 'node:assert/strict';
import { DemoImuAttitude as ImuAttitude, ImuAttitude as BoardPose,
  hasBoardAttitude, quaternionCssMatrix } from '../imu-attitude.mjs';

const close = (a, b, eps = .01) => assert.ok(Math.abs(a - b) < eps, `${a} differs from ${b}`);

test('USB 姿态直接消费四元数，低帧率和重复序号不重新积分；归零只改显示', () => {
  const pose = new BoardPose();
  const q = [Math.cos(Math.PI / 12), Math.sin(Math.PI / 12), 0, 0];
  close(pose.update(q, true, 1).roll, 30);
  close(pose.update(q, true, 1000).roll, 30);
  assert.equal(pose.zero(), true);
  close(pose.update(q.map(v => -v), true, 1000).roll, 0);
  close(pose.update([Math.SQRT1_2, Math.SQRT1_2, 0, 0], true, 1020).roll, 60);
});

test('板端四元数校验拒绝无效标志、错误长度、NaN 和非单位值；旧固件不自行融合', () => {
  const frame = { imu_ok: 1, quat_ok: 1, quat: [1, 0, 0, 0] };
  assert.equal(hasBoardAttitude(frame), true);
  for (const patch of [{ imu_ok: 0 }, { quat_ok: 0 }, { quat: undefined },
    { quat: [0, 0, 0, 0] }, { quat: [2, 0, 0, 0] }, { quat: [NaN, 0, 0, 1] }, { quat: [1, 0, 0] }]) {
    assert.equal(hasBoardAttitude({ ...frame, ...patch }), false);
  }
  const pose = new BoardPose();
  assert.equal(pose.update(frame.quat, false, 1), null);
  assert.equal(pose.zero(), false);
});

test('订阅暂停保持显示参考，uint32 回绕保持参考，板端重启清除参考', () => {
  const pose = new BoardPose(), q = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
  pose.update(q, true, 0xfffffff0); pose.zero(); pose.pause();
  close(pose.update(q, true, 4).yaw, 0);
  assert.equal(pose.update(q, false, 5), null);
  const revision = pose.revision;
  close(pose.update(q, true, 1).yaw, 90);
  assert.ok(pose.revision > revision);
});

test('重力初始化倾斜，首帧不从零缓慢爬升', () => {
  const pose = new ImuAttitude().update([-.5, Math.sqrt(3) / 4, .75], [0, 0, 0], true, 0, 1);
  close(pose.roll, 30); close(pose.pitch, 30); close(pose.yaw, 0);
  assert.equal(new ImuAttitude().update([0, 0, 0], [0, 0, 0], true, 0), null);
});

test('三轴角速度独立产生对应的 90 度转动，四元数保持归一化', () => {
  for (const [axis, key] of ['roll', 'pitch', 'yaw'].entries()) {
    const estimator = new ImuAttitude();
    estimator.update([0, 0, 1], [0, 0, 0], true, 0, 0);
    const gyro = [0, 0, 0]; gyro[axis] = 90;
    let pose;
    for (let i = 1; i <= 50; i++) pose = estimator.update([0, 0, 2], gyro, true, i * 20, i * 20);
    close(pose[key], 90); close(Math.hypot(...pose.quaternion), 1, 1e-10);
  }
});

test('重力校正倾斜误差，航向保持相对角度', () => {
  const estimator = new ImuAttitude();
  estimator.update([0, Math.SQRT1_2, Math.SQRT1_2], [0, 0, 0], true, 0);
  let pose;
  for (let i = 1; i <= 500; i++) pose = estimator.update([0, 0, 1], [0, 0, 0], true, i * 10);
  close(pose.roll, 0); close(pose.pitch, 0); close(pose.yaw, 0);
});

test('归零只改变显示参考，后续转动从零姿态继续', () => {
  const estimator = new ImuAttitude();
  assert.equal(estimator.zero(), false);
  estimator.update([0, .5, Math.sqrt(3) / 2], [0, 0, 0], true, 0);
  const physical = [...estimator.q];
  assert.equal(estimator.zero(), true); assert.deepEqual(estimator.q, physical);
  close(estimator.pose().roll, 0); close(estimator.pose().pitch, 0); close(estimator.pose().yaw, 0);
  const pose = estimator.update([0, 0, 2], [90, 0, 0], true, 100);
  close(pose.roll, 9);
});

test('无效数据和长间隔不积分漏失时段；重复序号忽略，重启清除参考', () => {
  const estimator = new ImuAttitude();
  estimator.update([0, 0, 1], [0, 0, 0], true, 0, 0);
  const pose = estimator.update([0, 0, 1], [0, 0, 90], true, 20, 20);
  assert.deepEqual(estimator.update([0, 0, 1], [0, 0, 90], true, 21, 20), pose);
  assert.equal(estimator.update([0, 0, 1], [0, 0, 90], true, 600, 20), null);
  assert.equal(estimator.update([0, 0, 1], [0, 0, 90], false, 40, 40), null);
  close(estimator.update([0, 0, 1], [0, 0, 90], true, 10000, 10000).yaw, pose.yaw);
  close(estimator.update([0, 0, 1], [0, 0, 90], true, 20000, 20000).yaw, pose.yaw);
  estimator.pause();
  close(estimator.update([0, 0, 1], [0, 0, 90], true, 20010, 20010).yaw, pose.yaw);
  assert.equal(estimator.update([0, 0, 1], [NaN, 0, 0], true, 20020, 20020), null);
  estimator.update([0, 0, 1], [0, 0, 0], true, 20040, 20040);
  estimator.zero();
  const restart = estimator.update([0, .5, Math.sqrt(3) / 2], [0, 0, 0], true, 20060, 1);
  close(restart.roll, 30); close(restart.yaw, 0);
});

test('批量收到数据时使用板端采样序号计时；支持 uint32 序号回绕', () => {
  const estimator = new ImuAttitude();
  estimator.update([0, 0, 1], [0, 0, 0], true, 0, 0xfffffff0);
  close(estimator.update([0, 0, 1], [0, 0, 100], true, 0, 4).yaw, 2);
});

test('CSS 旋转矩阵保持正交并将 IMU Z 轴映射为屏幕竖直轴', () => {
  const matrix = q => quaternionCssMatrix(q).slice(9, -1).split(',').map(Number);
  assert.deepEqual(matrix([1, 0, 0, 0]), [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  const m = matrix([Math.SQRT1_2, 0, 0, Math.SQRT1_2]);
  close(m[0], 0); close(m[2], 1); close(m[5], 1); close(m[8], -1);
  for (const start of [0, 4, 8]) close(Math.hypot(...m.slice(start, start + 3)), 1);
});
