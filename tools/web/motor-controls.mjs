// Parameters and units match the controllers in software/components/svc/.
export const MODE_NAMES = ['开环 PWM', '阻抗力反馈', '位置 PID', '力矩 / 电流 PI'];
export const MODE_KEYS = [[],
  ['theta_des', 'k', 'b', 'duty_max', 'theta_min', 'theta_max'],
  ['theta_des', 'pid_kp', 'pid_ki', 'pid_kd', 'duty_max', 'theta_min', 'theta_max'],
  ['tau_des', 'tq_kp', 'tq_ki', 'i_max', 'kt', 'duty_max']];
export const ANGLE_KEYS = ['theta_des', 'theta_min', 'theta_max'];
export const MODE_DESCRIPTIONS = [
  '直接发送有符号占空比，正负决定方向。当前固件开环不使用闭环 PWM 上限。',
  '围绕中心角产生弹簧与阻尼反馈：K × 角度误差 − B × 角速度。当前固件将结果直接作为 PWM 指令。',
  '编码器位置闭环：Kp × 误差 + Ki × 积分 − Kd × 角速度；目标角度受软限位约束。',
  '目标力矩通过 Kt 换算为目标电流，由 PI 控制 PWM。此模式使用电流及 PWM 限幅，不执行角度软限位。'
];
export function fromRadians(value, unit) { return unit === 'deg' ? value * 180 / Math.PI : value; }
export function toRadians(value, unit) { return unit === 'deg' ? value * Math.PI / 180 : value; }

export function validateMotorParams(p) {
  for (const [key, value] of Object.entries(p)) {
    if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error(`${key} 必须是有效数值`);
  }
  for (const key of ['k', 'b', 'pid_kp', 'pid_ki', 'pid_kd', 'tq_kp', 'tq_ki']) {
    if (key in p && p[key] < 0) throw new Error('刚度、阻尼和控制器增益必须大于或等于 0');
  }
  if ('duty_max' in p && !(p.duty_max > 0 && p.duty_max <= 1)) throw new Error('闭环 PWM 上限必须大于 0 且不超过 1');
  if ('i_max' in p && !(p.i_max > .05)) throw new Error('电流限幅必须大于 0.05 A');
  if ('kt' in p && !(p.kt > 1e-6)) throw new Error('Kt 必须大于 0.000001 N·m/A');
  if ('theta_min' in p && 'theta_max' in p && p.theta_min >= p.theta_max) throw new Error('软限位下限必须小于上限');
}

export function motorCommand(mode, values, duty = 0) {
  if (!Number.isInteger(mode) || mode < 0 || mode > 3) throw new Error('请选择支持的电机模式');
  if (mode === 0) {
    if (!Number.isFinite(duty) || Math.abs(duty) > 1) throw new Error('开环占空比必须在 −1 到 1 之间');
    return { cmd: 'duty', v: duty };
  }
  const p = {};
  for (const key of MODE_KEYS[mode]) p[key] = values[key];
  validateMotorParams(p);
  if (mode < 3 && (p.theta_des < p.theta_min || p.theta_des > p.theta_max)) {
    throw new Error('目标角度必须在软限位范围内，请修改目标或限位');
  }
  // One set updates the mode and its complete configuration together.
  return { cmd: 'set', ctrl_mode: mode, ...p };
}

export function torqueFromCurrent(current, kt) {
  if (!Number.isFinite(current) || !Number.isFinite(kt) || kt <= 1e-6) throw new Error('请先填写有效目标电流和 Kt');
  return current * kt;
}
export function currentTarget(tau, kt, limit) {
  validateMotorParams({ tau_des: tau, kt, i_max: limit });
  const requested = tau / kt;
  return { requested, applied: Math.max(-limit, Math.min(limit, requested)), limited: Math.abs(requested) > limit };
}
