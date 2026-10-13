// Real USB poses consume board quaternions; six-axis fusion below is demo-only.
const RAD = Math.PI / 180;
const IDENTITY = [1, 0, 0, 0]; // w, x, y, z
const vector3 = v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
const conjugate = ([w, x, y, z]) => [w, -x, -y, -z];
const normalize = q => { const n = Math.hypot(...q); return q.map(v => v / n); };

export function multiplyQuaternion([w, x, y, z], [a, b, c, d]) {
  return [w * a - x * b - y * c - z * d,
    w * b + x * a + y * d - z * c,
    w * c - x * d + y * a + z * b,
    w * d + x * c - y * b + z * a];
}

export function quaternionAngles([w, x, y, z]) {
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) / RAD,
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) / RAD,
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) / RAD
  };
}

function tiltQuaternion(acc, yaw) {
  const [ax, ay, az] = acc;
  const roll = Math.atan2(ay, az), pitch = Math.atan2(-ax, Math.hypot(ay, az));
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}

export function hasBoardAttitude(data) {
  return data?.imu_ok === 1 && data.quat_ok === 1 && Array.isArray(data.quat) &&
    data.quat.length === 4 && data.quat.every(Number.isFinite) &&
    Math.abs(Math.hypot(...data.quat) - 1) < .02;
}

export class ImuAttitude {
  constructor() { this.reset(); }
  reset() {
    this.q = null; this.reference = [...IDENTITY]; this.lastSeq = null;
    this.revision = (this.revision || 0) + 1;
  }
  pause() {} // Display reference survives subscription pauses.
  zero() {
    if (!this.q) return false;
    this.reference = conjugate(this.q); return true;
  }
  pose() {
    if (!this.q) return null;
    const relative = normalize(multiplyQuaternion(this.reference, this.q));
    return { quaternion: relative, ...quaternionAngles(relative) };
  }
  update(quaternion, valid, seq = null) {
    if (Number.isInteger(seq) && Number.isInteger(this.lastSeq) &&
        ((seq - this.lastSeq) >>> 0) > 0x7fffffff) this.reset();
    this.lastSeq = seq;
    if (!valid || !Array.isArray(quaternion) || quaternion.length !== 4 ||
        !quaternion.every(Number.isFinite) || Math.abs(Math.hypot(...quaternion) - 1) >= .02) return null;
    this.q = normalize(quaternion);
    return this.pose();
  }
}

export class DemoImuAttitude {
  constructor() { this.reset(); }
  reset() {
    this.q = null; this.reference = [...IDENTITY]; this.lastAt = null; this.lastSeq = null;
    this.revision = (this.revision || 0) + 1;
  }
  zero() {
    if (!this.q) return false;
    this.reference = conjugate(this.q);
    return true;
  }
  pause() { this.lastAt = null; this.lastSeq = null; }
  pose() {
    if (!this.q) return null;
    const relative = normalize(multiplyQuaternion(this.reference, this.q));
    return { quaternion: relative, ...quaternionAngles(relative) };
  }
  // Firmware seq counts 1kHz sampler ticks; demo uses monotonic receive time.
  update(acc, gyro, valid, atMs, seq = null) {
    if (!valid || !vector3(acc) || !vector3(gyro) || !Number.isFinite(atMs)) {
      this.pause();
      return null;
    }
    const previousAt = this.lastAt;
    const sequenceDelta = Number.isInteger(seq) && Number.isInteger(this.lastSeq)
      ? (seq - this.lastSeq) >>> 0 : null;
    if (sequenceDelta === 0) return previousAt !== null && atMs - previousAt > 500 ? null : this.pose();
    let dt = previousAt === null ? null : (atMs - previousAt) / 1000;
    const gap = dt === null || dt < 0 || (dt === 0 && sequenceDelta === null) || dt > .5 ||
      (sequenceDelta !== null && sequenceDelta > 500);
    if (sequenceDelta !== null && sequenceDelta > 0x7fffffff) this.reset(); // Board restart.
    this.lastAt = atMs; this.lastSeq = seq;
    const norm = Math.hypot(...acc), gravityUsable = norm >= .7 && norm <= 1.3;
    if (!this.q || gap) {
      if (!gravityUsable) return null;
      const yaw = this.q ? quaternionAngles(this.q).yaw * RAD : 0;
      this.q = tiltQuaternion(acc, yaw);
      return this.pose();
    }
    if (sequenceDelta !== null) dt = sequenceDelta / 1000;
    // Bound each quaternion integration step, including low-rate telemetry.
    const steps = Math.ceil(dt / .01), step = dt / steps;
    for (let i = 0; i < steps; i++) {
      const [w, x, y, z] = this.q;
      const omega = gyro.map(v => v * RAD);
      if (gravityUsable) {
        const [ax, ay, az] = acc.map(v => v / norm);
        const gx = 2 * (x * z - w * y), gy = 2 * (w * x + y * z), gz = 1 - 2 * (x * x + y * y);
        // Measured gravity cross predicted gravity corrects tilt, not heading.
        omega[0] += 2.5 * (ay * gz - az * gy);
        omega[1] += 2.5 * (az * gx - ax * gz);
        omega[2] += 2.5 * (ax * gy - ay * gx);
      }
      const derivative = multiplyQuaternion(this.q, [0, ...omega]);
      this.q = normalize(this.q.map((v, j) => v + derivative[j] * step / 2));
    }
    return this.pose();
  }
}

export function quaternionCssMatrix([w, x, y, z]) {
  const r = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
  ];
  // Sensor X = screen right, Y = toward viewer, Z = screen up.
  const axes = [0, 2, 1], signs = [1, -1, 1];
  const m = Array.from({ length: 3 }, (_, row) => Array.from({ length: 3 }, (_, col) =>
    signs[row] * signs[col] * r[axes[row]][axes[col]]));
  return `matrix3d(${[m[0][0], m[1][0], m[2][0], 0, m[0][1], m[1][1], m[2][1], 0,
    m[0][2], m[1][2], m[2][2], 0, 0, 0, 0, 1].map(v => Number(v.toFixed(8))).join(',')})`;
}
