export function hasFilteredImu(data) {
  return !!data?.imu_ok && data.imu_kf_en === 1 && data.imu_kf === 1 &&
    ['acc_f', 'gyro_f'].every(key => Array.isArray(data[key]) && data[key].length === 3 &&
      data[key].every(Number.isFinite));
}

// Only used by the demo simulator. Real USB measurements are filtered on board.
export class DemoImuKalman {
  constructor() { this.reset(); }
  reset() { this.x = null; this.p = []; this.at = 0; }
  step(acc, gyro, enabled, at) {
    if (!enabled) { this.reset(); return null; }
    const raw = [...acc, ...gyro], dt = (at - this.at) / 1000;
    if (!this.x || dt < 0 || dt > .25) {
      this.x = [...raw]; this.p = [.0025, .0025, .0025, 1, 1, 1];
    } else {
      raw.forEach((value, i) => {
        const r = i < 3 ? .0025 : 1, q = i < 3 ? .04 : 4;
        const p = this.p[i] + q * dt, k = p / (p + r);
        this.x[i] += k * (value - this.x[i]); this.p[i] = r * p / (p + r);
      });
    }
    this.at = at;
    return { acc_f: this.x.slice(0, 3), gyro_f: this.x.slice(3) };
  }
}
