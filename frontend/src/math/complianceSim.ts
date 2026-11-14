/**
 * コンプライアント追従シミュレーション (Berio IROS 2016 §V).
 *
 * 実機 Baxter の低ゲイン PD 制御による追従誤差を簡易シミュレートするため、
 * 加速度依存ラグ:
 *   x_actual(t) = x_desired(t) - k_lag · ẍ_desired(t)
 * を適用する。高加速度区間 (急発進・鋭角) で誤差が増大し、滑らかな箇所では
 * ほぼ誤差なし。論文の "compliant tracking" の本質的挙動を直感的に再現する。
 */


export interface ComplianceParams {
  /** 加速度ラグ係数 [s²]。0 で剛性追従、増やすほど柔軟 */
  k_lag: number;
}


export const DEFAULT_COMPLIANCE: ComplianceParams = { k_lag: 0.005 };


/**
 * desired (世界座標) 軌跡 → compliant actual 軌跡 を生成。
 * @param positions  Float32Array [N, 3]
 * @param ts         Float32Array [N]
 * @returns actual positions [N, 3]
 */
export function simulateCompliance(
  positions: Float32Array,
  ts: Float32Array,
  params: ComplianceParams,
): Float32Array {
  const N = ts.length;
  const out = new Float32Array(N * 3);
  if (N < 3) {
    out.set(positions);
    return out;
  }
  // 中央差分で ẍ を計算
  for (let k = 0; k < N; k++) {
    const k0 = Math.max(k - 1, 0);
    const k1 = Math.min(k + 1, N - 1);
    const dt0 = ts[k] - ts[k0] || 1e-6;
    const dt1 = ts[k1] - ts[k] || 1e-6;
    const dtAvg = 0.5 * (dt0 + dt1);
    for (let d = 0; d < 3; d++) {
      const x0 = positions[k0 * 3 + d];
      const x1 = positions[k * 3 + d];
      const x2 = positions[k1 * 3 + d];
      // ẍ = (x2 - 2 x1 + x0) / dtAvg²
      const acc = (x2 - 2 * x1 + x0) / (dtAvg * dtAvg);
      out[k * 3 + d] = x1 - params.k_lag * acc;
    }
  }
  return out;
}
