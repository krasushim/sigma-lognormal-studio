/**
 * v2 IK キャッシュ — IROS 2016 方式の反復補正を **サンプル空間**で実装。
 *
 * 各反復:
 *   1. desired_k (現在フィードする軌跡) を compliance に通して actual_k を得る
 *   2. actual_k を追従するよう 7-DOF IK で関節角を計算
 *   3. 誤差 e_k(t) = desired_0(t) − actual_k(t)  (desired_0 = オリジナル 2D 形状)
 *   4. 次反復のフィード: desired_{k+1}(t) = desired_k(t) + α · e_k(t)
 *
 * 収束時: actual_k ≈ desired_0 (= 2D で見ている形状) になる。
 *
 * 注: 旧版は ΣΛ の virtual target を補正する方式だったが、
 *      2D と 3D の形状を厳密に一致させるため sample-space 方式に変更。
 */
import type { TrajectoryPlan } from "../api/types";
import type { CanvasMappingParams } from "./canvasMapping";
import { DEFAULT_MAPPING } from "./canvasMapping";
import {
  buildHelical, evaluateHelical, HelicalTrajectory,
} from "./helical3D";
import {
  RobotConfig7, DEFAULT_ROBOT_7, ikTrajectory, armReach,
} from "./jacobianIK";
import { simulateCompliance, ComplianceParams, DEFAULT_COMPLIANCE } from "./complianceSim";


export interface IterationFrame {
  iteration: number;
  ts: Float32Array;             // [N]
  desired: Float32Array;        // [N, 3]   ← 常にオリジナル 2D 形状
  fedDesired: Float32Array;     // [N, 3]   ← この iter で robot にフィードする (補正済み)
  actual: Float32Array;         // [N, 3]   ← compliance を通した実軌跡
  penWorld: Float32Array;       // [N, 3]   ← FK(IK(actual)) のペン先
  jointAngles: Float32Array;    // [N, 7]
  isPenUp: Uint8Array;          // [N]
  trackingRMSE: number;         // actual と desired (オリジナル) の RMS [m]
  reachable: boolean;
}


export interface CacheV2Options {
  maxIter: number;
  alpha0: number;
  alphaSubseq: number;
  liftHeightM: number;
  sampleRateHz: number;
  compliance: ComplianceParams;
  mapping: CanvasMappingParams;
  robot: RobotConfig7;
  progressCb?: (frac: number, iter: number) => void;
}


export const DEFAULT_CACHE_OPTS: CacheV2Options = {
  maxIter: 5,
  alpha0: 1.0,
  alphaSubseq: 0.5,
  liftHeightM: 0.05,
  sampleRateHz: 80,
  compliance: DEFAULT_COMPLIANCE,
  mapping: DEFAULT_MAPPING,
  robot: DEFAULT_ROBOT_7,
};


export class IKCacheV2 {
  iterations: IterationFrame[] = [];
  totalDuration: number = 0;
  hasUnreachable = false;
  canvasScale = 1.0;
  effectiveMapping: CanvasMappingParams;
  /** オリジナル (iter 0 に fed する) 軌跡。全反復の「真の desired」 */
  desiredOrig: Float32Array = new Float32Array(0);
  ts: Float32Array = new Float32Array(0);

  constructor(plan: TrajectoryPlan, opts: Partial<CacheV2Options> = {}) {
    const o: CacheV2Options = { ...DEFAULT_CACHE_OPTS, ...opts };
    this.effectiveMapping = o.mapping;
    if (!plan || plan.stroke_groups.length === 0) return;

    // Auto-fit canvas to arm reach
    const fitted = autoFitMapping(plan, o.mapping, o.robot, o.liftHeightM);
    this.canvasScale = fitted.scale;
    this.effectiveMapping = fitted.mapping;

    // 2D timeseries ベースの 3D 軌跡 (オリジナル)
    const traj: HelicalTrajectory = buildHelical(
      plan, o.liftHeightM, this.effectiveMapping,
    );
    this.totalDuration = traj.totalDuration;

    // サンプリング時刻 (一様格子)
    const N = Math.max(40, Math.ceil(traj.totalDuration * o.sampleRateHz));
    const ts = new Float32Array(N);
    for (let k = 0; k < N; k++) ts[k] = (k / (N - 1)) * traj.totalDuration;
    this.ts = ts;

    // オリジナル desired (2D 軌跡を世界座標にマップしたもの)
    const { pos: desired0, isPenUp } = evaluateHelical(traj, ts);
    this.desiredOrig = desired0;

    // 反復ごとの "fedDesired" (robot にフィードする補正済み軌跡)
    let fedDesired = new Float32Array(desired0);   // iter 0 は原本を直接フィード

    for (let iter = 0; iter < o.maxIter; iter++) {
      const frame = this._runOneIteration(
        ts, desired0, fedDesired, isPenUp, o, iter,
      );
      this.iterations.push(frame);
      if (!frame.reachable) this.hasUnreachable = true;
      if (o.progressCb) o.progressCb((iter + 1) / o.maxIter, iter);

      // 次反復: fedDesired ← fedDesired + α · (desired_0 − actual_k)
      if (iter < o.maxIter - 1) {
        const alpha = iter === 0 ? o.alpha0 : o.alphaSubseq;
        const next = new Float32Array(fedDesired.length);
        for (let k = 0; k < fedDesired.length; k++) {
          next[k] = fedDesired[k] + alpha * (desired0[k] - frame.actual[k]);
        }
        fedDesired = next;
      }
    }
  }

  private _runOneIteration(
    ts: Float32Array,
    desired0: Float32Array,
    fedDesired: Float32Array,
    isPenUp: Uint8Array,
    opts: CacheV2Options,
    iter: number,
  ): IterationFrame {
    // 2. compliance simulation (fedDesired → actual)
    const actual = simulateCompliance(fedDesired, ts, opts.compliance);

    // 3. IK on actual
    const n = this.effectiveMapping.canvasNormal;
    const penAxis = new Float32Array(ts.length * 3);
    for (let k = 0; k < ts.length; k++) {
      penAxis[k * 3 + 0] = -n[0];
      penAxis[k * 3 + 1] = -n[1];
      penAxis[k * 3 + 2] = -n[2];
    }
    const ikRes = ikTrajectory(opts.robot, actual, penAxis, ts);
    const reachable = ikRes.reachableMask.every((v) => v === 1);

    // tracking RMSE (actual と desired_0 の差)
    let sumSq = 0;
    for (let k = 0; k < ts.length; k++) {
      const dx = desired0[k * 3 + 0] - actual[k * 3 + 0];
      const dy = desired0[k * 3 + 1] - actual[k * 3 + 1];
      const dz = desired0[k * 3 + 2] - actual[k * 3 + 2];
      sumSq += dx * dx + dy * dy + dz * dz;
    }
    const rmse = Math.sqrt(sumSq / ts.length);

    return {
      iteration: iter,
      ts,
      desired: desired0,
      fedDesired,
      actual,
      penWorld: ikRes.penWorld,
      jointAngles: ikRes.q,
      isPenUp,
      trackingRMSE: rmse,
      reachable,
    };
  }
}


/**
 * Canvas 全域 (4 隅 × {lift あり/なし}) が shoulder 原点から arm reach 以内に
 * 収まるよう、キャンバスサイズを等比縮小する。
 */
function autoFitMapping(
  plan: TrajectoryPlan,
  mapping: CanvasMappingParams,
  robot: RobotConfig7,
  liftHeightM: number,
  safetyFactor = 0.92,
): { mapping: CanvasMappingParams; scale: number } {
  const reach = armReach(robot) * safetyFactor;

  const imgAspect = plan.image_width / plan.image_height;
  const canAspect = mapping.canvasWidth / mapping.canvasHeight;
  let useW = mapping.canvasWidth;
  let useH = mapping.canvasHeight;
  if (imgAspect > canAspect) useH = mapping.canvasWidth / imgAspect;
  else useW = mapping.canvasHeight * imgAspect;

  const n = unit(mapping.canvasNormal);
  const up = unit(mapping.canvasUp);
  const right = unitCross(up, n);
  const shoulder: [number, number, number] = [0, robot.baseHeight, 0];

  function maxCornerDist(sw: number, sh: number): number {
    let md = 0;
    for (const du of [-0.5, 0.5]) {
      for (const dv of [-0.5, 0.5]) {
        for (const lift of [0, liftHeightM]) {
          const px = mapping.canvasCenter[0]
            + right[0] * (du * sw) + up[0] * (dv * sh)
            + n[0] * (lift - mapping.contactDepthM);
          const py = mapping.canvasCenter[1]
            + right[1] * (du * sw) + up[1] * (dv * sh)
            + n[1] * (lift - mapping.contactDepthM);
          const pz = mapping.canvasCenter[2]
            + right[2] * (du * sw) + up[2] * (dv * sh)
            + n[2] * (lift - mapping.contactDepthM);
          // wrist center = pen tip + penLength * canvasNormal
          const wx = px + n[0] * robot.penLength;
          const wy = py + n[1] * robot.penLength;
          const wz = pz + n[2] * robot.penLength;
          const d = Math.hypot(wx - shoulder[0], wy - shoulder[1], wz - shoulder[2]);
          if (d > md) md = d;
        }
      }
    }
    return md;
  }

  let scale = 1.0;
  let md = maxCornerDist(useW, useH);
  if (md <= reach) return { mapping, scale };

  for (let iter = 0; iter < 4; iter++) {
    scale = Math.min(scale, reach / md);
    md = maxCornerDist(useW * scale, useH * scale);
    if (md <= reach) break;
  }

  return {
    mapping: {
      ...mapping,
      canvasWidth: mapping.canvasWidth * scale,
      canvasHeight: mapping.canvasHeight * scale,
    },
    scale,
  };
}


function unit(v: [number, number, number]): [number, number, number] {
  const L = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / L, v[1] / L, v[2] / L];
}
function unitCross(
  a: [number, number, number], b: [number, number, number],
): [number, number, number] {
  return unit([
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ]);
}
