/**
 * 全 timeseries サンプルに対して IK を事前計算し、高速補間でサンプル取得する。
 */
import type { TrajectoryPlan, StrokeGroup } from "../api/types";
import {
  RobotParams, DEFAULT_ROBOT, JointAngles,
  inverseKinematics, homeTheta,
} from "./robotKinematics";
import { CanvasMappingParams, DEFAULT_MAPPING, imagePxToPenTarget } from "./canvasMapping";


export interface GroupCache {
  t: number[];              // 時刻列 [s]
  thetas: Float32Array;     // [N × 6] 連結
  penWorld: Float32Array;   // [N × 3] ペン先ワールド座標
  reachable: boolean[];     // 各サンプル到達可能か
  tEnd: number;             // = t[t.length-1]
}


export class IKCache {
  readonly robot: RobotParams;
  readonly mapping: CanvasMappingParams;
  readonly groups: GroupCache[] = [];
  readonly groupStartTimes: number[] = [];
  readonly totalDuration: number;
  readonly gapS: number;
  /** 1つでも到達不可だった場合 true */
  hasUnreachable = false;

  constructor(
    plan: TrajectoryPlan,
    mapping: CanvasMappingParams = DEFAULT_MAPPING,
    robot: RobotParams = DEFAULT_ROBOT,
    gapBetweenGroupsS: number = 0.3,
    progressCb?: (frac: number) => void,
  ) {
    this.mapping = mapping;
    this.robot = robot;
    this.gapS = gapBetweenGroupsS;

    const totalSamples = plan.stroke_groups.reduce(
      (a, g) => a + g.derived.timeseries.t.length, 0,
    );
    let processed = 0;

    let cumT = 0;
    for (const g of plan.stroke_groups) {
      this.groupStartTimes.push(cumT);
      const cache = this._solveGroup(g, plan.image_width, plan.image_height);
      this.groups.push(cache);
      cumT += cache.tEnd + this.gapS;
      processed += cache.t.length;
      if (progressCb) progressCb(processed / Math.max(totalSamples, 1));
    }
    this.totalDuration = Math.max(0, cumT - this.gapS);
  }

  private _solveGroup(
    g: StrokeGroup, imgW: number, imgH: number,
  ): GroupCache {
    const { t, x, y } = g.derived.timeseries;
    const N = t.length;
    const thetas = new Float32Array(N * 6);
    const penWorld = new Float32Array(N * 3);
    const reachable: boolean[] = new Array(N).fill(false);

    let prevTheta: JointAngles = homeTheta();
    for (let i = 0; i < N; i++) {
      const target = imagePxToPenTarget(
        [x[i], y[i]], imgW, imgH, this.mapping,
      );
      const tipVec = { x: target.tip[0], y: target.tip[1], z: target.tip[2] };
      const axVec = { x: target.axis[0], y: target.axis[1], z: target.axis[2] };
      const res = inverseKinematics(
        this.robot, tipVec, axVec, true, prevTheta,
      );
      for (let k = 0; k < 6; k++) thetas[i * 6 + k] = res.theta[k];
      penWorld[i * 3 + 0] = target.tip[0];
      penWorld[i * 3 + 1] = target.tip[1];
      penWorld[i * 3 + 2] = target.tip[2];
      reachable[i] = res.reachable;
      if (!res.reachable) this.hasUnreachable = true;
      prevTheta = res.theta;
    }
    return { t: [...t], thetas, penWorld, reachable, tEnd: t[N - 1] ?? 0 };
  }

  /** グローバル時刻 T_cur → {gi, localT} */
  resolveTime(T_cur: number): { gi: number; localT: number; active: boolean } {
    for (let gi = 0; gi < this.groups.length; gi++) {
      const start = this.groupStartTimes[gi];
      const endActive = start + this.groups[gi].tEnd;
      if (T_cur < endActive) {
        return { gi, localT: Math.max(0, T_cur - start), active: true };
      }
      // ペンアップ区間
      const endGap = endActive + this.gapS;
      if (T_cur < endGap) {
        return { gi, localT: this.groups[gi].tEnd, active: false };
      }
    }
    // 超過
    const lastGi = this.groups.length - 1;
    return { gi: lastGi, localT: this.groups[lastGi].tEnd, active: false };
  }

  /** 指定グループ・ローカル時刻の関節角を線形補間で返す */
  sampleJointAngles(gi: number, localT: number, out: Float32Array): void {
    const g = this.groups[gi];
    const N = g.t.length;
    if (N === 0) { out.fill(0); return; }
    // Binary search for nearest index
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (g.t[mid] < localT) lo = mid + 1; else hi = mid;
    }
    const i1 = lo;
    const i0 = Math.max(i1 - 1, 0);
    if (i1 === i0) {
      for (let k = 0; k < 6; k++) out[k] = g.thetas[i0 * 6 + k];
      return;
    }
    const t0 = g.t[i0], t1 = g.t[i1];
    const a = (localT - t0) / Math.max(t1 - t0, 1e-9);
    for (let k = 0; k < 6; k++) {
      const v0 = g.thetas[i0 * 6 + k];
      const v1 = g.thetas[i1 * 6 + k];
      out[k] = v0 + a * (v1 - v0);
    }
  }

  /** 指定グループ・ローカル時刻のペン先ワールド座標を線形補間で返す */
  samplePenWorld(gi: number, localT: number, out: [number, number, number]): void {
    const g = this.groups[gi];
    const N = g.t.length;
    if (N === 0) { out[0] = out[1] = out[2] = 0; return; }
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (g.t[mid] < localT) lo = mid + 1; else hi = mid;
    }
    const i1 = lo;
    const i0 = Math.max(i1 - 1, 0);
    if (i1 === i0) {
      out[0] = g.penWorld[i0 * 3 + 0];
      out[1] = g.penWorld[i0 * 3 + 1];
      out[2] = g.penWorld[i0 * 3 + 2];
      return;
    }
    const t0 = g.t[i0], t1 = g.t[i1];
    const a = (localT - t0) / Math.max(t1 - t0, 1e-9);
    for (let k = 0; k < 3; k++) {
      const v0 = g.penWorld[i0 * 3 + k];
      const v1 = g.penWorld[i1 * 3 + k];
      out[k] = v0 + a * (v1 - v0);
    }
  }
}
