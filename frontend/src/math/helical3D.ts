/**
 * 3D 軌跡生成 (IROS 2016 の helical primitive 思想を採用しつつ、
 * 描画用の 2D 軌跡との一致を最優先にした実装)
 *
 * 設計意図:
 *  - グループ内の軌跡は **backend が出力した `derived.timeseries.{x, y}` を
 *    そのまま使用**する (世界 z=0 = キャンバス表面にマップ)。
 *    これにより 2D 表示と 3D ロボット描画の形状が完全に一致する。
 *  - グループ間の pen-up は `sin(πt)` リフト + `smoothstep` XY ブレンドで
 *    滑らかなアーチ (端で速度 0) として生成。ΣΛ の helical primitive は
 *    再合成時のパラメータ不一致を避けるため使わない。
 *
 * 出力は「時間順に並んだサンプル列」の単一の連続軌跡。
 */
import type { TrajectoryPlan } from "../api/types";
import type { CanvasMappingParams } from "./canvasMapping";


/** 単一の連続 3D 軌跡 (サンプル列) */
export interface HelicalTrajectory {
  /** 絶対時刻 [s] (0 から totalDuration、単調増加) */
  ts: Float32Array;        // [N]
  /** ワールド位置 [m] */
  pos: Float32Array;       // [N, 3]
  /** 速度 [m/s] (中央差分で後から算出) */
  vel: Float32Array;       // [N, 3]
  /** 各サンプルがペンアップ区間か (1) 否か (0) */
  isPenUp: Uint8Array;     // [N]
  /** ペンアップ区間の時間範囲 (描画色分け用) */
  penUpIntervals: [number, number][];
  totalDuration: number;
}


/** デフォルトのペンアップ所要時間 [s] */
const DEFAULT_PEN_UP_DUR_S = 0.35;

/** ペンアップ区間のサンプル数 */
const PEN_UP_SAMPLES = 28;


function smoothstep(t: number): number {
  return t * t * (3 - 2 * t);
}


function unitDir3(v: [number, number, number]): [number, number, number] {
  const L = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / L, v[1] / L, v[2] / L];
}


function cross3(
  a: [number, number, number], b: [number, number, number],
): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}


/**
 * 画像ピクセル座標 → ワールド座標 (キャンバス上、指定 lift で法線方向にオフセット)
 */
function makeImgToWorld3(plan: TrajectoryPlan, mapping: CanvasMappingParams) {
  const n = unitDir3(mapping.canvasNormal);
  const up = unitDir3(mapping.canvasUp);
  const right = unitDir3(cross3(up, n));
  const imgAspect = plan.image_width / plan.image_height;
  const canAspect = mapping.canvasWidth / mapping.canvasHeight;
  let useW = mapping.canvasWidth;
  let useH = mapping.canvasHeight;
  if (imgAspect > canAspect) useH = mapping.canvasWidth / imgAspect;
  else useW = mapping.canvasHeight * imgAspect;
  const center = mapping.canvasCenter;

  return (px: number, py: number, lift = 0): [number, number, number] => {
    const u = px / plan.image_width - 0.5;
    const v = 0.5 - py / plan.image_height;
    const cx = center[0] + right[0] * (u * useW) + up[0] * (v * useH);
    const cy = center[1] + right[1] * (u * useW) + up[1] * (v * useH);
    const cz = center[2] + right[2] * (u * useW) + up[2] * (v * useH);
    // lift>0 で canvasNormal 方向 (robot 側) に離れる。contactDepth は逆向きの押し込み。
    const off = lift - mapping.contactDepthM;
    return [cx + n[0] * off, cy + n[1] * off, cz + n[2] * off];
  };
}


/**
 * TrajectoryPlan (2D) から連続 3D 軌跡を構築する。
 *
 * - グループ内: `derived.timeseries.{t, x, y}` を直接使用、z=0 (canvas 面)
 * - グループ間: 滑らかなペンアップアーチ (sin(πt) リフト + smoothstep XY)
 */
export function buildHelical(
  plan: TrajectoryPlan,
  liftHeightM: number,
  mapping: CanvasMappingParams,
  penUpDurationS: number = DEFAULT_PEN_UP_DUR_S,
): HelicalTrajectory {
  const groups = plan.stroke_groups;
  if (groups.length === 0) {
    return {
      ts: new Float32Array(0), pos: new Float32Array(0),
      vel: new Float32Array(0), isPenUp: new Uint8Array(0),
      penUpIntervals: [], totalDuration: 0,
    };
  }

  const toWorld = makeImgToWorld3(plan, mapping);

  // まず全サンプル数を計算 (in-group + 各グループ間の pen-up)
  let totalN = 0;
  for (const g of groups) totalN += g.derived.timeseries.t.length;
  totalN += (groups.length - 1) * PEN_UP_SAMPLES;

  const ts = new Float32Array(totalN);
  const pos = new Float32Array(totalN * 3);
  const isPenUp = new Uint8Array(totalN);
  const penUpIntervals: [number, number][] = [];

  let w = 0;     // write index
  let cumT = 0;  // 現在の累積時刻

  for (let gi = 0; gi < groups.length; gi++) {
    const g = groups[gi];
    const gts = g.derived.timeseries;
    const Ng = gts.t.length;
    // --- グループ内 ---
    for (let k = 0; k < Ng; k++) {
      ts[w] = cumT + gts.t[k];
      const wp = toWorld(gts.x[k], gts.y[k], 0);
      pos[w * 3 + 0] = wp[0];
      pos[w * 3 + 1] = wp[1];
      pos[w * 3 + 2] = wp[2];
      isPenUp[w] = 0;
      w++;
    }
    cumT += gts.t[Ng - 1] ?? 0;

    // --- グループ間 pen-up arc ---
    if (gi < groups.length - 1) {
      const next = groups[gi + 1];
      const startPx: [number, number] = [gts.x[Ng - 1], gts.y[Ng - 1]];
      const endPx: [number, number] = [
        next.action_plan.p0[0], next.action_plan.p0[1],
      ];
      const penUpStart = cumT;
      for (let k = 0; k < PEN_UP_SAMPLES; k++) {
        const u = k / (PEN_UP_SAMPLES - 1);
        const tk = u * penUpDurationS;
        const lift = liftHeightM * Math.sin(u * Math.PI);     // 端で 0, 中央で peak
        const s = smoothstep(u);                               // XY ease in/out
        const px = startPx[0] * (1 - s) + endPx[0] * s;
        const py = startPx[1] * (1 - s) + endPx[1] * s;
        ts[w] = cumT + tk;
        const wp = toWorld(px, py, lift);
        pos[w * 3 + 0] = wp[0];
        pos[w * 3 + 1] = wp[1];
        pos[w * 3 + 2] = wp[2];
        isPenUp[w] = 1;
        w++;
      }
      cumT += penUpDurationS;
      penUpIntervals.push([penUpStart, cumT]);
    }
  }

  // 速度は中央差分で算出
  const vel = new Float32Array(totalN * 3);
  for (let k = 0; k < totalN; k++) {
    const k0 = Math.max(k - 1, 0);
    const k1 = Math.min(k + 1, totalN - 1);
    const dt = (ts[k1] - ts[k0]) || 1e-6;
    for (let d = 0; d < 3; d++) {
      vel[k * 3 + d] = (pos[k1 * 3 + d] - pos[k0 * 3 + d]) / dt;
    }
  }

  return {
    ts, pos, vel, isPenUp, penUpIntervals, totalDuration: cumT,
  };
}


/**
 * 与えられたサンプル時刻列に対して軌跡を線形補間サンプリング。
 * @returns  { pos: Float32Array [M, 3], isPenUp: Uint8Array [M] }
 */
export function evaluateHelical(
  traj: HelicalTrajectory,
  ts: Float32Array,
): { pos: Float32Array; isPenUp: Uint8Array } {
  const M = ts.length;
  const pos = new Float32Array(M * 3);
  const isPenUp = new Uint8Array(M);
  const N = traj.ts.length;
  if (N === 0) return { pos, isPenUp };

  for (let k = 0; k < M; k++) {
    const T = ts[k];
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (traj.ts[mid] < T) lo = mid + 1; else hi = mid;
    }
    const i1 = lo;
    const i0 = Math.max(i1 - 1, 0);
    if (i0 === i1) {
      pos[k * 3 + 0] = traj.pos[i0 * 3 + 0];
      pos[k * 3 + 1] = traj.pos[i0 * 3 + 1];
      pos[k * 3 + 2] = traj.pos[i0 * 3 + 2];
      isPenUp[k] = traj.isPenUp[i0];
      continue;
    }
    const t0 = traj.ts[i0], t1 = traj.ts[i1];
    const a = (T - t0) / Math.max(t1 - t0, 1e-9);
    for (let d = 0; d < 3; d++) {
      const v0 = traj.pos[i0 * 3 + d];
      const v1 = traj.pos[i1 * 3 + d];
      pos[k * 3 + d] = v0 + a * (v1 - v0);
    }
    // isPenUp は bool なので補間せず、より近いインデックスを採用
    isPenUp[k] = a < 0.5 ? traj.isPenUp[i0] : traj.isPenUp[i1];
  }
  return { pos, isPenUp };
}
