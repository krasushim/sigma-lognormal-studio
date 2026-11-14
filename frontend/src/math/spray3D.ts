/**
 * Canvas 2D 上のスプレー描画 (3D ロボット表示用).
 *
 * 2D 版 `spray.ts` のスタンプロジックを再利用するため、
 * 3D world 座標を「仮想画像ピクセル座標」(= 描画先のオフスクリーン canvas 画素)
 * に写像してから同じレンダラを適用する。
 *
 * 主な入力:
 *   - 軌跡サンプル pos [N, 3] (world [m])
 *   - canvasMapping (world↔canvas UV 変換パラメータ)
 *   - isPenUp [N]: 1 のサンプルはペンが浮いている → スタンプしない
 *   - tCutoff: 現時刻まで描画
 */
import type { CanvasMappingParams } from "./canvasMapping";
import {
  DEFAULT_SPRAY, SprayParams, hexToRgb,
} from "./spray";


interface UVSample {
  u: number;      // 0..1
  v: number;      // 0..1
  t: number;      // seconds
  speed: number;  // m/s
  penUp: boolean;
}


/**
 * world (m) 位置 p を canvas 面 UV (0..1) に投影する。
 * canvas 面外のサンプルは範囲外の (u, v) を返す (呼び出し側で UV ∈ [0,1] 確認)。
 */
function worldToCanvasUV(
  p: [number, number, number],
  center: [number, number, number],
  right: [number, number, number],
  up: [number, number, number],
  useW: number,
  useH: number,
): [number, number] {
  const dx = p[0] - center[0];
  const dy = p[1] - center[1];
  const dz = p[2] - center[2];
  const localRight = dx * right[0] + dy * right[1] + dz * right[2];
  const localUp = dx * up[0] + dy * up[1] + dz * up[2];
  return [localRight / useW + 0.5, 0.5 - localUp / useH];
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


/**
 * 3D 軌跡サンプル → UV + 速度 に変換。
 */
export function projectSamplesToUV(
  pos: Float32Array,            // [N, 3]
  ts: Float32Array,             // [N]
  isPenUp: Uint8Array,          // [N]
  mapping: CanvasMappingParams,
  planImageW: number,
  planImageH: number,
): UVSample[] {
  const N = ts.length;
  const n = unit(mapping.canvasNormal);
  const up = unit(mapping.canvasUp);
  const right = unitCross(up, n);
  // canvas の実効寸法 (contain fit)
  const imgAspect = planImageW / planImageH;
  const canAspect = mapping.canvasWidth / mapping.canvasHeight;
  let useW = mapping.canvasWidth;
  let useH = mapping.canvasHeight;
  if (imgAspect > canAspect) useH = mapping.canvasWidth / imgAspect;
  else useW = mapping.canvasHeight * imgAspect;

  const samples: UVSample[] = new Array(N);
  for (let k = 0; k < N; k++) {
    const p: [number, number, number] = [
      pos[k * 3 + 0], pos[k * 3 + 1], pos[k * 3 + 2],
    ];
    const [u, v] = worldToCanvasUV(p, mapping.canvasCenter, right, up, useW, useH);
    // 速度: 中央差分 (m/s, world)
    const k0 = Math.max(k - 1, 0);
    const k1 = Math.min(k + 1, N - 1);
    const dt = (ts[k1] - ts[k0]) || 1e-6;
    const dx = pos[k1 * 3 + 0] - pos[k0 * 3 + 0];
    const dy = pos[k1 * 3 + 1] - pos[k0 * 3 + 1];
    const dz = pos[k1 * 3 + 2] - pos[k0 * 3 + 2];
    const speedWorld = Math.hypot(dx, dy, dz) / dt;     // m/s
    samples[k] = {
      u, v, t: ts[k], speed: speedWorld, penUp: isPenUp[k] === 1,
    };
  }
  return samples;
}


/**
 * オフスクリーン canvas に spray を描画する。
 * - tCutoff までのサンプルのみ描画
 * - pen-up サンプルはスキップ (ペンが浮いているのでインクが付かない)
 * - 2D 版と同じ AutoGraff 方式のブラシスタンピング
 */
export function renderSpray3DOnCanvas(
  ctx: CanvasRenderingContext2D,
  canvasW: number, canvasH: number,
  samples: UVSample[],
  tCutoff: number,
  params: SprayParams,
  colorHex: string = "#111827",
): void {
  if (samples.length < 2) return;
  ctx.clearRect(0, 0, canvasW, canvasH);

  // ブラシテクスチャ (sprayHex) 取得 — 2D 版の generateBrushTexture を内部で再利用
  // 直接 radial gradient で描画 (軽量版、2D spray.ts と同じ振る舞い)
  const [R, G, B] = hexToRgb(colorHex);

  // 画面内速度 (pixel/s) を "virtual" 基準スケールで近似する。
  //   mu_speed は m/s の固定定数 → pixel/s 相当へ変換するため、
  //   ここでは canvas pixel の「世界→画素」比を使う。
  //   簡易版: speed (m/s) / mu_speed_m (0.1 m/s) を適切に expt するだけで OK。
  //   w_min / w_max は canvas pixel 単位で設定済み (params.w_min, w_max)。
  //   AutoGraff の式: w = w_min + (w_max - w_min) * exp(-|v| / μ_x)
  const mu_speed_m = 0.5;     // 50cm/s 相当でスケール

  const stamp_dt = params.stamp_dt;
  const stamp_overlap = params.stamp_overlap;
  const stepMin = Math.max(stamp_dt * 0.03, 1e-4);
  const stepMax = stamp_dt;
  const maxStamps = 30000;

  const tEnd = Math.min(tCutoff, samples[samples.length - 1].t);
  let ti = samples[0].t;

  // 補間ヘルパ
  let si = 0;
  function sampleAt(t: number): { u: number; v: number; speed: number; penUp: boolean } {
    while (si + 1 < samples.length && samples[si + 1].t < t) si++;
    const i0 = si;
    const i1 = Math.min(si + 1, samples.length - 1);
    if (i0 === i1) return samples[i0];
    const s0 = samples[i0], s1 = samples[i1];
    const span = Math.max(s1.t - s0.t, 1e-9);
    const a = (t - s0.t) / span;
    return {
      u: s0.u + a * (s1.u - s0.u),
      v: s0.v + a * (s1.v - s0.v),
      speed: s0.speed + a * (s1.speed - s0.speed),
      penUp: a < 0.5 ? s0.penUp : s1.penUp,
    };
  }

  for (let it = 0; it < maxStamps && ti <= tEnd; it++) {
    const s = sampleAt(ti);
    if (!s.penUp) {
      // UV を pixel に変換
      const sx = s.u * canvasW;
      const sy = s.v * canvasH;
      // 範囲外は skip (canvas 面の外)
      if (sx >= 0 && sx < canvasW && sy >= 0 && sy < canvasH) {
        // AutoGraff width 式 (m/s 基準)
        const w = params.w_min
          + (params.w_max - params.w_min) * Math.exp(-s.speed / mu_speed_m);
        const radius = Math.max(w * 0.5, 0.5);

        // Core radial gradient stamp
        const ac = params.alpha;
        const grad = ctx.createRadialGradient(sx, sy, 0, sx, sy, radius);
        grad.addColorStop(0.0,  `rgba(${R},${G},${B},${ac})`);
        grad.addColorStop(0.45, `rgba(${R},${G},${B},${ac * 0.95})`);
        grad.addColorStop(0.75, `rgba(${R},${G},${B},${ac * 0.45})`);
        grad.addColorStop(1.0,  `rgba(${R},${G},${B},0)`);
        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.arc(sx, sy, radius, 0, Math.PI * 2);
        ctx.fill();

        // 中心濃化
        const innerR = Math.max(radius * 0.35, 0.6);
        ctx.fillStyle = `rgba(${R},${G},${B},${Math.min(ac * 0.9, 0.7)})`;
        ctx.beginPath();
        ctx.arc(sx, sy, innerR, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 適応的ステップ: 距離上限あり (点線化を防止)
    const pixelSpeed = Math.hypot(
      (s.u * canvasW - 0), (s.v * canvasH - 0),   // 位置差分を近似的に使う代わり、速度を画素速度へ
    );
    // 簡易: UV 速度から pixel 速度を近似 (duration 分だけ進むと仮定)
    // ここでは s.speed (m/s) × canvas_px_per_m を大雑把に使う
    const pxPerM = (canvasW) / (Math.max(1e-6, 0.3));   // canvas 物理幅 ~0.3m 想定
    const pxSp = Math.max(s.speed * pxPerM, 1);
    const radiusPx = params.w_min
      + (params.w_max - params.w_min) * Math.exp(-s.speed / mu_speed_m);
    const radiusHalf = Math.max(radiusPx * 0.5, 0.5);
    const stepByDist = pxSp > 1e-3
      ? (radiusHalf * stamp_overlap) / pxSp
      : stepMax;
    const step = Math.max(Math.min(stepMax, stepByDist), stepMin);
    ti += step;
  }
}


export { DEFAULT_SPRAY };
export type { SprayParams };
