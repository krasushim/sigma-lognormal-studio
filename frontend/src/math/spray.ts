/**
 * Canvas 2D brush rasterization renderer — faithful port of AutoGraff.
 * https://doc.gold.ac.uk/autograff/post/brush/
 *
 *   hat(x, s, func=erf) = func((1 - |x · 2.5|) · s) · 0.5 + 0.5
 *   hat_img(d², s)      — uses *squared* radial distance (as in the blog)
 *
 *   w(t) = w_min + (w_max - w_min) · exp(-|ẋ(t)| / μ_x),   μ_x = mean(|ẋ|)
 *
 *   alpha_blend(a, b, α) = a · (1 - b·α) + b·α
 *     ≡ Canvas2D の source-over with source_alpha = b·α
 *       (ブラシテクスチャの α チャンネルに b を焼き込み、drawImage の globalAlpha で α)
 *
 * 「飛散 (splatter)」は行わない。ブラシテクスチャのスタンピングのみ。
 */
import type { StrokeGroup } from "../api/types";


// ---- Abramowitz–Stegun 7.1.26: relative error < 1.5e-7 -------------------
function erf(x: number): number {
  const a1 = 0.254829592;
  const a2 = -0.284496736;
  const a3 = 1.421413741;
  const a4 = -1.453152027;
  const a5 = 1.061405429;
  const p = 0.3275911;
  const sign = x >= 0 ? 1 : -1;
  const ax = Math.abs(x);
  const t = 1.0 / (1.0 + p * ax);
  const y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * Math.exp(-ax * ax);
  return sign * y;
}

/** hat(u, s) = erf((1 - |u·2.5|)·s)·0.5 + 0.5. u は hat_img の中では squared 距離。 */
function hat(u: number, smoothness: number): number {
  return erf((1.0 - Math.abs(u * 2.5)) * smoothness) * 0.5 + 0.5;
}

/**
 * Uniform Catmull-Rom interpolation at parameter ``a ∈ [0, 1]`` between
 * P1 and P2, given neighbors P0 and P3. At a=0 returns P1, at a=1 returns P2.
 * Endpoint boundary handled by caller via P0=P1 / P3=P2 duplication.
 */
function catmull(
  p0: number, p1: number, p2: number, p3: number, a: number,
): number {
  const a2 = a * a;
  const a3 = a2 * a;
  return 0.5 * (
    (2 * p1)
    + (-p0 + p2) * a
    + (2 * p0 - 5 * p1 + 4 * p2 - p3) * a2
    + (-p0 + 3 * p1 - 3 * p2 + p3) * a3
  );
}


// ---- ブラシパラメータ ---------------------------------------------------

export interface SprayParams {
  /** world 座標での最小幅 (高速時の線の細さ) */
  w_min: number;
  /** world 座標での最大幅 (低速時の線の太さ) */
  w_max: number;
  /** μ_x: 速度正規化定数 [px/s]。**0 以下を渡すと各 group の平均速度で自動正規化する
   *  (AutoGraff 原典: μ_x = mean(|ẋ|))**。固定値を与えると従来互換 (blog 記事デフォルト)。 */
  mu_speed: number;
  /** 縦横比 (height/width)。1.0 で円、1.0以外で楕円 */
  ratio: number;
  /** ブラシ回転 [rad] */
  theta: number;
  /** super-ellipse power。2.0 で正円、大きくするほど角ばる */
  power: number;
  /** hat 関数の鋭さ。小さいほど soft、大きいほど edgy (2〜5) */
  smoothness: number;
  /** スタンプ1つあたりのアルファ (blog では 0.2) */
  alpha: number;
  /** スタンプ時間間隔の上限 [s] */
  stamp_dt: number;
  /** スタンプ間ピクセル距離の上限係数 (半径×この値)。高速時の点線化を防ぐ */
  stamp_overlap: number;
}

export const DEFAULT_SPRAY: SprayParams = {
  w_min: 3.0,
  w_max: 20.0,
  mu_speed: 0.0,     // 0 = auto: group 平均速度で正規化 (AutoGraff 原典式)
  ratio: 1.0,
  theta: 0.0,
  power: 2.0,
  smoothness: 2.3,
  alpha: 0.22,
  stamp_dt: 0.012,
  stamp_overlap: 0.5,
};


// ---- ブラシテクスチャ生成 & キャッシュ --------------------------------

interface BrushTextureKey {
  size: number;
  ratio: number;
  theta: number;
  power: number;
  smoothness: number;
  color: string;  // "r,g,b"
}

const brushCache = new Map<string, HTMLCanvasElement>();

function keyOf(k: BrushTextureKey): string {
  return `${k.size}|${k.ratio.toFixed(3)}|${k.theta.toFixed(3)}|${k.power.toFixed(2)}|${k.smoothness.toFixed(2)}|${k.color}`;
}

/**
 * AutoGraff の hat_img 相当。super-ellipse 距離 + 回転 + 鋭さで brush マスクを作成し、
 * 結果を RGBA 画像 (RGB=color, A = hat(uv, s) · 255) として返す。
 *
 * ブラシの有効範囲は中心から ±1 の正規化距離にマップ。drawImage 時は
 *   drawSize = 2 · R_screen_px
 * とすれば R_screen が visible radius の外周になる。
 */
function generateBrushTexture(
  color: [number, number, number],
  params: SprayParams,
  size: number,
): HTMLCanvasElement {
  const [R, G, B] = color;
  const key = keyOf({
    size,
    ratio: params.ratio,
    theta: params.theta,
    power: params.power,
    smoothness: params.smoothness,
    color: `${R},${G},${B}`,
  });
  const cached = brushCache.get(key);
  if (cached) return cached;

  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  const imgData = ctx.createImageData(size, size);
  const data = imgData.data;

  const half = size / 2;
  const ct = Math.cos(params.theta);
  const st = Math.sin(params.theta);
  const invWx = 1.0;                     // 半径正規化: ±1 が可視範囲
  const invWy = 1.0 / params.ratio;      // 縦を ratio で拡縮
  const pw = Math.max(params.power, 0.1);

  for (let py = 0; py < size; py++) {
    for (let px = 0; px < size; px++) {
      // 正規化座標 [-1..1]
      const nx = (px - half) / half;
      const ny = (py - half) / half;
      // 回転
      const xr = nx * ct - ny * st;
      const yr = nx * st + ny * ct;
      // 楕円スケール + super-ellipse
      const ax = Math.abs(xr * invWx);
      const ay = Math.abs(yr * invWy);
      const d = Math.pow(ax, pw) + Math.pow(ay, pw);  // 距離² 相当 (blog: x² + y² → super-ellipse)
      const v = Math.max(0, Math.min(1, hat(d, params.smoothness)));
      const idx = (py * size + px) * 4;
      data[idx]     = R;
      data[idx + 1] = G;
      data[idx + 2] = B;
      data[idx + 3] = Math.round(v * 255);
    }
  }
  ctx.putImageData(imgData, 0, 0);
  brushCache.set(key, canvas);
  return canvas;
}


// ---- メインレンダラ -----------------------------------------------------

/**
 * 1 stroke group を brush rasterization で描画する。
 * 「飛散 (splatter)」はなし。ブラシテクスチャのスタンピングのみ。
 */
export function renderSprayGroup(
  ctx: CanvasRenderingContext2D,
  group: StrokeGroup,
  rgb: [number, number, number],
  tCutoff: number,
  worldToScreenX: (wx: number) => number,
  worldToScreenY: (wy: number) => number,
  screenPerWorld: number,
  params: SprayParams,
  _seed: number,   // AutoGraff 方式では乱数なし (予約だが未使用)
): void {
  const { t, x, y, speed } = group.derived.timeseries;
  if (t.length < 2 || tCutoff <= 0) return;

  // μ_x: AutoGraff 原典に合わせ、正値が渡されればそれを固定値として採用し、
  // 0 以下 (= auto) なら group の平均速度で正規化する。
  // 速度プロファイル max が大きくても、mean で割ることで exp 項が 0 に飽和せず、
  // slow-at-endpoints/fast-at-mid の変化が幅に現れる。
  let muX = params.mu_speed;
  if (muX <= 0) {
    let sum = 0;
    let cnt = 0;
    for (const s of speed) {
      if (s > 1e-6) {
        sum += s;
        cnt++;
      }
    }
    muX = cnt > 0 ? sum / cnt : 160;
  }
  muX = Math.max(muX, 1e-6);

  // ブラシテクスチャのピクセルサイズ: 最大幅に依存。常に偶数で確保。
  const maxRadiusScreen = Math.max(
    (params.w_max * screenPerWorld) / 2,
    4,
  );
  // 十分な解像度 (ブラシ extent ±1 → size px = ceil(2·maxR) × 2 で余裕)
  const texSize = Math.min(
    512,
    Math.max(64, 2 * Math.ceil(maxRadiusScreen * 2) + 2),
  );
  const brushTex = generateBrushTexture(rgb, params, texSize);

  const prevAlpha = ctx.globalAlpha;
  const prevComp = ctx.globalCompositeOperation;
  ctx.globalCompositeOperation = "source-over";

  const stepMin = Math.max(params.stamp_dt * 0.03, 1e-4);
  const stepMax = params.stamp_dt;
  const maxStamps = 30000;

  let i = 0;
  const tEnd = Math.min(tCutoff, t[t.length - 1]);
  const nSamples = t.length;
  let ti = 0;
  for (let it = 0; it < maxStamps && ti <= tEnd; it++) {
    while (i + 1 < nSamples && t[i + 1] < ti) i++;
    if (i + 1 >= nSamples) break;
    const span = Math.max(t[i + 1] - t[i], 1e-9);
    const a = (ti - t[i]) / span;
    // Centripetal Catmull-Rom on (x, y, speed) using the 4 surrounding samples
    // so stamp positions follow a smooth curve instead of a faceted polyline.
    const im1 = i > 0 ? i - 1 : i;
    const ip1 = i + 1;
    const ip2 = i + 2 < nSamples ? i + 2 : ip1;
    const wx = catmull(x[im1], x[i], x[ip1], x[ip2], a);
    const wy = catmull(y[im1], y[i], y[ip1], y[ip2], a);
    const sp = Math.max(
      0.0,
      catmull(speed[im1], speed[i], speed[ip1], speed[ip2], a),
    );

    const sx = worldToScreenX(wx);
    const sy = worldToScreenY(wy);

    // AutoGraff: w_t = w_min + (w_max - w_min) · exp(-|ẋ| / μ_x)
    const wWorld =
      params.w_min +
      (params.w_max - params.w_min) * Math.exp(-sp / muX);
    const radius = Math.max(wWorld * screenPerWorld * 0.5, 0.5);
    const drawSize = 2 * radius;   // texture の ±1 領域に対応 → full diameter

    ctx.globalAlpha = params.alpha;
    ctx.drawImage(
      brushTex,
      sx - drawSize / 2,
      sy - drawSize / 2,
      drawSize,
      drawSize,
    );

    // 適応的ステップ (時間上限 / 距離上限の最小値)
    const pixelSpeed = sp * screenPerWorld;
    const stepByDist =
      pixelSpeed > 1e-3
        ? (radius * params.stamp_overlap) / pixelSpeed
        : stepMax;
    const step = Math.max(Math.min(stepMax, stepByDist), stepMin);
    ti += step;
  }

  ctx.globalAlpha = prevAlpha;
  ctx.globalCompositeOperation = prevComp;
}


// ---- 廃止された hex→rgb ヘルパはそのまま保持 ---------------------------

export function hexToRgb(hex: string): [number, number, number] {
  const m = hex.replace("#", "");
  if (m.length === 3) {
    return [
      parseInt(m[0] + m[0], 16),
      parseInt(m[1] + m[1], 16),
      parseInt(m[2] + m[2], 16),
    ];
  }
  if (m.length === 6) {
    return [
      parseInt(m.slice(0, 2), 16),
      parseInt(m.slice(2, 4), 16),
      parseInt(m.slice(4, 6), 16),
    ];
  }
  return [17, 24, 39];
}

// Back-compat no-op (callers in TrajectoryCanvas still import this)
export function mulberry32(seed: number): () => number {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), 1 | t);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
