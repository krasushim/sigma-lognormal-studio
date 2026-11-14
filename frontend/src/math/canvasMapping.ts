/**
 * 画像座標 (image px) ⇔ ワールド座標 (m) のマッピング。
 *
 * Y-up 右手系を採用。キャンバスは法線 (-Z) を robot 側に向け、
 * (x_world, y_world) 平面が画像 (x, y) に対応する (Y軸は Up, 画像Y軸と反転)。
 */

export interface CanvasMappingParams {
  canvasCenter: [number, number, number];   // (x, y, z) [m]
  canvasWidth: number;                       // 水平 [m]
  canvasHeight: number;                      // 垂直 [m]
  /** キャンバス法線 (robot 側を向く単位ベクトル) */
  canvasNormal: [number, number, number];
  /** キャンバス上の "上方向" (キャンバス面内) */
  canvasUp: [number, number, number];
  /** ペン先がキャンバスに押し当てられる深さ [m] (キャンバス法線の負方向へオフセット) */
  contactDepthM: number;
}

export const DEFAULT_MAPPING: CanvasMappingParams = {
  canvasCenter: [0, 0.80, 0.45],
  canvasWidth:  0.35,
  canvasHeight: 0.25,
  canvasNormal: [0, 0, -1],   // 法線は robot (+Z→-Z) を向く
  canvasUp:     [0, 1, 0],
  contactDepthM: 0.002,
};

export interface PenTarget {
  /** ペン先のワールド座標 [m] */
  tip: [number, number, number];
  /** ペン軸 (wrist → tip) のワールド単位ベクトル */
  axis: [number, number, number];
}

function cross3(a: number[], b: number[]): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(v: number[]): [number, number, number] {
  const L = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / L, v[1] / L, v[2] / L];
}

/**
 * 画像座標 (px) を pen target 姿勢 (tip 位置・axis 方向) に写像する。
 *
 * @param px 画像上の座標 [x_px, y_px]  (y は下向き)
 * @param imgW 画像の幅 [px]
 * @param imgH 画像の高さ [px]
 * @param params マッピングパラメータ
 */
export function imagePxToPenTarget(
  px: [number, number],
  imgW: number,
  imgH: number,
  params: CanvasMappingParams,
): PenTarget {
  const n = normalize(params.canvasNormal);        // キャンバス外向き法線
  const up = normalize(params.canvasUp);
  const right = normalize(cross3(up, n));           // Right = Up × Normal (右手系)

  // 画像アスペクト比にキャンバスを "contain" 合わせ (中央配置)
  const imgAspect = imgW / imgH;
  const canAspect = params.canvasWidth / params.canvasHeight;
  let useW = params.canvasWidth;
  let useH = params.canvasHeight;
  if (imgAspect > canAspect) {
    // 画像の方が横長 → 幅基準
    useW = params.canvasWidth;
    useH = params.canvasWidth / imgAspect;
  } else {
    useH = params.canvasHeight;
    useW = params.canvasHeight * imgAspect;
  }

  // 画像座標の正規化: 画像中心を原点、[-0.5, +0.5] に
  const u = px[0] / imgW - 0.5;
  const v = 0.5 - px[1] / imgH;  // y は下向き → 反転

  // キャンバス面上の位置 (ワールド座標)
  const tx = params.canvasCenter[0] + right[0] * (u * useW) + up[0] * (v * useH);
  const ty = params.canvasCenter[1] + right[1] * (u * useW) + up[1] * (v * useH);
  const tz = params.canvasCenter[2] + right[2] * (u * useW) + up[2] * (v * useH);

  // ペン先は法線の逆方向にわずかに押し込む
  const tipX = tx - n[0] * params.contactDepthM;
  const tipY = ty - n[1] * params.contactDepthM;
  const tipZ = tz - n[2] * params.contactDepthM;

  // ペン軸 (wrist→tip) = -法線
  const axis: [number, number, number] = [-n[0], -n[1], -n[2]];
  return { tip: [tipX, tipY, tipZ], axis };
}
