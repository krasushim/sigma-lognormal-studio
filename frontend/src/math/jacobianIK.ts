/**
 * 7-DOF アーム: 順運動学 + 数値ヤコビアン + 重み付き擬似逆 Jacobian IK
 * + null-space projection を用いた副タスク (ペン姿勢、関節姿勢)。
 *
 * Berio IROS 2016 §IV (Eq.6-9) の実装。
 *
 * 関節構成 (Y-up, Baxter 風):
 *   J1: S0 shoulder yaw   axis Y
 *   J2: S1 shoulder pitch axis X
 *   J3: E0 upper twist    axis Y    @ +Y(upperArm)
 *   J4: E1 elbow pitch    axis X
 *   J5: W0 forearm twist  axis Y    @ +Y(forearm)
 *   J6: W1 wrist pitch    axis X
 *   J7: W2 wrist roll     axis Y    @ +Y(wristLen)
 *   pen tip = +Y(penLength) from J7
 *
 * 主タスク: pen 位置追従 (Cartesian)
 * 副タスク 1: pen 姿勢 (-Z 方向 ≒ canvas 法線方向)
 * 副タスク 2: Liegeois 関節姿勢 H = (1/7) Σ ((q_i - q_d_i) / (q_max - q_min))²
 */


export type Joint7 = Float32Array;   // length 7


export interface RobotConfig7 {
  baseHeight: number;
  upperArm: number;
  forearm: number;
  wristLen: number;
  penLength: number;
  /** 7 関節の重み (W diag)。高いほど penalize */
  weights: number[];
  /** 関節姿勢ヒューリスティック (副タスク 2) */
  q_d: number[];
  /** 関節制約 [min, max] × 7 */
  jointLimits: [number, number][];
  /** 副タスクゲイン */
  k_psi: number;
  k_d: number;
  /** Damping (regularization) */
  lambda: number;
}


export const DEFAULT_ROBOT_7: RobotConfig7 = {
  baseHeight: 0.30,
  upperArm: 0.50,
  forearm: 0.40,
  wristLen: 0.10,
  penLength: 0.12,
  weights: [1, 3, 1, 1, 1, 1, 1],     // 肩 pitch (J2) を緩く penalize
  q_d:     [0, 0.8, 0, -1.5, 0, 1.1, 0],
  jointLimits: [
    [-Math.PI, Math.PI],
    [-Math.PI / 2, Math.PI / 2],
    [-Math.PI, Math.PI],
    [-Math.PI + 0.1, Math.PI / 4],
    [-Math.PI, Math.PI],
    [-Math.PI / 2, Math.PI / 2],
    [-Math.PI, Math.PI],
  ],
  k_psi: 1.0,
  k_d:   0.3,
  lambda: 0.01,
};


/** 腕の実質的な wrist-center 到達半径 [m] (shoulder 原点から) */
export function armReach(robot: RobotConfig7): number {
  return robot.upperArm + robot.forearm + robot.wristLen;
}


// ===== Forward Kinematics =====

export interface FKResult {
  /** 各関節の世界位置 [7, 3] */
  jointPos: number[][];
  /** 各関節の世界回転軸 (回転軸方向の単位ベクトル) [7, 3] */
  jointAxis: number[][];
  /** End-effector position (pen tip) */
  pen: [number, number, number];
  /** Pen direction (axis from wrist to pen tip), unit vector */
  penAxis: [number, number, number];
}


// 4x4 行列演算 (row-major、列ベクトル変換)
type Mat4 = Float64Array;   // length 16

function mat4Identity(): Mat4 {
  const m = new Float64Array(16);
  m[0] = m[5] = m[10] = m[15] = 1;
  return m;
}

function mat4Mul(a: Mat4, b: Mat4, out?: Mat4): Mat4 {
  const r = out ?? new Float64Array(16);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      r[i * 4 + j] =
        a[i * 4 + 0] * b[0 * 4 + j] +
        a[i * 4 + 1] * b[1 * 4 + j] +
        a[i * 4 + 2] * b[2 * 4 + j] +
        a[i * 4 + 3] * b[3 * 4 + j];
    }
  }
  return r;
}

function mat4Translate(x: number, y: number, z: number): Mat4 {
  const m = mat4Identity();
  m[3] = x; m[7] = y; m[11] = z;
  return m;
}

/** Rotation about X axis */
function mat4RotX(a: number): Mat4 {
  const c = Math.cos(a), s = Math.sin(a);
  const m = mat4Identity();
  m[5] = c; m[6] = -s; m[9] = s; m[10] = c;
  return m;
}

/** Rotation about Y axis */
function mat4RotY(a: number): Mat4 {
  const c = Math.cos(a), s = Math.sin(a);
  const m = mat4Identity();
  m[0] = c; m[2] = s; m[8] = -s; m[10] = c;
  return m;
}

/** transform a 3D point */
function mat4TransformPoint(m: Mat4, p: [number, number, number]): [number, number, number] {
  return [
    m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
    m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
    m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
  ];
}

/** transform a 3D direction (no translation) */
function mat4TransformDir(m: Mat4, p: [number, number, number]): [number, number, number] {
  return [
    m[0] * p[0] + m[1] * p[1] + m[2] * p[2],
    m[4] * p[0] + m[5] * p[1] + m[6] * p[2],
    m[8] * p[0] + m[9] * p[1] + m[10] * p[2],
  ];
}


// 関節定義: 各関節の (前リンクからの translate, rotation axis)
//   axis = "X" | "Y" を採用
const JOINT_AXES: ("X" | "Y")[] = ["Y", "X", "Y", "X", "Y", "X", "Y"];

/** 関節 i に到達するまでの translate を返す */
function jointTranslate(robot: RobotConfig7, i: number): [number, number, number] {
  // J1: (0, baseHeight, 0)
  // J2: (0, 0, 0)
  // J3: (0, upperArm, 0)
  // J4: (0, 0, 0)
  // J5: (0, forearm, 0)
  // J6: (0, 0, 0)
  // J7: (0, wristLen, 0)
  switch (i) {
    case 0: return [0, robot.baseHeight, 0];
    case 1: return [0, 0, 0];
    case 2: return [0, robot.upperArm, 0];
    case 3: return [0, 0, 0];
    case 4: return [0, robot.forearm, 0];
    case 5: return [0, 0, 0];
    case 6: return [0, robot.wristLen, 0];
    default: return [0, 0, 0];
  }
}


export function fk7(robot: RobotConfig7, q: Joint7): FKResult {
  const jointPos: number[][] = [];
  const jointAxis: number[][] = [];

  let acc = mat4Identity();

  for (let i = 0; i < 7; i++) {
    // 1) translate to joint i origin
    const tr = jointTranslate(robot, i);
    acc = mat4Mul(acc, mat4Translate(tr[0], tr[1], tr[2]));
    // record world joint position (origin of rotation)
    jointPos.push(mat4TransformPoint(acc, [0, 0, 0]));
    // joint axis in world: depends on axis type
    const axisLocal: [number, number, number] = JOINT_AXES[i] === "Y" ? [0, 1, 0] : [1, 0, 0];
    jointAxis.push(mat4TransformDir(acc, axisLocal));
    // 2) apply rotation
    const R = JOINT_AXES[i] === "Y" ? mat4RotY(q[i]) : mat4RotX(q[i]);
    acc = mat4Mul(acc, R);
  }

  // pen tip = (0, penLength, 0) in J7 frame
  const pen = mat4TransformPoint(acc, [0, robot.penLength, 0]);
  // pen axis = (0, 1, 0) direction in J7 frame
  const penAxis = mat4TransformDir(acc, [0, 1, 0]);

  return { jointPos, jointAxis, pen, penAxis };
}


// ===== Numerical Jacobian =====

/**
 * Position Jacobian J_x ∈ R^{3×7}:  J_x[:, i] = ∂p_pen / ∂q_i
 * 解析的に: revolute joint i は p_pen に対して a_i × (p_pen - p_i) を寄与する
 * ここで a_i は world 座標での回転軸、p_i は joint i の世界位置。
 */
export function jacobianPos(fk: FKResult): number[][] {
  const J: number[][] = [[0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0]];
  for (let i = 0; i < 7; i++) {
    const ax = fk.jointAxis[i];
    const dx = fk.pen[0] - fk.jointPos[i][0];
    const dy = fk.pen[1] - fk.jointPos[i][1];
    const dz = fk.pen[2] - fk.jointPos[i][2];
    // a × (p_pen - p_i)
    J[0][i] = ax[1] * dz - ax[2] * dy;
    J[1][i] = ax[2] * dx - ax[0] * dz;
    J[2][i] = ax[0] * dy - ax[1] * dx;
  }
  return J;
}


/**
 * Orientation Jacobian J_ψ ∈ R^{3×7}: 角速度寄与 = a_i (関節軸そのもの)
 */
export function jacobianOri(fk: FKResult): number[][] {
  const J: number[][] = [[0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0]];
  for (let i = 0; i < 7; i++) {
    J[0][i] = fk.jointAxis[i][0];
    J[1][i] = fk.jointAxis[i][1];
    J[2][i] = fk.jointAxis[i][2];
  }
  return J;
}


// ===== Linear algebra (small matrices) =====

/** 3x3 行列の逆 */
function inv3(m: number[][]): number[][] {
  const a = m[0][0], b = m[0][1], c = m[0][2];
  const d = m[1][0], e = m[1][1], f = m[1][2];
  const g = m[2][0], h = m[2][1], i = m[2][2];
  const A = e * i - f * h;
  const B = -(d * i - f * g);
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-12) {
    // Singular; return identity-like with damping (caller adds λI).
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  }
  const invDet = 1 / det;
  return [
    [A * invDet, -(b * i - c * h) * invDet, (b * f - c * e) * invDet],
    [B * invDet,  (a * i - c * g) * invDet, -(a * f - c * d) * invDet],
    [C * invDet, -(a * h - b * g) * invDet, (a * e - b * d) * invDet],
  ];
}


// ===== Weighted pseudo-inverse =====

/**
 * J*  = W^-1 J^T (J W^-1 J^T + λI)^-1   ∈ R^{7×3}
 */
function weightedPinv(
  J: number[][],         // [3, 7]
  weights: number[],     // [7]
  lambda: number,
): number[][] {
  // W^-1 = diag(1/w_i)
  // J W^-1 = J with each column divided by w_i
  // (J W^-1 J^T) ∈ R^{3,3}
  const M: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 7; k++) s += J[i][k] * (1 / weights[k]) * J[j][k];
      M[i][j] = s + (i === j ? lambda : 0);
    }
  }
  const Minv = inv3(M);
  // J* = (W^-1 J^T) Minv  ∈ R^{7,3}
  const out: number[][] = [];
  for (let r = 0; r < 7; r++) {
    out.push([0, 0, 0]);
    for (let c = 0; c < 3; c++) {
      let s = 0;
      for (let k = 0; k < 3; k++) {
        s += (1 / weights[r]) * J[k][r] * Minv[k][c];
      }
      out[r][c] = s;
    }
  }
  return out;
}


// ===== Null space projector =====

/**
 * N = I - J^T J*    ∈ R^{7×7}
 */
function nullProjector(J: number[][], Jstar: number[][]): number[][] {
  // J^T J* ∈ R^{7,7}
  const N: number[][] = [];
  for (let i = 0; i < 7; i++) {
    N.push([]);
    for (let j = 0; j < 7; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += J[k][i] * Jstar[j][k];
      N[i].push((i === j ? 1 : 0) - s);
    }
  }
  return N;
}


// ===== Posture cost gradient =====

function postureGradient(
  q: Float32Array,
  q_d: number[],
  limits: [number, number][],
): number[] {
  // ∇H_i = (2/7) (q_i - q_d_i) / (q_max_i - q_min_i)²
  const grad: number[] = [];
  for (let i = 0; i < 7; i++) {
    const span = limits[i][1] - limits[i][0] || 1;
    grad.push(-(2 / 7) * (q[i] - q_d[i]) / (span * span));
  }
  return grad;
}


// ===== Orientation error =====

/**
 * 現在のペン姿勢 (penAxis) と目標 (targetAxis) の差分を回転ベクトル e_ψ で表す。
 * cross(current, target) の長さは sin(angle) になる。
 */
function orientationError(
  current: [number, number, number],
  target: [number, number, number],
): [number, number, number] {
  // cross(current, target) → 回転軸 (魔法ベクトル)
  return [
    current[1] * target[2] - current[2] * target[1],
    current[2] * target[0] - current[0] * target[2],
    current[0] * target[1] - current[1] * target[0],
  ];
}


// ===== IK Step =====

export interface IKStepResult {
  qNext: Float32Array;
  residualPos: number;
  residualOri: number;
}


/**
 * 1 ステップの IK 更新。
 * @param robot       7DOF 設定
 * @param q           現在の関節角
 * @param targetPos   目標 pen tip [m]
 * @param targetAxis  目標 pen 軸 (wrist→tip 方向、unit)
 * @param dt          時間ステップ [s]
 */
export function ikStep(
  robot: RobotConfig7,
  q: Joint7,
  targetPos: [number, number, number],
  targetAxis: [number, number, number],
  dt: number,
): IKStepResult {
  const fk = fk7(robot, q);
  const J_x = jacobianPos(fk);
  const J_psi = jacobianOri(fk);

  // 主タスク残差: ẋ_desired = (target - current) / dt
  const ex = (targetPos[0] - fk.pen[0]) / dt;
  const ey = (targetPos[1] - fk.pen[1]) / dt;
  const ez = (targetPos[2] - fk.pen[2]) / dt;

  // 重み付き擬似逆
  const Jstar_x = weightedPinv(J_x, robot.weights, robot.lambda);

  // q̇ from primary task
  const qdot = new Float32Array(7);
  for (let i = 0; i < 7; i++) {
    qdot[i] = Jstar_x[i][0] * ex + Jstar_x[i][1] * ey + Jstar_x[i][2] * ez;
  }

  // null projector
  const N = nullProjector(J_x, Jstar_x);

  // 副タスク 1: ペン姿勢
  const e_psi = orientationError(fk.penAxis, targetAxis);
  const Jstar_psi = weightedPinv(J_psi, robot.weights, robot.lambda);
  const qdot_psi = new Float32Array(7);
  for (let i = 0; i < 7; i++) {
    qdot_psi[i] = Jstar_psi[i][0] * e_psi[0] + Jstar_psi[i][1] * e_psi[1] + Jstar_psi[i][2] * e_psi[2];
    qdot_psi[i] *= robot.k_psi;
  }
  // project into null space
  const qdot_psi_proj = matVec(N, qdot_psi);

  // 副タスク 2: 関節姿勢
  const grad_H = postureGradient(q, robot.q_d, robot.jointLimits);
  const qdot_d = grad_H.map((g) => g * robot.k_d);
  const qdot_d_proj = matVec(N, qdot_d);

  // 合成
  const qNext = new Float32Array(7);
  for (let i = 0; i < 7; i++) {
    let v = qdot[i] + qdot_psi_proj[i] + qdot_d_proj[i];
    qNext[i] = clampAng(q[i] + v * dt, robot.jointLimits[i]);
  }

  // 検証 FK
  const fkNext = fk7(robot, qNext);
  const residualPos = Math.hypot(
    targetPos[0] - fkNext.pen[0],
    targetPos[1] - fkNext.pen[1],
    targetPos[2] - fkNext.pen[2],
  );
  const residualOri = Math.hypot(
    fkNext.penAxis[0] - targetAxis[0],
    fkNext.penAxis[1] - targetAxis[1],
    fkNext.penAxis[2] - targetAxis[2],
  );
  return { qNext, residualPos, residualOri };
}


function matVec(M: number[][], v: ArrayLike<number>): Float32Array {
  const n = M.length;
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let j = 0; j < (M[i] as number[]).length; j++) s += M[i][j] * v[j];
    out[i] = s;
  }
  return out;
}

function clampAng(v: number, lim: [number, number]): number {
  return v < lim[0] ? lim[0] : v > lim[1] ? lim[1] : v;
}


/**
 * 軌跡全体 (時系列) を IK で追従。前サンプルを warm-start として連続 ikStep を呼ぶ。
 *
 * @param positions Float32Array [N, 3] 望まれるペン先位置 (世界座標)
 * @param axes      Float32Array [N, 3] 望まれるペン軸 (世界座標)
 * @param ts        Float32Array [N] サンプル時刻
 */
export function ikTrajectory(
  robot: RobotConfig7,
  positions: Float32Array,
  axes: Float32Array,
  ts: Float32Array,
  qInit?: Joint7,
): { q: Float32Array; reachableMask: Uint8Array; penWorld: Float32Array } {
  const N = ts.length;
  const q_out = new Float32Array(N * 7);
  const reach = new Uint8Array(N);
  const pen = new Float32Array(N * 3);

  let q: Joint7 = qInit ? new Float32Array(qInit) : new Float32Array(robot.q_d);

  // --- 最初のサンプル: warmup で収束まで十分反復 (ホームポーズから遠い可能性) ---
  const firstTarget: [number, number, number] = [positions[0], positions[1], positions[2]];
  const firstAxis: [number, number, number] = [axes[0], axes[1], axes[2]];
  for (let warm = 0; warm < 30; warm++) {
    const r = ikStep(robot, q, firstTarget, firstAxis, 0.02);
    q = r.qNext;
    if (r.residualPos < 1e-3) break;
  }

  for (let k = 0; k < N; k++) {
    const dt = k === 0 ? 0.02 : Math.max(ts[k] - ts[k - 1], 1e-3);
    const targetPos: [number, number, number] = [
      positions[k * 3 + 0], positions[k * 3 + 1], positions[k * 3 + 2],
    ];
    const targetAxis: [number, number, number] = [
      axes[k * 3 + 0], axes[k * 3 + 1], axes[k * 3 + 2],
    ];
    // 各サンプルで multi-pass (収束が速い warm-start なので 3 回で十分)
    let r = ikStep(robot, q, targetPos, targetAxis, dt);
    q = r.qNext;
    for (let refine = 0; refine < 2; refine++) {
      if (r.residualPos < 5e-4) break;
      r = ikStep(robot, q, targetPos, targetAxis, dt);
      q = r.qNext;
    }
    for (let i = 0; i < 7; i++) q_out[k * 7 + i] = q[i];
    reach[k] = r.residualPos < 0.02 ? 1 : 0;   // 2cm 以内なら OK
    // record actual pen tip
    const fkR = fk7(robot, q);
    pen[k * 3 + 0] = fkR.pen[0];
    pen[k * 3 + 1] = fkR.pen[1];
    pen[k * 3 + 2] = fkR.pen[2];
  }
  return { q: q_out, reachableMask: reach, penWorld: pen };
}
