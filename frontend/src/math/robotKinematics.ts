/**
 * 6DOF パラメトリック・アームの順運動学 (FK) と逆運動学 (IK)。
 *
 * Y-up 座標系 (Three.js デフォルト)。関節配置:
 *   J1: Y (base yaw)        — 基底回転
 *   J2: X (shoulder pitch)  — 肩ピッチ (arm を前方へ倒す)
 *   J3: X (elbow pitch)     — 肘ピッチ
 *   J4: Y (wrist yaw)       — 手首ヨー
 *   J5: X (wrist pitch)     — 手首ピッチ
 *   J6: Y (wrist roll)      — 手首ロール (ペンは軸対称なので事実上自由変数)
 *
 * 零位姿勢 (全関節0) では、ペンは +Y 方向に直立する:
 *   tip = (0, baseHeight + upperArm + forearm + penLength, 0)
 *
 * Chain (base → tip):
 *   origin → Ry(θ1) → +Y(baseHeight)
 *          → Rx(θ2) → +Y(upperArm)
 *          → Rx(θ3) → +Y(forearm)        = wrist center
 *          → Ry(θ4) → Rx(θ5) → Ry(θ6)
 *          → +Y(penLength)                 = pen tip
 *
 * IK (解析解):
 *   1. 目標ペン先 P_tip とペン軸 (ペンが +Y から指す向き) w から
 *      手首中心 P_wc = P_tip - penLength · w を計算
 *   2. J1..J3 を 2-link 平面 IK で求解 (エルボー up 構成)
 *   3. アーム末端 (手首) のワールド回転 R_arm = Ry(θ1)·Rx(θ2+θ3) を計算
 *      (J2, J3 は同じ回転軸の合成になるので θ2+θ3 で OK)
 *   4. target_local = R_arm^T · w を Y-X-Y オイラーで分解
 *      → θ4 = atan2(tx, tz),  θ5 = acos(ty),  θ6 = 0
 */
export interface RobotParams {
  baseHeight: number;     // ベース高さ [m]
  upperArm: number;       // 上腕長 [m]
  forearm: number;        // 前腕長 [m]
  penLength: number;      // ペン長 [m]
  jointLimits: Array<[number, number]>;   // 6 関節の [min, max]
}

export const DEFAULT_ROBOT: RobotParams = {
  baseHeight: 0.25,
  upperArm: 0.45,
  forearm: 0.35,
  penLength: 0.12,
  jointLimits: [
    [-Math.PI, Math.PI],        // J1
    [-Math.PI * 0.9, Math.PI * 0.9],  // J2
    [-Math.PI * 0.95, Math.PI * 0.95],// J3
    [-Math.PI, Math.PI],        // J4
    [-Math.PI * 0.95, Math.PI * 0.95],// J5
    [-Math.PI, Math.PI],        // J6
  ],
};

export type JointAngles = Float32Array;   // length 6

export interface Vec3 { x: number; y: number; z: number }

function dot3(a: Vec3, b: Vec3) { return a.x * b.x + a.y * b.y + a.z * b.z; }
function len3(v: Vec3) { return Math.hypot(v.x, v.y, v.z); }
function normalize3(v: Vec3): Vec3 {
  const L = len3(v) || 1;
  return { x: v.x / L, y: v.y / L, z: v.z / L };
}
function clamp(v: number, a: number, b: number) { return v < a ? a : v > b ? b : v; }
function clampLimit(v: number, lim: [number, number]) { return clamp(v, lim[0], lim[1]); }


// ===== Forward Kinematics =====

export interface FKResult {
  wristCenter: Vec3;    // J4 の原点 (手首中心)
  penTip: Vec3;         // ペン先
  shoulder: Vec3;
  elbow: Vec3;
}

export function forwardKinematics(
  robot: RobotParams, theta: JointAngles,
): FKResult {
  const [t1, t2, t3, t4, t5, _t6] = theta;
  const c1 = Math.cos(t1), s1 = Math.sin(t1);
  const c23 = Math.cos(t2 + t3), s23 = Math.sin(t2 + t3);
  const c2 = Math.cos(t2), s2 = Math.sin(t2);

  const shoulder: Vec3 = { x: 0, y: robot.baseHeight, z: 0 };

  // Upper arm tip (= elbow) in shoulder-local: Rx(θ2) · [0, upperArm, 0]
  //   = [0, upperArm·cos(θ2), upperArm·sin(θ2)]
  // Then rotate by Ry(θ1) and translate shoulder:
  const elbowLocal = { x: 0, y: robot.upperArm * c2, z: robot.upperArm * s2 };
  const elbow: Vec3 = {
    x: shoulder.x + elbowLocal.x * c1 + elbowLocal.z * s1,
    y: shoulder.y + elbowLocal.y,
    z: shoulder.z - elbowLocal.x * s1 + elbowLocal.z * c1,
  };

  // Forearm tip (= wrist center)
  // In shoulder frame: elbow + Rx(θ2+θ3)·[0, forearm, 0]
  //   = elbowLocal + [0, forearm·cos(θ2+θ3), forearm·sin(θ2+θ3)]
  const wristLocal = {
    x: 0,
    y: elbowLocal.y + robot.forearm * c23,
    z: elbowLocal.z + robot.forearm * s23,
  };
  const wristCenter: Vec3 = {
    x: shoulder.x + wristLocal.x * c1 + wristLocal.z * s1,
    y: shoulder.y + wristLocal.y,
    z: shoulder.z - wristLocal.x * s1 + wristLocal.z * c1,
  };

  // Pen tip:
  // R_arm (up to wrist center) = Ry(θ1) · Rx(θ2+θ3)
  // R_wrist = Ry(θ4) · Rx(θ5) · Ry(θ6)
  // Pen rest axis = (0, 1, 0) in local.
  // Pen axis in world = R_arm · R_wrist · (0,1,0)
  // Ry(θ6) · (0,1,0) = (0,1,0) (invariant)
  // Rx(θ5) · (0,1,0) = (0, cos θ5, sin θ5)
  // Ry(θ4) · (0, cos θ5, sin θ5) = (sin θ4 · sin θ5, cos θ5, cos θ4 · sin θ5)
  const ss = Math.sin(t5);
  const axisWristLocal = {
    x: Math.sin(t4) * ss,
    y: Math.cos(t5),
    z: Math.cos(t4) * ss,
  };
  // Apply R_arm = Ry(θ1) · Rx(θ2+θ3)
  // Rx(θ2+θ3) · axisWristLocal:
  const ax = axisWristLocal.x;
  const ay = axisWristLocal.y * c23 - axisWristLocal.z * s23;
  const az = axisWristLocal.y * s23 + axisWristLocal.z * c23;
  // Ry(θ1):
  const penAxisWorld = {
    x: ax * c1 + az * s1,
    y: ay,
    z: -ax * s1 + az * c1,
  };
  const penTip: Vec3 = {
    x: wristCenter.x + robot.penLength * penAxisWorld.x,
    y: wristCenter.y + robot.penLength * penAxisWorld.y,
    z: wristCenter.z + robot.penLength * penAxisWorld.z,
  };
  return { shoulder, elbow, wristCenter, penTip };
}


// ===== Inverse Kinematics =====

export interface IKResult {
  theta: JointAngles;
  reachable: boolean;
  residualPosM: number;
}

/**
 * ペン先ワールド位置 P_tip とペン軸ワールド方向 w から関節角を求める。
 * @param preferElbowUp 肘が上向き (target より高い) になる構成を選ぶ
 */
export function inverseKinematics(
  robot: RobotParams,
  penTipWorld: Vec3,
  penAxisWorld: Vec3,
  preferElbowUp: boolean = true,
  previousTheta?: JointAngles,
): IKResult {
  const w = normalize3(penAxisWorld);
  // 1. 手首中心 = ペン先 − ペン長 · ペン軸
  const wc: Vec3 = {
    x: penTipWorld.x - robot.penLength * w.x,
    y: penTipWorld.y - robot.penLength * w.y,
    z: penTipWorld.z - robot.penLength * w.z,
  };

  // 2. θ1 = atan2(wc.x, wc.z)   (+Z 前方を向く基底)
  const theta1 = Math.atan2(wc.x, wc.z);

  // 平面 IK: 肩座標系 (J1 適用後、shoulder 原点に移動) での (forward, up) 座標へ
  //   forward = sqrt(wc.x² + wc.z²)  (原点から見た水平距離)
  //   up      = wc.y - baseHeight
  const forward = Math.hypot(wc.x, wc.z);
  const up = wc.y - robot.baseHeight;
  const D2 = forward * forward + up * up;
  const D = Math.sqrt(D2);
  const L1 = robot.upperArm;
  const L2 = robot.forearm;
  const minReach = Math.abs(L1 - L2);
  const maxReach = L1 + L2;
  const reachable = D <= maxReach - 1e-4 && D >= minReach + 1e-4;
  let Dclamp = D;
  if (!reachable) Dclamp = Math.max(minReach + 1e-3, Math.min(maxReach - 1e-3, D));
  const D2c = Dclamp * Dclamp;

  // 3. θ3: elbow 内角 (肘における 2 本の辺の間の角度変化)
  //    Target 式: L1·cos θ2 + L2·cos(θ2+θ3) = up,
  //               L1·sin θ2 + L2·sin(θ2+θ3) = forward
  //    D² = L1² + L2² + 2 L1 L2 cos θ3
  //    → cos θ3 = (D² − L1² − L2²) / (2 L1 L2)
  const cos3 = clamp((D2c - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
  const elbowAbs = Math.acos(cos3);   // ∈ [0, π]
  const theta3 = preferElbowUp ? -elbowAbs : elbowAbs;

  // 4. θ2
  //   α = atan2(forward, up) = 肩から手首への方向 (+Y からの角度)
  //   β = atan2(L2·sin θ3, L1 + L2·cos θ3)
  //   elbow-up: θ2 = α − β;   elbow-down: θ2 = α + β
  const alpha = Math.atan2(forward, up);
  const beta = Math.atan2(L2 * Math.sin(elbowAbs), L1 + L2 * Math.cos(elbowAbs));
  const theta2 = preferElbowUp ? alpha - beta : alpha + beta;

  // 5. Wrist orientation (Y-X-Y): target_local = R_arm^T · w
  //    R_arm = Ry(θ1) · Rx(θ2+θ3)
  //    Ry(θ1)^T = Ry(-θ1);  Rx(θ2+θ3)^T = Rx(-(θ2+θ3))
  const s1 = Math.sin(theta1), c1 = Math.cos(theta1);
  // Rollback Ry(θ1):
  const w1x =  w.x * c1 - w.z * s1;
  const w1y =  w.y;
  const w1z =  w.x * s1 + w.z * c1;
  // Rollback Rx(θ2+θ3):
  const s23 = Math.sin(theta2 + theta3), c23 = Math.cos(theta2 + theta3);
  const tx =  w1x;
  const ty =  w1y * c23 + w1z * s23;
  const tz = -w1y * s23 + w1z * c23;

  // Y-X-Y 分解: R_wrist · (0,1,0) = (sin θ4 sin θ5, cos θ5, cos θ4 sin θ5) = (tx, ty, tz)
  const cos5 = clamp(ty, -1, 1);
  const theta5 = Math.acos(cos5);
  let theta4: number;
  const sin5 = Math.sqrt(Math.max(1 - cos5 * cos5, 0));
  if (sin5 > 1e-5) {
    theta4 = Math.atan2(tx / sin5, tz / sin5);
  } else {
    // 特異点: pen axis は local +Y に近い。θ4 は前値を維持 or 0
    theta4 = previousTheta ? previousTheta[3] : 0;
  }
  const theta6 = 0;  // 軸対称ペンでは未使用

  // 関節制約にクランプ
  const L = robot.jointLimits;
  const thetaOut = new Float32Array(6);
  thetaOut[0] = clampLimit(theta1, L[0]);
  thetaOut[1] = clampLimit(theta2, L[1]);
  thetaOut[2] = clampLimit(theta3, L[2]);
  thetaOut[3] = clampLimit(theta4, L[3]);
  thetaOut[4] = clampLimit(theta5, L[4]);
  thetaOut[5] = clampLimit(theta6, L[5]);

  // 残差
  const fk = forwardKinematics(robot, thetaOut);
  const dx = fk.penTip.x - penTipWorld.x;
  const dy = fk.penTip.y - penTipWorld.y;
  const dz = fk.penTip.z - penTipWorld.z;
  const residualPosM = Math.hypot(dx, dy, dz);
  return { theta: thetaOut, reachable, residualPosM };
}


export function homeTheta(): JointAngles {
  // ホーム姿勢: 肩を少し前に、肘を曲げ、前腕を前方に
  return new Float32Array([0, 0.6, -1.1, 0, 0.9, 0]);
}
