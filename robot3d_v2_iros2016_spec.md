# ロボットアーム 3D シミュレーション v2.0 — IROS 2016 ベース再実装仕様書

> 参照論文: **Berio, Calinon & Leymarie (IROS 2016)**
> *"Learning dynamic graffiti strokes with a compliant robot"*
>
> 既存 v1 (シンプル 6DOF + 解析 IK) を、論文に忠実な
> **7DOF 冗長アーム + Helical ΣΛ + 重み付き Jacobian IK + コンプライアント追従 + 反復補正**
> へ再実装する。

---

## 目次

1. [概要・目的](#1-概要目的)
2. [v1 → v2 主要差分](#2-v1--v2-主要差分)
3. [採用する論文手法](#3-採用する論文手法)
4. [全体アーキテクチャ](#4-全体アーキテクチャ)
5. [数学モデル](#5-数学モデル)
6. [7-DOF アーム構成](#6-7-dof-アーム構成)
7. [データフロー](#7-データフロー)
8. [ディレクトリ構成](#8-ディレクトリ構成)
9. [主要モジュール仕様](#9-主要モジュール仕様)
10. [UI / UX 設計](#10-ui--ux-設計)
11. [パラメータ](#11-パラメータ)
12. [段階的実装ロードマップ](#12-段階的実装ロードマップ)
13. [テスト計画](#13-テスト計画)
14. [付録: 論文方程式 ↔ 実装対応表](#14-付録-論文方程式--実装対応表)

---

## 1. 概要・目的

v1 では「解析的 IK で軌跡を完全に追従する」理想化された 6DOF アームを描画していたが、
これは Berio らが論文で**避けるべきと指摘した**剛性追従パラダイムに該当する。

v2 では、論文の以下の特徴を可視化することを目的とする:

- **冗長性の活用** (7-DOF): 主タスクを満たしつつ、ペン姿勢と「腕の自然な姿勢」を副タスクで同時達成
- **コンプライアント挙動**: 低ゲインによる追従誤差を**意図的に許容**し、ロボットらしい人間ぽい動きを表現
- **反復補正の収束過程**: 「初回再生 → 誤差検出 → 仮想ターゲット補正 → 再生 → ...」を**ステップ単位で見せる**
- **3D ペンアップ動作** (Helical ΣΛ): グループ間のペン浮き移動が自然な螺旋運動として表現される

この可視化により、SL モデル + 反復補正 + コンプライアント制御という研究内容を**インタラクティブに体感**できるようにする。

---

## 2. v1 → v2 主要差分

| 項目 | v1 (現行) | v2 (本仕様) |
|---|---|---|
| アーム DOF | 6-DOF | **7-DOF (Baxter 風冗長)** |
| IK | 2-link 解析 + Y-X-Y wrist 分解 | **重み付き擬似逆 Jacobian + null-space (主・副×2)** |
| ペン姿勢 | 手首回転で受動的に向ける | **副タスクで能動的にキャンバス法線へ揃える** |
| 関節構成評価 | なし | **副タスク (qd) で「腕がキャンバスに平行」姿勢を維持** |
| 追従精度 | 完全追従 (理想) | **コンプライアント追従 (誤差あり) → 反復補正で収束** |
| 軌跡形状 | グループ間にペンアップ間隔 (時間ギャップのみ) | **Helical ΣΛ で 3D 螺旋ペンアップ軌跡** |
| 反復ループ | なし | **iter=1..N の補正過程を視覚化** |
| キーポイント | なし | lognormal 交点 (生成) / 速度極小 or EM (再現) |

---

## 3. 採用する論文手法

### 3.1 Helical Stroke Primitive (Eq. 2-5)

論文 §III の 3D 拡張 ΣΛ。仮想ターゲットを `v_i ∈ R³` (3D) として、

```
φ_i(t) = θ_i + θ_i · (1 + erf( (ln(t - t0_i) - μ_i) / (σ_i √2) ))   ... (Eq.2)

p(t) = v_1 + ∫₀ᵗ Λ_j(τ) Σᵢ Φ_i(τ) (v_{i+1} - v_i) dτ              ... (Eq.3)

Φ_i(t) = ⎡ s(θ_i)·cos φ_i(t)  -s(θ_i)·sin φ_i(t)  0 ⎤
         ⎢ s(θ_i)·sin φ_i(t)  -s(θ_i)·cos φ_i(t)  0 ⎥             ... (Eq.4)
         ⎣        0                    0           1 ⎦

s(θ_i) = (2θ_i) / (2 sin θ_i)   if |sin θ_i| > 0  else  1            ... (Eq.5)
```

**意図**: 連続ストロークが xy 平面上で円弧、`x_3` 方向で線形動作 → 円筒側面の螺旋。
ペンアップ → 上昇 → 横移動 → 下降 が自然な ΣΛ 軌跡として表現される。

### 3.2 7-DOF アーム & Weighted Pseudo-Inverse IK (Eq. 6-8)

```
q̇ = J*_x · x̂̇                                ← 主タスク (Cartesian 位置)
  + N(J_x) · k_ψ · J*_ψ · e_ψ                 ← 副タスク 1 (ペン姿勢)
  + N(J_x) · k_d · ∇H                         ← 副タスク 2 (関節姿勢)

J*_x = W^-1 J_x^T · (J_x W^-1 J_x^T + λI)^-1   ← 重み付き擬似逆
N(J_x) = I - J_x^T · J*_x                       ← null-space projection

H = (1/7) Σ ((q_i - q_{d,i}) / (q_max,i - q_min,i))²   ← 関節姿勢コスト
```

- `W` は対角行列。論文では肩 2 関節 (高出力・低精度) を penalize
- `e_ψ` = 現在のエンドエフェクタ姿勢と「キャンバス法線へ垂直」目標姿勢の差
- `q_d` = 「腕がキャンバス面に平行」と なる Liegeois 風ヒューリスティック姿勢

### 3.3 コンプライアント追従シミュレーション

論文の実機 PD 制御 (Eq. 10) を**シミュレートする**ため、以下のいずれか/組合せで意図的な追従誤差を導入する:

a. **加速度依存ラグ**: `x_actual(t) = x_desired(t) − k_lag · ẍ_desired(t)`
   高加速度区間 (鋭角コーナー、急発進) で誤差が大きくなる
b. **1次ローパス**: `x_actual = lowpass(x_desired, τ)`
c. **方向依存ノイズ**: ロボットの関節空間誤差を Cartesian へ投影

v2 ではまず (a) を採用 (シンプル・直感的)。

### 3.4 反復補正 (Eq. 11)

```
v_i ← v_i + α · (x̂(ẑ_i) − A^-1 · x(z_i))
```

- `ẑ_i`: 生成軌跡上のキーポイント時刻 (連続するロークォルマル交点)
- `z_i`: 再現軌跡上の対応キーポイント時刻
- `α`: 学習率 (1 → 0.5 → 0.5 → ...)
- 4–10 回反復で収束

**シミュレーション特化版**: 実機キーポイント検出 (EM) は不要。生成側と同じインデックスでサンプル時刻を比較する **simulated key-point** を直接使用可能。

### 3.5 キーポイント検出 (生成側のみ実装)

連続する lognormal `Λ_j` と `Λ_{j+1}` の交点 = `t̂_j` (`j = 1..M-2`)。
始点 (t=0) と終点 (T_end) を加えて `M` 個。

数値解 (二分法など) で容易に求まる。再現側は時刻インデックスで対応。

---

## 4. 全体アーキテクチャ

```
TrajectoryPlan (2D)
    ↓
┌───────────────────────────────────────────────────────────┐
│ helical3D.ts                                              │
│  - 各 group の 2D ΣΛ を抽出                                 │
│  - グループ間に "lift target" (z = liftHeight) を挿入       │
│  - Helical ΣΛ で連続 3D 軌跡 p(t) ∈ R³ を生成               │
└───────────────────────────────────────────────────────────┘
    ↓ desired trajectory x̂(t) ∈ R³
┌───────────────────────────────────────────────────────────┐
│ canvasMapping.ts (拡張)                                    │
│  - p ∈ R³ canvas-local → world R³                          │
│  - x_3 (lift) はキャンバス法線方向のオフセットに変換          │
└───────────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────────┐
│ complianceSim.ts                                           │
│  - x_actual(t) = x̂(t) - k · ẍ̂(t)  (加速度依存ラグ)          │
│  - 反復ループで k を一定に保ったまま virtual target を補正    │
└───────────────────────────────────────────────────────────┘
    ↓ desired pose: position + orientation
┌───────────────────────────────────────────────────────────┐
│ jacobianIK.ts (新規)                                       │
│  - 7-DOF FK (chain of 7 joints with axis vectors)          │
│  - 数値ヤコビアン (微小角度差分 or 解析)                     │
│  - 重み付き擬似逆 (W diag, λ damped)                         │
│  - null-space projection for secondary tasks                │
│  - integrate q̇ → q over time (incremental)                  │
└───────────────────────────────────────────────────────────┘
    ↓ joint angles q[7] per sample
┌───────────────────────────────────────────────────────────┐
│ ikCacheV2.ts                                               │
│  - 全反復 × 全サンプル の関節角・ペン位置を pre-compute      │
│  - sampleAt(iteration, time) で O(log N) 取得               │
└───────────────────────────────────────────────────────────┘
    ↓
┌───────────────────────────────────────────────────────────┐
│ Robot3DView v2 (R3F + drei)                                │
│  - <RobotArm7DOF> (Baxter 風 7関節)                         │
│  - <DrawingCanvas3D> (既存)                                 │
│  - <DesiredTrail> (灰色: x̂)                                 │
│  - <ActualTrail> (赤/青: 反復ごと色相を変える x_actual)      │
│  - <KeyPointMarkers> (補正前後の対応関係)                    │
│  - <IterationStrip> (時系列で反復回数のサムネ)                │
└───────────────────────────────────────────────────────────┘
```

---

## 5. 数学モデル

### 5.1 Helical 3D ΣΛ の定式化 (Eq. 2-5)

仮想ターゲット `v_1, ..., v_M ∈ R³` と Stroke パラメータ `{t0_i, μ_i, σ_i, θ_i}` (i=1..M-1)。
各ストロークの 3D 速度 contribution は
- xy 成分: 円弧 (Φ の左上 2x2 ブロック)
- z 成分: 線形 (Φ の (3,3) = 1)

論文 Eq.3 を時間離散化で実装:
```
p(t_{k+1}) = p(t_k) + Δt · Λ_j(t_k) · Σᵢ Φ_i(t_k) · (v_{i+1} - v_i)
```

ここで `Λ_j(t)` はストローク `j` の lognormal 速度値 (Berio 2025 Eq.2 と同じ形)。

実装上の注意:
- xy 平面の円弧は SL\* と同じ計算 (現行 backend `sl_model.py` のものを 3D に拡張)
- z 軸は単純に `(v_{i+1,z} - v_{i,z})` の積み上げ

### 5.2 ペンアップ仮想ターゲット挿入

`stroke_groups[g]` の最終仮想ターゲット → `stroke_groups[g+1]` の `p0` の間に
**3 つの追加仮想ターゲット**を挿入する:

1. **Lift target**: `(prev_end.x, prev_end.y, +lift_height)` — ペン上昇
2. **Travel target**: `((prev_end + next_start).xy / 2, +lift_height)` — 中継 (上空での横移動の中点)
3. **Plant target**: `(next_start.x, next_start.y, +lift_height)` — 次グループ開始位置の上空

この 3 ターゲットにより螺旋的な lift→travel→plant 動作が自動生成される。
パラメータ `lift_height` はキャンバス法線方向のオフセット (m, デフォルト 0.05)。

### 5.3 7-DOF FK

各関節の軸 `a_i ∈ R³` (基底原点を基準としたユニットベクトル) と
リンク変位 `d_i ∈ R³` を定義。FK は Three.js Object3D 階層で表現:

```
T_world = ∏ᵢ (Rotate(a_i, q_i) · Translate(d_i))
```

ヤコビアンは数値差分 or 解析:
```
J_x[:, i] = a_i × (p_ee - p_i)         ← 回転関節の Cartesian 速度寄与
J_ψ[:, i] = a_i                          ← 角速度寄与
```
ここで `p_i` は関節 i の世界位置、`p_ee` はエンドエフェクタ位置 (FK で取得済み)。

### 5.4 重み付き擬似逆 + null space (Eq. 6-8)

```typescript
// 各反復ステップ:
const J_x = computePositionJacobian(q);           // [3, 7]
const J_psi = computeOrientationJacobian(q);      // [3, 7]
const W = diag([w1, w2, ..., w7]);                // 関節重み (肩を penalize)
const Jstar_x = invMatMul(W).matMul(J_x.T)
              .matMul(invMatMul(J_x.matMul(invMatMul(W)).matMul(J_x.T).addEye(λ)));
// or compute via Cholesky for numerical stability

const N = eye(7).sub(J_x.T.matMul(Jstar_x));      // 7x7 null space projector

// Primary task velocity
const qdot_x = Jstar_x.matMul(x_dot_desired);

// Secondary task: pen orientation
const e_psi = orientationError(currentPose, desiredPose);
const Jstar_psi = weightedPseudoInverse(J_psi, W, λ);
const qdot_psi = N.matMul(Jstar_psi.matMul(e_psi.scale(k_psi)));

// Secondary task: joint posture
const grad_H = computePostureGradient(q, q_d, q_min, q_max);  // ∇H
const qdot_d = N.matMul(grad_H.scale(k_d));

const qdot = qdot_x.add(qdot_psi).add(qdot_d);
q = q.add(qdot.scale(dt));
```

### 5.5 コンプライアント追従モデル

最も簡単な dynamic lag model:
```
x_actual(t) = x_desired(t) - k_lag · ẍ_desired(t)
```

`ẍ` は数値的に算出 (中央差分)。`k_lag` を増やすほど tracking error が増大。デフォルト `0.005`。

代替: 1次ローパス (要 state):
```
x_actual(t+Δt) = x_actual(t) + (Δt/τ) · (x_desired(t) - x_actual(t))
```

v2.0 は加速度ラグを採用、v2.1 で切替可能に。

### 5.6 反復補正 (Eq. 11)

各反復で:
```
1. 現在の virtual targets で desired trajectory を生成
2. compliance simulator を通して actual trajectory を得る
3. キーポイント時刻 ẑ_i を desired から計算 (lognormal 交点)
4. 同じ時刻 ẑ_i で actual の位置 x(z_i) をサンプル
5. 補正: v_i ← v_i + α · (x̂(ẑ_i) - x(z_i))
6. 次反復へ
```

### 5.7 キーポイント (生成側)

連続する lognormal `Λ_j(t)` と `Λ_{j+1}(t)` の交点を二分法で探す。
区間 `[max(t_peak_j, t0_{j+1}), t_peak_{j+1}]` で `Λ_j - Λ_{j+1} = 0` を解く。
ピーク時刻 `t_peak_j = t0_j + exp(μ_j - σ_j²)`。

---

## 6. 7-DOF アーム構成

Baxter 風の構成を採用 (簡易版):

| # | 名称 | 軸 (local) | 親への変位 (local) | 可動範囲 |
|---|---|---|---|---|
| 1 | S0 (shoulder yaw) | Y | (0, baseHeight, 0) | ±π |
| 2 | S1 (shoulder pitch) | X | (0, 0, 0) | ±π/2 |
| 3 | E0 (upper arm twist) | Y | (0, upperArm, 0) | ±π |
| 4 | E1 (elbow pitch) | X | (0, 0, 0) | (-π+0.1, π/4) |
| 5 | W0 (forearm twist) | Y | (0, forearm, 0) | ±π |
| 6 | W1 (wrist pitch) | X | (0, 0, 0) | ±π/2 |
| 7 | W2 (wrist roll) | Y | (0, wristLen, 0) | ±π |

リンク長 (m, パラメトリック):
```
baseHeight = 0.30
upperArm   = 0.40
forearm    = 0.30
wristLen   = 0.10
penLength  = 0.12
```

リーチ ≈ baseHeight + upperArm + forearm + wristLen + penLength = 1.22 m

論文の Baxter 実機より小型。Drawing canvas は (0, 0.85, 0.50)、サイズ 0.40 × 0.30 m。

---

## 7. データフロー

```
TrajectoryPlan (backend)              ← 既存
   │
   ▼ helical3D.buildHelical(plan, mapping, liftHeight)
3D Trajectory (R³ time series, with pen-up segments)
   │
   ▼ canvasMapping.toWorld()
World 3D Trajectory (m)
   │
   ▼ complianceSim.lag(trajectory, k_lag)
Actual World Trajectory (with tracking error)
   │
   ▼ jacobianIK.solveTrajectory(actual, robot, weights, λ)
Joint Angles q(t) ∈ R^7
   │
   ▼ keyPoints.detectActual(actual)
Key-points {z_i}
   │
   ▼ correction.update(virtualTargets, desired, actual, α)
Updated virtualTargets
   │
   └── 次反復へ (max iter 設定数まで)

すべての反復結果を ikCacheV2 に保存:
  ikCacheV2[iter][time] → {q, penWorld, desired, actual, keyPoints}
```

---

## 8. ディレクトリ構成

```
frontend/src/
├─ math/
│  ├─ helical3D.ts            # NEW: 3D ΣΛ 軌跡生成 + ペンアップ挿入
│  ├─ canvasMapping.ts        # 拡張: 3D 入力対応 (canvas + lift)
│  ├─ jacobianIK.ts           # NEW: 7-DOF FK + 数値ヤコビアン + 重み付き IK
│  ├─ complianceSim.ts        # NEW: 加速度ラグ追従シミュ
│  ├─ keyPoints.ts            # NEW: lognormal 交点 / actual 速度 minima
│  ├─ correction.ts           # NEW: virtual target iterative update
│  └─ ikCacheV2.ts            # NEW: 反復 × 時刻のキャッシュ
├─ components/
│  ├─ Robot3DView.tsx         # 既存を v2 ベースに置換
│  └─ robot3d/
│     ├─ RobotArm7DOF.tsx     # NEW: 7関節アーム (Baxter 風)
│     ├─ DrawingCanvas3D.tsx  # 既存 (差分なし)
│     ├─ TrailPair3D.tsx      # NEW: desired (灰) + actual (色) 2 本
│     ├─ KeyPointMarkers3D.tsx # NEW: キーポイント間の補正ベクトル
│     ├─ Arm3DAnimator.tsx    # 拡張: 反復番号も入力に
│     ├─ SceneLights.tsx      # 既存
│     └─ IterationStrip.tsx   # NEW: 反復ステップのサムネ + クリック切替
└─ store/
   └─ studioStore.ts          # 拡張: robot iteration state
```

---

## 9. 主要モジュール仕様

### 9.1 `helical3D.ts`

```typescript
export interface Vec3 { x: number; y: number; z: number }

export interface VirtualTarget3D extends Vec3 { isPenUp: boolean }

export interface HelicalStroke {
  t0: number;       // 起動時刻 [s]
  mu: number;
  sigma: number;
  theta: number;    // 円弧旋回角 [rad]
  vNext: Vec3;      // 次の virtual target
}

export interface HelicalTrajectory {
  p0: Vec3;
  virtualTargets: VirtualTarget3D[];   // M 個
  strokes: HelicalStroke[];            // M-1 個
  tEnd: number;
}

/**
 * TrajectoryPlan (2D, multi-group) → 単一の 3D helical trajectory に変換。
 * グループ間に lift / travel / plant の3 ターゲットを挿入する。
 */
export function buildHelical(
  plan: TrajectoryPlan,
  liftHeightM: number,
  mapping: CanvasMappingParams,
): HelicalTrajectory;

/** 与えられた t 列でサンプル → 3D 位置と速度 */
export function evaluateHelical(
  traj: HelicalTrajectory,
  ts: Float32Array,
): { pos: Float32Array; vel: Float32Array };  // [N, 3] flatten
```

### 9.2 `jacobianIK.ts`

```typescript
export interface RobotConfig7 {
  jointAxes: [number, number, number][];   // 7 unit vectors (local frame)
  jointOffsets: [number, number, number][];// 7 offsets (local frame)
  jointLimits: [number, number][];          // 7 pairs
  endEffectorOffset: [number, number, number]; // pen tip from last frame
  weights: number[];                        // [7] for W diag (high = penalize)
  q_d: number[];                            // [7] desired posture
}

export const DEFAULT_ROBOT_7: RobotConfig7;

export interface FKResult7 {
  jointWorld: Float32Array;     // [7, 3] world positions
  jointAxesWorld: Float32Array; // [7, 3]
  endEffector: { position: Vec3; rotation: Mat3 };
}

export function fk7(robot: RobotConfig7, q: Float32Array): FKResult7;

export function jacobianPosition(robot: RobotConfig7, fk: FKResult7): Float32Array; // [3, 7]
export function jacobianOrientation(robot: RobotConfig7, fk: FKResult7): Float32Array; // [3, 7]

export interface IKStepInput {
  robot: RobotConfig7;
  q_current: Float32Array;
  v_desired: Vec3;          // ẋ desired
  R_desired: Mat3;          // pen orientation target
  k_psi: number;            // secondary task gain
  k_d: number;
  lambda: number;           // damping
  dt: number;
}

export function ikStep(input: IKStepInput): {
  q_next: Float32Array;
  residual_pos: number;
  residual_orient: number;
};

/** 軌跡全体 (時系列) を IK で追従 */
export function ikTrajectory(
  robot: RobotConfig7,
  desiredPose: { positions: Float32Array; rotations: Float32Array },
  q_init: Float32Array,
  ts: Float32Array,
): {
  q: Float32Array;          // [N, 7]
  penWorld: Float32Array;   // [N, 3]
};
```

### 9.3 `complianceSim.ts`

```typescript
export interface ComplianceParams {
  k_lag: number;          // 加速度ラグ係数 [s²]
  noise_std?: number;     // 関節空間ガウシアンノイズ [rad]
}

export const DEFAULT_COMPLIANCE: ComplianceParams = { k_lag: 0.005, noise_std: 0.001 };

/**
 * 望ましい軌跡 (位置時系列) → コンプライアント挙動を模した実際の軌跡
 *   x_actual = x_desired - k_lag · ẍ_desired
 */
export function simulateCompliance(
  desiredPositions: Float32Array,    // [N, 3]
  ts: Float32Array,
  params: ComplianceParams,
): Float32Array;                      // [N, 3]
```

### 9.4 `keyPoints.ts`

```typescript
/**
 * 生成軌跡の lognormal 交点時刻 (key-point times)。
 * 始点・終点を含めて M 個。
 */
export function keyPointsFromGenerated(
  traj: HelicalTrajectory,
): number[];

/**
 * 再現 (actual) 軌跡から速度極小点を抽出してキーポイント時刻 z_i を得る。
 * シミュレーション特化版: desired のキーポイント時刻周辺で actual の速度極小を探索。
 */
export function keyPointsFromActual(
  desiredKeyT: number[],
  ts: Float32Array,
  actualPos: Float32Array,
  searchWindowS: number,
): number[];
```

### 9.5 `correction.ts`

```typescript
/**
 * 1 反復の virtual target 補正 (Eq. 11)
 *   v_i ← v_i + α · (x̂(ẑ_i) - x(z_i))
 */
export function correctVirtualTargets(
  current: VirtualTarget3D[],
  desiredKeyPositions: Vec3[],   // x̂(ẑ_i)
  actualKeyPositions: Vec3[],    // x(z_i) — 正規化済み (canvas 座標)
  alpha: number,
): VirtualTarget3D[];
```

### 9.6 `ikCacheV2.ts`

```typescript
export interface IterationFrame {
  desiredPos: Float32Array;   // [N, 3]
  actualPos: Float32Array;    // [N, 3]
  jointAngles: Float32Array;  // [N, 7]
  keyDesiredPos: Float32Array; // [M, 3]
  keyActualPos: Float32Array;  // [M, 3]
  ts: Float32Array;
  tEnd: number;
}

export class IKCacheV2 {
  iterations: IterationFrame[];
  totalDuration: number;
  hasUnreachable: boolean;

  constructor(plan, mapping, robot, complianceParams, opts: {
    maxIter: number;          // default 6
    alpha0: number;           // default 1.0
    alphaSubseq: number;      // default 0.5
    liftHeightM: number;
    progressCb?: (frac, iter) => void;
  });

  /** 指定反復・時刻の状態を補間取得 */
  sample(iter: number, t: number, out: { q: Float32Array; pen: Vec3 }): void;
}
```

### 9.7 React コンポーネント

#### `RobotArm7DOF.tsx`
7 つの `<group ref={jXRef}>` ネストを構築。各 ref に対し `useFrame` で関節角を適用。
リンクは円柱/球で表現 (v1 のスタイル踏襲)。

#### `TrailPair3D.tsx`
desired (灰) と actual (反復番号で色相) の 2 本のラインを描画。
`drawRange` で currentTime まで動的に切り取り。

#### `KeyPointMarkers3D.tsx`
反復時に desired と actual のキーポイント対応を矢印 (Arrow helper) で表示。

#### `IterationStrip.tsx`
画面下部にサムネ列。各反復のミニビュー (canvas 全体俯瞰)。
クリックで `selectedIter` 更新 → メインビューがその反復を再生。

---

## 10. UI / UX 設計

### 10.1 メイン画面構成

```
┌──────────────────────────────────────────────────────┐
│ 表示モード: [2D] [🤖 3D ロボット v2] ← active        │
├──────────────────────────────────────────────────────┤
│ ┌──────────────────────────────────────────────────┐ │
│ │ 反復ストリップ:                                   │ │
│ │  [iter 1] [iter 2] [iter 3] [iter 4] [iter 5]    │ │
│ │      ●        ○        ○        ○        ○      │ │
│ │   選択中                                          │ │
│ ├──────────────────────────────────────────────────┤ │
│ │ Robot 3D View                                     │ │
│ │  - desired trail (灰色)                           │ │
│ │  - actual trail (青→オレンジ→緑、反復毎に色変化) │ │
│ │  - 7-DOF Baxter 風 arm                            │ │
│ │  - キーポイント矢印 (副表示トグル)                  │ │
│ ├──────────────────────────────────────────────────┤ │
│ │ AnimationControls (Play/Pause/シーク 共有)        │ │
│ └──────────────────────────────────────────────────┘ │
│ ┌──────────────────────────────────────────────────┐ │
│ │ コンプライアント設定 + 反復制御                     │ │
│ │  k_lag:    [-----●------]   0.005                 │ │
│ │  最大反復:  [3] [5] [10]                           │ │
│ │  α 初期値:  [-----●------]   1.0                   │ │
│ │  α 後続:    [-----●------]   0.5                   │ │
│ │  Lift 高さ: [-----●------]   0.05 m                │ │
│ │  [▶ 再計算]                                        │ │
│ └──────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

### 10.2 反復切替の演出

- 各反復は独立した actual 軌跡を持つ
- ストリップでクリック → 該当反復の軌跡が actual trail として表示される
- 補助線: 選択反復の **キーポイント補正ベクトル** を矢印で表示

### 10.3 補助情報

- 右上ステータス:
  - 反復回数: `iter 3 / 6`
  - tracking RMSE: `4.2 mm` (反復毎に減少していく様子を見せる)
  - 関節空間効用 H: `0.13` (Liegeois 式)
- 警告:
  - リーチ範囲外: 既存と同様に上部にバナー
  - IK 特異点付近: 「λ damping を強めています」

---

## 11. パラメータ

### 11.1 デフォルト値

```typescript
// helical 3D
liftHeightM: 0.05

// 7-DOF arm
baseHeight: 0.30
upperArm:   0.40
forearm:    0.30
wristLen:   0.10
penLength:  0.12
weights:    [1, 5, 1, 1, 1, 1, 1]   // shoulder pitch (S1) を penalize

// canvas
canvasCenter: [0, 0.85, 0.50]
canvasWidth:  0.40
canvasHeight: 0.30

// IK
lambda:    0.01    // damped
k_psi:     2.0
k_d:       0.5
dt:        0.01    // integration step

// compliance
k_lag:     0.005
noise_std: 0.001

// iteration
maxIter:   6
alpha0:    1.0
alphaSubseq: 0.5
```

### 11.2 ユーザ調整可能 (UI で公開)

- `k_lag`: 0 (剛性) ↔ 0.02 (柔軟)
- `maxIter`: 1, 3, 5, 10
- `alpha0`, `alphaSubseq`: 0.1 - 1.5
- `liftHeightM`: 0.0 - 0.2
- `weights`: プリセット切替 (デフォルト / 全均等 / shoulder 重視)

---

## 12. 段階的実装ロードマップ

| Phase | 内容 | 必要時間目安 |
|---|---|---|
| **v2.0-α (MVP)** | helical3D + jacobianIK + 7DOF + 1反復のみ | 1 セッション |
| **v2.0-β** | complianceSim + 反復ループ + ikCacheV2 | 1 セッション |
| **v2.0** | UI (IterationStrip, KeyPointMarkers, 設定パネル) | 1 セッション |
| **v2.1** | URDF (Baxter 実機モデル) ロード対応 | 任意 |
| **v2.2** | キーポイント検出を EM 版に切替 (実機データ対応) | 任意 |
| **v3.0** | ROS2 Bridge / 実機接続 | 将来 |

各 phase で動作確認を挟む (ブラウザ表示で軌跡が描けることを確認)。

---

## 13. テスト計画

### 13.1 単体テスト (Vitest)

| 対象 | 内容 |
|---|---|
| `helical3D.evaluateHelical` | 単一ストロークで端点 → 端点に到達することを検証 |
| `helical3D` | グループ間で z 軸が `liftHeight` まで上がりまた下がる |
| `jacobianIK.fk7` | 既知の関節角でエンドエフェクタ位置が論文相当の式と一致 |
| `jacobianIK.jacobianPosition` | 数値差分との一致 (相対誤差 < 1e-3) |
| `jacobianIK.ikStep` | 単一サンプルで residual < tol、副タスクが null space で達成される |
| `complianceSim.simulateCompliance` | k_lag=0 で出力 == 入力、k_lag>0 で高加速度区間で誤差増加 |
| `keyPoints` | 既知の lognormal pair で交点を ε 以内で検出 |
| `correction.correctVirtualTargets` | 5 反復で MSE が単調減少 |

### 13.2 統合テスト (Playwright)

- 解析後に「3D ロボット v2」タブ切替 → IterationStrip に複数のタイル表示
- 任意のタイルクリック → メインビューがその反復を再生
- スライダ操作で k_lag を変更 → 再計算ボタンで反復軌跡が変化
- 動作中にコンソールエラーが出ない

### 13.3 性能

| 指標 | 目標 |
|---|---|
| 1反復の IK 計算 (300サンプル × 7DOF) | < 200ms |
| 6反復の事前計算合計 | < 1.5s |
| フレームレート | 60 fps (M2 MacBook) |

---

## 14. 付録: 論文方程式 ↔ 実装対応表

| 論文式 | 内容 | 実装ファイル / 関数 |
|---|---|---|
| Eq.1 | Lognormal 速度 Λ_i | `helical3D.ts: lognormal()` (既存 sl_model 流用可) |
| Eq.2 | 円弧旋回 φ_i (erf 経由) | `helical3D.ts: phaseAngle()` |
| Eq.3 | 軌跡 p(t) (3D 積分) | `helical3D.ts: evaluateHelical()` |
| Eq.4 | 行列 Φ_i (3D, 螺旋) | `helical3D.ts: phiMatrix()` |
| Eq.5 | スケール係数 s(θ_i) | `helical3D.ts: arcScale()` |
| Eq.6 | IK ((主×1 + 副×2) 重ね合せ) | `jacobianIK.ts: ikStep()` |
| Eq.7 | 重み付き擬似逆 J*_x | `jacobianIK.ts: weightedPseudoInverse()` |
| Eq.8 | Null-space projector | `jacobianIK.ts: nullSpaceProjector()` |
| Eq.9 | 関節姿勢コスト H | `jacobianIK.ts: postureCost(), postureGradient()` |
| Eq.10 | PD トルク制御 | (実装外: シミュレーションでは省略 — `complianceSim` で代替) |
| Eq.11 | Virtual target 補正 | `correction.ts: correctVirtualTargets()` |

---

*本仕様書バージョン: 2.0 / 対象プロジェクト: Sigma-Lognormal Studio*
*基底論文: Berio, Calinon & Leymarie (IROS 2016)*
