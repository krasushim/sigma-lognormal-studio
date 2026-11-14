# ロボットアーム 3D 描画シミュレーション — 実装仕様書

> Sigma-Lognormal Studio の `TrajectoryPlan` (2D 時系列 `x(t), y(t)`) を入力として、
> 仮想キャンバス上に**ロボットアームが軌跡を描く 3D シミュレーション**を
> スタジオ内で表示する機能の実装仕様。

---

## 目次

1. [概要・目的](#1-概要目的)
2. [機能要件 (v1 スコープ)](#2-機能要件-v1-スコープ)
3. [非機能要件](#3-非機能要件)
4. [全体アーキテクチャ](#4-全体アーキテクチャ)
5. [技術スタック](#5-技術スタック)
6. [ディレクトリ構成](#6-ディレクトリ構成)
7. [データフロー](#7-データフロー)
8. [主要モジュール仕様](#8-主要モジュール仕様)
9. [UI 設計 (Studio 統合)](#9-ui-設計-studio-統合)
10. [パラメータとカスタマイズ](#10-パラメータとカスタマイズ)
11. [パフォーマンス要件](#11-パフォーマンス要件)
12. [テスト計画](#12-テスト計画)
13. [将来拡張 (v2+)](#13-将来拡張-v2)
14. [付録: 数学的基礎](#14-付録-数学的基礎)

---

## 1. 概要・目的

### 1.1 ねらい

- **SL/GL モデルで再構成された人間らしい軌跡**を、**産業用ロボットアームで再現する様子を可視化**する
- タグ作成の kinematics (速度プロファイル + 仮想ターゲット経由の円弧) が、ロボットという**別のアクチュエータ**に移植された際の挙動を直感的に確認できる
- 将来的な**実ロボットへの転送**（URDF / ROS2 経由）の前段として、オフラインシミュレーション環境を提供

### 1.2 動作環境

- **ブラウザ内 3D 描画**: WebGL2 + Three.js
- **実行場所**: Sigma-Lognormal Studio のタブ追加（新規ルーティングなし）
- **バックエンド変更**: 不要（既存 `TrajectoryPlan` をそのまま利用）

### 1.3 v1 / 将来拡張の区分

- v1: パラメトリック 6DOF アーム + 仮想キャンバス + CCD-IK + 軌跡トレイル
- v2: URDF ローディング (UR5, Panda 等の実機モデル)
- v3: WebXR (VR プレビュー)、物理シミュ (rapier3d)、実ロボット接続 (ROS2 bridge)

---

## 2. 機能要件 (v1 スコープ)

### 2.1 可視化

1. **6DOF パラメトリックロボットアーム** (base, shoulder, upper arm, elbow, forearm, wrist, end-effector/pen) が 3D 空間に立体表示される
2. 正面に**仮想キャンバス** (平板、画像アスペクト比に一致) が配置される
3. 既存の `TrajectoryPlan.stroke_groups` を順番にキャンバス上に描いていく
4. ペン先の**軌跡 (インク)** がキャンバスに蓄積される（色・濃さは既存スプレーパラメータを流用）
5. **関節位置・角度**がリアルタイムに計算・可視化される（IK 結果）

### 2.2 操作

- カメラ: **OrbitControls** (マウス/タッチでの自由視点)
- タイムライン同期: Studio の既存アニメーション制御 (`currentTime`, `playing`, `speed`) と共有
- 既存の Play/Pause/シーク/スピードが**そのまま 3D シーンに反映**される
- シーンリセット、キャンバスクリアボタン

### 2.3 ロボット構成

v1 では固定の 6DOF 構成（仮想アーム）を持つ。関節:

| # | 名称 | 軸 | 可動範囲 (rad) |
|---|---|---|---|
| 1 | Base yaw | Z | ±π |
| 2 | Shoulder pitch | Y | −π/2 … +π |
| 3 | Elbow pitch | Y | −π … +π/2 |
| 4 | Wrist pitch | Y | ±π |
| 5 | Wrist yaw | Z | ±π |
| 6 | Wrist roll | X | ±π |

リンク長 (m): `L_base=0.20`, `L_upper=0.35`, `L_forearm=0.30`, `L_wrist=0.15`, `L_pen=0.12`

合計リーチ ≈ 0.92m。キャンバス (0.40 × 0.30 m) が十分収まる。

---

## 3. 非機能要件

| カテゴリ | 要件 |
|---|---|
| パフォーマンス | 60 fps 安定 (M2 MacBook, 1280×720 viewport 想定) |
| IK 精度 | ペン先誤差 < 1 mm (キャンバス上) |
| IK 計算時間 | 事前計算方式: 600 サンプルで < 200ms (初回のみ) |
| レイテンシ | タイムライン操作 → 3D 反映: < 16ms |
| ブラウザ | Chrome/Safari/Firefox latest (WebGL2 必須) |

---

## 4. 全体アーキテクチャ

```
┌───────────────────────────────────────────────────────────┐
│ Sigma-Lognormal Studio (existing)                         │
│  ┌───────────┬─────────────────────────┬───────────┐       │
│  │ Upload/   │ 中央ペイン               │ Variation │       │
│  │ Draw      │  ┌─────────────────────┐ │ Panel     │       │
│  │ Panel     │  │ [2D View] [3D View] │ │           │       │
│  │           │  │ ←── new tab switch  │ │           │       │
│  │ Params    │  ├─────────────────────┤ │           │       │
│  │ Panel     │  │                     │ │           │       │
│  │           │  │  <2D TrajectoryCvs> │ │           │       │
│  │           │  │   OR                │ │           │       │
│  │           │  │  <Robot3DView>  ◄── NEW             │    │
│  │           │  │                     │ │           │       │
│  │           │  ├─────────────────────┤ │           │       │
│  │           │  │ AnimationControls   │ │           │       │
│  │           │  │  (共有)             │ │           │       │
│  │           │  └─────────────────────┘ │           │       │
│  └───────────┴─────────────────────────┴───────────┘       │
└───────────────────────────────────────────────────────────┘

Robot3DView 内部:
  ┌───────────────────────────────────────┐
  │ <Canvas> (React Three Fiber)          │
  │  ├─ <ambientLight/>, <directionalLight/>
  │  ├─ <OrbitControls/>                  │
  │  ├─ <gridHelper/>, <axesHelper/> (任意)
  │  ├─ <RobotArm jointAngles=... />      │
  │  ├─ <DrawingCanvas3D width=... h=... />
  │  ├─ <PenTrail3D points=... />          │
  │  └─ <PenTip position=... />            │
  └───────────────────────────────────────┘

Data pipeline:
  TrajectoryPlan (2D)
    → canvasMapping (2D→3D world)
    → ikCache.solveAll(poses)     ← CCD IK をサンプル毎に適用 (warm-start)
    → ikTable: Float32Array [N, 6]
    → at render frame:
        lookup joint angles at currentTime
        → <RobotArm jointAngles={...} />
        → extend <PenTrail3D>
```

---

## 5. 技術スタック

| 役割 | ライブラリ | バージョン |
|---|---|---|
| 3D 基盤 | three | ≥ 0.165 |
| React 統合 | @react-three/fiber | ≥ 8.17 |
| ヘルパ集 | @react-three/drei | ≥ 9.114 |
| TypeScript 型 | @types/three | ≥ 0.165 |

(v2 で追加予定)
| URDF ローダ | urdf-loader | ≥ 0.12 |
| 物理 | @dimforge/rapier3d-compat | ≥ 0.14 |

---

## 6. ディレクトリ構成

```
frontend/
├─ src/
│  ├─ components/
│  │  ├─ Robot3DView.tsx          # <Canvas> ラッパ (Three.js シーン)
│  │  └─ robot3d/
│  │     ├─ RobotArm.tsx          # 6DOF パラメトリックアーム
│  │     ├─ DrawingCanvas3D.tsx   # 仮想キャンバス平面
│  │     ├─ PenTrail3D.tsx        # ペン跡 (Line2 or Points)
│  │     ├─ PenTip.tsx            # ペン先マーカ
│  │     └─ SceneLights.tsx       # 共通照明
│  ├─ math/
│  │  ├─ robotKinematics.ts        # DH パラメータ + FK + CCD IK
│  │  ├─ canvasMapping.ts          # 画像座標 → ワールド座標
│  │  └─ ikCache.ts                # 全サンプル IK 事前計算
│  ├─ store/
│  │  └─ studioStore.ts           # viewMode: "2d" | "3d", ikCache を保持
│  └─ pages/
│     └─ Studio.tsx               # タブ切替 UI の追加
```

---

## 7. データフロー

### 7.1 入力

既存の `TrajectoryPlan`（変更なし）:
```
plan.image_width, plan.image_height
plan.stroke_groups[].derived.timeseries.{t, x, y, speed}
plan.stroke_groups[].derived.t_end
```

### 7.2 変換 pipeline

```
(1) 画像座標 (x, y) ∈ [0, W] × [0, H]
   ↓  canvasMapping.imagePxToCanvasWorld(p)
(2) キャンバスワールド座標 (x_w, y_w, z_w) [meters]
    ・キャンバス中心 = [0.6, 0.0, 1.2] m (例)
    ・キャンバス面: YZ 平面に平行 (x 軸が法線)
    ・ペン先は x_w - 0.002m (紙面接触) にオフセット
   ↓  ikCache.solve(targetPose) using CCD
(3) 関節角 [θ1..θ6] ∈ R^6
   ↓  React Three Fiber での FK 連鎖
(4) 各リンクの transform (position + rotation)
   ↓  render
(5) 3D シーン (アーム + ペン先 + 軌跡)
```

### 7.3 事前計算 (plan 読込時に1度)

```typescript
// 解析完了 → plan 更新イベント
useEffect(() => {
  if (!plan) return;
  const cache = new IKCache(plan, mappingParams, robotParams);
  cache.solveAll();   // 全 timeseries サンプルについて IK を解く (~100-200ms)
  setIkCache(cache);
}, [plan, mappingParams, robotParams]);
```

### 7.4 描画フレーム (60fps)

```typescript
useFrame(() => {
  const [groupIdx, localT] = resolveTime(currentTime, groupStartTimes);
  const angles = ikCache.sampleJointAngles(groupIdx, localT);
  setArmAngles(angles);
  const penPos = ikCache.samplePenWorld(groupIdx, localT);
  trailRef.current.extend(penPos, currentTime);   // 巻き戻しで trim
});
```

---

## 8. 主要モジュール仕様

### 8.1 `math/robotKinematics.ts`

**6DOF パラメトリックアームの DH パラメータ**を定義し、FK と IK を提供。

```typescript
export interface DHParam { a: number; alpha: number; d: number; thetaOffset: number; }
export interface RobotParams {
  linkLengths: { base: number; upper: number; forearm: number; wrist: number; pen: number };
  dhTable: DHParam[];   // 6 entries
  jointLimits: Array<[number, number]>; // 6 pairs of [min, max]
}

export const DEFAULT_ROBOT: RobotParams = {
  linkLengths: { base: 0.20, upper: 0.35, forearm: 0.30, wrist: 0.15, pen: 0.12 },
  dhTable: [
    // standard Craig DH for a 6DOF anthropomorphic arm with spherical wrist
    // ...
  ],
  jointLimits: [[-π, π], [-π/2, π], [-π, π/2], [-π, π], [-π, π], [-π, π]],
};

export function forwardKinematics(
  params: RobotParams, theta: Float32Array
): { linkTransforms: Matrix4[]; endEffector: Matrix4 };

export function ccdInverseKinematics(
  params: RobotParams,
  targetPose: Matrix4,             // 目標 end-effector transform
  initialTheta: Float32Array,      // warm-start (前サンプルの解)
  opts?: {
    maxIter?: number;              // default 20
    posToleranceM?: number;        // default 1e-4 (0.1mm)
    rotToleranceRad?: number;      // default 1e-3
    damping?: number;              // Damped Least Squares の λ
  }
): { theta: Float32Array; converged: boolean; residualPos: number };
```

実装方式:
- **CCD (Cyclic Coordinate Descent)** を優先。向きが比較的似ているペン姿勢タスクに高速 (< 1ms per sample, warm-start)
- リーチ範囲外は最近接を返す
- 関節限界は clamp で処理

### 8.2 `math/canvasMapping.ts`

画像座標をキャンバスワールド座標に変換。

```typescript
export interface CanvasMappingParams {
  // キャンバスのワールド位置・向き
  canvasCenter: [number, number, number];   // (x, y, z) [m]
  canvasNormal: [number, number, number];   // facing direction (unit)
  canvasUp:     [number, number, number];   // world-up in canvas plane
  canvasWidth:  number;                     // [m]  (horizontal)
  canvasHeight: number;                     // [m]  (vertical)
  // キャンバスへのペン接触深さ
  contactDepthM: number;                    // 0.002 (1-2mm 接触)
}

export const DEFAULT_MAPPING: CanvasMappingParams = {
  canvasCenter: [0.55, 0.0, 1.10],
  canvasNormal: [-1, 0, 0],   // 壁面がロボット側を向く
  canvasUp:     [0, 0, 1],
  canvasWidth:  0.40,
  canvasHeight: 0.30,
  contactDepthM: 0.002,
};

/** 画像座標 [0, W]×[0, H] を world [m] に写像。戻り値は位置と姿勢 (ペン向き) */
export function imagePxToTargetPose(
  px: [number, number],
  planImageW: number,
  planImageH: number,
  params: CanvasMappingParams,
): Matrix4;   // ペン先のキャンバスへの接触ポーズ (位置 + 法線方向)
```

画像アスペクトがキャンバスと異なる場合は `aspectFit = "contain"` で中央寄せ（空白領域が生じる）。

### 8.3 `math/ikCache.ts`

軌跡全サンプル分の IK 解を事前計算してキャッシュ。

```typescript
export class IKCache {
  private table: Float32Array;       // shape [N, 6]  (全グループ連結)
  private groupOffsets: number[];    // グループごとの先頭インデックス
  private groupTimes: number[][];    // グループごとの t 列

  constructor(plan: TrajectoryPlan, mapping: CanvasMappingParams, robot: RobotParams);

  solveAll(progressCb?: (frac: number) => void): void;

  /** 指定グループ・ローカル時刻の関節角度を補間取得 */
  sampleJointAngles(groupIdx: number, t: number): Float32Array;

  /** ペン先のワールド座標 (FK で再現) */
  samplePenWorld(groupIdx: number, t: number): [number, number, number];

  /** キャンバス上のローカル 2D 座標 (U, V) [0..1] */
  samplePenOnCanvas(groupIdx: number, t: number): [number, number];
}
```

warm-start:
```
for i in samples:
  θ_i = ccd(target_i, initial=θ_{i-1})
```

最初の θ_0 はホームポジション (`[0, π/4, -π/2, 0, π/2, 0]` 相当) を使う。

### 8.4 `components/robot3d/RobotArm.tsx`

```tsx
interface Props { jointAngles: Float32Array; params?: RobotParams; }

export function RobotArm({ jointAngles, params = DEFAULT_ROBOT }: Props) {
  // DH パラメータから各リンクの transform を FK で計算し、
  // <group> の position/rotation として表現する
  const transforms = useMemo(
    () => forwardKinematics(params, jointAngles).linkTransforms,
    [jointAngles, params],
  );

  return (
    <group>
      {/* Base cylinder */}
      <mesh position={[0, 0, params.linkLengths.base / 2]}>
        <cylinderGeometry args={[0.08, 0.10, params.linkLengths.base, 24]} />
        <meshStandardMaterial color="#555" metalness={0.5} roughness={0.4} />
      </mesh>
      {/* Upper arm link (cylinder) positioned/rotated by transforms[1] */}
      <LinkSegment transform={transforms[1]} length={params.linkLengths.upper} radius={0.05} />
      {/* Forearm, wrist, pen ... */}
      ...
    </group>
  );
}
```

`LinkSegment` は `<primitive object={mesh}>` や `<group>` でマトリクス適用。

スタイル: 金属風 (`metalness=0.6, roughness=0.3`)、ジョイント部は別色（黒）。

### 8.5 `components/robot3d/DrawingCanvas3D.tsx`

キャンバス平面 (固定) を描画。

```tsx
export function DrawingCanvas3D({ mapping }: { mapping: CanvasMappingParams }) {
  return (
    <mesh
      position={mapping.canvasCenter}
      rotation={quaternionFromNormal(mapping.canvasNormal, mapping.canvasUp)}
    >
      <planeGeometry args={[mapping.canvasWidth, mapping.canvasHeight]} />
      <meshStandardMaterial color="#f5f5f0" side={DoubleSide} />
    </mesh>
  );
}
```

### 8.6 `components/robot3d/PenTrail3D.tsx`

ペン跡を 3D で描画。実装候補:

A. **MeshLine** ベース (推奨): 可変太さの線をキャンバス面上に描画
B. **InstancedMesh** でスフィア列 (スプレースタンプ風)

```tsx
interface Props {
  points: Float32Array;      // [N, 3]  (x,y,z)
  widths: Float32Array;      // [N]     (半径 m)
  currentIndex: number;      // 現在時刻までの点数
  color?: string;
}
```

currentTime と groupStartTimes からインデックスを算出し、`points.slice(0, currentIndex)` を描画。

スプレー感を保つ場合は、2D 版と同様に **speed に応じた半径**を事前計算してキャッシュし、ポイントを `<instancedMesh>` で投入。

### 8.7 `components/Robot3DView.tsx`

```tsx
interface Props {
  plan: TrajectoryPlan;
  currentTime: number;
  groupStartTimes: number[];
  mappingParams?: CanvasMappingParams;
  robotParams?: RobotParams;
}

export function Robot3DView({ plan, currentTime, groupStartTimes, ... }: Props) {
  const [ikCache, setIkCache] = useState<IKCache | null>(null);
  const [computing, setComputing] = useState(false);

  useEffect(() => {
    setComputing(true);
    const cache = new IKCache(plan, mapping, robot);
    // IK の事前計算は Web Worker に移譲することも可 (v1 は main thread で OK)
    cache.solveAll((frac) => { /* optional progress callback */ });
    setIkCache(cache);
    setComputing(false);
  }, [plan, mapping, robot]);

  const [armAngles, setArmAngles] = useState<Float32Array>(new Float32Array(6));
  const [penHistory, setPenHistory] = useState<Float32Array>(new Float32Array());

  useFrame(() => {
    if (!ikCache) return;
    const { gi, localT } = resolveTime(currentTime, groupStartTimes);
    setArmAngles(ikCache.sampleJointAngles(gi, localT));
    // trail 更新 (巻き戻しで trim)
    ...
  });

  return (
    <Canvas camera={{ position: [1.8, -1.8, 1.3], fov: 40 }} shadows>
      <SceneLights />
      <OrbitControls target={[0.55, 0, 1.10]} />
      <gridHelper args={[4, 20]} />
      <DrawingCanvas3D mapping={mapping} />
      <RobotArm jointAngles={armAngles} params={robot} />
      <PenTrail3D points={penHistory} />
      {computing && <Html center><div>IK 計算中…</div></Html>}
    </Canvas>
  );
}
```

---

## 9. UI 設計 (Studio 統合)

### 9.1 タブ切り替え

中央ペインに **「2D 表示」/「3D シミュレーション」** タブを新設。
アニメーション制御 (Play/Pause/速度/シーク) は**両者で共通**。

### 9.2 3D ビュー上部のコントロール (spray コントロールの隣)

| コントロール | 内容 | デフォルト |
|---|---|---|
| カメラリセット | 定位置のカメラに戻す | — |
| キャンバス位置 | プリセット (正面/斜め/上) | 正面 |
| トレイル表示 | ON/OFF | ON |
| グリッド表示 | ON/OFF | ON |
| ロボット可視 | ON/OFF | ON |
| ペン速度色 | グラデーション ON/OFF | OFF |

### 9.3 ステータスバー

| 情報 | 表示 |
|---|---|
| キャッシュ状態 | "IK 事前計算中 60%" / "準備完了" |
| 現在の関節角 | `θ = [0.12, 0.45, -0.78, 0.03, 0.52, 0.00] rad` (6 要素) |
| ペン位置 | `(x, y, z) = (0.55, 0.12, 1.20) m` |

### 9.4 既存機能との連携

- **Studio の stage セレクタは無効化** (3D ビューは常に final ステージを描画)
- **Variation のサムネ**をクリックで 3D ビューでも表示可能

---

## 10. パラメータとカスタマイズ

### 10.1 Canvas マッピング

```ts
canvasCenter:  [0.55, 0, 1.10] m  // ロボット基底から 0.55m 前方
canvasWidth:   0.40 m
canvasHeight:  0.30 m
contactDepthM: 0.002 m
```

ユーザが調整可能 (v2):
- キャンバスサイズ (0.10–1.00 m)
- キャンバス位置 (スライダ)
- アスペクト強制 (image と一致/キャンバス固定)

### 10.2 ロボット

v1 は固定。v2 で選択可:
- `PARAM_6DOF` (デフォルト, 自家製)
- `UR5` (URDF)
- `FRANKA_PANDA` (URDF)

### 10.3 レンダリング

- 背景色: #e2e8f0 (Studio 全体と調和)
- 床グリッド: 4m × 4m, 20 分割
- 照明: ambient 0.4 + directional 0.8 (1, -1, 1), shadow-casting
- 反射: ロボット金属面は環境反射 `Environment preset="studio"`

---

## 11. パフォーマンス要件

| 指標 | 目標 |
|---|---|
| 初回 IK キャッシュ構築 | < 300ms (600 サンプル, warm-start CCD) |
| フレームレート | 60 fps (M2 MacBook) |
| メモリ使用 | < 60 MB |
| GPU 負荷 | < 30% |
| アニメーション巻き戻し/シーク | < 5ms で反映 |

キャッシュが重い場合 (>500ms) は **Web Worker に移譲**。

---

## 12. テスト計画

### 12.1 単体テスト (Vitest)

- `forwardKinematics`: 既知の関節角で end-effector 位置を検証 (ホームポジション → z=0.92m 直上)
- `ccdInverseKinematics`: ランダム位置でリーチ内なら residual < 1mm
- `imagePxToTargetPose`: 画像中央 → キャンバス中心、画像隅 → キャンバス隅
- `IKCache.sampleJointAngles`: 時刻補間の連続性 (隣接時刻で角度差が < 閾値)

### 12.2 Visual (Playwright)

- 既存ジョブで 3D ビュー切替 → キャンバスに何か描画されているかスクリーンショット比較
- カメラ操作: ドラッグ前後のビュー変化

### 12.3 パフォーマンステスト

- 600 サンプル `IKCache.solveAll()` 実行時間計測 (< 300ms)
- `<Canvas>` 内で requestAnimationFrame 30 秒間、frame drop 数をカウント

---

## 13. 将来拡張 (v2+)

### v2: URDF + 複数ロボット

- `urdf-loader` で `UR5`, `Franka Emika Panda`, `KUKA iiwa` の URDF を読み込み
- 関節構成が変わっても `forwardKinematics` は DH から自動生成
- Selector UI で切替

### v2.1: 物理シミュレーション

- `@dimforge/rapier3d-compat` で関節重力・慣性を模擬
- IK 解をトルク制御に変換 (簡易インバースダイナミクス)

### v2.2: スタイル統合

- 既存 Canvas 2D スプレーレンダラの style (`alpha`, `w_max`, `smoothness`) を
  3D トレイルに適用 (テクスチャスタンプ方式)
- 速度依存の太さは 3D でも再現 (w(t) = w_min + (w_max-w_min)·exp(-|ẋ|/μ_x))

### v3: 実ロボット接続

- **ROS2 bridge** (WebSocket) 経由で実機へ関節角を送出
- Gazebo / MuJoCo のシミュで dry-run
- セーフティ (jerk 制限、衝突判定)

### v3.1: WebXR

- `@react-three/xr` で VR 内に 3D シーンを配置
- コントローラでキャンバス位置を直接つかんで移動

### v3.2: 複数アーム協調

- 2本のアームが同時にタグの別の部分を描く (負荷分散)
- 各 stroke group を最適にアーム割当

---

## 14. 付録: 数学的基礎

### 14.1 DH (Denavit-Hartenberg) 変換

各リンク $i$ の変換行列:
$$
T_i = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)
$$

End-effector: $T_{ee} = T_1 T_2 \cdots T_6 T_{\text{pen}}$

### 14.2 CCD (Cyclic Coordinate Descent) IK

1. 現ポーズ $\theta$ から end-effector 位置 $p_{ee}$ と目標 $p_{target}$ の差 $\Delta p = p_{target} - p_{ee}$
2. 末端から基底へ関節 $i$ を逆順に巡り、各関節について：
   - 関節位置から end-effector への方向ベクトル $\vec{e}$ を計算
   - 関節位置から目標への方向ベクトル $\vec{t}$ を計算
   - $\vec{e}$ を $\vec{t}$ に合わせる回転を関節軸に射影して $\Delta\theta_i$ を得る
3. $\theta_i \leftarrow \text{clamp}(\theta_i + \Delta\theta_i, \text{limits})$
4. 残差 $\|\Delta p\|$ が閾値以下 or max_iter に到達するまで 1–3 を繰り返す

**Warm-start**: 前サンプルの $\theta$ から開始すると 1–3 回の反復で収束。

### 14.3 Damped Least Squares (代替)

ヤコビアン $J$ を用いて:
$$
\Delta\theta = J^T (JJ^T + \lambda^2 I)^{-1} \Delta x
$$

特異点近傍で安定。実装コストは CCD より高いが精度が良い。
v1 では CCD を採用、v2 で hybrid (CCD で粗く → DLS で精密化) を検討。

### 14.4 ペン姿勢制約

ペン先はキャンバス法線と反対向き（キャンバスに押し付ける姿勢）:
$$
R_{ee,z} = -n_{canvas}
$$

上向き (`ee_y` = canvas の上方向) を additional DoF として使い、wrist roll `θ_6` で調整。

### 14.5 時間同期の計算

アニメーション現在時刻 $T_{cur}$ と group 開始時刻 $[T_0, T_1, \ldots]$ (pen-up gap 込み) から:
1. $gi = \max\{i : T_i \le T_{cur}\}$
2. $t_{local} = T_{cur} - T_{gi}$
3. group gi 内で $t_{local} > \text{t\_end}_{gi}$ なら pen-up (ペン先をキャンバスから離す)

---

*本仕様書バージョン: 1.0 / 対象プロジェクト: Sigma-Lognormal Studio*
