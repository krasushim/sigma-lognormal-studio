import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Html } from "@react-three/drei";

import type { TrajectoryPlan } from "../api/types";
import { IKCacheV2, DEFAULT_CACHE_OPTS, CacheV2Options } from "../math/ikCacheV2";
import { DEFAULT_MAPPING } from "../math/canvasMapping";
import { DEFAULT_ROBOT_7 } from "../math/jacobianIK";

import { SceneLights } from "./robot3d/SceneLights";
import { RobotArm7DOF } from "./robot3d/RobotArm7DOF";
import { DrawingCanvas3D } from "./robot3d/DrawingCanvas3D";
import { TrailPair3D } from "./robot3d/TrailPair3D";
import { IterationStrip } from "./robot3d/IterationStrip";
import { Arm3DAnimatorV2 } from "./robot3d/Arm3DAnimatorV2";
import { CanvasSprayOverlay } from "./robot3d/CanvasSprayOverlay";


interface Props {
  plan: TrajectoryPlan;
  currentTime: number;
  totalDuration: number;
}


/**
 * v2: Berio IROS 2016 ベース。
 *  - 7-DOF アーム + 重み付き Jacobian IK + null-space 副タスク
 *  - 3D Helical ΣΛ で pen-up 軌跡
 *  - Compliance シム + 反復補正
 *  - IterationStrip で各反復ステップの結果を切替表示
 */
export function Robot3DViewV2({ plan, currentTime }: Props) {
  const [cache, setCache] = useState<IKCacheV2 | null>(null);
  const [building, setBuilding] = useState(false);
  const [progress, setProgress] = useState(0);
  const [selectedIter, setSelectedIter] = useState(0);

  // ユーザ調整可能パラメータ
  const [kLag, setKLag] = useState(0);
  const [maxIter, setMaxIter] = useState(5);
  const [liftHeight, setLiftHeight] = useState(0.05);
  const [sprayOn, setSprayOn] = useState(true);
  const [sprayAlpha, setSprayAlpha] = useState(0.6);
  const [sprayWMax, setSprayWMax] = useState(14);

  // 関節角共有 ref (animator → arm)
  const armAnglesRef = useRef<Float32Array>(new Float32Array(7));

  // パラメータが変わるたびに再構築
  const cacheKey = useMemo(
    () => `${plan?.schema_version}-${kLag}-${maxIter}-${liftHeight}`,
    [plan, kLag, maxIter, liftHeight],
  );
  useEffect(() => {
    if (!plan || plan.stroke_groups.length === 0) {
      setCache(null);
      return;
    }
    setBuilding(true);
    setProgress(0);
    const id = setTimeout(() => {
      try {
        const opts: Partial<CacheV2Options> = {
          ...DEFAULT_CACHE_OPTS,
          maxIter,
          liftHeightM: liftHeight,
          compliance: { k_lag: kLag },
          progressCb: (frac) => setProgress(frac),
        };
        const c = new IKCacheV2(plan, opts);
        setCache(c);
        setSelectedIter(c.iterations.length - 1);
      } finally {
        setBuilding(false);
        setProgress(1);
      }
    }, 30);
    return () => clearTimeout(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cacheKey]);

  const frame = cache?.iterations[selectedIter] ?? null;

  // currentTime ref 共有
  const currentTimeRef = useRef<number>(currentTime);
  useEffect(() => { currentTimeRef.current = currentTime; }, [currentTime]);
  const getCurrentTime = useMemo(() => () => currentTimeRef.current, []);
  const getJointAngles = useMemo(() => () => armAnglesRef.current, []);

  return (
    <div className="relative w-full h-full bg-slate-200 rounded overflow-hidden flex flex-col">
      {cache && cache.iterations.length > 0 && (
        <IterationStrip
          iterations={cache.iterations}
          selected={selectedIter}
          onSelect={setSelectedIter}
        />
      )}

      <div className="flex-1 min-h-0 relative">
        <Canvas
          shadows
          camera={{ position: [1.2, 1.0, -1.2], fov: 32, near: 0.05, far: 20 }}
        >
          <color attach="background" args={["#e2e8f0"]} />
          <SceneLights />

          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
            <planeGeometry args={[3, 3]} />
            <meshStandardMaterial color="#cbd5e1" roughness={0.9} />
          </mesh>
          <Grid
            args={[3, 3]} cellColor="#94a3b8" sectionColor="#64748b"
            cellSize={0.1} sectionSize={0.5}
            position={[0, 0.001, 0]} fadeDistance={5}
          />

          <DrawingCanvas3D mapping={cache?.effectiveMapping ?? DEFAULT_MAPPING} />

          {frame && (
            <>
              <RobotArm7DOF
                robot={DEFAULT_ROBOT_7}
                getJointAngles={getJointAngles}
              />
              <Arm3DAnimatorV2
                frame={frame}
                getCurrentTime={getCurrentTime}
                anglesRef={armAnglesRef}
              />
              {/* spray ON のときは spray オーバーレイを使う。OFF のときは細線 trail */}
              {sprayOn ? (
                <CanvasSprayOverlay
                  frame={frame}
                  mapping={cache?.effectiveMapping ?? DEFAULT_MAPPING}
                  planImageWidth={plan.image_width}
                  planImageHeight={plan.image_height}
                  getCurrentTime={getCurrentTime}
                  sprayParams={{ alpha: sprayAlpha, w_max: sprayWMax }}
                />
              ) : (
                <TrailPair3D frame={frame} getCurrentTime={getCurrentTime} />
              )}
            </>
          )}

          <OrbitControls
            target={[
              DEFAULT_MAPPING.canvasCenter[0],
              DEFAULT_MAPPING.canvasCenter[1] - 0.1,
              DEFAULT_MAPPING.canvasCenter[2] / 2,
            ]}
            enableDamping dampingFactor={0.1}
            minDistance={0.4} maxDistance={4}
          />

          {building && (
            <Html center>
              <div className="bg-white/90 px-4 py-2 rounded shadow text-xs text-slate-700 border border-slate-300">
                IK 計算中… {(progress * 100).toFixed(0)}%
              </div>
            </Html>
          )}
        </Canvas>

        {frame && (
          <div className="absolute top-2 right-2 bg-white/90 text-xs px-3 py-1.5 rounded shadow border border-slate-300 space-y-0.5">
            <div className="font-mono">iter {selectedIter} / {(cache?.iterations.length ?? 1) - 1}</div>
            <div className="font-mono">RMSE: {(frame.trackingRMSE * 1000).toFixed(2)} mm</div>
            {cache && cache.canvasScale < 0.999 && (
              <div className="font-mono text-slate-500">
                canvas×{cache.canvasScale.toFixed(2)} (auto-fit)
              </div>
            )}
          </div>
        )}
        {cache?.hasUnreachable && (
          <div className="absolute top-2 left-2 bg-amber-100 text-amber-800 text-xs px-2 py-1 rounded border border-amber-300">
            ⚠ 一部のポイントがアームの届かない位置です
          </div>
        )}
      </div>

      <div className="bg-white border-t px-3 py-2 text-xs flex flex-wrap gap-4 items-center">
        <label className="flex items-center gap-2">
          <span className="font-semibold text-slate-700">k_lag (柔軟):</span>
          <input
            type="range" min={0} max={0.02} step={0.001}
            value={kLag} onChange={(e) => setKLag(Number(e.target.value))}
            className="w-24"
          />
          <span className="font-mono w-14 text-right">{kLag.toFixed(3)}</span>
        </label>
        <label className="flex items-center gap-2">
          <span className="font-semibold text-slate-700">最大反復:</span>
          <select
            value={maxIter}
            onChange={(e) => setMaxIter(Number(e.target.value))}
            className="border rounded px-1 py-0.5"
          >
            {[1, 2, 3, 5, 8, 10].map((n) => <option key={n} value={n}>{n}</option>)}
          </select>
        </label>
        <label className="flex items-center gap-2">
          <span className="font-semibold text-slate-700">Lift 高さ:</span>
          <input
            type="range" min={0} max={0.2} step={0.005}
            value={liftHeight} onChange={(e) => setLiftHeight(Number(e.target.value))}
            className="w-24"
          />
          <span className="font-mono w-14 text-right">{(liftHeight * 100).toFixed(1)}cm</span>
        </label>
        <span className="border-l border-slate-300 h-4 mx-1" />
        <label className="flex items-center gap-1 cursor-pointer">
          <input
            type="checkbox" checked={sprayOn}
            onChange={(e) => setSprayOn(e.target.checked)}
          />
          <span className="font-semibold text-slate-700">🎨 スプレー</span>
        </label>
        {sprayOn && (
          <>
            <label className="flex items-center gap-2">
              <span>透明度:</span>
              <input
                type="range" min={0.05} max={1.0} step={0.01}
                value={sprayAlpha}
                onChange={(e) => setSprayAlpha(Number(e.target.value))}
                className="w-24"
              />
              <span className="font-mono w-10 text-right">{sprayAlpha.toFixed(2)}</span>
            </label>
            <label className="flex items-center gap-2">
              <span>最大太さ:</span>
              <input
                type="range" min={3} max={40} step={1}
                value={sprayWMax}
                onChange={(e) => setSprayWMax(Number(e.target.value))}
                className="w-24"
              />
              <span className="font-mono w-10 text-right">{sprayWMax}px</span>
            </label>
          </>
        )}
      </div>
    </div>
  );
}
