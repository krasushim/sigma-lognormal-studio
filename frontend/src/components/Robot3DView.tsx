import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Html } from "@react-three/drei";

import type { TrajectoryPlan } from "../api/types";
import { IKCache } from "../math/ikCache";
import { DEFAULT_MAPPING, CanvasMappingParams } from "../math/canvasMapping";
import { DEFAULT_ROBOT, RobotParams, JointAngles, homeTheta } from "../math/robotKinematics";

import { SceneLights } from "./robot3d/SceneLights";
import { RobotArm } from "./robot3d/RobotArm";
import { DrawingCanvas3D } from "./robot3d/DrawingCanvas3D";
import { PenTrail3D } from "./robot3d/PenTrail3D";
import { Arm3DAnimator } from "./robot3d/Arm3DAnimator";


interface Props {
  plan: TrajectoryPlan;
  currentTime: number;
  totalDuration: number;
  mapping?: CanvasMappingParams;
  robot?: RobotParams;
  showGrid?: boolean;
  showTrail?: boolean;
}


export function Robot3DView({
  plan, currentTime,
  mapping = DEFAULT_MAPPING, robot = DEFAULT_ROBOT,
  showGrid = true, showTrail = true,
}: Props) {
  const [cache, setCache] = useState<IKCache | null>(null);
  const [building, setBuilding] = useState(false);
  const [buildProgress, setBuildProgress] = useState(0);

  // プランが更新されたら IK を事前計算
  useEffect(() => {
    if (!plan || plan.stroke_groups.length === 0) {
      setCache(null);
      return;
    }
    setBuilding(true);
    setBuildProgress(0);
    // 非同期化 (UI ブロック回避のため setTimeout で次ティックに回す)
    const id = setTimeout(() => {
      try {
        const c = new IKCache(plan, mapping, robot, 0.3, (frac) => {
          setBuildProgress(frac);
        });
        setCache(c);
      } finally {
        setBuilding(false);
        setBuildProgress(1);
      }
    }, 20);
    return () => clearTimeout(id);
  }, [plan, mapping, robot]);

  // レンダリングループで参照するため ref で関節角・時刻を共有
  const jointAnglesRef = useRef<JointAngles>(homeTheta());
  const currentTimeRef = useRef<number>(currentTime);
  useEffect(() => { currentTimeRef.current = currentTime; }, [currentTime]);

  const getCurrentTime = useMemo(() => () => currentTimeRef.current, []);
  const getJointAngles = useMemo(() => () => jointAnglesRef.current, []);

  return (
    <div className="relative w-full h-full bg-slate-200 rounded overflow-hidden">
      <Canvas
        shadows
        camera={{ position: [1.1, 0.85, -1.1], fov: 35, near: 0.05, far: 20 }}
      >
        <color attach="background" args={["#e2e8f0"]} />
        <SceneLights />

        {/* Ground */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
          <planeGeometry args={[3, 3]} />
          <meshStandardMaterial color="#cbd5e1" roughness={0.9} />
        </mesh>
        {showGrid && (
          <Grid
            args={[3, 3]}
            cellColor="#94a3b8"
            sectionColor="#64748b"
            cellSize={0.1}
            sectionSize={0.5}
            position={[0, 0.001, 0]}
            fadeDistance={5}
          />
        )}

        <DrawingCanvas3D mapping={mapping} />
        <RobotArm robot={robot} getJointAngles={getJointAngles} />

        {cache && (
          <Arm3DAnimator
            cache={cache}
            getCurrentTime={getCurrentTime}
            jointAnglesRef={jointAnglesRef}
          />
        )}
        {cache && showTrail && (
          <PenTrail3D cache={cache} getCurrentTime={getCurrentTime} />
        )}

        <OrbitControls
          target={[mapping.canvasCenter[0], mapping.canvasCenter[1] - 0.1, mapping.canvasCenter[2] / 2]}
          enableDamping
          dampingFactor={0.1}
          minDistance={0.4}
          maxDistance={4}
        />

        {building && (
          <Html center>
            <div className="bg-white/90 px-4 py-2 rounded shadow text-xs text-slate-700 border border-slate-300">
              IK 事前計算中… {(buildProgress * 100).toFixed(0)}%
            </div>
          </Html>
        )}
      </Canvas>

      {cache?.hasUnreachable && (
        <div className="absolute top-2 right-2 bg-amber-100 text-amber-800 text-xs px-2 py-1 rounded border border-amber-300">
          ⚠ 一部のポイントがアームの届かない位置です
        </div>
      )}
    </div>
  );
}
