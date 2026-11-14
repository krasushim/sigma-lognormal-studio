import { useRef } from "react";
import type { Group } from "three";
import { useFrame } from "@react-three/fiber";
import { RobotParams, DEFAULT_ROBOT, JointAngles } from "../../math/robotKinematics";


interface Props {
  robot?: RobotParams;
  /** 現在の関節角 (Float32Array length 6) — useFrame 側でミューテート可能 */
  getJointAngles: () => JointAngles;
}

const COLOR_LINK = "#5b6b7a";
const COLOR_JOINT = "#1f2937";
const COLOR_PEN = "#0f172a";
const COLOR_PEN_TIP = "#dc2626";


/**
 * 6DOF パラメトリック・アーム。
 * 関節角はレンダリングフレーム毎に getJointAngles() で取得し、
 * Three.js の Object3D 階層として FK を構成する。
 *
 * 各関節の軸は次の通り:
 *   J1: Y, J2: X, J3: X, J4: Y, J5: X, J6: Y
 */
export function RobotArm({ robot = DEFAULT_ROBOT, getJointAngles }: Props) {
  const j1Ref = useRef<Group>(null);
  const j2Ref = useRef<Group>(null);
  const j3Ref = useRef<Group>(null);
  const j4Ref = useRef<Group>(null);
  const j5Ref = useRef<Group>(null);
  const j6Ref = useRef<Group>(null);

  useFrame(() => {
    const t = getJointAngles();
    if (j1Ref.current) j1Ref.current.rotation.y = t[0];
    if (j2Ref.current) j2Ref.current.rotation.x = t[1];
    if (j3Ref.current) j3Ref.current.rotation.x = t[2];
    if (j4Ref.current) j4Ref.current.rotation.y = t[3];
    if (j5Ref.current) j5Ref.current.rotation.x = t[4];
    if (j6Ref.current) j6Ref.current.rotation.y = t[5];
  });

  const r = robot;

  return (
    <group>
      {/* --- Fixed base plate (on ground) --- */}
      <mesh position={[0, 0.015, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.14, 0.16, 0.03, 32]} />
        <meshStandardMaterial color="#334155" metalness={0.4} roughness={0.7} />
      </mesh>

      {/* J1 (Y rotation at origin) */}
      <group ref={j1Ref}>
        {/* Base column */}
        <mesh position={[0, r.baseHeight / 2 + 0.03, 0]} castShadow>
          <cylinderGeometry args={[0.08, 0.095, r.baseHeight, 24]} />
          <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
        </mesh>
        {/* Shoulder joint housing */}
        <group position={[0, r.baseHeight, 0]}>
          <mesh castShadow>
            <sphereGeometry args={[0.07, 24, 16]} />
            <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
          </mesh>

          {/* J2 (X rotation) */}
          <group ref={j2Ref}>
            {/* Upper arm (cylinder from shoulder to elbow) */}
            <mesh position={[0, r.upperArm / 2, 0]} castShadow>
              <cylinderGeometry args={[0.055, 0.05, r.upperArm, 20]} />
              <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
            </mesh>

            <group position={[0, r.upperArm, 0]}>
              {/* Elbow joint */}
              <mesh castShadow>
                <sphereGeometry args={[0.055, 20, 14]} />
                <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
              </mesh>

              {/* J3 (X rotation) */}
              <group ref={j3Ref}>
                {/* Forearm */}
                <mesh position={[0, r.forearm / 2, 0]} castShadow>
                  <cylinderGeometry args={[0.045, 0.04, r.forearm, 20]} />
                  <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
                </mesh>

                <group position={[0, r.forearm, 0]}>
                  {/* Wrist joint housing */}
                  <mesh castShadow>
                    <sphereGeometry args={[0.045, 20, 14]} />
                    <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
                  </mesh>

                  {/* J4 (Y rotation) */}
                  <group ref={j4Ref}>
                    {/* J5 (X rotation) */}
                    <group ref={j5Ref}>
                      {/* J6 (Y rotation — unused for round pen but included) */}
                      <group ref={j6Ref}>
                        {/* Pen (cylinder from wrist to tip) */}
                        <mesh position={[0, r.penLength / 2, 0]} castShadow>
                          <cylinderGeometry args={[0.01, 0.006, r.penLength, 12]} />
                          <meshStandardMaterial color={COLOR_PEN} metalness={0.1} roughness={0.8} />
                        </mesh>
                        {/* Pen tip marker */}
                        <mesh position={[0, r.penLength, 0]} castShadow>
                          <sphereGeometry args={[0.007, 12, 8]} />
                          <meshStandardMaterial color={COLOR_PEN_TIP} />
                        </mesh>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}
