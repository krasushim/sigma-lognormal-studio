import { useRef } from "react";
import type { Group } from "three";
import { useFrame } from "@react-three/fiber";
import { RobotConfig7, DEFAULT_ROBOT_7 } from "../../math/jacobianIK";


interface Props {
  robot?: RobotConfig7;
  /** 関節角 (length 7) を毎フレーム取得する関数 */
  getJointAngles: () => Float32Array;
}

const COLOR_LINK = "#52525b";
const COLOR_JOINT = "#1f2937";
const COLOR_PEN = "#0f172a";
const COLOR_PEN_TIP = "#dc2626";
const COLOR_BASE = "#334155";


/**
 * Baxter 風 7-DOF パラメトリックアーム。
 * 関節構成 (Y-up):
 *   J1 Y  S0 shoulder yaw    @ +Y(baseHeight)
 *   J2 X  S1 shoulder pitch
 *   J3 Y  E0 upper twist     @ +Y(upperArm)
 *   J4 X  E1 elbow pitch
 *   J5 Y  W0 forearm twist   @ +Y(forearm)
 *   J6 X  W1 wrist pitch
 *   J7 Y  W2 wrist roll      @ +Y(wristLen)
 *   pen tip @ +Y(penLength)
 */
export function RobotArm7DOF({ robot = DEFAULT_ROBOT_7, getJointAngles }: Props) {
  const j1 = useRef<Group>(null);
  const j2 = useRef<Group>(null);
  const j3 = useRef<Group>(null);
  const j4 = useRef<Group>(null);
  const j5 = useRef<Group>(null);
  const j6 = useRef<Group>(null);
  const j7 = useRef<Group>(null);

  useFrame(() => {
    const t = getJointAngles();
    if (j1.current) j1.current.rotation.y = t[0];
    if (j2.current) j2.current.rotation.x = t[1];
    if (j3.current) j3.current.rotation.y = t[2];
    if (j4.current) j4.current.rotation.x = t[3];
    if (j5.current) j5.current.rotation.y = t[4];
    if (j6.current) j6.current.rotation.x = t[5];
    if (j7.current) j7.current.rotation.y = t[6];
  });

  const r = robot;

  return (
    <group>
      {/* ベースプレート */}
      <mesh position={[0, 0.015, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.16, 0.18, 0.03, 32]} />
        <meshStandardMaterial color={COLOR_BASE} metalness={0.4} roughness={0.7} />
      </mesh>

      {/* J1 (Y rotation) */}
      <group ref={j1}>
        {/* 基底コラム */}
        <mesh position={[0, r.baseHeight / 2 + 0.03, 0]} castShadow>
          <cylinderGeometry args={[0.085, 0.10, r.baseHeight, 28]} />
          <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
        </mesh>

        {/* 肩部 (J2 X) */}
        <group position={[0, r.baseHeight, 0]}>
          <mesh castShadow>
            <sphereGeometry args={[0.075, 28, 18]} />
            <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
          </mesh>

          <group ref={j2}>
            {/* 上腕 */}
            <mesh position={[0, r.upperArm / 2, 0]} castShadow>
              <cylinderGeometry args={[0.058, 0.052, r.upperArm, 22]} />
              <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
            </mesh>

            <group position={[0, r.upperArm, 0]}>
              {/* 上腕ツイスト関節 (J3 Y) — 上腕の先 */}
              <mesh castShadow>
                <sphereGeometry args={[0.058, 22, 14]} />
                <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
              </mesh>

              <group ref={j3}>
                {/* 肘部 (J4 X) */}
                <group ref={j4}>
                  {/* 前腕 */}
                  <mesh position={[0, r.forearm / 2, 0]} castShadow>
                    <cylinderGeometry args={[0.048, 0.042, r.forearm, 20]} />
                    <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
                  </mesh>

                  <group position={[0, r.forearm, 0]}>
                    {/* 手首 (J5 Y) */}
                    <mesh castShadow>
                      <sphereGeometry args={[0.045, 20, 14]} />
                      <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
                    </mesh>

                    <group ref={j5}>
                      {/* J6 X */}
                      <group ref={j6}>
                        {/* wrist link */}
                        <mesh position={[0, r.wristLen / 2, 0]} castShadow>
                          <cylinderGeometry args={[0.035, 0.030, r.wristLen, 16]} />
                          <meshStandardMaterial color={COLOR_LINK} metalness={0.55} roughness={0.4} />
                        </mesh>

                        <group position={[0, r.wristLen, 0]}>
                          {/* J7 Y */}
                          <mesh castShadow>
                            <sphereGeometry args={[0.030, 16, 12]} />
                            <meshStandardMaterial color={COLOR_JOINT} metalness={0.7} roughness={0.3} />
                          </mesh>
                          <group ref={j7}>
                            {/* ペン本体 */}
                            <mesh position={[0, r.penLength / 2, 0]} castShadow>
                              <cylinderGeometry args={[0.012, 0.006, r.penLength, 14]} />
                              <meshStandardMaterial color={COLOR_PEN} metalness={0.1} roughness={0.85} />
                            </mesh>
                            {/* ペン先 */}
                            <mesh position={[0, r.penLength, 0]} castShadow>
                              <sphereGeometry args={[0.008, 14, 10]} />
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
      </group>
    </group>
  );
}
