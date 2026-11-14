import { MutableRefObject } from "react";
import { useFrame } from "@react-three/fiber";
import type { IKCache } from "../../math/ikCache";
import type { JointAngles } from "../../math/robotKinematics";


interface Props {
  cache: IKCache;
  getCurrentTime: () => number;
  jointAnglesRef: MutableRefObject<JointAngles>;
}

/**
 * 毎フレーム IK キャッシュから関節角をサンプリングして共有 ref を更新する。
 * (RobotArm 側は useFrame で ref を読んで rotation に適用する)
 */
export function Arm3DAnimator({ cache, getCurrentTime, jointAnglesRef }: Props) {
  useFrame(() => {
    const T = getCurrentTime();
    const { gi, localT } = cache.resolveTime(T);
    cache.sampleJointAngles(gi, localT, jointAnglesRef.current);
  });
  return null;
}
