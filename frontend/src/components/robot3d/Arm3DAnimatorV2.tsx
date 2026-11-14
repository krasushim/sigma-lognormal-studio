import { MutableRefObject } from "react";
import { useFrame } from "@react-three/fiber";
import type { IterationFrame } from "../../math/ikCacheV2";


interface Props {
  frame: IterationFrame;
  getCurrentTime: () => number;
  anglesRef: MutableRefObject<Float32Array>;
}

/**
 * 毎フレーム IterationFrame.jointAngles から 7 関節角を線形補間してサンプリングし、
 * 共有 ref に書き込む。RobotArm7DOF 側はこの ref を読んで rotation を更新する。
 */
export function Arm3DAnimatorV2({ frame, getCurrentTime, anglesRef }: Props) {
  useFrame(() => {
    const T = getCurrentTime();
    const ts = frame.ts;
    const N = ts.length;
    if (N === 0) return;
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (ts[mid] < T) lo = mid + 1; else hi = mid;
    }
    const i1 = lo;
    const i0 = Math.max(i1 - 1, 0);
    const t0 = ts[i0], t1 = ts[i1];
    const a = i0 === i1 ? 0 : (T - t0) / Math.max(t1 - t0, 1e-9);
    const out = anglesRef.current;
    for (let k = 0; k < 7; k++) {
      const v0 = frame.jointAngles[i0 * 7 + k];
      const v1 = frame.jointAngles[i1 * 7 + k];
      out[k] = v0 + a * (v1 - v0);
    }
  });
  return null;
}
