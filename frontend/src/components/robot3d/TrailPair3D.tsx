import { useEffect, useMemo } from "react";
import { useFrame } from "@react-three/fiber";
import {
  BufferAttribute, BufferGeometry, Color,
  Line as ThreeLine, LineBasicMaterial,
} from "three";
import type { IterationFrame } from "../../math/ikCacheV2";


interface Props {
  frame: IterationFrame;
  getCurrentTime: () => number;
  desiredColor?: string;
  actualColor?: string;
}

/**
 * desired (灰) と actual (色) の 2 本のラインを 3D で描画。
 * currentTime で末尾を切り取る (drawRange)。
 */
export function TrailPair3D({
  frame, getCurrentTime,
  desiredColor = "#94a3b8",
  actualColor = "#dc2626",
}: Props) {
  const { desiredLine, actualLine } = useMemo(() => {
    const dGeo = new BufferGeometry();
    dGeo.setAttribute("position", new BufferAttribute(frame.desired, 3));
    dGeo.setDrawRange(0, 0);
    // depthTest: false → キャンバスに遮蔽されずに裏表どちらからも描画される。
    // renderOrder を持ち上げて canvas より後に描く。
    const dMat = new LineBasicMaterial({
      color: new Color(desiredColor),
      transparent: true, opacity: 0.6,
      depthTest: false,
    });
    const dLine = new ThreeLine(dGeo, dMat);
    dLine.renderOrder = 10;

    // actual (compliance 出力) を物理的ペン跡として描画。
    // penWorld (FK(q) = IK の推定) は IK 残差分だけズレるので trail には使わない。
    const aGeo = new BufferGeometry();
    aGeo.setAttribute("position", new BufferAttribute(frame.actual, 3));
    aGeo.setDrawRange(0, 0);
    const aMat = new LineBasicMaterial({
      color: new Color(actualColor),
      transparent: true, opacity: 0.95,
      depthTest: false,
    });
    const aLine = new ThreeLine(aGeo, aMat);
    aLine.renderOrder = 11;

    return { desiredLine: dLine, actualLine: aLine };
  }, [frame, desiredColor, actualColor]);

  useEffect(() => {
    return () => {
      desiredLine.geometry.dispose();
      (desiredLine.material as LineBasicMaterial).dispose();
      actualLine.geometry.dispose();
      (actualLine.material as LineBasicMaterial).dispose();
    };
  }, [desiredLine, actualLine]);

  useFrame(() => {
    const T = getCurrentTime();
    const N = frame.ts.length;
    let idx = 0;
    let lo = 0, hi = N - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (frame.ts[mid] < T) lo = mid + 1; else hi = mid;
    }
    idx = Math.max(1, Math.min(lo + 1, N));
    desiredLine.geometry.setDrawRange(0, idx);
    actualLine.geometry.setDrawRange(0, idx);
  });

  return (
    <>
      <primitive object={desiredLine} />
      <primitive object={actualLine} />
    </>
  );
}
