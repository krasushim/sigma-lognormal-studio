import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import {
  BufferAttribute, BufferGeometry,
  Line as ThreeLine, LineBasicMaterial,
} from "three";
import type { IKCache } from "../../math/ikCache";


interface Props {
  cache: IKCache;
  getCurrentTime: () => number;
  color?: string;
  maxSamples?: number;
}


/**
 * 各 group の全サンプルを連結したペン先軌跡を 3D で描画する。
 * currentTime に応じて描画範囲 (drawRange) を毎フレーム動的に制御。
 */
export function PenTrail3D({
  cache, getCurrentTime, color = "#111827", maxSamples = 5000,
}: Props) {
  const lineRef = useRef<ThreeLine | null>(null);

  // 全 group の penWorld を 1 本のバッファに連結
  const { positions, groupIndexRanges } = useMemo(() => {
    const total = cache.groups.reduce((a, g) => a + g.penWorld.length / 3, 0);
    const capped = Math.min(total, maxSamples);
    const pos = new Float32Array(capped * 3);
    const ranges: { start: number; end: number; groupIdx: number }[] = [];
    let w = 0;
    for (let gi = 0; gi < cache.groups.length; gi++) {
      const arr = cache.groups[gi].penWorld;
      const n = arr.length / 3;
      if (w + n > capped) break;
      pos.set(arr.subarray(0, n * 3), w * 3);
      ranges.push({ start: w, end: w + n, groupIdx: gi });
      w += n;
    }
    return { positions: pos, groupIndexRanges: ranges };
  }, [cache, maxSamples]);

  const line = useMemo(() => {
    const geo = new BufferGeometry();
    geo.setAttribute("position", new BufferAttribute(positions, 3));
    geo.setDrawRange(0, 0);
    const mat = new LineBasicMaterial({
      color, linewidth: 2, transparent: true, opacity: 0.95,
    });
    return new ThreeLine(geo, mat);
  }, [positions, color]);

  useEffect(() => {
    return () => {
      line.geometry.dispose();
      (line.material as LineBasicMaterial).dispose();
    };
  }, [line]);

  useFrame(() => {
    const T = getCurrentTime();
    const { gi, localT } = cache.resolveTime(T);
    let endIdx = 0;
    for (const r of groupIndexRanges) {
      if (r.groupIdx < gi) {
        endIdx = r.end;
      } else if (r.groupIdx === gi) {
        const g = cache.groups[r.groupIdx];
        let lo = 0, hi = g.t.length - 1;
        while (lo < hi) {
          const mid = (lo + hi) >> 1;
          if (g.t[mid] < localT) lo = mid + 1; else hi = mid;
        }
        const inGroup = Math.max(1, Math.min(lo + 1, g.t.length));
        endIdx = r.start + inGroup;
        break;
      }
    }
    line.geometry.setDrawRange(0, endIdx);
  });

  // Forward ref for parent access if needed.
  useEffect(() => { lineRef.current = line; }, [line]);

  return <primitive object={line} />;
}
