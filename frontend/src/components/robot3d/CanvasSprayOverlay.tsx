import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import {
  CanvasTexture, ClampToEdgeWrapping, DoubleSide,
  LinearFilter, Quaternion, Vector3,
} from "three";
import type { IterationFrame } from "../../math/ikCacheV2";
import type { CanvasMappingParams } from "../../math/canvasMapping";
import {
  projectSamplesToUV, renderSpray3DOnCanvas,
  DEFAULT_SPRAY, SprayParams,
} from "../../math/spray3D";


interface Props {
  frame: IterationFrame;
  mapping: CanvasMappingParams;
  planImageWidth: number;
  planImageHeight: number;
  getCurrentTime: () => number;
  sprayParams?: Partial<SprayParams>;
  color?: string;
  /** オフスクリーン canvas の画素サイズ (長辺) */
  resolution?: number;
}


/**
 * Canvas 平面の 1mm 手前 (robot 側) に半透明オーバーレイ平面を配置し、
 * 2D Canvas API で描いた spray を CanvasTexture として貼る。
 *
 *  - frame.actual (compliance 出力) をサンプルとして使用
 *  - frame.isPenUp でペン浮き区間をスキップ
 *  - currentTime に応じて毎フレーム再描画 (スクラブ対応)
 */
export function CanvasSprayOverlay({
  frame, mapping, planImageWidth, planImageHeight,
  getCurrentTime,
  sprayParams,
  color = "#111827",
  resolution = 1024,
}: Props) {
  // オフスクリーン canvas (アスペクト比はキャンバス平面に一致)
  const offscreen = useMemo(() => {
    const asp = mapping.canvasWidth / mapping.canvasHeight;
    const c = document.createElement("canvas");
    if (asp >= 1) {
      c.width = resolution;
      c.height = Math.max(16, Math.round(resolution / asp));
    } else {
      c.height = resolution;
      c.width = Math.max(16, Math.round(resolution * asp));
    }
    return c;
  }, [mapping.canvasWidth, mapping.canvasHeight, resolution]);

  const texture = useMemo(() => {
    const t = new CanvasTexture(offscreen);
    t.wrapS = t.wrapT = ClampToEdgeWrapping;
    t.minFilter = LinearFilter;
    t.magFilter = LinearFilter;
    t.needsUpdate = true;
    return t;
  }, [offscreen]);

  // Offscreen canvas を 2D context で取得
  const ctx = useMemo(
    () => offscreen.getContext("2d") ?? null,
    [offscreen],
  );

  useEffect(() => () => texture.dispose(), [texture]);

  // 3D サンプル → UV + 速度 変換 (frame 毎に 1 回計算してキャッシュ)
  const samplesRef = useRef<ReturnType<typeof projectSamplesToUV> | null>(null);
  useEffect(() => {
    samplesRef.current = projectSamplesToUV(
      frame.actual, frame.ts, frame.isPenUp,
      mapping, planImageWidth, planImageHeight,
    );
    // frame/mapping が変わったら即時再描画 (最大時刻で)
    if (ctx) {
      const p = { ...DEFAULT_SPRAY, ...(sprayParams ?? {}) };
      renderSpray3DOnCanvas(
        ctx, offscreen.width, offscreen.height,
        samplesRef.current, frame.ts[frame.ts.length - 1] ?? 0,
        p, color,
      );
      texture.needsUpdate = true;
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [frame, mapping, planImageWidth, planImageHeight, color, sprayParams]);

  // 毎フレーム: currentTime までを再描画
  // 時刻が前後してもスクラブで整合する (毎回クリア+再描画)
  useFrame(() => {
    if (!ctx || !samplesRef.current) return;
    const T = getCurrentTime();
    const p = { ...DEFAULT_SPRAY, ...(sprayParams ?? {}) };
    renderSpray3DOnCanvas(
      ctx, offscreen.width, offscreen.height,
      samplesRef.current, T,
      p, color,
    );
    texture.needsUpdate = true;
  });

  // キャンバス面の +normal 方向 (robot 側) に 1mm 手前へ配置して z-fighting を回避
  const offsetAlongNormal = 0.001;
  const pos = useMemo<[number, number, number]>(() => [
    mapping.canvasCenter[0] + mapping.canvasNormal[0] * offsetAlongNormal,
    mapping.canvasCenter[1] + mapping.canvasNormal[1] * offsetAlongNormal,
    mapping.canvasCenter[2] + mapping.canvasNormal[2] * offsetAlongNormal,
  ], [mapping.canvasCenter, mapping.canvasNormal]);

  // planeGeometry は初期法線 +Z なので、mapping.canvasNormal に合わせて回転
  const quat = useMemo(() => {
    const q = new Quaternion().setFromUnitVectors(
      new Vector3(0, 0, 1),
      new Vector3(...mapping.canvasNormal).normalize(),
    );
    return [q.x, q.y, q.z, q.w] as [number, number, number, number];
  }, [mapping.canvasNormal]);

  // 実効 canvas 寸法 (画像アスペクト比に合わせて contain fit したサイズで平面を張る)
  const [useW, useH] = useMemo(() => {
    const imgAsp = planImageWidth / planImageHeight;
    const canAsp = mapping.canvasWidth / mapping.canvasHeight;
    if (imgAsp > canAsp) return [mapping.canvasWidth, mapping.canvasWidth / imgAsp];
    return [mapping.canvasHeight * imgAsp, mapping.canvasHeight];
  }, [mapping.canvasWidth, mapping.canvasHeight, planImageWidth, planImageHeight]);

  return (
    <mesh position={pos} quaternion={quat}>
      <planeGeometry args={[useW, useH]} />
      <meshBasicMaterial
        map={texture}
        transparent
        depthWrite={false}
        side={DoubleSide}
      />
    </mesh>
  );
}
