import { useMemo } from "react";
import type { GmlDocument, MpcResult } from "../api/types";

interface Props {
  doc: GmlDocument;
  selected: number[] | null;
  width?: number;
  height?: number;
  /** Playback time in seconds (undefined = render all strokes fully). */
  currentTime?: number;
  /** Global start time per stroke (by doc.strokes index). Parallel to ``doc.strokes``. */
  strokeStarts?: number[];
  /** MPC 推定結果 (あれば経由点楕円 + 平滑軌跡をオーバーレイ表示) */
  mpcResult?: MpcResult | null;
}

const PALETTE = [
  "#2563eb", "#7c3aed", "#db2777", "#dc2626",
  "#ea580c", "#ca8a04", "#16a34a", "#0891b2",
];

// MPC オーバーレイの色定数
const MPC_TRAJ_COLOR  = "#111827";   // 黒に近いダークグレー
const MPC_ELLIPSE_STROKE = "#10b981"; // エメラルド
const MPC_CENTER_COLOR   = "#10b981";

interface RenderedStroke {
  key: number;
  d: string;
  color: string;
  dim: boolean;
  headX: number | null;
  headY: number | null;
}

export function GmlPreview({
  doc, selected, width = 400, height = 300,
  currentTime, strokeStarts, mpcResult,
}: Props) {
  const [sw, sh] = doc.screen_bounds;
  const scale = useMemo(() => {
    if (sw <= 0 || sh <= 0) return 1;
    return Math.min(width / sw, height / sh);
  }, [sw, sh, width, height]);

  const selSet = useMemo(
    () => (selected ? new Set(selected) : null),
    [selected],
  );

  const rendered = useMemo<RenderedStroke[]>(() => {
    return doc.strokes.map((s, i) => {
      const color = PALETTE[i % PALETTE.length];
      const isSelected = selSet === null || selSet.has(s.index);
      const pts = s.points;
      if (pts.length === 0) {
        return { key: s.index, d: "", color, dim: !isSelected, headX: null, headY: null };
      }

      if (currentTime != null && strokeStarts && i < strokeStarts.length) {
        const t0 = pts[0][2];
        const tLast = pts[pts.length - 1][2];
        const strokeDur = Math.max(tLast - t0, 0);
        const elapsed = currentTime - strokeStarts[i];
        if (elapsed <= 0) {
          return { key: s.index, d: "", color, dim: !isSelected, headX: null, headY: null };
        }
        if (elapsed >= strokeDur) {
          const d = pts
            .map((p, k) => `${k === 0 ? "M" : "L"}${p[0].toFixed(1)},${p[1].toFixed(1)}`)
            .join(" ");
          const last = pts[pts.length - 1];
          return { key: s.index, d, color, dim: !isSelected, headX: last[0], headY: last[1] };
        }
        const targetLocal = elapsed;
        let lastCompleteIdx = 0;
        for (let k = 0; k < pts.length; k++) {
          const localT = pts[k][2] - t0;
          if (localT <= targetLocal) lastCompleteIdx = k;
          else break;
        }
        const segs: string[] = [];
        for (let k = 0; k <= lastCompleteIdx; k++) {
          const p = pts[k];
          segs.push(`${k === 0 ? "M" : "L"}${p[0].toFixed(1)},${p[1].toFixed(1)}`);
        }
        let headX: number | null = null;
        let headY: number | null = null;
        if (lastCompleteIdx + 1 < pts.length) {
          const a = pts[lastCompleteIdx];
          const b = pts[lastCompleteIdx + 1];
          const tA = a[2] - t0;
          const tB = b[2] - t0;
          const span = Math.max(tB - tA, 1e-6);
          const frac = Math.min(Math.max((targetLocal - tA) / span, 0), 1);
          const ix = a[0] + (b[0] - a[0]) * frac;
          const iy = a[1] + (b[1] - a[1]) * frac;
          segs.push(`L${ix.toFixed(1)},${iy.toFixed(1)}`);
          headX = ix;
          headY = iy;
        } else {
          const last = pts[pts.length - 1];
          headX = last[0];
          headY = last[1];
        }
        return { key: s.index, d: segs.join(" "), color, dim: !isSelected, headX, headY };
      }

      // Static mode
      const d = pts
        .map((p, k) => `${k === 0 ? "M" : "L"}${p[0].toFixed(1)},${p[1].toFixed(1)}`)
        .join(" ");
      return { key: s.index, d, color, dim: !isSelected, headX: null, headY: null };
    });
  }, [doc.strokes, selSet, currentTime, strokeStarts]);

  const strokeWidth = Math.max(1.5 / scale, 1);
  const showHead = currentTime != null;

  return (
    <svg
      width={width}
      height={height}
      viewBox={`0 0 ${sw} ${sh}`}
      preserveAspectRatio="xMidYMid meet"
      className="bg-slate-50 rounded border border-slate-200"
    >
      {/* 元の GML ストローク */}
      {rendered.map((r) =>
        r.d ? (
          <path
            key={r.key}
            d={r.d}
            fill="none"
            stroke={r.dim ? "#cbd5e1" : r.color}
            strokeWidth={strokeWidth}
            strokeLinecap="round"
            strokeLinejoin="round"
            opacity={r.dim ? 0.4 : 1}
          />
        ) : null,
      )}

      {/* ペン先 (アニメーション時) */}
      {showHead &&
        rendered.map((r) =>
          r.headX != null && r.headY != null && !r.dim ? (
            <circle
              key={`head-${r.key}`}
              cx={r.headX}
              cy={r.headY}
              r={strokeWidth * 2}
              fill={r.color}
              stroke="#fff"
              strokeWidth={strokeWidth * 0.5}
            />
          ) : null,
        )}

      {/* MPC オーバーレイ */}
      {mpcResult?.strokes.map((ms) => (
        <g key={`mpc-${ms.stroke_index}`}>
          {/* 平滑化された MPC 軌跡 */}
          {ms.traj_x.length > 0 && (
            <path
              d={ms.traj_x
                .map((x, k) =>
                  `${k === 0 ? "M" : "L"}${x.toFixed(1)},${ms.traj_y[k].toFixed(1)}`
                )
                .join(" ")}
              fill="none"
              stroke={MPC_TRAJ_COLOR}
              strokeWidth={strokeWidth * 1.8}
              strokeLinecap="round"
              strokeLinejoin="round"
              opacity={0.85}
            />
          )}

          {/* ガウス経由点: 信頼楕円 + 中心点 */}
          {ms.ellipses.map((el, ei) => (
            <g
              key={`el-${ei}`}
              transform={`rotate(${el.angle_deg.toFixed(2)},${el.x.toFixed(1)},${el.y.toFixed(1)})`}
            >
              <ellipse
                cx={el.x}
                cy={el.y}
                rx={Math.max(el.rx, 0.5)}
                ry={Math.max(el.ry, 0.5)}
                fill={MPC_ELLIPSE_STROKE}
                fillOpacity={0.08}
                stroke={MPC_ELLIPSE_STROKE}
                strokeWidth={strokeWidth * 0.8}
                strokeOpacity={0.7}
              />
            </g>
          ))}
          {ms.ellipses.map((el, ei) => (
            <circle
              key={`ctr-${ei}`}
              cx={el.x}
              cy={el.y}
              r={strokeWidth * 1.6}
              fill={MPC_CENTER_COLOR}
              opacity={0.9}
            />
          ))}
        </g>
      ))}
    </svg>
  );
}
