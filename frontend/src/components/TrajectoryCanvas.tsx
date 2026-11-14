import { useEffect, useMemo, useRef, useState } from "react";
import type { TrajectoryPlan } from "../api/types";
import { useStudio } from "../store/studioStore";
import {
  DEFAULT_SPRAY, SprayParams, hexToRgb, renderSprayGroup,
} from "../math/spray";

interface Props {
  plan: TrajectoryPlan;
  showContour?: boolean;
  showDomPoints?: boolean;
  showActionPlan?: boolean;
  showTrajectory?: boolean;
  showVirtualTargets?: boolean;
  spray?: boolean;
  sprayParams?: Partial<SprayParams>;
  /** spray 使用時にすべての group を 1 色に揃える (グラフィティは通常単色) */
  sprayMonoColor?: string | null;
  interactive?: boolean;
  /** if set, only draw trajectory up to this absolute time (s). undefined = full */
  currentTime?: number;
  /** group start times in absolute timeline (s). Required when currentTime is given. */
  groupStartTimes?: number[];
  uid?: string;
  /** 背景に重ねる raster 画像 (解析ステージ表示用) */
  backgroundImageUrl?: string | null;
}

const STROKE_COLORS = [
  "#111827", "#7f1d1d", "#1e3a8a", "#3f6212",
  "#581c87", "#831843", "#0c4a6e", "#713f12",
  "#064e3b", "#312e81", "#155e75", "#7c2d12",
];
const colorAt = (i: number) => STROKE_COLORS[i % STROKE_COLORS.length];

/**
 * Build a smooth SVG path through the timeseries samples.
 *
 * Uses centripetal Catmull-Rom → cubic Bézier conversion so the curve passes
 * through every sample point with C1 continuity, eliminating the faceted
 * corners of a plain ``M...L...L`` polyline. Endpoints are handled by
 * duplicating the boundary control points.
 */
function buildPathUpTo(
  t: number[], x: number[], y: number[], tCutoff: number,
): string {
  if (!t.length) return "";
  // Collect (x, y) samples up to the cutoff, inserting a linearly interpolated
  // point at the cutoff boundary so animation ends on the right spot.
  const px: number[] = [];
  const py: number[] = [];
  if (t[0] <= tCutoff) {
    px.push(x[0]);
    py.push(y[0]);
  }
  for (let i = 1; i < t.length; i++) {
    if (t[i] > tCutoff) {
      const t0 = t[i - 1];
      const a = (tCutoff - t0) / Math.max(t[i] - t0, 1e-9);
      px.push(x[i - 1] + a * (x[i] - x[i - 1]));
      py.push(y[i - 1] + a * (y[i] - y[i - 1]));
      return catmullRomPath(px, py);
    }
    px.push(x[i]);
    py.push(y[i]);
  }
  return catmullRomPath(px, py);
}

/**
 * Convert a sample polyline to a cubic-Bézier SVG path via Catmull-Rom
 * splines. Produces ``M ... C ... C ...`` for n≥2 points.
 */
function catmullRomPath(px: number[], py: number[]): string {
  const n = px.length;
  if (n === 0) return "";
  if (n === 1) return `M ${fmt(px[0])},${fmt(py[0])}`;
  if (n === 2) {
    return `M ${fmt(px[0])},${fmt(py[0])} L ${fmt(px[1])},${fmt(py[1])}`;
  }
  let d = `M ${fmt(px[0])},${fmt(py[0])}`;
  // Tension parameter for uniform Catmull-Rom → Bézier (α = 0.5 classical).
  const k = 1 / 6;
  for (let i = 0; i < n - 1; i++) {
    const p0x = i === 0 ? px[0] : px[i - 1];
    const p0y = i === 0 ? py[0] : py[i - 1];
    const p1x = px[i], p1y = py[i];
    const p2x = px[i + 1], p2y = py[i + 1];
    const p3x = i + 2 < n ? px[i + 2] : px[n - 1];
    const p3y = i + 2 < n ? py[i + 2] : py[n - 1];
    const c1x = p1x + (p2x - p0x) * k;
    const c1y = p1y + (p2y - p0y) * k;
    const c2x = p2x - (p3x - p1x) * k;
    const c2y = p2y - (p3y - p1y) * k;
    d += ` C ${fmt(c1x)},${fmt(c1y)} ${fmt(c2x)},${fmt(c2y)} ${fmt(p2x)},${fmt(p2y)}`;
  }
  return d;
}

function fmt(v: number): string {
  return v.toFixed(2);
}

export function TrajectoryCanvas({
  plan,
  showContour = true,
  showDomPoints = true,
  showActionPlan = true,
  showTrajectory = true,
  showVirtualTargets = true,
  spray = false,
  sprayParams,
  sprayMonoColor = "#111827",
  interactive = true,
  currentTime,
  groupStartTimes,
  backgroundImageUrl,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const [drag, setDrag] = useState<{ groupIdx: number; targetIdx: number } | null>(null);
  const updateTarget = useStudio((s) => s.updateTargetLocal);
  const refit = useStudio((s) => s.refitCurrent);
  const select = useStudio((s) => s.selectStroke);
  const selected = useStudio((s) => s.selected);

  const { minX, minY, vw, vh } = useMemo(() => {
    const xs: number[] = [];
    const ys: number[] = [];
    for (const g of plan.stroke_groups) {
      for (const [x, y] of g.input_contour) { xs.push(x); ys.push(y); }
      for (const [x, y] of g.action_plan.virtual_targets) { xs.push(x); ys.push(y); }
      const { x: tx, y: ty } = g.derived.timeseries;
      for (let i = 0; i < tx.length; i++) { xs.push(tx[i]); ys.push(ty[i]); }
      xs.push(g.action_plan.p0[0]);
      ys.push(g.action_plan.p0[1]);
    }
    if (xs.length === 0) {
      return { minX: 0, minY: 0, vw: plan.image_width, vh: plan.image_height };
    }
    const xmin = Math.min(...xs) - 20;
    const ymin = Math.min(...ys) - 20;
    const xmax = Math.max(...xs) + 20;
    const ymax = Math.max(...ys) + 20;
    return {
      minX: xmin, minY: ymin,
      vw: Math.max(xmax - xmin, 10),
      vh: Math.max(ymax - ymin, 10),
    };
  }, [plan]);

  const sz = Math.max(vw, vh);

  // ---- Canvas spray 描画 (spray=true のみ) ----
  useEffect(() => {
    if (!spray) return;
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const dpr = window.devicePixelRatio || 1;
    const cw = container.clientWidth;
    const ch = container.clientHeight;
    if (cw === 0 || ch === 0) return;

    // viewBox と同じ fit 方式 (xMidYMid meet) で canvas を設定
    const viewportAspect = cw / ch;
    const worldAspect = vw / vh;
    let screenPerWorld: number;
    let offsetX: number;
    let offsetY: number;
    if (viewportAspect > worldAspect) {
      screenPerWorld = ch / vh;
      offsetX = (cw - vw * screenPerWorld) / 2;
      offsetY = 0;
    } else {
      screenPerWorld = cw / vw;
      offsetX = 0;
      offsetY = (ch - vh * screenPerWorld) / 2;
    }

    canvas.width = Math.floor(cw * dpr);
    canvas.height = Math.floor(ch * dpr);
    canvas.style.width = `${cw}px`;
    canvas.style.height = `${ch}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cw, ch);

    const worldToScreenX = (wx: number) => offsetX + (wx - minX) * screenPerWorld;
    const worldToScreenY = (wy: number) => offsetY + (wy - minY) * screenPerWorld;

    const params: SprayParams = { ...DEFAULT_SPRAY, ...(sprayParams ?? {}) };

    for (let gi = 0; gi < plan.stroke_groups.length; gi++) {
      const g = plan.stroke_groups[gi];
      let tCutoff = g.derived.t_end;
      if (currentTime != null && groupStartTimes) {
        tCutoff = Math.max(0, currentTime - groupStartTimes[gi]);
        tCutoff = Math.min(tCutoff, g.derived.t_end);
      }
      if (tCutoff <= 0) continue;
      const color = sprayMonoColor ?? colorAt(gi);
      renderSprayGroup(
        ctx, g, hexToRgb(color),
        tCutoff,
        worldToScreenX, worldToScreenY, screenPerWorld,
        params,
        gi * 1000 + 17,
      );
    }
  }, [plan, spray, sprayParams, sprayMonoColor, currentTime, groupStartTimes, minX, minY, vw, vh]);

  // ---- SVG 相互作用 ----
  const svgToWorld = (e: React.MouseEvent): [number, number] => {
    const r = svgRef.current!.getBoundingClientRect();
    const fx = (e.clientX - r.left) / r.width;
    const fy = (e.clientY - r.top) / r.height;
    return [minX + fx * vw, minY + fy * vh];
  };
  const onMouseMove = (e: React.MouseEvent) => {
    if (drag == null) return;
    const xy = svgToWorld(e);
    updateTarget(drag.groupIdx, drag.targetIdx, xy);
  };
  const onMouseUp = () => {
    if (drag != null) { setDrag(null); refit(); }
  };

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full rounded overflow-hidden"
      style={{ backgroundColor: spray ? "#f1f5f9" : "#e2e8f0" }}
    >
      {spray && (
        <canvas
          ref={canvasRef}
          className="absolute inset-0 pointer-events-none"
        />
      )}
      <svg
        ref={svgRef}
        viewBox={`${minX} ${minY} ${vw} ${vh}`}
        preserveAspectRatio="xMidYMid meet"
        className="absolute inset-0 w-full h-full"
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
      >
        {backgroundImageUrl && (
          <image
            href={backgroundImageUrl}
            x={0}
            y={0}
            width={plan.image_width}
            height={plan.image_height}
            preserveAspectRatio="xMidYMid meet"
            style={{ opacity: 0.9 }}
          />
        )}
        {plan.stroke_groups.map((g, gi) => {
          const color = colorAt(gi);
          let tCutoff = g.derived.t_end;
          if (currentTime != null && groupStartTimes) {
            tCutoff = Math.max(0, currentTime - groupStartTimes[gi]);
            tCutoff = Math.min(tCutoff, g.derived.t_end);
          }
          const { t, x, y } = g.derived.timeseries;
          const trajD = buildPathUpTo(t, x, y, tCutoff);
          const actionD = (() => {
            const { p0, virtual_targets } = g.action_plan;
            let d = `M ${p0[0]},${p0[1]}`;
            for (const [vx, vy] of virtual_targets) d += ` L ${vx},${vy}`;
            return d;
          })();
          const tipPos = (() => {
            if (currentTime == null || !groupStartTimes) return null;
            if (tCutoff <= 0 || tCutoff >= g.derived.t_end) return null;
            let i = 0;
            while (i + 1 < t.length && t[i + 1] < tCutoff) i++;
            if (i + 1 >= t.length) return [x[i], y[i]];
            const a = (tCutoff - t[i]) / Math.max(t[i + 1] - t[i], 1e-9);
            return [x[i] + a * (x[i + 1] - x[i]), y[i] + a * (y[i + 1] - y[i])];
          })();

          return (
            <g key={`grp-${gi}`}>
              {showContour && g.input_contour.length > 1 && (
                <polyline
                  points={g.input_contour.map(([x, y]) => `${x},${y}`).join(" ")}
                  fill="none"
                  stroke="#94a3b8"
                  strokeWidth={sz * 0.0025}
                  strokeLinecap="round"
                  opacity={0.5}
                />
              )}
              {showDomPoints && g.dominant_points.map(([x, y], i) => (
                <circle
                  key={`dom-${gi}-${i}`}
                  cx={x} cy={y}
                  r={sz * 0.004}
                  fill={color}
                  opacity={0.35}
                />
              ))}
              {showActionPlan && (
                <path
                  d={actionD}
                  fill="none"
                  stroke={color}
                  strokeWidth={sz * 0.003}
                  strokeDasharray={`${vw * 0.008},${vw * 0.006}`}
                  opacity={0.5}
                />
              )}

              {/* Non-spray plain trajectory line */}
              {!spray && showTrajectory && trajD && (
                <path
                  d={trajD} fill="none"
                  stroke={color}
                  strokeWidth={sz * 0.005}
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              )}

              {/* Pen tip indicator */}
              {tipPos && (
                <>
                  <circle cx={tipPos[0]} cy={tipPos[1]}
                          r={sz * 0.012}
                          fill={spray ? (sprayMonoColor ?? color) : color}
                          opacity={0.35} />
                  <circle cx={tipPos[0]} cy={tipPos[1]}
                          r={sz * 0.005} fill="white"
                          stroke={spray ? (sprayMonoColor ?? color) : color}
                          strokeWidth={sz * 0.002} />
                </>
              )}

              {showVirtualTargets && (
                <>
                  <circle
                    cx={g.action_plan.p0[0]} cy={g.action_plan.p0[1]}
                    r={sz * 0.008} fill="#16a34a"
                    stroke="white" strokeWidth={sz * 0.0015}
                  />
                  {g.action_plan.virtual_targets.map(([vx, vy], i) => {
                    const isSel = selected?.groupIdx === gi && selected?.strokeIdx === i;
                    return (
                      <circle
                        key={`vt-${gi}-${i}`}
                        cx={vx} cy={vy}
                        r={sz * 0.007}
                        fill={isSel ? "#dc2626" : color}
                        stroke="white" strokeWidth={sz * 0.0015}
                        style={{ cursor: interactive ? "grab" : "default" }}
                        onMouseDown={(e) => {
                          if (!interactive) return;
                          e.stopPropagation();
                          setDrag({ groupIdx: gi, targetIdx: i });
                          select(gi, i);
                        }}
                      />
                    );
                  })}
                </>
              )}
            </g>
          );
        })}
      </svg>
    </div>
  );
}
