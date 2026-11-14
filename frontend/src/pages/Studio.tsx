import { useEffect, useMemo, useState } from "react";
import type { TrajectoryPlan } from "../api/types";
import { UploadPanel } from "../components/UploadPanel";
import { DrawPanel } from "../components/DrawPanel";
import { GmlPanel } from "../components/GmlPanel";
import { TrajectoryCanvas } from "../components/TrajectoryCanvas";
import { VelocityChart } from "../components/VelocityChart";
import { ParamsPanel } from "../components/ParamsPanel";
import { VariationPanel } from "../components/VariationPanel";
import { AnimationControls } from "../components/AnimationControls";
import { AnalysisProgress } from "../components/AnalysisProgress";
import { Robot3DViewV2 } from "../components/Robot3DViewV2";
import { useStudio } from "../store/studioStore";

const PEN_UP_GAP_S = 0.3;

type Stage = "final" | "initial" | "mask" | "skeleton" | "input";

/**
 * Auto-derive spray max thickness from a plan.
 *
 * The spray width formula is ``w = w_min + (w_max - w_min) * exp(-speed / μ)``
 * with μ auto-normalised to group mean speed (see spray.ts). So ``w_max`` only
 * needs to match the canvas scale. We take ~3% of the longer image side and
 * give a mild bump for high-speed data so that slow endpoints still look like
 * real spray on tags with speeds in the thousands of px/s.
 */
function autoSprayWMax(plan: TrajectoryPlan | null): number {
  if (!plan) return 10;
  const imgDim = Math.max(plan.image_width || 0, plan.image_height || 0, 1);
  let speedMax = 0;
  for (const g of plan.stroke_groups) {
    for (const s of g.derived.timeseries.speed) {
      if (s > speedMax) speedMax = s;
    }
  }
  // 画像寸法の 3% を基準に、速度スケール (基準 300 px/s 超) で log 的に加算。
  const speedBoost = 1.0 + 0.25 * Math.log10(Math.max(speedMax / 300, 1));
  const base = imgDim * 0.03 * speedBoost;
  return Math.round(Math.max(8, Math.min(base, 60)));
}

export function Studio() {
  const plan = useStudio((s) => s.plan);
  const status = useStudio((s) => s.status);
  const errorMsg = useStudio((s) => s.errorMsg);
  const inputImageUrl = useStudio((s) => s.inputImageUrl);
  const progress = useStudio((s) => s.progress);
  const progressStep = useStudio((s) => s.progressStep);
  const progressExtra = useStudio((s) => s.progressExtra);
  const [mode, setMode] = useState<"upload" | "draw" | "gml">("upload");
  const [stage, setStage] = useState<Stage>("final");
  const [viewMode, setViewMode] = useState<"2d" | "3d">("2d");
  const [layers, setLayers] = useState({
    contour: true, domPoints: false, actionPlan: false, trajectory: true,
    targets: false, spray: true,
  });

  // Spray params (UI で調整可)
  const [sprayAlpha, setSprayAlpha] = useState(0.6);
  const [sprayWMax, setSprayWMax] = useState(10);
  // 新しい plan が来たら w_max を自動再計算 (速度プロファイル最大値と画像寸法から)
  useEffect(() => {
    if (!plan) return;
    setSprayWMax(autoSprayWMax(plan));
  }, [plan]);

  // Animation state
  const [currentTime, setCurrentTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1.0);

  // 表示するストローク: final / initial を切替
  const displayPlan = useMemo(() => {
    if (!plan) return null;
    if (stage === "initial" && plan.initial_stroke_groups && plan.initial_stroke_groups.length > 0) {
      return { ...plan, stroke_groups: plan.initial_stroke_groups };
    }
    return plan;
  }, [plan, stage]);

  // 背景画像の選択 (mask / skeleton / input のいずれか)
  const backgroundUrl = useMemo(() => {
    if (!plan) return null;
    if (stage === "mask" && plan.mask_png_b64) {
      return `data:image/png;base64,${plan.mask_png_b64}`;
    }
    if (stage === "skeleton" && plan.skeleton_png_b64) {
      return `data:image/png;base64,${plan.skeleton_png_b64}`;
    }
    if (stage === "input") {
      return inputImageUrl;
    }
    return null;
  }, [plan, stage, inputImageUrl]);

  // 軌跡を描画するかどうか (入力/マスク/スケルトンだけを見たいときは非表示)
  const showStrokesForStage = stage === "final" || stage === "initial";

  const { totalDuration, groupStartTimes } = useMemo(() => {
    if (!displayPlan) return { totalDuration: 0, groupStartTimes: [] as number[] };
    const starts: number[] = [];
    let acc = 0;
    for (const g of displayPlan.stroke_groups) {
      starts.push(acc);
      acc += g.derived.t_end + PEN_UP_GAP_S;
    }
    const total = Math.max(0, acc - PEN_UP_GAP_S);
    return { totalDuration: total, groupStartTimes: starts };
  }, [displayPlan]);

  // Reset animation when plan changes
  useEffect(() => {
    setCurrentTime(totalDuration);
    setPlaying(false);
  }, [totalDuration]);

  return (
    <div className="h-full flex flex-col">
      <header className="bg-slate-900 text-white px-4 py-3 flex items-center gap-3">
        <h1 className="font-bold">Sigma-Lognormal Studio</h1>
        <span className="text-xs text-slate-400">
          Berio 2015 + 2025 · グラフィティタグ軌跡の解析と合成
        </span>
      </header>

      <div className="flex-1 grid grid-cols-12 gap-3 p-3 overflow-hidden">
        {/* Left panel */}
        <aside className="col-span-3 space-y-3 overflow-y-auto">
          <div className="flex gap-1 bg-white rounded p-1 text-xs">
            <button
              className={`flex-1 py-1 rounded ${mode === "upload" ? "bg-slate-900 text-white" : ""}`}
              onClick={() => setMode("upload")}
            >
              画像
            </button>
            <button
              className={`flex-1 py-1 rounded ${mode === "draw" ? "bg-slate-900 text-white" : ""}`}
              onClick={() => setMode("draw")}
            >
              描画
            </button>
            <button
              className={`flex-1 py-1 rounded ${mode === "gml" ? "bg-slate-900 text-white" : ""}`}
              onClick={() => setMode("gml")}
            >
              GML
            </button>
          </div>
          {mode === "upload" && <UploadPanel />}
          {mode === "draw" && <DrawPanel />}
          {mode === "gml" && <GmlPanel />}
          <ParamsPanel />
        </aside>

        {/* Center: canvas + animation + velocity */}
        <main className="col-span-6 flex flex-col gap-3 min-h-0">
          <div className="flex-1 bg-white rounded-lg shadow-sm flex flex-col overflow-hidden">
            {/* 2D / 3D ビュー切替 */}
            {plan && (
              <div className="flex gap-2 items-center p-2 text-xs bg-slate-200 border-b">
                <span className="font-semibold text-slate-700 mr-2">表示モード:</span>
                {([
                  ["2d", "2D 軌跡"],
                  ["3d", "🤖 3D ロボット"],
                ] as const).map(([k, label]) => (
                  <button
                    key={k}
                    onClick={() => setViewMode(k)}
                    className={`px-3 py-1 rounded font-medium ${
                      viewMode === k
                        ? "bg-indigo-600 text-white"
                        : "bg-white text-slate-700 hover:bg-indigo-100"
                    }`}
                  >
                    {label}
                  </button>
                ))}
              </div>
            )}
            {/* ステージ選択 (2D のみ) */}
            {plan && viewMode === "2d" && (
              <div className="flex gap-1 p-2 text-xs bg-slate-100 border-b">
                {(() => {
                  const kind = plan.source?.kind;
                  const isRaster = kind === "image";
                  const allStages = [
                    ["input", "① 入力", true],
                    ["mask", "② マスク", isRaster],
                    ["skeleton", "③ スケルトン", isRaster],
                    ["initial", "④ 解析初期推定", true],
                    ["final", "⑤ 最終 (diff-refit)", true],
                  ] as const;
                  return allStages
                    .filter(([, , show]) => show)
                    .map(([k, label]) => (
                      <button
                        key={k}
                        onClick={() => setStage(k)}
                        className={`px-3 py-1 rounded font-medium ${
                          stage === k
                            ? "bg-slate-900 text-white"
                            : "bg-white text-slate-700 hover:bg-slate-200"
                        }`}
                      >
                        {label}
                      </button>
                    ));
                })()}
              </div>
            )}
            {viewMode === "2d" && (
            <div className="flex items-center gap-3 text-xs text-slate-600 p-2 flex-wrap border-b">
              {(
                [
                  ["contour", "輪郭"],
                  ["domPoints", "優位点"],
                  ["actionPlan", "Action plan"],
                  ["trajectory", "軌跡"],
                  ["targets", "仮想ターゲット"],
                  ["spray", "🎨 スプレー"],
                ] as const
              ).map(([k, label]) => (
                <label key={k} className="flex items-center gap-1 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={layers[k]}
                    onChange={(e) => setLayers((L) => ({ ...L, [k]: e.target.checked }))}
                  />
                  {label}
                </label>
              ))}
              {layers.spray && (
                <div className="flex items-center gap-4 ml-auto">
                  <label className="flex items-center gap-2">
                    <span>透明度:</span>
                    <input
                      type="range"
                      min={0.05}
                      max={1.0}
                      step={0.01}
                      value={sprayAlpha}
                      onChange={(e) => setSprayAlpha(Number(e.target.value))}
                      className="w-28"
                    />
                    <span className="font-mono w-10 text-right">{sprayAlpha.toFixed(2)}</span>
                  </label>
                  <label className="flex items-center gap-2">
                    <span>最大太さ:</span>
                    <input
                      type="range"
                      min={3}
                      max={80}
                      step={1}
                      value={sprayWMax}
                      onChange={(e) => setSprayWMax(Number(e.target.value))}
                      className="w-28"
                    />
                    <span className="font-mono w-10 text-right">{sprayWMax}px</span>
                  </label>
                </div>
              )}
            </div>
            )}
            <div className="flex-1 min-h-0 p-3">
              {plan && viewMode === "3d" ? (
                <Robot3DViewV2
                  plan={plan}
                  currentTime={currentTime}
                  totalDuration={totalDuration}
                />
              ) : displayPlan ? (
                <TrajectoryCanvas
                  plan={displayPlan}
                  showContour={showStrokesForStage && layers.contour}
                  showDomPoints={showStrokesForStage && layers.domPoints}
                  showActionPlan={showStrokesForStage && layers.actionPlan}
                  showTrajectory={showStrokesForStage && layers.trajectory}
                  showVirtualTargets={showStrokesForStage && layers.targets}
                  spray={showStrokesForStage && layers.spray}
                  sprayParams={{ alpha: sprayAlpha, w_max: sprayWMax }}
                  currentTime={showStrokesForStage ? currentTime : undefined}
                  groupStartTimes={showStrokesForStage ? groupStartTimes : undefined}
                  backgroundImageUrl={backgroundUrl}
                  uid="main"
                  interactive={stage === "final"}
                />
              ) : (
                <div className="h-full flex items-center justify-center text-slate-400 text-sm">
                  {status === "loading" ? (
                    <AnalysisProgress
                      progress={progress}
                      step={progressStep}
                      extra={progressExtra}
                    />
                  ) : errorMsg ? (
                    <span className="text-red-600 text-center">{errorMsg}</span>
                  ) : (
                    "画像または描画から解析を開始してください"
                  )}
                </div>
              )}
            </div>
            {plan && totalDuration > 0 && (
              <AnimationControls
                totalDuration={totalDuration}
                currentTime={currentTime}
                setCurrentTime={setCurrentTime}
                playing={playing}
                setPlaying={setPlaying}
                speed={speed}
                setSpeed={setSpeed}
              />
            )}
          </div>
          {plan && <VelocityChart plan={plan} />}
        </main>

        {/* Right: variation panel */}
        <aside className="col-span-3 space-y-3 overflow-y-auto">
          <VariationPanel />
        </aside>
      </div>
    </div>
  );
}
