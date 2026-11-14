import { useStudio } from "../store/studioStore";

export function ParamsPanel() {
  const plan = useStudio((s) => s.plan);
  const selected = useStudio((s) => s.selected);
  if (!plan) return null;

  const totalStrokes = plan.stroke_groups.reduce((s, g) => s + g.strokes.length, 0);
  const totalArc = plan.stroke_groups.reduce((s, g) => s + g.derived.arc_length_px, 0);
  const totalT = plan.stroke_groups.reduce((s, g) => s + g.derived.t_end, 0);

  const selectedGroup =
    selected != null ? plan.stroke_groups[selected.groupIdx] : null;
  const selectedStroke =
    selectedGroup != null && selected?.strokeIdx != null
      ? selectedGroup.strokes[selected.strokeIdx]
      : null;

  return (
    <div className="p-4 bg-white rounded-lg shadow-sm space-y-2 text-sm">
      <h2 className="font-semibold text-slate-700">パラメータ</h2>
      <div className="text-slate-600 space-y-0.5">
        <div>モデル: <span className="font-mono">{plan.model}</span></div>
        <div>ストローク群: <span className="font-mono">{plan.stroke_groups.length}</span></div>
        <div>合計 m: <span className="font-mono">{totalStrokes}</span></div>
        <div>合計 T_end: <span className="font-mono">{totalT.toFixed(2)} s</span></div>
        <div>合計 弧長: <span className="font-mono">{totalArc.toFixed(1)} px</span></div>
        <div>全体 MSE: <span className="font-mono">{plan.overall_quality.reconstruction_mse_px.toFixed(2)} px</span></div>
      </div>

      <hr className="my-2" />
      <h3 className="font-semibold text-slate-700">グローバル</h3>
      <div className="text-slate-600 space-y-0.5 font-mono text-xs">
        <div>μ = {plan.global_params.mu.toFixed(4)}</div>
        <div>σ = {plan.global_params.sigma.toFixed(4)}</div>
        <div>Δt_min = {plan.global_params.delta_t_min.toFixed(2)}</div>
        <div>Δt_max = {plan.global_params.delta_t_max.toFixed(2)}</div>
      </div>

      <hr className="my-2" />
      <h3 className="font-semibold text-slate-700">ストローク群</h3>
      <div className="max-h-32 overflow-y-auto text-xs space-y-0.5">
        {plan.stroke_groups.map((g, i) => (
          <div
            key={i}
            className={`flex justify-between px-1 py-0.5 rounded cursor-pointer hover:bg-slate-100 ${
              selected?.groupIdx === i ? "bg-indigo-100" : ""
            }`}
          >
            <span className="font-mono">g{i}</span>
            <span className="text-slate-600">m={g.strokes.length}</span>
            <span className="text-slate-600">{g.derived.arc_length_px.toFixed(0)}px</span>
            <span className="text-slate-600">{g.quality.reconstruction_mse_px.toFixed(2)}</span>
          </div>
        ))}
      </div>

      {selectedGroup && (
        <>
          <hr className="my-2" />
          <h3 className="font-semibold text-slate-700">
            選択: g{selectedGroup.group_index}
            {selectedStroke && ` / stroke #${selectedStroke.index}`}
          </h3>
          {selectedStroke && (
            <div className="font-mono text-xs text-slate-600 space-y-0.5">
              <div>Δt = {selectedStroke.delta_t.toFixed(3)}</div>
              <div>μ = {selectedStroke.mu.toFixed(4)}</div>
              <div>σ = {selectedStroke.sigma.toFixed(4)}</div>
              {selectedStroke.phi !== null && <div>φ = {selectedStroke.phi.toFixed(3)} rad</div>}
            </div>
          )}
        </>
      )}
    </div>
  );
}
