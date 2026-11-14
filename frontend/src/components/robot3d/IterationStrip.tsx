import type { IterationFrame } from "../../math/ikCacheV2";


interface Props {
  iterations: IterationFrame[];
  selected: number;
  onSelect: (i: number) => void;
}


/** 反復ステップを画面下部に並べるサムネ列 (RMSE 推移) */
export function IterationStrip({ iterations, selected, onSelect }: Props) {
  if (iterations.length === 0) return null;
  return (
    <div className="flex gap-1 items-center px-3 py-2 bg-slate-100 border-b text-xs">
      <span className="text-slate-700 font-semibold mr-2">反復:</span>
      {iterations.map((f) => {
        const isActive = f.iteration === selected;
        const rmseMm = (f.trackingRMSE * 1000).toFixed(1);
        return (
          <button
            key={f.iteration}
            onClick={() => onSelect(f.iteration)}
            className={`px-2 py-1 rounded font-mono text-[11px] border transition ${
              isActive
                ? "bg-indigo-600 text-white border-indigo-700"
                : "bg-white text-slate-700 border-slate-300 hover:bg-indigo-50"
            }`}
            title={`iter ${f.iteration} · tracking RMSE = ${rmseMm} mm`}
          >
            #{f.iteration} · {rmseMm}mm
          </button>
        );
      })}
    </div>
  );
}
