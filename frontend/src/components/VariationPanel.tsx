import { useState } from "react";
import { useStudio } from "../store/studioStore";
import { TrajectoryCanvas } from "./TrajectoryCanvas";

export function VariationPanel() {
  const plan = useStudio((s) => s.plan);
  const variations = useStudio((s) => s.variations);
  const generate = useStudio((s) => s.generateVariations);
  const load = useStudio((s) => s.loadVariation);
  const status = useStudio((s) => s.status);

  const [n, setN] = useState(6);
  const [offset, setOffset] = useState(5);
  const [dtScale, setDtScale] = useState(0.15);
  const [phiScale, setPhiScale] = useState(0.1);
  const [xyScale, setXyScale] = useState(0.05);
  const [seed, setSeed] = useState(42);

  const run = () =>
    generate(
      n,
      {
        target_offset_px: offset,
        delta_t_scale: dtScale,
        phi_scale: phiScale,
        global_scale_xy: xyScale,
      },
      seed,
    );

  return (
    <div className="p-4 bg-white rounded-lg shadow-sm space-y-3 text-sm">
      <h2 className="font-semibold text-slate-700">バリエーション生成</h2>
      {!plan && <div className="text-slate-400 text-xs">先に解析してください</div>}
      {plan && (
        <>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <label className="flex items-center justify-between">
              n:
              <input
                type="number"
                min={1}
                max={16}
                value={n}
                onChange={(e) => setN(+e.target.value)}
                className="w-14 border rounded px-1 ml-1"
              />
            </label>
            <label className="flex items-center justify-between">
              seed:
              <input
                type="number"
                value={seed}
                onChange={(e) => setSeed(+e.target.value)}
                className="w-20 border rounded px-1 ml-1"
              />
            </label>
            <label>
              target±px: {offset.toFixed(1)}
              <input type="range" min={0} max={30} step={0.5}
                value={offset} onChange={(e) => setOffset(+e.target.value)} className="w-full" />
            </label>
            <label>
              Δt scale: {dtScale.toFixed(2)}
              <input type="range" min={0} max={0.5} step={0.01}
                value={dtScale} onChange={(e) => setDtScale(+e.target.value)} className="w-full" />
            </label>
            <label>
              φ scale: {phiScale.toFixed(2)}
              <input type="range" min={0} max={0.5} step={0.01}
                value={phiScale} onChange={(e) => setPhiScale(+e.target.value)} className="w-full" />
            </label>
            <label>
              XY scale: {xyScale.toFixed(2)}
              <input type="range" min={0} max={0.3} step={0.01}
                value={xyScale} onChange={(e) => setXyScale(+e.target.value)} className="w-full" />
            </label>
          </div>
          <button
            onClick={run}
            disabled={status === "loading"}
            className="w-full bg-indigo-600 text-white rounded py-1.5 text-sm disabled:opacity-40"
          >
            {status === "loading" ? "生成中..." : "生成"}
          </button>

          {variations.length > 0 && (
            <div className="grid grid-cols-3 gap-2 pt-2">
              {variations.map((v, i) => (
                <div
                  key={i}
                  className="aspect-square border rounded cursor-pointer hover:border-indigo-500 overflow-hidden"
                  onClick={() => load(i)}
                  title={`#${i} groups=${v.stroke_groups.length}`}
                >
                  <TrajectoryCanvas
                    plan={v}
                    showContour={false}
                    showDomPoints={false}
                    showActionPlan={false}
                    showVirtualTargets={false}
                    interactive={false}
                  />
                </div>
              ))}
            </div>
          )}
        </>
      )}
    </div>
  );
}
