import { useRef, useState } from "react";
import { useStudio } from "../store/studioStore";

type Model = "SL_STAR" | "GL_STAR" | "MPC_STAR";

export function DrawPanel() {
  const [points, setPoints] = useState<[number, number, number][]>([]);
  const [isDrawing, setDrawing] = useState(false);
  const startT = useRef<number>(0);
  const svgRef = useRef<SVGSVGElement>(null);
  const [model, setModel] = useState<Model>("GL_STAR");
  const analyze = useStudio((s) => s.analyzeStrokePoints);
  const runMpcFromPoints = useStudio((s) => s.runMpcFromPoints);
  const status = useStudio((s) => s.status);

  const canvasSize = { w: 400, h: 300 };

  const xyFromEvent = (e: React.MouseEvent<SVGSVGElement>): [number, number] => {
    const rect = svgRef.current!.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  };

  const onDown = (e: React.MouseEvent<SVGSVGElement>) => {
    setDrawing(true);
    startT.current = performance.now();
    setPoints([[...xyFromEvent(e), 0]]);
  };
  const onMove = (e: React.MouseEvent<SVGSVGElement>) => {
    if (!isDrawing) return;
    const t = (performance.now() - startT.current) / 1000;
    const [x, y] = xyFromEvent(e);
    setPoints((p) => [...p, [x, y, t]]);
  };
  const onUp = () => setDrawing(false);

  const submit = () => {
    if (points.length < 4) return;
    if (model === "MPC_STAR") {
      runMpcFromPoints(points, canvasSize.w, canvasSize.h);
    } else {
      analyze(points, model);
    }
  };

  const pathD = points.length
    ? `M ${points.map((p) => `${p[0]},${p[1]}`).join(" L ")}`
    : "";

  return (
    <div className="p-4 bg-white rounded-lg shadow-sm space-y-3">
      <h2 className="font-semibold text-sm text-slate-700">
        キャンバスに描く (時刻付き)
      </h2>
      <svg
        ref={svgRef}
        width={canvasSize.w}
        height={canvasSize.h}
        className="bg-slate-50 rounded border border-slate-200 cursor-crosshair"
        onMouseDown={onDown}
        onMouseMove={onMove}
        onMouseUp={onUp}
        onMouseLeave={onUp}
      >
        {pathD && (
          <path
            d={pathD}
            stroke="#111"
            strokeWidth={3}
            fill="none"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        )}
      </svg>
      <div className="flex items-center gap-2 text-sm">
        <label>モデル:</label>
        <select
          className="border rounded px-2 py-1"
          value={model}
          onChange={(e) => setModel(e.target.value as any)}
        >
          <option value="SL_STAR">SL*</option>
          <option value="GL_STAR">GL*</option>
          <option value="MPC_STAR">MPC</option>
        </select>
        <span className="ml-auto text-slate-500">{points.length} 点</span>
      </div>
      <div className="flex gap-2">
        <button
          onClick={submit}
          disabled={points.length < 4 || status === "loading"}
          className="flex-1 bg-indigo-600 text-white rounded py-1.5 text-sm disabled:opacity-40"
        >
          {status === "loading" ? "解析中..." : "解析"}
        </button>
        <button
          onClick={() => setPoints([])}
          className="px-3 py-1.5 border rounded text-sm"
        >
          クリア
        </button>
      </div>
    </div>
  );
}
