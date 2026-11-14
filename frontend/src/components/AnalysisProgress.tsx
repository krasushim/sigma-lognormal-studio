interface Props {
  progress: number;                // 0..1
  step: string | null;
  extra: Record<string, unknown>;
}

const ANALYZE_STEPS = [
  { id: "preprocess", label: "① 前処理 (CLAHE + リサイズ)",          range: [0.00, 0.08] as [number, number] },
  { id: "mask",       label: "② インクマスク生成",                     range: [0.08, 0.15] as [number, number] },
  { id: "contours",   label: "③ スケルトン化・輪郭抽出",               range: [0.15, 0.25] as [number, number] },
  { id: "segment",    label: "④ 優位点検出・弧フィッティング",          range: [0.25, 0.45] as [number, number] },
  { id: "diff_refit", label: "⑤ 微分可能最適化 (Adam)",                range: [0.45, 0.92] as [number, number] },
  { id: "finalize",   label: "⑥ 時間軸正規化・仕上げ",                 range: [0.92, 1.00] as [number, number] },
];

const MPC_STEPS = [
  { id: "mpc", label: "MPC ストローク推定 (ICP + chamfer 最適化)", range: [0.00, 1.00] as [number, number] },
];

function isMpcStep(step: string | null): boolean {
  return step === "mpc" || step === "mpc_init";
}

export function AnalysisProgress({ progress, step, extra }: Props) {
  const isMpc = isMpcStep(step);
  const STEPS = isMpc ? MPC_STEPS : ANALYZE_STEPS;
  const curIdx = STEPS.findIndex((s) => s.id === step);
  const pctOverall = Math.max(0, Math.min(100, progress * 100));

  // ステップ補足テキスト
  let stepDetail = "";
  if (isMpc && extra) {
    const stroke = (extra as any).stroke;
    const total  = (extra as any).total;
    const n_via  = (extra as any).n_via;
    const err    = (extra as any).error_px;
    if (stroke != null && total != null && total > 0) {
      stepDetail = `ストローク ${stroke}/${total}`;
      if (n_via != null) stepDetail += ` · n_via=${n_via}`;
      if (err   != null) stepDetail += ` · ${Number(err).toFixed(1)} px`;
    }
  } else if (step === "diff_refit" && extra) {
    const it = (extra as any).iter;
    const total = (extra as any).total;
    const loss = (extra as any).loss;
    if (it != null && total != null) {
      stepDetail = `iter ${it}/${total}` + (loss != null ? ` · L=${Number(loss).toFixed(4)}` : "");
    }
  } else if (step === "segment" && extra) {
    const g = (extra as any).group;
    const t = (extra as any).total;
    if (g != null && t != null) stepDetail = `${g}/${t} グループ処理中`;
  } else if (step === "contours" && extra) {
    const n = (extra as any).n_contours;
    if (n != null) stepDetail = `${n} 輪郭抽出`;
  }

  const title = isMpc ? "MPC 推定中…" : "解析中…";

  return (
    <div className="w-full max-w-lg px-6 py-8 bg-white rounded-xl shadow border border-slate-200 text-slate-700">
      <div className="text-sm font-semibold text-slate-800 mb-3">{title}</div>

      {/* 全体プログレスバー */}
      <div className="h-2 w-full bg-slate-200 rounded overflow-hidden mb-1">
        <div
          className="h-full bg-gradient-to-r from-indigo-500 to-purple-500 transition-[width] duration-150"
          style={{ width: `${pctOverall}%` }}
        />
      </div>
      <div className="text-xs text-slate-500 tabular-nums mb-4">
        {pctOverall.toFixed(0)}%
      </div>

      {/* ステップリスト */}
      <ul className="space-y-1.5 text-xs">
        {STEPS.map((s, i) => {
          const done = i < curIdx || progress >= s.range[1];
          const active = i === curIdx || (isMpc && step === "mpc");
          const icon = done ? "✔" : active ? "●" : "○";
          return (
            <li
              key={s.id}
              className={`flex items-center gap-2 ${
                done ? "text-emerald-600"
                  : active ? "text-indigo-700 font-semibold"
                  : "text-slate-400"
              }`}
            >
              <span className="w-4 text-center">{icon}</span>
              <span className="flex-1">{s.label}</span>
              {active && stepDetail && (
                <span className="text-xs text-indigo-600/70 font-mono">{stepDetail}</span>
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}
