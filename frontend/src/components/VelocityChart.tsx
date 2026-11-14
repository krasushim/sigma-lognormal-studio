import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer,
} from "recharts";
import type { TrajectoryPlan } from "../api/types";

const COLORS = [
  "#2563eb", "#dc2626", "#16a34a", "#ea580c",
  "#9333ea", "#0891b2", "#ca8a04", "#db2777",
  "#65a30d", "#7c3aed", "#0d9488", "#e11d48",
];

interface Props { plan: TrajectoryPlan }

export function VelocityChart({ plan }: Props) {
  // 各 group の時系列を t-shift して連結 (group 間に小さな gap)
  const series: { name: string; data: { t: number; speed: number }[] }[] = [];
  let offset = 0;
  const gap = 0.3;
  let totalT = 0;
  for (let gi = 0; gi < plan.stroke_groups.length; gi++) {
    const g = plan.stroke_groups[gi];
    const { t, speed } = g.derived.timeseries;
    const data = t.map((ti, i) => ({ t: ti + offset, speed: speed[i] }));
    series.push({ name: `g${gi}`, data });
    offset += g.derived.t_end + gap;
    totalT = Math.max(totalT, offset);
  }

  return (
    <div className="h-48 bg-white rounded p-2">
      <div className="text-xs text-slate-500 pl-1">
        速度プロファイル |v(t)| [px/s] · 全 {plan.stroke_groups.length} グループ
      </div>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart margin={{ top: 5, right: 10, bottom: 18, left: 30 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
          <XAxis
            type="number"
            dataKey="t"
            domain={[0, totalT || 1]}
            tick={{ fontSize: 10 }}
            label={{ value: "t [s]", position: "insideBottom", offset: -8, fontSize: 11 }}
          />
          <YAxis tick={{ fontSize: 10 }} />
          <Tooltip
            formatter={(v: any) => Number(v).toFixed(1)}
            labelFormatter={(v: any) => `t=${Number(v).toFixed(2)}s`}
          />
          {series.map((s, i) => (
            <Line
              key={s.name}
              data={s.data}
              type="monotone"
              dataKey="speed"
              stroke={COLORS[i % COLORS.length]}
              strokeWidth={1.4}
              dot={false}
              isAnimationActive={false}
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
