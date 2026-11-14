import axios from "axios";
import type {
  GmlDocument, GmlListResponse, MpcResult, StrokeGroup, TrajectoryPlan, VariationRequest,
} from "./types";

const api = axios.create({ baseURL: "" });

export async function analyzeImage(
  file: File,
  opts: {
    model: "SL_STAR" | "GL_STAR";
    angle_thresh_deg?: number;
    dt_min?: number;
    dt_max?: number;
  },
): Promise<TrajectoryPlan> {
  const form = new FormData();
  form.append("image", file);
  form.append("model", opts.model);
  if (opts.angle_thresh_deg != null)
    form.append("angle_thresh_deg", String(opts.angle_thresh_deg));
  if (opts.dt_min != null) form.append("dt_min", String(opts.dt_min));
  if (opts.dt_max != null) form.append("dt_max", String(opts.dt_max));
  const r = await api.post<TrajectoryPlan>("/api/analyze", form);
  return r.data;
}

export interface ProgressEvent {
  step: string;
  progress: number;
  [key: string]: unknown;
}

/**
 * NDJSON ストリーミング版 analyze。解析中の進捗 (step/progress) を onProgress に逐次流し、
 * 完了時に最終 TrajectoryPlan を返す。
 */
export async function analyzeImageStream(
  file: File,
  opts: {
    model: "SL_STAR" | "GL_STAR";
  },
  onProgress: (ev: ProgressEvent) => void,
): Promise<TrajectoryPlan> {
  const form = new FormData();
  form.append("image", file);
  form.append("model", opts.model);
  return analyzeStreamInner(form, onProgress);
}

export async function analyzeStrokePointsStream(
  points: [number, number, number][],
  opts: { model: "SL_STAR" | "GL_STAR" },
  onProgress: (ev: ProgressEvent) => void,
): Promise<TrajectoryPlan> {
  const form = new FormData();
  form.append("stroke_points", JSON.stringify(points));
  form.append("model", opts.model);
  return analyzeStreamInner(form, onProgress);
}

async function analyzeStreamInner(
  form: FormData,
  onProgress: (ev: ProgressEvent) => void,
): Promise<TrajectoryPlan> {
  const res = await fetch("/api/analyze/stream", { method: "POST", body: form });
  if (!res.ok || !res.body) {
    throw new Error(`HTTP ${res.status}`);
  }
  const reader = res.body.getReader();
  const decoder = new TextDecoder();
  let buf = "";
  let plan: TrajectoryPlan | null = null;
  // Read until stream ends
  // Each line is one NDJSON object.
  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    buf += decoder.decode(value, { stream: true });
    let idx: number;
    while ((idx = buf.indexOf("\n")) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      let obj: ProgressEvent;
      try {
        obj = JSON.parse(line);
      } catch {
        continue;
      }
      if (obj.step === "done") {
        plan = (obj as unknown as { plan: TrajectoryPlan }).plan ?? null;
      } else if (obj.step === "error") {
        const detail = (obj as { detail?: string }).detail ?? "analysis failed";
        throw new Error(detail);
      } else {
        onProgress(obj);
      }
    }
  }
  // flush tail
  if (buf.trim()) {
    try {
      const tail = JSON.parse(buf.trim()) as ProgressEvent;
      if (tail.step === "done") {
        plan = (tail as unknown as { plan: TrajectoryPlan }).plan ?? plan;
      } else if (tail.step === "error") {
        throw new Error((tail as { detail?: string }).detail ?? "analysis failed");
      }
    } catch {
      /* ignore */
    }
  }
  if (!plan) throw new Error("サーバから最終プランが返りませんでした");
  return plan;
}

export async function analyzeStrokePoints(
  points: [number, number, number][],
  opts: { model: "SL_STAR" | "GL_STAR" },
): Promise<TrajectoryPlan> {
  const form = new FormData();
  form.append("stroke_points", JSON.stringify(points));
  form.append("model", opts.model);
  const r = await api.post<TrajectoryPlan>("/api/analyze", form);
  return r.data;
}

export async function refit(plan: TrajectoryPlan): Promise<TrajectoryPlan> {
  const r = await api.post<TrajectoryPlan>("/api/refit", {
    plan,
    max_iter: 10,
    tol_px: 1.0,
  });
  return r.data;
}

export async function variations(
  req: VariationRequest,
): Promise<TrajectoryPlan[]> {
  const r = await api.post<{ variations: TrajectoryPlan[] }>(
    "/api/variations",
    req,
  );
  return r.data.variations;
}

// ---- GML コーパス ----

export async function fetchGmlList(
  offset: number,
  limit: number,
  q?: string,
  opts?: { random?: boolean; seed?: number },
): Promise<GmlListResponse> {
  const params: Record<string, string | number | boolean> = { offset, limit };
  if (q) params.q = q;
  if (opts?.random) params.random = true;
  if (opts?.seed != null) params.seed = opts.seed;
  const r = await api.get<GmlListResponse>("/api/gml/list", { params });
  return r.data;
}

export async function fetchGmlDoc(id: number): Promise<GmlDocument> {
  const r = await api.get<GmlDocument>(`/api/gml/${id}`);
  return r.data;
}

export async function previewGmlFile(file: File): Promise<GmlDocument> {
  const form = new FormData();
  form.append("gml_file", file);
  const r = await api.post<GmlDocument>("/api/gml/preview", form);
  return r.data;
}

export type GmlSource = { id: number } | { file: File };

function appendGmlSource(form: FormData, source: GmlSource): void {
  if ("id" in source) {
    form.append("gml_id", String(source.id));
  } else {
    form.append("gml_file", source.file);
  }
}

export async function analyzeGml(
  source: GmlSource,
  selected: number[] | null,
  opts: { model: "SL_STAR" | "GL_STAR" },
): Promise<TrajectoryPlan> {
  const form = new FormData();
  appendGmlSource(form, source);
  if (selected && selected.length) {
    form.append("gml_stroke_indices", selected.join(","));
  }
  form.append("model", opts.model);
  const r = await api.post<TrajectoryPlan>("/api/analyze", form);
  return r.data;
}

export async function analyzeGmlStream(
  source: GmlSource,
  selected: number[] | null,
  opts: { model: "SL_STAR" | "GL_STAR" },
  onProgress: (ev: ProgressEvent) => void,
): Promise<TrajectoryPlan> {
  const form = new FormData();
  appendGmlSource(form, source);
  if (selected && selected.length) {
    form.append("gml_stroke_indices", selected.join(","));
  }
  form.append("model", opts.model);
  return analyzeStreamInner(form, onProgress);
}

// ---- MPC → TrajectoryPlan 変換 ----

function _arcLength(x: number[], y: number[]): number {
  let len = 0;
  for (let i = 1; i < x.length; i++) {
    len += Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  return len;
}

/**
 * MpcResult を TrajectoryPlan に変換し、アニメーション・ロボットアーム描画に利用できるようにする。
 *
 * origPointsPerStroke: 各ストロークの元 GML/描画点列 [x_px, y_px, t_s][]。
 * 省略可能（省略時は input_contour が空になるが動作に支障なし）。
 */
export function mpcResultToTrajectoryPlan(
  result: MpcResult,
  origPointsPerStroke?: [number, number, number][][],
): TrajectoryPlan {
  const [sw, sh] = result.screen_bounds;

  const strokeGroups: StrokeGroup[] = result.strokes.map((sr, gi) => {
    const n = sr.traj_x.length;
    const dt = n > 1 ? sr.duration_s / (n - 1) : sr.duration_s;
    const t = Array.from({ length: n }, (_, i) => i * dt);

    // 中心差分による速度（両端は前進/後退差分）
    const vx = new Array<number>(n).fill(0);
    const vy = new Array<number>(n).fill(0);
    for (let i = 1; i < n - 1; i++) {
      vx[i] = (sr.traj_x[i + 1] - sr.traj_x[i - 1]) / (2 * dt);
      vy[i] = (sr.traj_y[i + 1] - sr.traj_y[i - 1]) / (2 * dt);
    }
    if (n > 1) {
      vx[n - 1] = (sr.traj_x[n - 1] - sr.traj_x[n - 2]) / dt;
      vy[n - 1] = (sr.traj_y[n - 1] - sr.traj_y[n - 2]) / dt;
    }
    const speed = vx.map((vxi, i) => Math.hypot(vxi, vy[i]));
    const viaCenters: [number, number][] = sr.ellipses.map(e => [e.x, e.y]);
    const origPts = origPointsPerStroke?.[gi] ?? [];

    return {
      group_index: gi,
      action_plan: {
        p0: [sr.traj_x[0], sr.traj_y[0]] as [number, number],
        virtual_targets: viaCenters,
      },
      strokes: [{
        index: 0,
        delta_t: sr.duration_s,
        mu: 0,
        sigma: 0,
        phi: null,
        phi1: null,
        phi2: null,
      }],
      derived: {
        timeseries: { dt, t, x: sr.traj_x, y: sr.traj_y, vx, vy, speed },
        t_end: sr.duration_s,
        arc_length_px: _arcLength(sr.traj_x, sr.traj_y),
        critical_points: viaCenters,
      },
      input_contour: origPts.map(([x, y]) => [x, y] as [number, number]),
      dominant_points: viaCenters,
      quality: { reconstruction_mse_px: sr.reconstruction_error_px, n_iter_used: sr.n_via },
    } satisfies StrokeGroup;
  });

  const avgErr = result.strokes.length > 0
    ? result.strokes.reduce((s, r) => s + r.reconstruction_error_px, 0) / result.strokes.length
    : 0;

  return {
    schema_version: "2.0",
    model: "MPC_STAR",
    image_width: sw,
    image_height: sh,
    global_params: {
      mu: 0, sigma: 0, delta_t_min: 0, delta_t_max: 1,
      use_min_time_reparam: false, lambda_smooth: 0, seed: 0,
    },
    stroke_groups: strokeGroups,
    overall_quality: { reconstruction_mse_px: avgErr, n_iter_used: result.n_via },
  };
}

// ---- MPC ストローク推定 ----

export interface MpcOptions {
  n_via?: number;
  order?: number;
  dt?: number;
  auto_optimize?: boolean;
  tol_px?: number;
  min_via?: number;
  max_via?: number;
}

// ---- MPC NDJSON ストリーム共通読み取り ----

async function _mpcStreamInner(
  fetchPromise: Promise<Response>,
  onProgress: (ev: ProgressEvent) => void,
): Promise<MpcResult> {
  const res = await fetchPromise;
  if (!res.ok || !res.body) throw new Error(`HTTP ${res.status}`);
  const reader = res.body.getReader();
  const decoder = new TextDecoder();
  let buf = "";
  let result: MpcResult | null = null;

  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    buf += decoder.decode(value, { stream: true });
    let idx: number;
    while ((idx = buf.indexOf("\n")) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      let obj: any;
      try { obj = JSON.parse(line); } catch { continue; }
      if (obj.step === "done") {
        result = obj.result as MpcResult;
      } else if (obj.step === "error") {
        throw new Error(obj.detail ?? "MPC failed");
      } else {
        onProgress(obj as ProgressEvent);
      }
    }
  }
  if (!result) throw new Error("サーバから MPC 結果が返りませんでした");
  return result;
}

export async function fetchMpcStream(
  source: GmlSource,
  selected: number[] | null,
  opts: MpcOptions = {},
  onProgress: (ev: ProgressEvent) => void = () => {},
): Promise<MpcResult> {
  const form = new FormData();
  appendGmlSource(form, source);
  if (selected && selected.length) form.append("gml_stroke_indices", selected.join(","));
  if (opts.n_via        != null) form.append("n_via",         String(opts.n_via));
  if (opts.order        != null) form.append("order",         String(opts.order));
  if (opts.dt           != null) form.append("dt",            String(opts.dt));
  if (opts.auto_optimize != null) form.append("auto_optimize", String(opts.auto_optimize));
  if (opts.tol_px       != null) form.append("tol_px",        String(opts.tol_px));
  if (opts.min_via      != null) form.append("min_via",       String(opts.min_via));
  if (opts.max_via      != null) form.append("max_via",       String(opts.max_via));
  return _mpcStreamInner(fetch("/api/mpc/stream", { method: "POST", body: form }), onProgress);
}

export async function fetchMpcFromPointsStream(
  strokes: [number, number, number][][],
  screenW: number,
  screenH: number,
  opts: MpcOptions = {},
  onProgress: (ev: ProgressEvent) => void = () => {},
): Promise<MpcResult> {
  return _mpcStreamInner(
    fetch("/api/mpc/from-points/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        strokes, screen_w: screenW, screen_h: screenH,
        n_via: opts.n_via ?? 5, order: opts.order ?? 4, dt: opts.dt ?? 0.005,
        auto_optimize: opts.auto_optimize ?? true, tol_px: opts.tol_px ?? 3.0,
        min_via: opts.min_via ?? 3, max_via: opts.max_via ?? 15,
      }),
    }),
    onProgress,
  );
}

export async function fetchMpcResult(
  source: GmlSource,
  selected: number[] | null,
  opts: MpcOptions = {},
): Promise<MpcResult> {
  const form = new FormData();
  appendGmlSource(form, source);
  if (selected && selected.length) {
    form.append("gml_stroke_indices", selected.join(","));
  }
  if (opts.n_via        != null) form.append("n_via",         String(opts.n_via));
  if (opts.order        != null) form.append("order",         String(opts.order));
  if (opts.dt           != null) form.append("dt",            String(opts.dt));
  if (opts.auto_optimize != null) form.append("auto_optimize", String(opts.auto_optimize));
  if (opts.tol_px       != null) form.append("tol_px",        String(opts.tol_px));
  if (opts.min_via      != null) form.append("min_via",       String(opts.min_via));
  if (opts.max_via      != null) form.append("max_via",       String(opts.max_via));
  const r = await api.post<MpcResult>("/api/mpc", form);
  return r.data;
}

/**
 * 生の描画点列 (複数ストローク) から MPC を実行する。GML ファイル不要。
 * DrawPanel などで手描きストロークをそのまま MPC に渡す用途。
 *
 * strokes: 各ストロークの [x_px, y_px, t_s][] 配列の配列
 */
export async function fetchMpcFromPoints(
  strokes: [number, number, number][][],
  screenW: number,
  screenH: number,
  opts: MpcOptions = {},
): Promise<MpcResult> {
  const r = await api.post<MpcResult>("/api/mpc/from-points", {
    strokes,
    screen_w: screenW,
    screen_h: screenH,
    n_via:          opts.n_via         ?? 5,
    order:          opts.order         ?? 4,
    dt:             opts.dt            ?? 0.005,
    auto_optimize:  opts.auto_optimize ?? true,
    tol_px:         opts.tol_px        ?? 3.0,
    min_via:        opts.min_via       ?? 3,
    max_via:        opts.max_via       ?? 15,
  });
  return r.data;
}
