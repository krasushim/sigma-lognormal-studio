export interface GlobalParams {
  mu: number;
  sigma: number;
  delta_t_min: number;
  delta_t_max: number;
  use_min_time_reparam: boolean;
  lambda_smooth: number;
  seed: number;
}

export interface ActionPlan {
  p0: [number, number];
  virtual_targets: [number, number][];
}

export interface StrokeParams {
  index: number;
  delta_t: number;
  mu: number;
  sigma: number;
  phi: number | null;
  phi1: number | null;
  phi2: number | null;
}

export interface TimeSeries {
  dt: number;
  t: number[];
  x: number[];
  y: number[];
  vx: number[];
  vy: number[];
  speed: number[];
}

export interface DerivedData {
  timeseries: TimeSeries;
  t_end: number;
  arc_length_px: number;
  critical_points: [number, number][];
}

export interface Quality {
  reconstruction_mse_px: number;
  n_iter_used: number;
}

export interface StrokeGroup {
  group_index: number;
  action_plan: ActionPlan;
  strokes: StrokeParams[];
  derived: DerivedData;
  input_contour: [number, number][];
  dominant_points: [number, number][];
  quality: Quality;
}

export interface SourceInfo {
  kind: "image" | "drawing" | "gml";
  gml_id?: number | null;
  gml_filename?: string | null;
  gml_client?: string | null;
  gml_screen_bounds?: [number, number] | null;
  gml_stroke_count?: number | null;
  gml_selected_indices?: number[] | null;
}

export interface TrajectoryPlan {
  schema_version: string;
  model: "SL_STAR" | "GL_STAR" | "MPC_STAR";
  image_width: number;
  image_height: number;
  global_params: GlobalParams;
  stroke_groups: StrokeGroup[];
  overall_quality: Quality;
  // 解析過程の可視化用 (stages)
  mask_png_b64?: string | null;
  skeleton_png_b64?: string | null;
  initial_stroke_groups?: StrokeGroup[];
  // 入力ソース (v2.1)
  source?: SourceInfo | null;
}

// ---- GML コーパス関連 ----
export interface GmlStroke {
  index: number;
  n_points: number;
  duration_s: number;
  points: [number, number, number][];  // [x_px, y_px, t_s]
}

export interface GmlDocument {
  gml_id: number | null;
  filename: string;
  spec: string | null;
  client_name: string | null;
  screen_bounds: [number, number];
  strokes: GmlStroke[];
}

export interface GmlIndexEntry {
  gml_id: number;
  filename: string;
  client_name: string | null;
  n_strokes: number;
  total_duration_s: number;
  screen_bounds: [number, number];
}

export interface GmlListResponse {
  total: number;
  offset: number;
  limit: number;
  items: GmlIndexEntry[];
}

export interface VariationPerturbations {
  target_offset_px: number;
  delta_t_scale: number;
  phi_scale: number;
  global_scale_xy: number;
}

export interface VariationRequest {
  plan: TrajectoryPlan;
  n_variations: number;
  perturbations: VariationPerturbations;
  seed: number;
}

// ---- MPC ストローク推定 ----

export interface MpcEllipse {
  x: number;
  y: number;
  rx: number;       // semi-axis along major direction (px)
  ry: number;       // semi-axis along minor direction (px)
  angle_deg: number;
}

export interface MpcStrokeResult {
  stroke_index: number;
  n_via: number;
  order: number;
  ellipses: MpcEllipse[];
  traj_x: number[];
  traj_y: number[];
  duration_s: number;
  reconstruction_error_px: number;
  converged: boolean;
  error_history: number[];
  n_via_start: number;
}

export interface MpcResult {
  gml_id: number | null;
  screen_bounds: [number, number];
  strokes: MpcStrokeResult[];
  n_via: number;
  order: number;
  auto_optimize: boolean;
  tol_px: number;
}
