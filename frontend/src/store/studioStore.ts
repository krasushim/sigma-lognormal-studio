import { create } from "zustand";
import type {
  GmlDocument, MpcResult, TrajectoryPlan, VariationPerturbations,
} from "../api/types";
import * as api from "../api/client";

type Status = "idle" | "loading" | "error";

interface StudioState {
  plan: TrajectoryPlan | null;
  status: Status;
  errorMsg: string;
  selected: { groupIdx: number; strokeIdx: number | null } | null;
  variations: TrajectoryPlan[];
  /** 解析ステージ表示のためのソース画像 URL */
  inputImageUrl: string | null;
  /** 解析中の進捗 (0..1) */
  progress: number;
  /** 現在の解析ステップ識別子 */
  progressStep: string | null;
  /** 追加情報 (iter/total, loss, etc.) */
  progressExtra: Record<string, unknown>;
  /** GML プレビュー (選択中のみ設定) */
  gmlDoc: GmlDocument | null;
  /** 解析に使うストローク index の subset (null=全選択) */
  gmlSelected: number[] | null;
  /** MPC ストローク推定結果 */
  mpcResult: MpcResult | null;
  /** MPC 解析中フラグ */
  mpcLoading: boolean;

  analyzeImage: (file: File, model: "SL_STAR" | "GL_STAR") => Promise<void>;
  analyzeStrokePoints: (
    points: [number, number, number][],
    model: "SL_STAR" | "GL_STAR",
  ) => Promise<void>;
  analyzeGmlId: (
    id: number,
    selected: number[] | null,
    model: "SL_STAR" | "GL_STAR",
  ) => Promise<void>;
  analyzeGmlFile: (
    file: File,
    selected: number[] | null,
    model: "SL_STAR" | "GL_STAR",
  ) => Promise<void>;
  setGmlDoc: (doc: GmlDocument | null) => void;
  setGmlSelected: (selected: number[] | null) => void;
  setPlan: (plan: TrajectoryPlan) => void;
  runMpc: (
    source: api.GmlSource,
    selected: number[] | null,
    opts?: api.MpcOptions,
  ) => Promise<void>;
  /** DrawPanel などで手描きした生点列を MPC で解析し、plan をセットする */
  runMpcFromPoints: (
    points: [number, number, number][],
    screenW: number,
    screenH: number,
    opts?: api.MpcOptions,
  ) => Promise<void>;
  clearMpc: () => void;
  selectStroke: (groupIdx: number, strokeIdx: number | null) => void;
  updateTargetLocal: (
    groupIdx: number,
    targetIdx: number,
    xy: [number, number],
  ) => void;
  refitCurrent: () => Promise<void>;
  generateVariations: (
    n: number,
    perturb: VariationPerturbations,
    seed: number,
  ) => Promise<void>;
  loadVariation: (i: number) => void;
}

export const useStudio = create<StudioState>((set, get) => ({
  plan: null,
  status: "idle",
  errorMsg: "",
  selected: null,
  variations: [],
  inputImageUrl: null,
  progress: 0,
  progressStep: null,
  progressExtra: {},
  gmlDoc: null,
  gmlSelected: null,
  mpcResult: null,
  mpcLoading: false,

  async runMpc(source, selected, opts = {}) {
    set({
      mpcLoading: true, mpcResult: null,
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: null,
      progress: 0.01, progressStep: "mpc", progressExtra: { stroke: 0, total: 0 },
    });
    try {
      const result = await api.fetchMpcStream(source, selected, opts, (ev) => {
        set({
          progress: Number(ev.progress) || 0,
          progressStep: String(ev.step),
          progressExtra: { ...ev },
        });
      });
      const { gmlDoc } = get();
      const origPerStroke = result.strokes.map(sr => {
        const gs = gmlDoc?.strokes.find(s => s.index === sr.stroke_index);
        return gs?.points ?? [];
      });
      const plan = api.mpcResultToTrajectoryPlan(result, origPerStroke);
      set({
        mpcResult: result, mpcLoading: false,
        plan, status: "idle", progress: 1, progressStep: "done",
      });
    } catch (e: any) {
      set({
        mpcLoading: false, status: "error",
        errorMsg: (e as any)?.message ?? String(e),
      });
      throw e;
    }
  },

  async runMpcFromPoints(points, screenW, screenH, opts = {}) {
    set({
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: null,
      progress: 0.01, progressStep: "mpc", progressExtra: { stroke: 0, total: 1 },
    });
    try {
      const result = await api.fetchMpcFromPointsStream(
        [points], screenW, screenH, opts,
        (ev) => {
          set({
            progress: Number(ev.progress) || 0,
            progressStep: String(ev.step),
            progressExtra: { ...ev },
          });
        },
      );
      const plan = api.mpcResultToTrajectoryPlan(result, [points]);
      set({ plan, mpcResult: result, status: "idle", progress: 1, progressStep: "done" });
    } catch (e: any) {
      set({
        status: "error",
        errorMsg: (e as any)?.message ?? String(e),
      });
    }
  },

  clearMpc() {
    set({ mpcResult: null });
  },

  async analyzeImage(file, model) {
    const url = URL.createObjectURL(file);
    set({
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: url,
      progress: 0, progressStep: "preprocess", progressExtra: {},
    });
    try {
      const plan = await api.analyzeImageStream(file, { model }, (ev) => {
        set({
          progress: Number(ev.progress) || 0,
          progressStep: String(ev.step),
          progressExtra: { ...ev },
        });
      });
      set({ plan, status: "idle", progress: 1, progressStep: "done" });
    } catch (e: any) {
      set({
        status: "error",
        errorMsg: e?.response?.data?.detail ?? e?.message ?? String(e),
        progress: 0, progressStep: null,
      });
    }
  },

  async analyzeStrokePoints(points, model) {
    set({
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: null,
      progress: 0, progressStep: "preprocess", progressExtra: {},
    });
    try {
      const plan = await api.analyzeStrokePointsStream(points, { model }, (ev) => {
        set({
          progress: Number(ev.progress) || 0,
          progressStep: String(ev.step),
          progressExtra: { ...ev },
        });
      });
      set({ plan, status: "idle", progress: 1, progressStep: "done" });
    } catch (e: any) {
      set({
        status: "error",
        errorMsg: e?.response?.data?.detail ?? e?.message ?? String(e),
        progress: 0, progressStep: null,
      });
    }
  },

  async analyzeGmlId(id, selected, model) {
    set({
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: null,
      progress: 0, progressStep: "preprocess", progressExtra: {},
    });
    try {
      const plan = await api.analyzeGmlStream(
        { id }, selected, { model }, (ev) => {
          set({
            progress: Number(ev.progress) || 0,
            progressStep: String(ev.step),
            progressExtra: { ...ev },
          });
        },
      );
      set({ plan, status: "idle", progress: 1, progressStep: "done" });
    } catch (e: any) {
      set({
        status: "error",
        errorMsg: e?.response?.data?.detail ?? e?.message ?? String(e),
        progress: 0, progressStep: null,
      });
    }
  },

  async analyzeGmlFile(file, selected, model) {
    set({
      status: "loading", errorMsg: "", plan: null,
      variations: [], selected: null, inputImageUrl: null,
      progress: 0, progressStep: "preprocess", progressExtra: {},
    });
    try {
      const plan = await api.analyzeGmlStream(
        { file }, selected, { model }, (ev) => {
          set({
            progress: Number(ev.progress) || 0,
            progressStep: String(ev.step),
            progressExtra: { ...ev },
          });
        },
      );
      set({ plan, status: "idle", progress: 1, progressStep: "done" });
    } catch (e: any) {
      set({
        status: "error",
        errorMsg: e?.response?.data?.detail ?? e?.message ?? String(e),
        progress: 0, progressStep: null,
      });
    }
  },

  setGmlDoc(doc) {
    const selected = doc ? doc.strokes.map((s) => s.index) : null;
    set({ gmlDoc: doc, gmlSelected: selected });
  },

  setGmlSelected(selected) { set({ gmlSelected: selected }); },

  setPlan(plan) { set({ plan }); },

  selectStroke(groupIdx, strokeIdx) { set({ selected: { groupIdx, strokeIdx } }); },

  updateTargetLocal(groupIdx, targetIdx, xy) {
    const { plan } = get();
    if (!plan) return;
    const groups = plan.stroke_groups.map((g, gi) => {
      if (gi !== groupIdx) return g;
      const vt = g.action_plan.virtual_targets.map(
        (p, k) => (k === targetIdx ? xy : p) as [number, number],
      );
      const p0 = targetIdx === -1 ? xy : g.action_plan.p0;
      return { ...g, action_plan: { p0, virtual_targets: vt } };
    });
    set({ plan: { ...plan, stroke_groups: groups } });
  },

  async refitCurrent() {
    const { plan } = get();
    if (!plan) return;
    set({ status: "loading", errorMsg: "" });
    try {
      const next = await api.refit(plan);
      set({ plan: next, status: "idle" });
    } catch (e: any) {
      set({ status: "error", errorMsg: e?.response?.data?.detail ?? String(e) });
    }
  },

  async generateVariations(n, perturb, seed) {
    const { plan } = get();
    if (!plan) return;
    set({ status: "loading", errorMsg: "" });
    try {
      const vs = await api.variations({
        plan, n_variations: n, perturbations: perturb, seed,
      });
      set({ variations: vs, status: "idle" });
    } catch (e: any) {
      set({ status: "error", errorMsg: e?.response?.data?.detail ?? String(e) });
    }
  },

  loadVariation(i) {
    const { variations } = get();
    if (!variations[i]) return;
    set({ plan: variations[i] });
  },
}));
