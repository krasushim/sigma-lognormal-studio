"""
POST /api/analyze       — 同期エンドポイント (最終 TrajectoryPlan を返す)
POST /api/analyze/stream — NDJSON ストリーミング (解析中の進捗イベント + 完了時のプラン)
"""
from __future__ import annotations

import asyncio
import base64
import copy
import json
import logging
import math
from typing import Any, Callable, Optional

import cv2
import numpy as np
from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from fastapi.responses import StreamingResponse

from sigma_lognormal.contour import (
    extract_all_contours, resample_arclen, smooth_gaussian,
)
from sigma_lognormal.segmentation import find_dominant_points
from sigma_lognormal.arc_fit import fit_phi_pair, find_inflection_local
from sigma_lognormal.sharpness import compute_sharpness, estimate_delta_t
from sigma_lognormal.fitting import build_initial_sl_plan, iterative_target_refinement
from sigma_lognormal.sl_model import (
    max_delta_t_for_overlap, rescale_params_in_time, evaluate_trajectory,
)
from sigma_lognormal.diff_refit import differentiable_refit

from app.api.convert import GroupResult, results_to_plan
from app.gml.corpus import get_corpus
from app.gml.parser import GmlParseError, parse_gml
from app.gml.to_contours import (
    parse_indices_csv, rasterize_strokes, strokes_to_contours,
)
from app.schemas.trajectory import (
    GlobalParams, SourceInfo, TrajectoryPlan,
)

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["analyze"])


# ============================================================================
# プログレスステップ定義
# ============================================================================

ProgressCb = Optional[Callable[[str, float, dict], None]]


# ============================================================================
# 1 輪郭の解析 (SL* 初期プラン + 反復ターゲット補正)
# ============================================================================

def _analyze_single_contour(
    contour: np.ndarray,
    *,
    model: str,
    angle_thresh_deg: float,
    dt_min: float,
    dt_max: float,
    include_inflections: bool,
    max_refit_iter: int,
    mu_global: float,
    sigma_global: float,
    min_arc_spacing_px: float,
    max_dominant_points: int,
    nonzero_speed_safety: float,
) -> Optional[GroupResult]:
    if len(contour) < 8:
        return None

    dt_cap = max_delta_t_for_overlap(
        mu_global, sigma_global, safety_factor=nonzero_speed_safety,
    )
    dt_max_eff = min(dt_max, dt_cap)
    dt_min_eff = min(dt_min, dt_max_eff * 0.7)

    use_inflections = include_inflections and model == "SL_STAR"
    arc_len = float(np.sum(np.linalg.norm(np.diff(contour, axis=0), axis=1)))
    if min_arc_spacing_px <= 0:
        min_arc_spacing_px = max(arc_len / 25.0, 12.0)
    dom_idx = find_dominant_points(
        contour,
        angle_thresh_deg=angle_thresh_deg,
        include_inflections=use_inflections,
        min_arc_spacing_px=min_arc_spacing_px,
        max_dominant_points=max_dominant_points,
    )
    if len(dom_idx) < 2:
        return None

    phi_pairs: list[tuple[float, float]] = []
    for a, b in zip(dom_idx[:-1], dom_idx[1:]):
        seg = contour[a: b + 1]
        if model == "GL_STAR":
            infl_local = find_inflection_local(seg)
            phi_pairs.append(fit_phi_pair(seg, inflection_local_idx=infl_local))
        else:
            phi_pairs.append(fit_phi_pair(seg, inflection_local_idx=None))

    segment_lengths = [dom_idx[k + 1] - dom_idx[k] for k in range(len(dom_idx) - 1)]
    mean_len = max(int(np.mean(segment_lengths)) // 4, 6)
    l_sup = min(mean_len, 40)
    sharp_vals = np.zeros(len(dom_idx), dtype=np.float64)
    for k, idx in enumerate(dom_idx):
        sharp_vals[k] = compute_sharpness(contour, idx, l_sup=l_sup)
    dt_vals = estimate_delta_t(sharp_vals[1:], dt_min=dt_min_eff, dt_max=dt_max_eff)
    if len(dt_vals) > 0:
        dt_vals = np.concatenate([[0.0], dt_vals[:-1]])
    if len(dt_vals) > 1:
        dt_vals[1:] = np.clip(dt_vals[1:], dt_min_eff, dt_max_eff)

    initial = build_initial_sl_plan(
        contour=contour, dom_indices=dom_idx,
        phi_pairs=phi_pairs, delta_t_values=dt_vals,
        mu_global=mu_global, sigma_global=sigma_global,
        dt_min=dt_min_eff, dt_max=dt_max_eff,
    )
    refined, mse_px, n_iter = iterative_target_refinement(
        initial, contour=contour, dom_indices=dom_idx,
        max_iter=max_refit_iter, tol_px=1.0,
    )
    return GroupResult(
        params=refined, contour=contour,
        dominant_points=contour[dom_idx],
        mse_px=mse_px, n_iter_used=n_iter,
    )


# ============================================================================
# パイプライン本体 (同期)
# ============================================================================

def _run_pipeline(
    *,
    raw: Optional[bytes],
    stroke_points_json: Optional[str],
    gml_raw: Optional[bytes] = None,
    gml_filename: Optional[str] = None,
    gml_id: Optional[int] = None,
    gml_stroke_indices: Optional[str] = None,
    model: str,
    max_dim: int,
    angle_thresh_deg: float,
    dt_min: float,
    dt_max: float,
    max_refit_iter: int,
    include_inflections: bool,
    use_min_time_reparam: bool,
    seed: int,
    min_arc_px: float,
    max_groups: int,
    min_arc_spacing_px: float,
    max_dominant_points: int,
    nonzero_speed_safety: float,
    target_total_duration_s: float,
    gap_between_groups_s: float,
    use_diff_refit: bool,
    diff_n_iter: int,
    diff_lr: float,
    diff_lambda_smooth: float,
    diff_work_size: int,
    diff_stroke_radius_px: float,
    include_stages: bool,
    on_progress: ProgressCb = None,
) -> TrajectoryPlan:
    def _emit(step: str, progress: float, **extra: Any) -> None:
        if on_progress is not None:
            try:
                on_progress(step, progress, extra)
            except Exception:
                pass

    if model not in ("SL_STAR", "GL_STAR"):
        raise HTTPException(400, "model は SL_STAR または GL_STAR")
    input_count = sum(
        1 for x in (raw, stroke_points_json, gml_raw, gml_id) if x is not None and x != ""
    )
    if input_count == 0:
        raise HTTPException(
            400, "image / stroke_points / gml_file / gml_id のいずれかが必要です",
        )
    if input_count > 1:
        raise HTTPException(
            400, "image / stroke_points / gml_file / gml_id は排他です",
        )

    _emit("preprocess", 0.03)

    contours: list[np.ndarray]
    target_mask: Optional[np.ndarray] = None
    img_w: int
    img_h: int
    source_info: Optional[SourceInfo] = None

    if raw is not None:
        if len(raw) > 20 * 1024 * 1024:
            raise HTTPException(400, "画像は 20MB 以下にしてください")
        _emit("mask", 0.08)
        try:
            ext = extract_all_contours(
                raw, max_dim=max_dim, min_arc_px=min_arc_px,
                max_groups=max_groups,
            )
        except Exception as e:
            raise HTTPException(400, f"輪郭抽出に失敗: {e}")
        contours = ext["contours"]
        img_w, img_h = ext["image_width"], ext["image_height"]
        target_mask = ext.get("mask")
        source_info = SourceInfo(kind="image")
        _emit("contours", 0.22, n_contours=len(contours))
    elif gml_raw is not None or gml_id is not None:
        # --- GML 入力 (spec §8) ---
        try:
            if gml_raw is not None:
                if len(gml_raw) > 2 * 1024 * 1024:
                    raise HTTPException(400, "GML は 2MB 以下にしてください")
                doc = parse_gml(
                    gml_raw, filename=gml_filename or "<upload>",
                )
            else:
                corpus = get_corpus()
                try:
                    doc = corpus.get(int(gml_id))  # type: ignore[arg-type]
                except FileNotFoundError:
                    raise HTTPException(404, f"gml_id {gml_id} が見つかりません")
                except ValueError as e:
                    raise HTTPException(400, str(e))
        except GmlParseError as e:
            raise HTTPException(400, f"GML のパースに失敗: {e}")
        try:
            selected = parse_indices_csv(gml_stroke_indices)
        except ValueError as e:
            raise HTTPException(400, f"gml_stroke_indices が不正: {e}")
        valid = {s.index for s in doc.strokes}
        if selected is not None:
            bad = [i for i in selected if i not in valid]
            if bad:
                raise HTTPException(
                    400, f"存在しない stroke index: {bad}",
                )
        contours = strokes_to_contours(doc, selected=selected)
        if not contours:
            raise HTTPException(
                400, "GML から利用可能なストロークがありません (各 stroke は 4 点以上必要)",
            )
        img_w, img_h = doc.screen_bounds
        if use_diff_refit:
            target_mask = rasterize_strokes(
                contours, img_w, img_h,
                thickness=max(int(diff_stroke_radius_px * 2), 3),
            )
        source_info = SourceInfo(
            kind="gml",
            gml_id=doc.gml_id,
            gml_filename=doc.filename,
            gml_client=doc.client_name,
            gml_screen_bounds=doc.screen_bounds,
            gml_stroke_count=len(doc.strokes),
            gml_selected_indices=selected,
        )
        _emit("contours", 0.22, n_contours=len(contours))
    else:
        try:
            pts = json.loads(stroke_points_json or "[]")
            if not isinstance(pts, list) or len(pts) < 4:
                raise ValueError("stroke_points は4点以上必要")
            arr = np.array([[p[0], p[1]] for p in pts], dtype=np.float32)
        except Exception as e:
            raise HTTPException(400, f"stroke_points のパースに失敗: {e}")
        arr = resample_arclen(arr, step=1.0)
        contours = [smooth_gaussian(arr, sigma_px=1.5)]
        img_w = int(arr[:, 0].max() + 20) if len(arr) else 800
        img_h = int(arr[:, 1].max() + 20) if len(arr) else 600
        mask = np.zeros((img_h, img_w), dtype=np.uint8)
        pts_int = arr.round().astype(np.int32)
        cv2.polylines(
            mask, [pts_int], isClosed=False, color=255,
            thickness=max(int(diff_stroke_radius_px * 2), 3),
        )
        target_mask = mask
        source_info = SourceInfo(kind="drawing")
        _emit("contours", 0.22, n_contours=1)

    if not contours:
        raise HTTPException(400, "輪郭が抽出できませんでした (画像を確認してください)")

    mu_global = math.log(0.2)
    sigma_global = math.log(1.4)

    _emit("segment", 0.28)
    results: list[GroupResult] = []
    for ci, c in enumerate(contours):
        gr = _analyze_single_contour(
            c, model=model, angle_thresh_deg=angle_thresh_deg,
            dt_min=dt_min, dt_max=dt_max,
            include_inflections=include_inflections,
            max_refit_iter=max_refit_iter,
            mu_global=mu_global, sigma_global=sigma_global,
            min_arc_spacing_px=min_arc_spacing_px,
            max_dominant_points=max_dominant_points,
            nonzero_speed_safety=nonzero_speed_safety,
        )
        if gr is not None:
            results.append(gr)
        _emit(
            "segment", 0.28 + 0.17 * (ci + 1) / len(contours),
            group=ci + 1, total=len(contours),
        )

    if not results:
        raise HTTPException(400, "全輪郭で優位点を検出できませんでした")

    # 初期プラン snapshot
    initial_snapshot: Optional[list[GroupResult]] = None
    if include_stages:
        initial_snapshot = copy.deepcopy(results)

    # 微分可能最適化 (最も時間がかかるステージ)
    if use_diff_refit and target_mask is not None and results:
        _emit("diff_refit", 0.48, iter=0, total=diff_n_iter)
        try:
            diff_dt_max = max_delta_t_for_overlap(
                mu_global, sigma_global, safety_factor=nonzero_speed_safety,
            )
            params_list = [gr.params for gr in results]

            def _diff_cb(frac: float, loss: float) -> None:
                cur = int(frac * diff_n_iter)
                _emit(
                    "diff_refit",
                    0.48 + 0.44 * frac,
                    iter=cur, total=diff_n_iter, loss=round(float(loss), 5),
                )

            refined_params, diff_info = differentiable_refit(
                params_list, target_mask,
                mu_global=mu_global, sigma_global=sigma_global,
                dt_min=max(dt_min * 0.1, 0.01),
                dt_max=diff_dt_max,
                n_iter=diff_n_iter, lr=diff_lr,
                lambda_smooth=diff_lambda_smooth,
                work_size=diff_work_size,
                stroke_radius_world_px=diff_stroke_radius_px,
                progress_callback=_diff_cb,
            )
            try:
                from scipy.spatial import cKDTree
                for gr, new_p in zip(results, refined_params):
                    gr.params = new_p
                    if gr.contour is not None and len(gr.contour) > 1:
                        ev = evaluate_trajectory(new_p)
                        xy = np.stack([ev["x"], ev["y"]], axis=-1)
                        tree = cKDTree(np.asarray(gr.contour, dtype=np.float64))
                        dists, _ = tree.query(xy)
                        gr.mse_px = float(np.sqrt(np.mean(dists ** 2)))
            except ImportError:
                for gr, new_p in zip(results, refined_params):
                    gr.params = new_p
            logger.info(
                "diff_refit: L_I %.5f → %.5f (%.1f%% image cost reduction)",
                diff_info["initial_LI"], diff_info["final_LI"],
                (1.0 - diff_info["final_LI"] / max(diff_info["initial_LI"], 1e-9)) * 100.0,
            )
        except Exception:
            logger.exception("diff_refit failed, keeping analytic estimates")

    _emit("finalize", 0.93)

    # 時間軸再スケール
    if target_total_duration_s > 0 and len(results) > 0:
        n = len(results)
        visible_durations = []
        for gr in results:
            ev = evaluate_trajectory(gr.params, trim_zero_speed=True)
            visible_durations.append(float(ev["t_end"]))
        total_active = sum(visible_durations)
        target_active = max(
            target_total_duration_s - (n - 1) * gap_between_groups_s, 0.1,
        )
        if total_active > target_active:
            alpha = target_active / total_active
            for gr in results:
                rescale_params_in_time(gr.params, alpha)
            if initial_snapshot is not None:
                for gr in initial_snapshot:
                    rescale_params_in_time(gr.params, alpha)
            logger.info(
                "time-rescale: active %.2fs → %.2fs (α=%.3f)",
                total_active, target_active, alpha,
            )

    logger.info(
        "analyze: %d contours → %d stroke groups", len(contours), len(results),
    )

    global_params = GlobalParams(
        mu=mu_global, sigma=sigma_global,
        delta_t_min=dt_min, delta_t_max=dt_max,
        use_min_time_reparam=use_min_time_reparam,
        lambda_smooth=0.01, seed=seed,
    )
    plan = results_to_plan(
        results,
        image_width=img_w, image_height=img_h,
        global_params=global_params,
        model_kind=model,
        decimate_ts=2,
    )
    if source_info is not None:
        plan.source = source_info

    if include_stages:
        if target_mask is not None:
            try:
                from skimage.morphology import thin as _thin
                mask_u8 = (
                    target_mask if target_mask.dtype == np.uint8
                    else (np.clip(target_mask, 0, 1) * 255).astype(np.uint8)
                )
                ok, buf = cv2.imencode(".png", mask_u8)
                if ok:
                    plan.mask_png_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
                sk = _thin(mask_u8 > 127)
                sk_u8 = (sk.astype(np.uint8)) * 255
                ok2, buf2 = cv2.imencode(".png", sk_u8)
                if ok2:
                    plan.skeleton_png_b64 = base64.b64encode(buf2.tobytes()).decode("ascii")
            except Exception:
                logger.exception("stages PNG encoding failed")
        if initial_snapshot is not None:
            initial_plan = results_to_plan(
                initial_snapshot,
                image_width=img_w, image_height=img_h,
                global_params=global_params,
                model_kind=model,
                decimate_ts=2,
            )
            plan.initial_stroke_groups = initial_plan.stroke_groups

    _emit("finalize", 0.99)
    return plan


# ============================================================================
# フォーム引数ヘルパ
# ============================================================================

def _collect_form_kwargs(
    model: str, max_dim: int, angle_thresh_deg: float,
    dt_min: float, dt_max: float, max_refit_iter: int,
    include_inflections: bool, use_min_time_reparam: bool, seed: int,
    min_arc_px: float, max_groups: int,
    min_arc_spacing_px: float, max_dominant_points: int,
    nonzero_speed_safety: float,
    target_total_duration_s: float, gap_between_groups_s: float,
    use_diff_refit: bool, diff_n_iter: int, diff_lr: float,
    diff_lambda_smooth: float, diff_work_size: int,
    diff_stroke_radius_px: float, include_stages: bool,
) -> dict:
    return dict(
        model=model, max_dim=max_dim, angle_thresh_deg=angle_thresh_deg,
        dt_min=dt_min, dt_max=dt_max, max_refit_iter=max_refit_iter,
        include_inflections=include_inflections,
        use_min_time_reparam=use_min_time_reparam, seed=seed,
        min_arc_px=min_arc_px, max_groups=max_groups,
        min_arc_spacing_px=min_arc_spacing_px,
        max_dominant_points=max_dominant_points,
        nonzero_speed_safety=nonzero_speed_safety,
        target_total_duration_s=target_total_duration_s,
        gap_between_groups_s=gap_between_groups_s,
        use_diff_refit=use_diff_refit, diff_n_iter=diff_n_iter,
        diff_lr=diff_lr, diff_lambda_smooth=diff_lambda_smooth,
        diff_work_size=diff_work_size,
        diff_stroke_radius_px=diff_stroke_radius_px,
        include_stages=include_stages,
    )


# ============================================================================
# 同期エンドポイント (既存と互換)
# ============================================================================

@router.post("/analyze", response_model=TrajectoryPlan)
async def analyze(
    image: Optional[UploadFile] = File(None),
    stroke_points: Optional[str] = Form(None),
    gml_file: Optional[UploadFile] = File(None),
    gml_id: Optional[int] = Form(None),
    gml_stroke_indices: Optional[str] = Form(None),
    model: str = Form("GL_STAR"),
    max_dim: int = Form(1024),
    angle_thresh_deg: float = Form(35.0),
    dt_min: float = Form(0.15),
    dt_max: float = Form(0.60),
    max_refit_iter: int = Form(10),
    include_inflections: bool = Form(True),
    use_min_time_reparam: bool = Form(True),
    seed: int = Form(42),
    min_arc_px: float = Form(15.0),
    max_groups: int = Form(20),
    min_arc_spacing_px: float = Form(0.0),
    max_dominant_points: int = Form(15),
    nonzero_speed_safety: float = Form(0.4),
    target_total_duration_s: float = Form(0.0),
    gap_between_groups_s: float = Form(0.3),
    use_diff_refit: bool = Form(True),
    diff_n_iter: int = Form(180),
    diff_lr: float = Form(5e-3),
    diff_lambda_smooth: float = Form(0.01),
    diff_work_size: int = Form(192),
    diff_stroke_radius_px: float = Form(5.0),
    include_stages: bool = Form(True),
) -> TrajectoryPlan:
    raw = await image.read() if image is not None else None
    gml_raw = await gml_file.read() if gml_file is not None else None
    gml_name = gml_file.filename if gml_file is not None else None
    kwargs = _collect_form_kwargs(
        model, max_dim, angle_thresh_deg, dt_min, dt_max, max_refit_iter,
        include_inflections, use_min_time_reparam, seed,
        min_arc_px, max_groups, min_arc_spacing_px, max_dominant_points,
        nonzero_speed_safety, target_total_duration_s, gap_between_groups_s,
        use_diff_refit, diff_n_iter, diff_lr, diff_lambda_smooth,
        diff_work_size, diff_stroke_radius_px, include_stages,
    )
    return _run_pipeline(
        raw=raw, stroke_points_json=stroke_points,
        gml_raw=gml_raw, gml_filename=gml_name, gml_id=gml_id,
        gml_stroke_indices=gml_stroke_indices,
        **kwargs,
    )


# ============================================================================
# ストリーミングエンドポイント (NDJSON)
# ============================================================================

@router.post("/analyze/stream")
async def analyze_stream(
    image: Optional[UploadFile] = File(None),
    stroke_points: Optional[str] = Form(None),
    gml_file: Optional[UploadFile] = File(None),
    gml_id: Optional[int] = Form(None),
    gml_stroke_indices: Optional[str] = Form(None),
    model: str = Form("GL_STAR"),
    max_dim: int = Form(1024),
    angle_thresh_deg: float = Form(35.0),
    dt_min: float = Form(0.15),
    dt_max: float = Form(0.60),
    max_refit_iter: int = Form(10),
    include_inflections: bool = Form(True),
    use_min_time_reparam: bool = Form(True),
    seed: int = Form(42),
    min_arc_px: float = Form(15.0),
    max_groups: int = Form(20),
    min_arc_spacing_px: float = Form(0.0),
    max_dominant_points: int = Form(15),
    nonzero_speed_safety: float = Form(0.4),
    target_total_duration_s: float = Form(0.0),
    gap_between_groups_s: float = Form(0.3),
    use_diff_refit: bool = Form(True),
    diff_n_iter: int = Form(180),
    diff_lr: float = Form(5e-3),
    diff_lambda_smooth: float = Form(0.01),
    diff_work_size: int = Form(192),
    diff_stroke_radius_px: float = Form(5.0),
    include_stages: bool = Form(True),
):
    raw = await image.read() if image is not None else None
    gml_raw = await gml_file.read() if gml_file is not None else None
    gml_name = gml_file.filename if gml_file is not None else None
    kwargs = _collect_form_kwargs(
        model, max_dim, angle_thresh_deg, dt_min, dt_max, max_refit_iter,
        include_inflections, use_min_time_reparam, seed,
        min_arc_px, max_groups, min_arc_spacing_px, max_dominant_points,
        nonzero_speed_safety, target_total_duration_s, gap_between_groups_s,
        use_diff_refit, diff_n_iter, diff_lr, diff_lambda_smooth,
        diff_work_size, diff_stroke_radius_px, include_stages,
    )

    loop = asyncio.get_event_loop()
    queue: asyncio.Queue = asyncio.Queue()
    SENTINEL = object()

    def progress_cb(step: str, progress: float, extra: dict) -> None:
        payload = {"step": step, "progress": progress, **(extra or {})}
        try:
            loop.call_soon_threadsafe(queue.put_nowait, payload)
        except RuntimeError:
            pass

    def worker() -> None:
        try:
            plan = _run_pipeline(
                raw=raw, stroke_points_json=stroke_points,
                gml_raw=gml_raw, gml_filename=gml_name, gml_id=gml_id,
                gml_stroke_indices=gml_stroke_indices,
                on_progress=progress_cb, **kwargs,
            )
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "done", "progress": 1.0,
                 "plan": plan.model_dump(mode="json")},
            )
        except HTTPException as e:
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "error", "status": e.status_code,
                 "detail": str(e.detail)},
            )
        except Exception as e:
            logger.exception("analyze_stream worker failed")
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "error", "status": 500, "detail": str(e)},
            )
        finally:
            loop.call_soon_threadsafe(queue.put_nowait, SENTINEL)

    loop.run_in_executor(None, worker)

    async def gen():
        while True:
            msg = await queue.get()
            if msg is SENTINEL:
                return
            yield json.dumps(msg, ensure_ascii=False) + "\n"
            if msg.get("step") in ("done", "error"):
                # sentinel を待たずに終了しても OK、worker 側は独立
                pass

    return StreamingResponse(gen(), media_type="application/x-ndjson")
