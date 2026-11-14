"""MPC stroke estimation endpoint.

POST /api/mpc                    — 同期 (MpcResult を直接返す)
POST /api/mpc/stream             — NDJSON ストリーミング (進捗イベント + 完了時 result)
POST /api/mpc/from-points        — 生点列入力・同期
POST /api/mpc/from-points/stream — 生点列入力・ストリーミング

Auto-optimize mode: n_via loop emits one progress event per iteration.
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Callable, Optional

import numpy as np
from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from fastapi.responses import StreamingResponse

from app.gml.corpus import get_corpus
from app.gml.parser import GmlParseError, parse_gml
from app.schemas.mpc import MpcEllipse, MpcFromPointsRequest, MpcResult, MpcStrokeResult
from sigma_lognormal.mpc_stroke import (
    optimize_mpc_stroke,
    run_mpc_on_stroke,
)

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/mpc", tags=["mpc"])

MAX_UPLOAD_BYTES = 2 * 1024 * 1024

ProgressCb = Optional[Callable[[str, float, dict], None]]


# ── Schema helper ─────────────────────────────────────────────────────────────

def _to_schema(result, error_history: list[float]) -> MpcStrokeResult:
    return MpcStrokeResult(
        stroke_index=result.stroke_index,
        n_via=result.n_via,
        order=result.order,
        ellipses=[
            MpcEllipse(x=e.x, y=e.y, rx=e.rx, ry=e.ry, angle_deg=e.angle_deg)
            for e in result.ellipses
        ],
        traj_x=result.traj_x,
        traj_y=result.traj_y,
        duration_s=result.duration_s,
        reconstruction_error_px=result.reconstruction_error_px,
        converged=result.converged,
        error_history=error_history,
        n_via_start=result.n_via_start,
    )


# ── Core processing loop ──────────────────────────────────────────────────────

def _process_stroke_list(
    strokes: list[tuple[int, np.ndarray]],  # [(stroke_index, pts_px), ...]
    screen_w: int,
    screen_h: int,
    n_via: int,
    order: int,
    dt: float,
    auto_optimize: bool,
    tol_px: float,
    min_via: int,
    max_via: int,
    on_progress: ProgressCb = None,
) -> tuple[list[MpcStrokeResult], int]:
    """Run MPC on a list of (stroke_index, pts_px) pairs.

    Emits progress events via *on_progress* if provided:
      - ``{"step": "mpc", "progress": f, "stroke": i, "total": N}`` before each stroke
      - ``{"step": "mpc", ..., "n_via": k, "error_px": e}`` per n_via iteration (auto mode)
      - ``{"step": "mpc", ..., "error_px": e, "n_via": k}`` after each stroke
    """
    def _emit(progress: float, **extra: Any) -> None:
        if on_progress is not None:
            try:
                on_progress("mpc", progress, extra)
            except Exception:
                pass

    total = len(strokes)
    stroke_results: list[MpcStrokeResult] = []
    final_n_via = n_via

    for i, (stroke_index, pts) in enumerate(strokes):
        base_frac = i / max(total, 1)
        next_frac = (i + 1) / max(total, 1)
        _emit(base_frac * 0.95 + 0.02,
              stroke=i + 1, total=total, stroke_index=stroke_index)

        try:
            if auto_optimize:
                n_steps = max(max_via - min_via + 1, 1)

                def _via_cb(n_via_k: int, err: float) -> None:
                    within = (n_via_k - min_via) / n_steps
                    frac = (base_frac + within * (next_frac - base_frac)) * 0.95 + 0.02
                    _emit(frac,
                          stroke=i + 1, total=total, stroke_index=stroke_index,
                          n_via=n_via_k, error_px=round(err, 2))

                result, history = optimize_mpc_stroke(
                    pts_px=pts, screen_w=screen_w, screen_h=screen_h,
                    order=order, dt=dt, tol_px=tol_px,
                    min_via=min_via, max_via=max_via,
                    stroke_index=stroke_index,
                    progress_cb=_via_cb,
                )
            else:
                result = run_mpc_on_stroke(
                    pts_px=pts, screen_w=screen_w, screen_h=screen_h,
                    n_via=n_via, order=order, dt=dt,
                    stroke_index=stroke_index,
                )
                result.converged = result.reconstruction_error_px <= tol_px
                history = [result.reconstruction_error_px]

        except Exception as exc:
            logger.warning("MPC failed for stroke %d: %s", stroke_index, exc)
            continue

        final_n_via = result.n_via
        stroke_results.append(_to_schema(result, history))

        _emit(next_frac * 0.95 + 0.02,
              stroke=i + 1, total=total, stroke_index=stroke_index,
              n_via=result.n_via, error_px=result.reconstruction_error_px)

    return stroke_results, final_n_via


# ── GML ロード・フィルタ ヘルパ ────────────────────────────────────────────────

async def _load_gml_doc(
    gml_file: Optional[UploadFile],
    gml_id: Optional[int],
):
    if gml_file is not None:
        raw = await gml_file.read()
        if not raw:
            raise HTTPException(400, "空ファイルです")
        if len(raw) > MAX_UPLOAD_BYTES:
            raise HTTPException(400, f"ファイルが大きすぎます (最大 {MAX_UPLOAD_BYTES // 1024} KB)")
        try:
            doc = parse_gml(raw, filename=gml_file.filename or "<upload>")
        except GmlParseError as exc:
            raise HTTPException(400, f"GML のパースに失敗: {exc}") from exc
        return doc, None
    elif gml_id is not None:
        corpus = get_corpus()
        try:
            doc = corpus.get(gml_id)
        except FileNotFoundError:
            raise HTTPException(404, f"gml_id {gml_id} がコーパスに見つかりません")
        except (ValueError, GmlParseError) as exc:
            raise HTTPException(400, str(exc))
        return doc, gml_id
    else:
        raise HTTPException(400, "gml_id または gml_file のいずれかを指定してください")


def _filter_strokes(doc, gml_stroke_indices: Optional[str]) -> list[tuple[int, np.ndarray]]:
    selected: Optional[set[int]] = None
    if gml_stroke_indices:
        try:
            selected = {int(x.strip()) for x in gml_stroke_indices.split(",") if x.strip()}
        except ValueError:
            raise HTTPException(400, "gml_stroke_indices の形式が不正です")

    result = []
    for s in doc.strokes:
        if selected is not None and s.index not in selected:
            continue
        pts = np.array(s.points, dtype=float)
        if len(pts) >= 4:
            result.append((s.index, pts))
    return result


# ── ストリーミング共通ヘルパ ───────────────────────────────────────────────────

def _make_stream_response(worker_fn: Callable) -> StreamingResponse:
    """Run *worker_fn* in a thread, stream NDJSON progress to client."""
    loop = asyncio.get_event_loop()
    queue: asyncio.Queue = asyncio.Queue()
    SENTINEL = object()

    def progress_cb(step: str, progress: float, extra: dict) -> None:
        payload = {"step": step, "progress": round(progress, 4), **(extra or {})}
        try:
            loop.call_soon_threadsafe(queue.put_nowait, payload)
        except RuntimeError:
            pass

    def run_worker() -> None:
        try:
            result = worker_fn(progress_cb)
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "done", "progress": 1.0, "result": result.model_dump(mode="json")},
            )
        except HTTPException as e:
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "error", "status": e.status_code, "detail": str(e.detail)},
            )
        except Exception as e:
            logger.exception("MPC stream worker failed")
            loop.call_soon_threadsafe(
                queue.put_nowait,
                {"step": "error", "status": 500, "detail": str(e)},
            )
        finally:
            loop.call_soon_threadsafe(queue.put_nowait, SENTINEL)

    loop.run_in_executor(None, run_worker)

    async def gen():
        while True:
            msg = await queue.get()
            if msg is SENTINEL:
                return
            yield json.dumps(msg, ensure_ascii=False) + "\n"

    return StreamingResponse(gen(), media_type="application/x-ndjson")


# ── 同期エンドポイント ─────────────────────────────────────────────────────────

@router.post("", response_model=MpcResult)
async def estimate_mpc(
    gml_id: Optional[int] = Form(None),
    gml_file: Optional[UploadFile] = File(None),
    gml_stroke_indices: Optional[str] = Form(None, description="カンマ区切りのストローク index。省略=全選択"),
    n_via: int = Form(5, ge=2, le=20, description="ガウス経由点数"),
    order: int = Form(4, ge=2, le=6, description="システム次数"),
    dt: float = Form(0.005, gt=0, description="MPCタイムステップ (s)"),
    auto_optimize: bool = Form(False, description="True=誤差が tol_px 以下になるまで n_via を自動増加"),
    tol_px: float = Form(3.0, gt=0, description="収束判定の再構成誤差しきい値 (px)"),
    min_via: int = Form(3, ge=2, le=20, description="自動最適化の最小経由点数"),
    max_via: int = Form(15, ge=2, le=30, description="自動最適化の最大経由点数"),
) -> MpcResult:
    doc, resolved_id = await _load_gml_doc(gml_file, gml_id)
    strokes = _filter_strokes(doc, gml_stroke_indices)
    if not strokes:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
    sw, sh = doc.screen_bounds
    results, final_n_via = _process_stroke_list(
        strokes, sw, sh, n_via, order, dt, auto_optimize, tol_px, min_via, max_via,
    )
    if not results:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
    return MpcResult(gml_id=resolved_id, screen_bounds=(sw, sh), strokes=results,
                     n_via=final_n_via, order=order, auto_optimize=auto_optimize, tol_px=tol_px)


# ── ストリーミングエンドポイント (GML) ─────────────────────────────────────────

@router.post("/stream")
async def estimate_mpc_stream(
    gml_id: Optional[int] = Form(None),
    gml_file: Optional[UploadFile] = File(None),
    gml_stroke_indices: Optional[str] = Form(None),
    n_via: int = Form(5, ge=2, le=20),
    order: int = Form(4, ge=2, le=6),
    dt: float = Form(0.005, gt=0),
    auto_optimize: bool = Form(False),
    tol_px: float = Form(3.0, gt=0),
    min_via: int = Form(3, ge=2, le=20),
    max_via: int = Form(15, ge=2, le=30),
):
    """NDJSON ストリーミング版。進捗イベントを逐次送出し、完了時に result を含む done イベントを送る。"""
    doc, resolved_id = await _load_gml_doc(gml_file, gml_id)
    strokes = _filter_strokes(doc, gml_stroke_indices)
    if not strokes:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
    sw, sh = doc.screen_bounds

    def worker(on_progress: ProgressCb) -> MpcResult:
        on_progress("mpc", 0.01, {"stroke": 0, "total": len(strokes)})
        results, final_n_via = _process_stroke_list(
            strokes, sw, sh, n_via, order, dt, auto_optimize, tol_px, min_via, max_via,
            on_progress=on_progress,
        )
        if not results:
            raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
        return MpcResult(gml_id=resolved_id, screen_bounds=(sw, sh), strokes=results,
                         n_via=final_n_via, order=order, auto_optimize=auto_optimize, tol_px=tol_px)

    return _make_stream_response(worker)


# ── 生点列エンドポイント (同期) ────────────────────────────────────────────────

@router.post("/from-points", response_model=MpcResult)
async def estimate_mpc_from_points(body: MpcFromPointsRequest) -> MpcResult:
    """Run MPC on raw stroke-point arrays (no GML file required)."""
    sw, sh = body.screen_w, body.screen_h
    strokes: list[tuple[int, np.ndarray]] = []
    for idx, raw_pts in enumerate(body.strokes):
        if len(raw_pts) < 4:
            continue
        pts = np.array(raw_pts, dtype=float)
        if pts.shape[1] < 3:
            raise HTTPException(400, "各点は [x_px, y_px, t_s] の3要素が必要です")
        strokes.append((idx, pts))
    if not strokes:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")

    results, final_n_via = _process_stroke_list(
        strokes, sw, sh, body.n_via, body.order, body.dt,
        body.auto_optimize, body.tol_px, body.min_via, body.max_via,
    )
    if not results:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
    return MpcResult(gml_id=None, screen_bounds=(sw, sh), strokes=results,
                     n_via=final_n_via, order=body.order,
                     auto_optimize=body.auto_optimize, tol_px=body.tol_px)


# ── 生点列エンドポイント (ストリーミング) ──────────────────────────────────────

@router.post("/from-points/stream")
async def estimate_mpc_from_points_stream(body: MpcFromPointsRequest):
    """生点列入力の NDJSON ストリーミング版。"""
    sw, sh = body.screen_w, body.screen_h
    strokes: list[tuple[int, np.ndarray]] = []
    for idx, raw_pts in enumerate(body.strokes):
        if len(raw_pts) < 4:
            continue
        pts = np.array(raw_pts, dtype=float)
        if pts.shape[1] < 3:
            raise HTTPException(400, "各点は [x_px, y_px, t_s] の3要素が必要です")
        strokes.append((idx, pts))
    if not strokes:
        raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")

    def worker(on_progress: ProgressCb) -> MpcResult:
        on_progress("mpc", 0.01, {"stroke": 0, "total": len(strokes)})
        results, final_n_via = _process_stroke_list(
            strokes, sw, sh, body.n_via, body.order, body.dt,
            body.auto_optimize, body.tol_px, body.min_via, body.max_via,
            on_progress=on_progress,
        )
        if not results:
            raise HTTPException(422, "有効なストロークが見つかりません (最低4点必要)")
        return MpcResult(gml_id=None, screen_bounds=(sw, sh), strokes=results,
                         n_via=final_n_via, order=body.order,
                         auto_optimize=body.auto_optimize, tol_px=body.tol_px)

    return _make_stream_response(worker)
