"""
Conversion between internal SLStarParams (per stroke group) and the
pydantic TrajectoryPlan schema (v2.0, multi-group).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sigma_lognormal.sl_model import (
    SLStarParams, evaluate_trajectory, critical_points,
)
from app.schemas.trajectory import (
    TrajectoryPlan, ActionPlan, StrokeParams, GlobalParams,
    DerivedData, TimeSeries, Quality, StrokeGroup,
)


@dataclass
class GroupResult:
    """analyze.py / refit.py から convert に渡す中間表現。"""
    params: SLStarParams
    contour: np.ndarray | None = None         # [N, 2]
    dominant_points: np.ndarray | None = None # [m+1, 2]
    mse_px: float = 0.0
    n_iter_used: int = 0


def _ts_to_timeseries(ev: dict, decimate: int = 1) -> TimeSeries:
    idx = slice(None, None, max(decimate, 1))
    t = ev["t"][idx]
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    return TimeSeries(
        dt=dt,
        t=[round(float(v), 5) for v in t],
        x=[round(float(v), 3) for v in ev["x"][idx]],
        y=[round(float(v), 3) for v in ev["y"][idx]],
        vx=[round(float(v), 3) for v in ev["vx"][idx]],
        vy=[round(float(v), 3) for v in ev["vy"][idx]],
        speed=[round(float(v), 3) for v in ev["speed"][idx]],
    )


def _group_from_result(gr: GroupResult, group_index: int, decimate_ts: int) -> StrokeGroup:
    p = gr.params
    ev = evaluate_trajectory(p)
    cps = critical_points(p, eval_result=ev)
    xy = np.stack([ev["x"], ev["y"]], axis=-1)
    arc_len = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    strokes = [
        StrokeParams(
            index=j,
            delta_t=float(p.delta_t[j]),
            mu=float(p.mu[j]),
            sigma=float(p.sigma[j]),
            phi=float(p.phi[j]),
        )
        for j in range(p.m())
    ]
    return StrokeGroup(
        group_index=group_index,
        action_plan=ActionPlan(
            p0=(float(p.p0[0]), float(p.p0[1])),
            virtual_targets=[(float(v[0]), float(v[1])) for v in p.virtual_targets],
        ),
        strokes=strokes,
        derived=DerivedData(
            timeseries=_ts_to_timeseries(ev, decimate=decimate_ts),
            t_end=float(ev["t_end"]),
            arc_length_px=round(arc_len, 2),
            critical_points=[(float(p_[0]), float(p_[1])) for p_ in cps],
        ),
        input_contour=(
            [(float(p_[0]), float(p_[1])) for p_ in gr.contour]
            if gr.contour is not None and len(gr.contour) > 0 else []
        ),
        dominant_points=(
            [(float(p_[0]), float(p_[1])) for p_ in gr.dominant_points]
            if gr.dominant_points is not None and len(gr.dominant_points) > 0 else []
        ),
        quality=Quality(
            reconstruction_mse_px=round(float(gr.mse_px), 3),
            n_iter_used=int(gr.n_iter_used),
        ),
    )


def results_to_plan(
    results: list[GroupResult],
    *,
    image_width: int,
    image_height: int,
    global_params: GlobalParams,
    model_kind: str = "SL_STAR",
    decimate_ts: int = 2,
) -> TrajectoryPlan:
    groups = [
        _group_from_result(gr, i, decimate_ts) for i, gr in enumerate(results)
    ]
    # 全体 MSE は各 group の重み (頂点数) 付き平均
    if groups:
        total_n = sum(max(len(g.dominant_points), 1) for g in groups)
        weighted = sum(
            g.quality.reconstruction_mse_px * max(len(g.dominant_points), 1)
            for g in groups
        )
        overall_mse = round(weighted / max(total_n, 1), 3)
        max_iter = max(g.quality.n_iter_used for g in groups)
    else:
        overall_mse = 0.0
        max_iter = 0
    return TrajectoryPlan(
        schema_version="2.1",
        model=model_kind,
        image_width=image_width,
        image_height=image_height,
        global_params=global_params,
        stroke_groups=groups,
        overall_quality=Quality(
            reconstruction_mse_px=overall_mse,
            n_iter_used=max_iter,
        ),
    )


def group_to_params(group: StrokeGroup) -> SLStarParams:
    m = len(group.strokes)
    phi = np.zeros(m, dtype=np.float64)
    dt = np.zeros(m, dtype=np.float64)
    mu = np.zeros(m, dtype=np.float64)
    sigma = np.zeros(m, dtype=np.float64)
    for j, s in enumerate(group.strokes):
        phi[j] = s.phi if s.phi is not None else 0.0
        dt[j] = s.delta_t
        mu[j] = s.mu
        sigma[j] = s.sigma
    return SLStarParams(
        p0=np.array(group.action_plan.p0, dtype=np.float64),
        virtual_targets=np.array(group.action_plan.virtual_targets, dtype=np.float64),
        delta_t=dt, mu=mu, sigma=sigma, phi=phi,
    )


def plan_to_params_list(plan: TrajectoryPlan) -> list[SLStarParams]:
    return [group_to_params(g) for g in plan.stroke_groups]
