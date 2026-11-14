"""POST /api/refit — 編集済み plan の各 stroke_group を再反復。"""
from __future__ import annotations

import numpy as np
from fastapi import APIRouter

from sigma_lognormal.fitting import iterative_target_refinement

from app.api.convert import GroupResult, results_to_plan, group_to_params
from app.schemas.trajectory import TrajectoryPlan
from app.schemas.requests import RefitRequest

router = APIRouter(prefix="/api", tags=["refit"])


@router.post("/refit", response_model=TrajectoryPlan)
async def refit(req: RefitRequest) -> TrajectoryPlan:
    plan = req.plan
    results: list[GroupResult] = []
    for group in plan.stroke_groups:
        params = group_to_params(group)
        # contour: group.input_contour を優先、なければ dom points を contour と見なす
        if group.input_contour:
            contour = np.asarray(group.input_contour, dtype=np.float64)
            cur_targets = np.vstack([params.p0[None, :], params.virtual_targets])
            dom_idx = []
            for p in cur_targets:
                d = np.linalg.norm(contour - p, axis=1)
                dom_idx.append(int(np.argmin(d)))
        elif group.dominant_points:
            contour = np.array(group.dominant_points, dtype=np.float64)
            dom_idx = list(range(len(contour)))
        else:
            results.append(GroupResult(
                params=params, contour=None, dominant_points=None,
                mse_px=group.quality.reconstruction_mse_px,
                n_iter_used=0,
            ))
            continue
        refined, mse, n_iter = iterative_target_refinement(
            params, contour=contour, dom_indices=dom_idx,
            max_iter=req.max_iter, tol_px=req.tol_px,
        )
        results.append(GroupResult(
            params=refined, contour=contour,
            dominant_points=contour[dom_idx],
            mse_px=mse, n_iter_used=n_iter,
        ))
    return results_to_plan(
        results,
        image_width=plan.image_width,
        image_height=plan.image_height,
        global_params=plan.global_params,
        model_kind=plan.model,
        decimate_ts=2,
    )
