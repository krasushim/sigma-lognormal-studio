"""POST /api/variations — 各 group を独立に摂動してバリエーション生成。"""
from __future__ import annotations

import numpy as np
from fastapi import APIRouter

from sigma_lognormal.variation import perturb

from app.api.convert import GroupResult, results_to_plan, group_to_params
from app.schemas.requests import VariationRequest, VariationResponse

router = APIRouter(prefix="/api", tags=["variations"])


@router.post("/variations", response_model=VariationResponse)
async def variations(req: VariationRequest) -> VariationResponse:
    rng = np.random.default_rng(req.seed)
    base_params = [group_to_params(g) for g in req.plan.stroke_groups]

    out_plans = []
    for _ in range(req.n_variations):
        per_seed = int(rng.integers(1 << 30))
        results: list[GroupResult] = []
        for k, base in enumerate(base_params):
            p = perturb(
                base,
                target_offset_px=req.perturbations.target_offset_px,
                delta_t_scale=req.perturbations.delta_t_scale,
                phi_scale=req.perturbations.phi_scale,
                global_scale_xy=req.perturbations.global_scale_xy,
                seed=per_seed + k,
            )
            results.append(GroupResult(params=p))
        out_plans.append(
            results_to_plan(
                results,
                image_width=req.plan.image_width,
                image_height=req.plan.image_height,
                global_params=req.plan.global_params,
                model_kind=req.plan.model,
                decimate_ts=4,
            )
        )
    return VariationResponse(variations=out_plans)
