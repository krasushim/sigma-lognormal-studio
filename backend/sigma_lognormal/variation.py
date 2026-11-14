"""
Parameter-perturbation variation generator — Berio 2015 §7.1.
"""
from __future__ import annotations

import numpy as np

from .sl_model import SLStarParams


def perturb(
    plan: SLStarParams,
    *,
    target_offset_px: float = 5.0,
    delta_t_scale: float = 0.15,
    phi_scale: float = 0.1,
    global_scale_xy: float = 0.05,
    seed: int = 0,
) -> SLStarParams:
    rng = np.random.default_rng(seed)
    m = plan.m()

    # Target offsets
    off = rng.uniform(-target_offset_px, target_offset_px, size=(m, 2))
    vt_new = plan.virtual_targets + off

    # Δt multiplicative
    dt_mult = rng.uniform(1.0 - delta_t_scale, 1.0 + delta_t_scale, size=m)
    dt_new = np.clip(plan.delta_t * dt_mult, 1e-3, None)

    # φ multiplicative
    phi_mult = rng.uniform(1.0 - phi_scale, 1.0 + phi_scale, size=m)
    phi_new = plan.phi * phi_mult

    # Global X/Y non-isotropic scale applied around p0
    sx = rng.uniform(1.0 - global_scale_xy, 1.0 + global_scale_xy)
    sy = rng.uniform(1.0 - global_scale_xy, 1.0 + global_scale_xy)
    rel = vt_new - plan.p0
    rel[:, 0] *= sx
    rel[:, 1] *= sy
    vt_new = plan.p0 + rel

    return SLStarParams(
        p0=plan.p0.copy(),
        virtual_targets=vt_new,
        delta_t=dt_new,
        mu=plan.mu.copy(),
        sigma=plan.sigma.copy(),
        phi=phi_new,
    )


def generate_variations(
    plan: SLStarParams,
    n: int,
    *,
    target_offset_px: float = 5.0,
    delta_t_scale: float = 0.15,
    phi_scale: float = 0.1,
    global_scale_xy: float = 0.05,
    seed: int = 42,
) -> list[SLStarParams]:
    rng = np.random.default_rng(seed)
    return [
        perturb(
            plan,
            target_offset_px=target_offset_px,
            delta_t_scale=delta_t_scale,
            phi_scale=phi_scale,
            global_scale_xy=global_scale_xy,
            seed=int(rng.integers(1 << 30)),
        )
        for _ in range(n)
    ]
