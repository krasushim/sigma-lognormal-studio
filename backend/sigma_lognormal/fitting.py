"""
Iterative target refinement — Berio 2015 Eq.22.

Given an initial SL* plan (from arc_fit + sharpness + segmentation), iteratively
move each virtual target w_j towards minimizing MSE between the generated
trajectory's critical points c_j and the dominant contour points p(z_j):

  w_j ← w_j + (p(z_j) - c_j)

Repeat until MSE < tol or max_iter reached.
"""
from __future__ import annotations

import math

import numpy as np

from .sl_model import SLStarParams, evaluate_trajectory, critical_points


def build_initial_sl_plan(
    contour: np.ndarray,
    dom_indices: list[int],
    phi_pairs: list[tuple[float, float]] | None = None,
    delta_t_values: np.ndarray | None = None,
    mu_global: float = math.log(0.2),
    sigma_global: float = math.log(1.4),
    dt_min: float = 0.15,
    dt_max: float = 0.60,
) -> SLStarParams:
    """
    Constructs SLStarParams with:
      p0 = contour[dom_indices[0]]
      virtual_targets = contour[dom_indices[1:]]
      phi_j ≈ mean of φ1, φ2 (SL has single symmetric turning angle)
      delta_t_j from sharpness (or mid-range default)
      mu, sigma = global constants across strokes (Berio 2015 §5 assumption)
    """
    if len(dom_indices) < 2:
        raise ValueError("少なくとも2つの優位点が必要です")

    dom_pts = contour[dom_indices]            # [m+1, 2]
    p0 = dom_pts[0].copy()
    vt = dom_pts[1:].copy()                   # [m, 2]
    m = vt.shape[0]

    if phi_pairs is None:
        phi = np.zeros(m, dtype=np.float64)
    else:
        phi = np.array([(p2 - p1) / 2.0 for (p1, p2) in phi_pairs], dtype=np.float64)
    # δ = 2φ が ±2π に近づくと D = ||Δw|| / sinc(δ/(2π)) が特異化する。
    # 実用上 1ストロークで ±90° を超える曲がりは稀なので、余裕を持って ±0.7π にクランプ。
    phi_limit = math.pi * 0.7
    phi = np.clip(phi, -phi_limit, phi_limit)

    if delta_t_values is None:
        delta_t = np.full(m, (dt_min + dt_max) / 2.0, dtype=np.float64)
    else:
        delta_t = np.asarray(delta_t_values, dtype=np.float64)
        if len(delta_t) != m:
            # Aggressive fallback: pad/truncate
            delta_t = np.full(m, (dt_min + dt_max) / 2.0, dtype=np.float64)

    mu_arr = np.full(m, mu_global, dtype=np.float64)
    sigma_arr = np.full(m, sigma_global, dtype=np.float64)
    return SLStarParams(
        p0=p0, virtual_targets=vt,
        delta_t=delta_t, mu=mu_arr, sigma=sigma_arr, phi=phi,
    )


def iterative_target_refinement(
    plan: SLStarParams,
    contour: np.ndarray,
    dom_indices: list[int],
    max_iter: int = 10,
    tol_px: float = 1.0,
    damping: float = 1.0,
) -> tuple[SLStarParams, float, int]:
    """
    Iteratively adjusts virtual_targets to minimize ||c_j - contour[z_j]|| MSE.
    Returns (refined_params, final_mse_px, iters_used).
    """
    target_pts = contour[dom_indices]   # [m+1, 2]
    cur = SLStarParams(
        p0=plan.p0.copy(),
        virtual_targets=plan.virtual_targets.copy(),
        delta_t=plan.delta_t.copy(),
        mu=plan.mu.copy(),
        sigma=plan.sigma.copy(),
        phi=plan.phi.copy(),
    )

    last_mse = float("inf")
    for it in range(max_iter):
        ev = evaluate_trajectory(cur)
        cps = critical_points(cur, eval_result=ev)  # [m+1, 2]
        diff = target_pts - cps                     # [m+1, 2]
        mse = float(np.mean(np.sum(diff * diff, axis=1)))
        if math.sqrt(mse) < tol_px:
            return cur, math.sqrt(mse), it
        # Move virtual targets (skip p0 which is fixed to dom_pts[0])
        # cps[0] corresponds to p0 = plan.p0, so we don't shift p0.
        shift = damping * diff[1:]    # [m, 2]
        cur.virtual_targets += shift
        last_mse = math.sqrt(mse)
    return cur, last_mse, max_iter
