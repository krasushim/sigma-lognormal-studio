"""
Differentiable refitting — Berio 2025 §IV.

Given initial SL* params (from the analytic pipeline) and the target ink mask,
jointly optimize all stroke groups' virtual targets, Δt, δ against
  L = λ_annealed · T_end + L_I,
  L_I = Σ_s MSE(I_s, Î_s)     (multi-scale)

Uses Adam + cosine annealing + linear λ warmup (30 steps), matching the paper.
"""
from __future__ import annotations

import logging
import math
from typing import Callable, Optional

import cv2
import numpy as np
import torch

from .diff_model import DiffSLStar
from .diff_raster import soft_raster_polyline, multiscale_mse
from .sl_model import SLStarParams

logger = logging.getLogger(__name__)

LAMBDA_WARMUP_STEPS = 30


def _build_model(
    gp: SLStarParams,
    scale: float,
    mu_global: float,
    sigma_global: float,
    dt_min: float,
    dt_max: float,
    device: torch.device,
) -> DiffSLStar:
    p0 = torch.tensor(gp.p0 * scale, dtype=torch.float32, device=device)
    tgt = torch.tensor(gp.virtual_targets * scale, dtype=torch.float32, device=device)
    dt = torch.tensor(gp.delta_t, dtype=torch.float32, device=device)
    delta = torch.tensor(2.0 * gp.phi, dtype=torch.float32, device=device)  # δ = 2φ
    return DiffSLStar(
        p0, tgt, dt, delta,
        mu_global=mu_global, sigma_global=sigma_global,
        dt_min=dt_min, dt_max=dt_max,
    ).to(device)


def differentiable_refit(
    groups: list[SLStarParams],
    target_mask: np.ndarray,
    *,
    mu_global: float,
    sigma_global: float,
    dt_min: float = 0.02,
    dt_max: float = 0.35,
    n_iter: int = 200,
    lr: float = 5e-3,
    lambda_smooth: float = 0.01,
    work_size: int = 192,
    stroke_radius_world_px: float = 5.0,
    sharpness: float = 0.8,
    n_samples_per_group: int = 120,
    device: Optional[torch.device] = None,
    progress_callback: Optional[Callable[[float, float], None]] = None,
) -> tuple[list[SLStarParams], dict]:
    """
    すべての stroke group をマスクに対して同時フィット。
    戻り値: (refined_groups, info)  info = {"initial_loss", "final_loss", "iterations"}
    """
    device = device or torch.device("cpu")

    H_full, W_full = target_mask.shape[:2]
    scale = min(1.0, work_size / max(H_full, W_full))
    H = max(int(round(H_full * scale)), 16)
    W = max(int(round(W_full * scale)), 16)

    # Target mask をダウンサンプルして 0..1 float 化
    tgt = (target_mask > 127).astype(np.float32) if target_mask.dtype == np.uint8 else \
          np.clip(target_mask, 0.0, 1.0).astype(np.float32)
    tgt_s = cv2.resize(tgt, (W, H), interpolation=cv2.INTER_AREA)
    target_t = torch.tensor(tgt_s, dtype=torch.float32, device=device)

    # レンダラの半径 (work px)
    r_work = max(stroke_radius_world_px * scale, 1.0)

    # 各 group の微分可能モデルを構築
    models: list[DiffSLStar] = [
        _build_model(gp, scale, mu_global, sigma_global, dt_min, dt_max, device)
        for gp in groups if gp.m() >= 1
    ]
    if not models:
        return groups, {"initial_loss": 0.0, "final_loss": 0.0, "iterations": 0}

    params = [p for m in models for p in m.parameters()]
    optimizer = torch.optim.Adam(params, lr=lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(n_iter, 1), eta_min=lr * 0.01,
    )

    def _compute_loss(step: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Returns (loss, L_I). L_I is the image term alone for logging."""
        comps: list[torch.Tensor] = []
        t_end_total = torch.zeros((), device=device)
        for model in models:
            T_end = model.t_end()
            t_end_total = t_end_total + T_end
            # 時刻サンプル — detach() で T_end に gradient 経由しないようにする
            t = torch.linspace(0.0, T_end.detach().item(), n_samples_per_group, device=device)
            pos, _ = model(t)
            comps.append(soft_raster_polyline(pos, r_work, H, W, sharpness))
        # 全 group の soft-union
        stacked = torch.stack(comps, dim=0).clamp(1e-7, 1.0 - 1e-7)
        neg_log = torch.log(1.0 - stacked).sum(dim=0)
        total_render = (1.0 - torch.exp(neg_log)).clamp(0.0, 1.0)

        L_I = multiscale_mse(total_render, target_t)
        lambda_annealed = lambda_smooth * min(step / max(LAMBDA_WARMUP_STEPS, 1), 1.0)
        loss = lambda_annealed * t_end_total + L_I
        return loss, L_I

    # 初期損失 (参考)
    with torch.no_grad():
        init_loss, init_LI = _compute_loss(0)
        init_LI_val = float(init_LI.item())

    last_loss = float("inf")
    last_LI = init_LI_val
    for step in range(n_iter):
        optimizer.zero_grad()
        loss, L_I = _compute_loss(step)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, max_norm=10.0)
        optimizer.step()
        scheduler.step()
        for m in models:
            m.project_()
        last_loss = float(loss.item())
        last_LI = float(L_I.item())
        if progress_callback is not None and step % 10 == 0:
            try:
                progress_callback(step / n_iter, last_loss)
            except Exception:
                pass

    logger.info(
        "diff_refit: %d iters, L_I %.5f → %.5f", n_iter, init_LI_val, last_LI,
    )

    # Export & スケール戻し
    refined: list[SLStarParams] = []
    model_idx = 0
    for gp in groups:
        if gp.m() < 1:
            refined.append(gp)
            continue
        exp = models[model_idx].export()
        model_idx += 1
        new_p0 = exp["p0"] / scale
        new_targets = exp["targets"] / scale
        m = int(new_targets.shape[0])
        refined.append(SLStarParams(
            p0=new_p0,
            virtual_targets=new_targets,
            delta_t=exp["delta_t"],
            mu=np.full(m, exp["mu_global"], dtype=np.float64),
            sigma=np.full(m, exp["sigma_global"], dtype=np.float64),
            phi=exp["delta"] / 2.0,
        ))

    return refined, {
        "initial_LI": init_LI_val,
        "final_LI": last_LI,
        "final_loss": last_loss,
        "iterations": n_iter,
    }
