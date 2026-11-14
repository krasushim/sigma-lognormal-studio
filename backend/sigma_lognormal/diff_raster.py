"""
Pure-PyTorch differentiable soft raster of polylines (thick line tube).
Used as the image cost backbone for Berio 2025 §IV loss.

  α(x) = sigmoid((r − SDF(x)) · β)
  comp = 1 − Π_k (1 − α_k)   (soft union over segments)
"""
from __future__ import annotations

import torch
import torch.nn.functional as F


def soft_raster_polyline(
    pos: torch.Tensor,      # [T, 2]  (x, y) in work-size coords
    radius: float,          # half-width in work px
    H: int,
    W: int,
    sharpness: float = 0.8,
) -> torch.Tensor:
    """
    `pos` を結ぶ折れ線を太さ 2·radius で HxW キャンバスにソフトラスタ化。
    戻り値 α [H, W] ∈ [0, 1]。
    """
    if pos.shape[0] < 2:
        return torch.zeros(H, W, device=pos.device, dtype=pos.dtype)
    device = pos.device
    dtype = pos.dtype

    yy, xx = torch.meshgrid(
        torch.arange(H, device=device, dtype=dtype),
        torch.arange(W, device=device, dtype=dtype),
        indexing="ij",
    )
    pixels = torch.stack([xx.reshape(-1), yy.reshape(-1)], dim=-1)  # [P, 2]

    a = pos[:-1]  # [K, 2]
    b = pos[1:]   # [K, 2]
    d = b - a
    d_len_sq = (d * d).sum(-1).clamp(min=1e-9)   # [K]

    pa = pixels.unsqueeze(0) - a.unsqueeze(1)    # [K, P, 2]
    t = (pa * d.unsqueeze(1)).sum(-1) / d_len_sq.unsqueeze(1)  # [K, P]
    t = t.clamp(0.0, 1.0)
    foot = a.unsqueeze(1) + t.unsqueeze(-1) * d.unsqueeze(1)   # [K, P, 2]
    sdf = (pixels.unsqueeze(0) - foot).norm(dim=-1)            # [K, P]

    alpha = torch.sigmoid((radius - sdf) * sharpness)          # [K, P]
    # 安定のため clamp
    alpha = alpha.clamp(1e-7, 1.0 - 1e-7)
    neg_log = torch.log(1.0 - alpha).sum(dim=0)                # [P]
    comp = (1.0 - torch.exp(neg_log)).clamp(0.0, 1.0)
    return comp.view(H, W)


def multiscale_mse(
    rendered: torch.Tensor,
    target: torch.Tensor,
    scales: tuple[float, ...] = (1.0, 0.5, 0.25),
) -> torch.Tensor:
    """
    Berio 2025 §IV.A Eq.7 — multi-scale MSE.
    rendered, target are [H, W] float in [0, 1].
    """
    r = rendered.unsqueeze(0).unsqueeze(0)
    t = target.unsqueeze(0).unsqueeze(0)
    terms = []
    for s in scales:
        if abs(s - 1.0) < 1e-6:
            r_s, t_s = r, t
        else:
            nh = max(int(round(rendered.shape[0] * s)), 4)
            nw = max(int(round(rendered.shape[1] * s)), 4)
            r_s = F.interpolate(r, size=(nh, nw), mode="bilinear", align_corners=False)
            t_s = F.interpolate(t, size=(nh, nw), mode="bilinear", align_corners=False)
        terms.append(F.mse_loss(r_s, t_s))
    return sum(terms) / len(terms)
