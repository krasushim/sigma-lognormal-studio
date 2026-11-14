"""
Differentiable PyTorch port of SL* trajectory model (Berio 2015 + 2025).

Uses the analytic position form from Berio 2025 Eq.4 so the entire forward pass
is closed-form (no numerical integration), making gradients clean.

Optimizable parameters per stroke group:
  targets  : [m, 2]  virtual targets (p_1 .. p_m)
  log_dt   : [m-1]   log(Δt_j) for j=1..m-1   (Δt_0 ≡ 0 per Berio 2015 §3.2)
  delta    : [m]     δ_j = 2 φ_j              (turning angle)

Fixed:
  p0          : [2]
  μ_global, σ_global  (Berio 2015 §5 global-constant assumption)
"""
from __future__ import annotations

import math

import torch
import torch.nn as nn


EPS = 1e-6
DELTA_EPS = 1e-4


class DiffSLStar(nn.Module):
    def __init__(
        self,
        p0: torch.Tensor,           # [2]
        targets_init: torch.Tensor, # [m, 2]
        delta_t_init: torch.Tensor, # [m]  (delta_t_init[0] ≡ 0)
        delta_init: torch.Tensor,   # [m]
        mu_global: float,
        sigma_global: float,
        dt_min: float,
        dt_max: float,
        delta_clamp: float = math.pi * 1.4,  # ±0.7π on φ = ±1.4π on δ
    ):
        super().__init__()
        self.register_buffer("p0", p0.detach().float())
        self.targets = nn.Parameter(targets_init.detach().float().clone())

        m = targets_init.shape[0]
        self.m = m
        dt_rest = delta_t_init[1:].clamp(min=max(dt_min, 1e-3), max=dt_max)
        self.log_dt = nn.Parameter(torch.log(dt_rest).float().clone())
        self.delta = nn.Parameter(delta_init.detach().float().clone())

        self.mu_global = float(mu_global)
        self.sigma_global = float(sigma_global)
        self.dt_min = float(dt_min)
        self.dt_max = float(dt_max)
        self.delta_clamp = float(delta_clamp)

    # ------------------------------------------------------------------
    # Clamping helpers (project parameters after each optimizer step)
    # ------------------------------------------------------------------

    @torch.no_grad()
    def project_(self) -> None:
        """Clamp raw parameters to feasibility region in-place."""
        # Δt ∈ [dt_min, dt_max]  →  log_dt ∈ [ln dt_min, ln dt_max]
        self.log_dt.clamp_(min=math.log(self.dt_min), max=math.log(self.dt_max))
        self.delta.clamp_(min=-self.delta_clamp, max=self.delta_clamp)

    # ------------------------------------------------------------------
    # Forward evaluation
    # ------------------------------------------------------------------

    def _effective_dt(self) -> torch.Tensor:
        """[m] with dt[0]=0, rest from log_dt."""
        dt_rest = torch.exp(self.log_dt)
        z = torch.zeros(1, dtype=dt_rest.dtype, device=dt_rest.device)
        return torch.cat([z, dt_rest], dim=0)

    def t_end(self) -> torch.Tensor:
        dt = self._effective_dt()
        t0 = torch.cumsum(dt * math.sinh(3.0 * self.sigma_global), dim=0)
        return t0[-1] + math.exp(self.mu_global + 3.0 * self.sigma_global)

    def forward(self, t: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """t: [T] → pos [T,2], vel [T,2]"""
        dt = self._effective_dt()
        sigma = self.sigma_global
        mu = self.mu_global

        t0 = torch.cumsum(dt * math.sinh(3.0 * sigma), dim=0)  # [m]

        all_targets = torch.cat([self.p0.unsqueeze(0), self.targets], dim=0)   # [m+1, 2]
        diffs = all_targets[1:] - all_targets[:-1]                              # [m, 2]
        lengths = torch.norm(diffs, dim=1) + EPS                                # [m]
        theta = torch.atan2(diffs[:, 1], diffs[:, 0])                           # [m]
        delta = self.delta                                                      # [m]

        # D = lengths / sinc(δ/(2π))  with normalized sinc: sinc(x)=sin(πx)/(πx)
        sinc_arg = delta / (2.0 * math.pi)
        pi_x = math.pi * sinc_arg
        sinc_val = torch.where(
            pi_x.abs() < 1e-6,
            torch.ones_like(pi_x),
            torch.sin(pi_x) / pi_x,
        )
        # 保護: sinc_val が 0 近傍で爆発しないよう小さな下限
        sinc_val = torch.where(
            sinc_val.abs() < 1e-3,
            torch.sign(sinc_val + 1e-9) * 1e-3,
            sinc_val,
        )
        D = lengths / sinc_val                                                  # [m]

        # Per-stroke time-dependent terms
        phi = torch.clamp(t.unsqueeze(0) - t0.unsqueeze(1), min=0.0) + EPS      # [m, T]
        log_phi = torch.log(phi)
        z = (log_phi - mu) / (sigma * math.sqrt(2.0))
        w = 0.5 * (1.0 + torch.erf(z))                                          # [m, T]
        # Lognormal speed Λ(t)
        lognormal = (
            1.0 / (sigma * math.sqrt(2.0 * math.pi) * phi)
            * torch.exp(-0.5 * ((log_phi - mu) / sigma) ** 2)
        )

        # phase(t) = θ − δ/2 + δ · w
        theta_b = theta.unsqueeze(1)
        delta_b = delta.unsqueeze(1)
        phase_0 = theta_b - delta_b * 0.5
        phase = phase_0 + delta_b * w                                           # [m, T]

        # 位置 (Berio 2025 Eq.4)
        arc_mask = (delta_b.abs() > DELTA_EPS).float()
        D_b = D.unsqueeze(1)
        safe_delta = torch.where(
            delta_b.abs() < DELTA_EPS,
            torch.full_like(delta_b, DELTA_EPS),
            delta_b,
        )
        arc_x = (D_b / safe_delta) * (torch.sin(phase) - torch.sin(phase_0))
        arc_y = (D_b / safe_delta) * (-torch.cos(phase) + torch.cos(phase_0))
        line_x = D_b * torch.cos(theta_b) * w
        line_y = D_b * torch.sin(theta_b) * w
        x_j = arc_mask * arc_x + (1.0 - arc_mask) * line_x
        y_j = arc_mask * arc_y + (1.0 - arc_mask) * line_y

        # 速度 (magnitude+direction)
        vx_j = D_b * lognormal * torch.cos(phase)
        vy_j = D_b * lognormal * torch.sin(phase)

        pos_x = self.p0[0] + x_j.sum(dim=0)
        pos_y = self.p0[1] + y_j.sum(dim=0)
        vel_x = vx_j.sum(dim=0)
        vel_y = vy_j.sum(dim=0)
        pos = torch.stack([pos_x, pos_y], dim=-1)
        vel = torch.stack([vel_x, vel_y], dim=-1)
        return pos, vel

    # ------------------------------------------------------------------
    # Export back to numpy-friendly form
    # ------------------------------------------------------------------

    @torch.no_grad()
    def export(self) -> dict:
        return {
            "p0": self.p0.cpu().numpy().copy(),
            "targets": self.targets.cpu().numpy().copy(),
            "delta_t": self._effective_dt().cpu().numpy().copy(),
            "delta": self.delta.cpu().numpy().copy(),
            "mu_global": self.mu_global,
            "sigma_global": self.sigma_global,
        }
