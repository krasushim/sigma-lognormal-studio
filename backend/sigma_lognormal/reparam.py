"""
Reparameterization between {Ac, T, Δt} and {σ, μ, t0}.

Reference: Berio 2025 Eq.5
    σ_i  = sqrt(-ln(1 - Ac_i))
    μ_i  = 3σ_i - ln((-1 + exp(6σ_i)) / T_i)
    t0_i = t0_{i-1} + Δt_i · sinh(3σ_i)
"""
from __future__ import annotations

import math

import numpy as np
import torch


EPS = 1e-8


# ---------------------------------------------------------------------------
# NumPy versions (for analysis pipeline / fitting)
# ---------------------------------------------------------------------------

def ac_to_sigma(ac: np.ndarray) -> np.ndarray:
    ac_clipped = np.clip(ac, EPS, 1.0 - EPS)
    return np.sqrt(-np.log(1.0 - ac_clipped))


def compute_mu(sigma: np.ndarray, T: np.ndarray) -> np.ndarray:
    """μ_i = 3σ_i - ln((-1 + exp(6σ_i)) / T_i)"""
    T_safe = np.clip(T, EPS, None)
    return 3.0 * sigma - np.log((np.expm1(6.0 * sigma)) / T_safe)


def compute_t0(sigma: np.ndarray, delta_t: np.ndarray) -> np.ndarray:
    """t0_i = t0_{i-1} + Δt_i · sinh(3σ_i) (with t0_0 = 0)."""
    offsets = delta_t * np.sinh(3.0 * sigma)
    t0 = np.cumsum(offsets)
    # Shift so t0_0 = 0 (first stroke starts at time 0).
    # Interpretation: delta_t[0] = time before the first stroke starts. Here we assume it's 0.
    # To match the paper's "Δt_i relative to previous", we keep cumsum starting from Δt[0]·sinh(3σ[0]).
    return t0


def ac_t_dt_to_sigma_mu_t0(
    ac: np.ndarray, T: np.ndarray, delta_t: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (sigma, mu, t0)."""
    sigma = ac_to_sigma(ac)
    mu = compute_mu(sigma, T)
    t0 = compute_t0(sigma, delta_t)
    return sigma, mu, t0


def compute_t_end(mu: np.ndarray, sigma: np.ndarray, t0: np.ndarray) -> float:
    """T_end = t0_m + exp(μ_m + 3σ_m) (Berio 2025 Eq.6)."""
    return float(t0[-1] + math.exp(mu[-1] + 3.0 * sigma[-1]))


# ---------------------------------------------------------------------------
# Torch versions (for differentiable pipeline)
# ---------------------------------------------------------------------------

def torch_ac_to_sigma(ac: torch.Tensor) -> torch.Tensor:
    ac_clipped = ac.clamp(EPS, 1.0 - EPS)
    return torch.sqrt(-torch.log(1.0 - ac_clipped))


def torch_compute_mu(sigma: torch.Tensor, T: torch.Tensor) -> torch.Tensor:
    T_safe = T.clamp(min=EPS)
    return 3.0 * sigma - torch.log(torch.expm1(6.0 * sigma) / T_safe)


def torch_compute_t0(sigma: torch.Tensor, delta_t: torch.Tensor) -> torch.Tensor:
    offsets = delta_t * torch.sinh(3.0 * sigma)
    return torch.cumsum(offsets, dim=0)


def torch_t_end(mu: torch.Tensor, sigma: torch.Tensor, t0: torch.Tensor) -> torch.Tensor:
    return t0[-1] + torch.exp(mu[-1] + 3.0 * sigma[-1])
