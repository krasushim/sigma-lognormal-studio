"""
Sigma-Lognormal (SL*) trajectory model.
Uses analytic position form from Berio 2025 Eq.4, combined with SL* virtual-target parameterization
from Berio 2015 Eq.6-10.

Parameters per stroke j (SL*):
  Δt_j   : time offset relative to previous stroke
  μ_j, σ_j : lognormal shape (can be per-stroke or global constants)
  φ_j    : half turning angle  (→ δ_j = 2 φ_j)

Action plan:
  p0                    : starting point [2]
  virtual_targets [m,2] : targets w_1, ..., w_m

Eqs (summary):
  φ(t)   = ReLU(t - t0_j) + ε
  w_j(t) = 0.5 (1 + erf((ln φ - μ_j) / (σ_j √2)))               ∈ [0,1]
  θ_j    = atan2(Δw_y, Δw_x)              Δw = w_j - w_{j-1}
  δ_j    = 2 φ_j
  D_j    = ||Δw|| / sinc(δ_j / (2π))       (sinc(x) = sin(πx)/(πx))
  phase(t) = θ_j - δ_j/2 + δ_j · w_j(t)
  # Circular arc case (|δ_j| > ε)
  x_j(t) = (D_j/δ_j) [ sin(phase) - sin(phase_0), -cos(phase) + cos(phase_0) ]
  # Straight line case (|δ_j| ≤ ε)
  x_j(t) = D_j [cos θ_j, sin θ_j] · w_j(t)
  p(t)  = p0 + Σ_j x_j(t)
  v(t)  = Σ_j Λ_j(t) · D_j · [cos phase(t), sin phase(t)]
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


EPS = 1e-6
DELTA_EPS = 1e-4   # threshold for straight-line fallback


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------

@dataclass
class SLStarParams:
    """Immutable container for SL* params (NumPy, per-stroke arrays)."""
    p0: np.ndarray                # [2]
    virtual_targets: np.ndarray   # [m, 2]
    delta_t: np.ndarray           # [m]
    mu: np.ndarray                # [m]  (log-delay)
    sigma: np.ndarray             # [m]  (log-response, aliased σ)
    phi: np.ndarray               # [m]  (half turning angle, radians)

    def m(self) -> int:
        return int(self.virtual_targets.shape[0])

    def all_targets(self) -> np.ndarray:
        """[m+1, 2] with p0 as index 0."""
        return np.vstack([self.p0[None, :], self.virtual_targets])


# ---------------------------------------------------------------------------
# Per-stroke geometry helpers
# ---------------------------------------------------------------------------

def sinc_norm(x: np.ndarray) -> np.ndarray:
    """Normalized sinc: sinc(x) = sin(πx) / (πx), with sinc(0)=1."""
    return np.sinc(x)


def compute_theta_delta_D(params: SLStarParams) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (theta [m], delta [m], D [m]) for each stroke."""
    all_pts = params.all_targets()  # [m+1, 2]
    diffs = all_pts[1:] - all_pts[:-1]  # [m, 2]
    lengths = np.linalg.norm(diffs, axis=1)  # [m]
    theta = np.arctan2(diffs[:, 1], diffs[:, 0])
    delta = 2.0 * params.phi
    # D_j = ||Δw|| / sinc(δ / (2π))
    sinc_val = sinc_norm(delta / (2.0 * math.pi))
    sinc_val = np.where(np.abs(sinc_val) < EPS, 1.0, sinc_val)
    D = lengths / sinc_val
    return theta, delta, D


def compute_t0(sigma: np.ndarray, delta_t: np.ndarray) -> np.ndarray:
    """
    t0_i = t0_{i-1} + Δt_i · sinh(3σ_i)  (Berio 2025 Eq.5)
    with t0_1 convention = Δt_1 · sinh(3σ_1).
    """
    offsets = delta_t * np.sinh(3.0 * sigma)
    return np.cumsum(offsets)


def compute_t_end(mu: np.ndarray, sigma: np.ndarray, t0: np.ndarray) -> float:
    return float(t0[-1] + math.exp(mu[-1] + 3.0 * sigma[-1]))


def rescale_params_in_time(
    params: SLStarParams, alpha: float,
) -> None:
    """
    Time-scaling transform: new_t = α · old_t ⇒
      Δt_j    → α · Δt_j
      μ_j     → μ_j + ln α     (σ, φ, virtual_targets, p0 は不変)
    形状 (軌跡) はそのまま、速度プロファイルだけが 1/α 倍に速くなる。
    """
    if alpha <= 0:
        raise ValueError("alpha must be > 0")
    params.delta_t = params.delta_t * alpha
    params.mu = params.mu + math.log(alpha)


def max_delta_t_for_overlap(mu: float, sigma: float, safety_factor: float = 0.7) -> float:
    """
    隣接 lognormal の間で速度が (ほぼ) ゼロに落ちない最大 Δt を返す。

    Δt · sinh(3σ) ≤ exp(μ + 3σ)  のとき、
      t0_{j+1} ≤ t0_j + exp(μ + 3σ)  (= ストローク j の 3σ 実効終端)
    となりストロークがオーバーラップする。

    valley を十分浅くするために safety_factor (< 1) を掛ける。
    safety_factor=1.0 で「ちょうど触れる」、0.7 程度で実用的な overlap。
    """
    if sigma <= 0.0 or safety_factor <= 0.0:
        return float("inf")
    gap_limit = math.exp(mu + 3.0 * sigma) / math.sinh(3.0 * sigma)
    return float(safety_factor * gap_limit)


# ---------------------------------------------------------------------------
# Lognormal speed & direction
# ---------------------------------------------------------------------------

def _phi_of(t: np.ndarray, t0: float) -> np.ndarray:
    return np.maximum(t - t0, 0.0) + EPS


def lognormal_speed(
    t: np.ndarray, mu: float, sigma: float, t0: float
) -> np.ndarray:
    """Λ_j(t) ≥ 0."""
    phi = _phi_of(t, t0)
    inv = 1.0 / (sigma * math.sqrt(2.0 * math.pi) * phi)
    expo = -0.5 * ((np.log(phi) - mu) / sigma) ** 2
    return inv * np.exp(expo)


def lognormal_cdf(
    t: np.ndarray, mu: float, sigma: float, t0: float
) -> np.ndarray:
    """w_j(t) = 0.5 (1 + erf(...)) ∈ [0, 1]."""
    from scipy.special import erf
    phi = _phi_of(t, t0)
    return 0.5 * (1.0 + erf((np.log(phi) - mu) / (sigma * math.sqrt(2.0))))


# ---------------------------------------------------------------------------
# Analytic stroke position (Berio 2025 Eq.4)
# ---------------------------------------------------------------------------

def stroke_position(
    t: np.ndarray,
    theta: float, delta: float, D: float,
    mu: float, sigma: float, t0: float,
) -> np.ndarray:
    """Returns x_j(t) [T, 2] — displacement of stroke j at times t."""
    w = lognormal_cdf(t, mu, sigma, t0)  # [T]
    phase_0 = theta - delta / 2.0
    phase = phase_0 + delta * w

    out = np.zeros((t.shape[0], 2), dtype=np.float64)
    if abs(delta) > DELTA_EPS:
        inv_delta = 1.0 / delta
        out[:, 0] = (D * inv_delta) * (np.sin(phase) - math.sin(phase_0))
        out[:, 1] = (D * inv_delta) * (-np.cos(phase) + math.cos(phase_0))
    else:
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        out[:, 0] = D * cos_t * w
        out[:, 1] = D * sin_t * w
    return out


def stroke_velocity(
    t: np.ndarray,
    theta: float, delta: float, D: float,
    mu: float, sigma: float, t0: float,
) -> np.ndarray:
    """Returns ẋ_j(t) [T, 2]."""
    w = lognormal_cdf(t, mu, sigma, t0)
    Lambda = lognormal_speed(t, mu, sigma, t0)
    phase = theta - delta / 2.0 + delta * w
    out = np.stack([np.cos(phase), np.sin(phase)], axis=-1)  # [T, 2]
    return (D * Lambda)[:, None] * out


# ---------------------------------------------------------------------------
# Full trajectory
# ---------------------------------------------------------------------------

def evaluate_trajectory(
    params: SLStarParams,
    n_samples: int | None = None,
    trim_zero_speed: bool = True,
    trim_frac: float = 0.02,
) -> dict:
    """
    Samples the SL* trajectory at `n_samples` uniformly spaced times in [t_start, t_end_trim].
    If n_samples is None, uses 5*m (paper default) or 200, whichever larger.

    trim_zero_speed=True: lognormal 尾 (先頭/末尾) で speed < trim_frac * peak_speed の
      区間を取り除き、可視時系列に zero-speed 区間が含まれないようにする。
    """
    theta, delta, D = compute_theta_delta_D(params)
    t0 = compute_t0(params.sigma, params.delta_t)
    T_end_raw = compute_t_end(params.mu, params.sigma, t0)
    n_samples = n_samples if n_samples is not None else max(5 * params.m(), 200)

    # --- トリムのためのピーク推定用オーバーサンプル ---
    if trim_zero_speed:
        # 粗いサンプルから speed を計算し、lead/tail カットオフを決定
        n_coarse = max(200, n_samples)
        t_coarse = np.linspace(0.0, T_end_raw, n_coarse)
        vel_coarse = np.zeros((n_coarse, 2), dtype=np.float64)
        for j in range(params.m()):
            vel_coarse += stroke_velocity(
                t_coarse, theta[j], delta[j], D[j],
                params.mu[j], params.sigma[j], t0[j],
            )
        sp_coarse = np.linalg.norm(vel_coarse, axis=1)
        sp_peak = float(np.max(sp_coarse)) if sp_coarse.size else 0.0
        threshold = max(sp_peak * trim_frac, 1e-6)
        above = np.where(sp_coarse >= threshold)[0]
        if len(above) >= 2:
            t_start = float(t_coarse[above[0]])
            t_end = float(t_coarse[above[-1]])
        else:
            t_start, t_end = 0.0, T_end_raw
    else:
        t_start, t_end = 0.0, T_end_raw

    t = np.linspace(t_start, t_end, n_samples)
    pos = np.tile(params.p0, (t.shape[0], 1)).astype(np.float64)
    vel = np.zeros_like(pos)
    for j in range(params.m()):
        pos += stroke_position(
            t, theta[j], delta[j], D[j],
            params.mu[j], params.sigma[j], t0[j],
        )
        vel += stroke_velocity(
            t, theta[j], delta[j], D[j],
            params.mu[j], params.sigma[j], t0[j],
        )
    speed = np.linalg.norm(vel, axis=1)
    return {
        "t": t - t_start,   # 時間軸は可視範囲の先頭を 0 にシフト
        "x": pos[:, 0],
        "y": pos[:, 1],
        "vx": vel[:, 0],
        "vy": vel[:, 1],
        "speed": speed,
        "t_end": float(t_end - t_start),
        "theta": theta,
        "delta": delta,
        "D": D,
        "t0": t0,
    }


def critical_points(
    params: SLStarParams,
    eval_result: dict | None = None,
) -> np.ndarray:
    """
    Finds 'critical points' c_j along the trajectory where consecutive lognormal
    speeds intersect (v_j(t) == v_{j+1}(t)). For j = 0 (first), use start; for
    j = m (last), use end. Returns [m+1, 2] in world coordinates. Used by
    iterative target refinement (Berio 2015 Eq.22).
    """
    ev = eval_result if eval_result is not None else evaluate_trajectory(params)
    t = ev["t"]
    m = params.m()

    # Per-stroke speed profiles Λ_j at sample times.
    speeds = np.zeros((m, t.shape[0]))
    for j in range(m):
        speeds[j] = lognormal_speed(t, params.mu[j], params.sigma[j], ev["t0"][j])

    pos_xy = np.stack([ev["x"], ev["y"]], axis=-1)
    cps = np.zeros((m + 1, 2))
    cps[0] = params.p0
    cps[-1] = pos_xy[-1]
    for j in range(m - 1):
        # Find the earliest t where speeds[j+1] >= speeds[j] (after the peak of j).
        diff = speeds[j + 1] - speeds[j]
        # Find sign change from negative to positive (speeds[j] was dominant, now j+1 overtakes)
        sign = np.sign(diff)
        changes = np.where(np.diff(sign) > 0)[0]
        if len(changes) > 0:
            idx = int(changes[0])
        else:
            # Fallback: take peak of stroke j+1 or midpoint
            idx = int(np.argmax(speeds[j + 1]))
        cps[j + 1] = pos_xy[idx]
    return cps
