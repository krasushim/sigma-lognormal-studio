"""MPC-based stroke estimation from GML trajectory data.

Converts raw stroke points (x_px, y_px, t_s) into smooth MPC trajectories
parameterised by Gaussian via-points.  The mathematics follow:

    Berio et al. "Generating Graffiti Tag Letterforms with
    Model Predictive Control" GI 2017

Coordinates flow:
  1. Input pixels normalised to [0, 1] before running MPC (preserves default
     sigma parameters).
  2. Outputs are scaled back to pixel space for the API response.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import reduce
from typing import Callable, Optional

import numpy as np
from numpy.linalg import inv, pinv
from scipy.spatial.distance import cdist

mul = lambda mats: reduce(np.dot, mats)  # noqa: E731

# ── Dynamic system ────────────────────────────────────────────────────────────

class DynSys:
    """Linear time-invariant system with Euler discretisation."""

    def __init__(self, order: int, dt: float = 0.01, dim: int = 2) -> None:
        self.order = order
        self.dim = dim
        self.dt = dt

        A1 = np.zeros((order, order))
        A1[:order-1, 1:order] = np.eye(order - 1)
        B1 = np.zeros((order, 1))
        B1[-1] = 1.

        A1 = np.eye(order) + A1 * dt
        B1 = B1 * dt

        self.A = np.kron(A1, np.eye(dim))
        self.B = np.kron(B1, np.eye(dim))
        self.C = np.kron(
            np.hstack([np.ones((1, 1)), np.zeros((1, order - 1))]),
            np.eye(dim),
        )

    def y(self, x: np.ndarray) -> np.ndarray:
        return self.C @ x

    def num_timesteps(self, duration: float) -> int:
        return max(2, int(duration / self.dt))

    def matrices(self) -> tuple[np.ndarray, np.ndarray]:
        return self.A, self.B


# ── Iterative (Riccati) MPC ───────────────────────────────────────────────────

def iterative_mpc(
    sys: DynSys,
    MuQ: np.ndarray,
    Q: np.ndarray,
    r: float,
) -> np.ndarray:
    A, B = sys.matrices()
    order, dim = sys.order, sys.dim
    n = MuQ.shape[1]
    cDim = order * dim

    R = np.eye(dim) * r
    P = np.zeros((cDim, cDim, n))
    P[:, :, -1] = Q[(n-1)*cDim:, (n-1)*cDim:]
    d = np.zeros((cDim, n))

    for i in range(n - 2, 0, -1):
        Qi = Q[i*cDim:(i+1)*cDim, i*cDim:(i+1)*cDim]
        Pi1 = P[:, :, i + 1]
        BPB_R_inv = inv(mul([B.T, Pi1, B]) + R)
        P[:, :, i] = (
            Qi - mul([A.T,
                      mul([Pi1, B, BPB_R_inv, B.T, Pi1]) - Pi1,
                      A])
        )
        d[:, i] = mul([
            A.T - mul([A.T, Pi1, B, BPB_R_inv, B.T]),
            mul([Pi1, A @ MuQ[:, i] - MuQ[:, i + 1]]) + d[:, i + 1],
        ])

    x0 = MuQ[:, 0].copy()
    x = np.zeros((cDim, n))
    u = np.zeros((dim, n))
    x_t = x0

    for i in range(n):
        x[:, i] = x_t
        p = P[:, :, i]
        mu = MuQ[:, i]
        G = mul([pinv(mul([B.T, p, B]) + R), B.T])
        K = G @ p @ A
        M_ff = -G @ (p @ (A @ mu - mu) + d[:, i])
        u[:, i] = K @ (mu - x_t) + M_ff
        x_t = A @ x_t + B @ u[:, i]

    return x


# ── Reference construction ────────────────────────────────────────────────────

def _via_points_uniform(m: int, n: int) -> tuple:
    a, b = 0.0, n - 1
    return range(m), [int(a + i * (b - a) / (m - 1)) for i in range(m)]


def _via_points_timed(m: int, n: int, t_norm: np.ndarray) -> tuple:
    T = [int(np.clip(tau * (n - 1), 0, n - 1)) for tau in t_norm]
    return range(m), T


def make_reference(
    Mu: np.ndarray,
    Sigma: np.ndarray,
    n: int,
    sys: DynSys,
    t_norm: Optional[np.ndarray] = None,
    end_weight: float = 1e10,
) -> tuple[np.ndarray, np.ndarray]:
    order, dim = sys.order, sys.dim
    cDim = order * dim
    muDim = Mu.shape[0]
    m = Mu.shape[1]

    Lambda = np.zeros((muDim, m * muDim))
    for i in range(m):
        Lambda[:, i*muDim:(i+1)*muDim] = np.linalg.inv(Sigma[:, :, i])

    Q = np.zeros((cDim * n, cDim * n))
    MuQ = np.zeros((cDim, n))

    if t_norm is not None:
        I, T = _via_points_timed(m, n, t_norm)
    else:
        I, T = _via_points_uniform(m, n)

    for i, t in zip(I, T):
        Q[t*cDim:t*cDim+muDim, t*cDim:t*cDim+muDim] = Lambda[:, i*muDim:(i+1)*muDim]
        MuQ[:muDim, t] = Mu[:, i]

    ind = T[-1]
    if end_weight > 0.0:
        for i in range(muDim, cDim):
            Q[ind*cDim+i, ind*cDim+i] = end_weight
    return MuQ, Q


def SHM_r(d: float, order: int, duration: float, m: int) -> float:
    """Control-cost r from simple-harmonic-motion amplitude d."""
    period = duration / max(m - 1, 1)
    omega = (2.0 * np.pi) / period
    return 1.0 / ((d * omega ** order) ** 2)


# ── Via-point selection ───────────────────────────────────────────────────────

def _arc_lengths(pts: np.ndarray) -> np.ndarray:
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segs)])


def resample_arclength(pts: np.ndarray, n: int) -> np.ndarray:
    arc = _arc_lengths(pts)
    total = arc[-1]
    if total < 1e-12:
        return np.tile(pts[0], (n, 1))
    target = np.linspace(0.0, total, n)
    return np.column_stack([
        np.interp(target, arc, pts[:, 0]),
        np.interp(target, arc, pts[:, 1]),
    ])


def _rdp_mask(pts: np.ndarray, eps: float) -> np.ndarray:
    n = len(pts)
    keep = np.ones(n, dtype=bool)
    stack = [(0, n - 1)]
    while stack:
        start, end = stack.pop()
        if end - start < 2:
            continue
        seg = pts[end] - pts[start]
        seg_len = np.linalg.norm(seg)
        sub = pts[start+1:end]
        if seg_len < 1e-12:
            dists = np.linalg.norm(sub - pts[start], axis=1)
        else:
            dists = np.abs(np.cross(seg, sub - pts[start])) / seg_len
        idx_rel = int(np.argmax(dists))
        if dists[idx_rel] > eps:
            stack.append((start, start + idx_rel + 1))
            stack.append((start + idx_rel + 1, end))
        else:
            keep[start+1:end] = False
    return keep


def select_via_points(pts: np.ndarray, n_via: int) -> np.ndarray:
    """RDP simplification → arc-length resampling to exactly n_via points."""
    if len(pts) <= n_via:
        return resample_arclength(pts, n_via)

    total = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    lo, hi = 0.0, total
    simplified = pts
    for _ in range(60):
        eps = (lo + hi) / 2.0
        mask = _rdp_mask(pts, eps)
        cand = pts[mask]
        if len(cand) > n_via:
            lo = eps
        else:
            simplified = cand
            if len(cand) == n_via:
                break
            hi = eps
    return resample_arclength(simplified, n_via)


# ── Gaussian estimation ───────────────────────────────────────────────────────

def estimate_stroke_gaussians(
    xy: np.ndarray,        # (N, 2) pixel coords
    t: np.ndarray,         # (N,) time in seconds
    n_via: int,
    sigma_min: float,
    sigma_speed_scale: float,
    end_sigma_scale: float = 0.3,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Estimate (Mu, Sigma, t_norm) in normalised [0,1] coordinate space.

    Parameters
    ----------
    xy               : (N, 2) stroke points (any coordinate space)
    t                : (N,) timestamps in seconds
    n_via            : number of via-points
    sigma_min        : minimum covariance eigenvalue
    sigma_speed_scale: scales normalised speed → Sigma magnitude
    end_sigma_scale  : tighter anchor at start/end

    Returns
    -------
    Mu     : (2, n_via)
    Sigma  : (2, 2, n_via)
    t_norm : (n_via,) normalised time in [0, 1]
    """
    duration = float(t[-1] - t[0])
    if duration < 1e-9:
        duration = 1.0

    via_xy = select_via_points(xy, n_via)          # (n_via, 2)
    arc = _arc_lengths(xy)
    total_arc = arc[-1]
    via_arc = np.linspace(0.0, total_arc, n_via)

    via_t = np.interp(via_arc, arc, t)
    t_norm = (via_t - t[0]) / duration

    # Speed
    dt_raw = np.diff(t)
    dt_raw = np.where(dt_raw < 1e-9, 1e-9, dt_raw)
    vel = np.diff(xy, axis=0) / dt_raw[:, None]
    speed_full = np.concatenate([[np.linalg.norm(vel[0])],
                                  np.linalg.norm(vel, axis=1)])

    # Curvature
    if len(xy) > 2:
        d1 = np.gradient(xy, axis=0)
        d2 = np.gradient(d1, axis=0)
        cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
        denom = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5 + 1e-12
        curvature = np.abs(cross) / denom
    else:
        curvature = np.zeros(len(xy))

    via_speed = np.interp(via_arc, arc, speed_full)
    via_curv  = np.interp(via_arc, arc, curvature)

    max_speed = via_speed.max() if via_speed.max() > 0 else 1.0
    max_curv  = via_curv.max()  if via_curv.max()  > 0 else 1.0
    norm_speed = via_speed / max_speed
    norm_curv  = via_curv  / max_curv

    Mu = via_xy.T  # (2, n_via)
    Sigma = np.zeros((2, 2, n_via))

    for i in range(n_via):
        corner = 1.0 / (1.0 + 8.0 * norm_curv[i])
        s = (sigma_min + sigma_speed_scale * norm_speed[i]) * corner
        s = max(s, sigma_min)

        if i == 0 or i == n_via - 1:
            s = max(s * end_sigma_scale, sigma_min)

        if i > 0:
            tangent = via_xy[i] - via_xy[i - 1]
        elif n_via > 1:
            tangent = via_xy[1] - via_xy[0]
        else:
            tangent = np.array([1.0, 0.0])

        tlen = np.linalg.norm(tangent)
        tangent = tangent / tlen if tlen > 1e-12 else np.array([1.0, 0.0])
        normal = np.array([-tangent[1], tangent[0]])

        R = np.column_stack([tangent, normal])
        S = np.diag([(s * 1.5) ** 2, (s * 0.5) ** 2])
        Sigma[:, :, i] = R @ S @ R.T

    return Mu, Sigma, t_norm


# ── Ellipse parameters for visualisation ─────────────────────────────────────

@dataclass
class EllipseParams:
    x: float
    y: float
    rx: float        # semi-axis along major direction (px)
    ry: float        # semi-axis along minor direction (px)
    angle_deg: float # rotation of major axis from +X (degrees)


def sigma_to_ellipse(
    sigma: np.ndarray,
    cx: float,
    cy: float,
    scale: float = 2.0,
) -> EllipseParams:
    """Convert a 2×2 covariance to SVG-ready ellipse parameters.

    scale=2 gives the 2-σ confidence ellipse.
    """
    vals, vecs = np.linalg.eigh(sigma)   # ascending eigenvalues
    vals = np.maximum(vals, 0.0)
    rx = float(np.sqrt(vals[1])) * scale
    ry = float(np.sqrt(vals[0])) * scale
    angle_deg = float(np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1])))
    return EllipseParams(x=cx, y=cy, rx=rx, ry=ry, angle_deg=angle_deg)


# ── Reconstruction error ─────────────────────────────────────────────────────

def chamfer_error_px(traj_px: np.ndarray, orig_px: np.ndarray) -> float:
    """Mean of per-original-point minimum distances to the MPC trajectory.

    traj_px : (T, 2)  MPC trajectory in pixel space
    orig_px : (N, 2)  original GML data points in pixel space

    Each original point contributes min_j ||orig[i] - traj[j]|| to the mean,
    so the metric penalises regions of the original stroke that the MPC
    trajectory does not pass through.
    """
    D = cdist(orig_px, traj_px)   # (N, T)
    return float(np.mean(D.min(axis=1)))


# ── Main pipeline function ────────────────────────────────────────────────────

@dataclass
class StrokeMpcResult:
    stroke_index: int
    # Via-point ellipses in pixel space
    ellipses: list[EllipseParams]
    # Downsampled trajectory in pixel space
    traj_x: list[float]
    traj_y: list[float]
    duration_s: float
    n_via: int
    order: int
    # Reconstruction quality
    reconstruction_error_px: float = 0.0
    converged: bool = False
    # n_via of the first iteration (= min_via for auto mode, = n_via for fixed)
    n_via_start: int = 0


def run_mpc_on_stroke(
    pts_px: np.ndarray,  # (N, 3) float: [x_px, y_px, t_s]
    screen_w: int,
    screen_h: int,
    n_via: int = 5,
    order: int = 4,
    dt: float = 0.005,
    stroke_index: int = 0,
    sigma_min: float = 5e-4,
    sigma_speed_scale: float = 0.02,
    ellipse_scale: float = 2.0,
    max_traj_pts: int = 400,
    icp_max_iter: int = 8,
    icp_alpha: float = 0.5,
) -> StrokeMpcResult:
    """Run the MPC pipeline on one GML stroke with ICP via-point refinement.

    Points are internally normalised to [0,1] and results are scaled back to
    pixel space.  After the initial MPC solve, ICP iterations shift each
    via-point toward the nearest original data point at its time slot,
    minimising the chamfer distance (mean per-data-point minimum distance to
    the MPC trajectory).  The best chamfer error across all ICP steps is
    reported in ``reconstruction_error_px``.
    """
    xy_px = pts_px[:, :2]
    t_s   = pts_px[:, 2].copy()
    for i in range(1, len(t_s)):
        if t_s[i] <= t_s[i - 1]:
            t_s[i] = t_s[i - 1] + 1e-4

    w, h = float(screen_w), float(screen_h)
    xy_norm = xy_px / np.array([[w, h]])

    Mu_n, Sigma_n, t_norm = estimate_stroke_gaussians(
        xy_norm, t_s, n_via,
        sigma_min=sigma_min,
        sigma_speed_scale=sigma_speed_scale,
    )

    duration = float(t_s[-1] - t_s[0])
    if duration < 0.05:
        duration = 0.15 * n_via

    sys_ = DynSys(order, dt=dt)
    n_steps = sys_.num_timesteps(duration)

    bbox_diag = np.linalg.norm(Mu_n.max(axis=1) - Mu_n.min(axis=1)) + 1e-6
    d_scale = max(bbox_diag * 0.20, sigma_min)
    r = SHM_r(d_scale, order, duration, n_via)

    # ICP loop: iteratively shift via-points toward nearest original data point
    # at each via-point's time slot, minimising chamfer distance.
    best_err   = float("inf")
    best_y_px  = None
    best_Mu_n  = Mu_n.copy()

    for _ in range(icp_max_iter):
        MuQ, Q = make_reference(Mu_n, Sigma_n, n_steps, sys_,
                                 t_norm=t_norm, end_weight=1e10)
        x      = iterative_mpc(sys_, MuQ, Q, r=r)
        y_norm = sys_.y(x)          # (2, n_steps)
        y_px_full = y_norm * np.array([[w], [h]])   # (2, n_steps)

        err = chamfer_error_px(y_px_full.T, xy_px)

        if err < best_err:
            best_err  = err
            best_y_px = y_px_full.copy()
            best_Mu_n = Mu_n.copy()

        # ICP update: move each via-point toward the nearest original point
        # relative to the current MPC position at that via-point's time slot.
        updated = False
        for i in range(n_via):
            t_idx = int(np.clip(t_norm[i] * (n_steps - 1), 0, n_steps - 1))
            mpc_pos_px = y_px_full[:, t_idx]
            nearest_idx = int(np.argmin(np.linalg.norm(xy_px - mpc_pos_px, axis=1)))
            nearest_norm = xy_px[nearest_idx] / np.array([w, h])
            delta = nearest_norm - Mu_n[:, i]
            if np.linalg.norm(delta) > 1e-6:
                Mu_n[:, i] += icp_alpha * delta
                updated = True

        if not updated:
            break

    Mu_n  = best_Mu_n
    y_px  = best_y_px if best_y_px is not None else y_px_full

    S_mat = np.array([[w, 0.0], [0.0, h]])
    Mu_px = Mu_n * np.array([[w], [h]])
    Sigma_px = np.zeros_like(Sigma_n)
    for i in range(n_via):
        Sigma_px[:, :, i] = S_mat @ Sigma_n[:, :, i] @ S_mat.T

    # Downsample for JSON response
    step = max(1, y_px.shape[1] // max_traj_pts)
    y_ds = y_px[:, ::step]

    ellipses = [
        sigma_to_ellipse(Sigma_px[:, :, i],
                         float(Mu_px[0, i]),
                         float(Mu_px[1, i]),
                         scale=ellipse_scale)
        for i in range(n_via)
    ]

    return StrokeMpcResult(
        stroke_index=stroke_index,
        ellipses=ellipses,
        traj_x=[float(v) for v in y_ds[0]],
        traj_y=[float(v) for v in y_ds[1]],
        duration_s=round(duration, 4),
        n_via=n_via,
        order=order,
        reconstruction_error_px=round(best_err, 3),
        converged=False,
        n_via_start=n_via,
    )


# ── Iterative n_via optimiser ─────────────────────────────────────────────────

def optimize_mpc_stroke(
    pts_px: np.ndarray,
    screen_w: int,
    screen_h: int,
    order: int = 4,
    dt: float = 0.005,
    tol_px: float = 3.0,
    min_via: int = 3,
    max_via: int = 15,
    stroke_index: int = 0,
    sigma_min: float = 5e-4,
    sigma_speed_scale: float = 0.02,
    ellipse_scale: float = 2.0,
    max_traj_pts: int = 400,
    progress_cb: Optional[Callable[[int, float], None]] = None,
) -> tuple[StrokeMpcResult, list[float]]:
    """Increase n_via until the MPC trajectory matches the original within tol_px.

    Returns the best result found and the per-iteration error history.

    Strategy
    --------
    1. Run MPC for n_via = min_via, min_via+1, …, max_via.
    2. Keep the result with the lowest error seen so far.
    3. Stop early once reconstruction_error_px ≤ tol_px.
    4. If the error stops improving for ``_PATIENCE`` consecutive steps,
       stop early to avoid wasted computation.
    """
    _PATIENCE = 3   # stop if error hasn't improved for this many steps
    best: Optional[StrokeMpcResult] = None
    error_history: list[float] = []
    no_improve = 0

    for n_via in range(min_via, max_via + 1):
        try:
            result = run_mpc_on_stroke(
                pts_px, screen_w, screen_h,
                n_via=n_via, order=order, dt=dt,
                stroke_index=stroke_index,
                sigma_min=sigma_min,
                sigma_speed_scale=sigma_speed_scale,
                ellipse_scale=ellipse_scale,
                max_traj_pts=max_traj_pts,
            )
        except Exception:
            error_history.append(float("inf"))
            continue

        err = result.reconstruction_error_px
        error_history.append(err)

        if progress_cb is not None:
            try:
                progress_cb(n_via, err)
            except Exception:
                pass

        improved = best is None or err < best.reconstruction_error_px
        if improved:
            best = result
            no_improve = 0
        else:
            no_improve += 1

        if err <= tol_px:
            break
        if no_improve >= _PATIENCE:
            break

    assert best is not None
    best.converged = best.reconstruction_error_px <= tol_px
    best.n_via_start = min_via
    return best, error_history
