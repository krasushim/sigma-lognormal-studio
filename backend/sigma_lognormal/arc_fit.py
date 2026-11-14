"""
Arc fitting and φ estimation — Berio 2015 §6.2 / Eq.19.

Given a contour segment, fit a circular arc and return the interior angle θ.
With θ, set φ = θ/2 (with sign depending on direction).

Handles inflections: if the signed area of the segment changes sign mid-way,
split at the inflection and fit two arcs → (θ_1, θ_2) → (φ_1, φ_2).
"""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import brentq


def signed_area(poly: np.ndarray) -> float:
    """Signed area of the region between poly and its chord (shoelace - triangle)."""
    if len(poly) < 3:
        return 0.0
    # Area between polyline and chord = polyline area - triangle from endpoints
    # Shoelace for closed polygon (poly + chord)
    closed = np.vstack([poly, poly[0:1]])
    sa = 0.5 * np.sum(
        closed[:-1, 0] * closed[1:, 1] - closed[1:, 0] * closed[:-1, 1]
    )
    return float(sa)


def _arc_area_eq(theta: float, chord_len: float, target_area: float) -> float:
    """
    Eq.19 of Berio 2015:
        0.5 * (c / (2 sin(θ/2)))^2 * (θ - sin θ) - A_s = 0
    where A_s = |target_area|.
    """
    s = math.sin(theta / 2.0)
    if abs(s) < 1e-9:
        return -target_area
    r = chord_len / (2.0 * s)
    segment_area = 0.5 * r * r * (theta - math.sin(theta))
    return segment_area - target_area


def fit_arc_interior_angle(segment: np.ndarray) -> float:
    """
    Find θ ∈ (0, 2π) such that the circular segment with chord |p_end - p_start|
    and area equal to |signed_area(segment)| closes the polyline area.
    Returns signed θ (sign from signed_area).
    """
    if len(segment) < 2:
        return 0.0
    chord = float(np.linalg.norm(segment[-1] - segment[0]))
    if chord < 1e-6:
        return 0.0
    sa = signed_area(segment)
    target = abs(sa)
    # For very flat segments (target ~ 0), θ ≈ 0.
    if target < 1e-3:
        return 0.0
    # Try to bracket a root in (small, near 2π). The equation is monotonic in θ for θ∈(0, π) typically.
    lo, hi = 1e-3, 2.0 * math.pi - 1e-3
    f_lo = _arc_area_eq(lo, chord, target)
    f_hi = _arc_area_eq(hi, chord, target)
    if f_lo * f_hi > 0:
        # fallback: approximate by max-deviation / chord
        v = segment[-1] - segment[0]
        nrm = np.linalg.norm(v) + 1e-9
        u = np.array([-v[1] / nrm, v[0] / nrm])
        dev = np.max(np.abs((segment - segment[0]) @ u))
        theta_approx = 4.0 * math.atan2(2.0 * dev, chord)
        theta_abs = max(min(theta_approx, math.pi * 0.99), 1e-3)
    else:
        theta_abs = brentq(_arc_area_eq, lo, hi, args=(chord, target), xtol=1e-5)
    # Apply sign
    return float(theta_abs if sa >= 0 else -theta_abs)


def fit_phi_pair(
    segment: np.ndarray,
    inflection_local_idx: int | None = None,
) -> tuple[float, float]:
    """
    Returns (φ_1, φ_2) for a contour segment (GL*).
    If inflection_local_idx is provided, splits segment at that index and fits
    two arcs.

    Convention:
      φ_1 = -θ_1 / 2   (angle between tangent at p_start and chord direction)
      φ_2 = +θ_2 / 2   (angle between chord direction and tangent at p_end)
    For a single arc (no inflection), (φ_1, φ_2) = (-θ/2, +θ/2) → symmetric.
    """
    if len(segment) < 2:
        return 0.0, 0.0
    if inflection_local_idx is None or inflection_local_idx <= 0 or inflection_local_idx >= len(segment) - 1:
        theta = fit_arc_interior_angle(segment)
        return -theta / 2.0, theta / 2.0
    s1 = segment[: inflection_local_idx + 1]
    s2 = segment[inflection_local_idx:]
    theta_1 = fit_arc_interior_angle(s1)
    theta_2 = fit_arc_interior_angle(s2)
    return -theta_1 / 2.0, theta_2 / 2.0


def find_inflection_local(segment: np.ndarray) -> int | None:
    """
    Returns a local inflection index within [1, len-2] or None if the segment
    has consistent curvature sign.
    """
    if len(segment) < 5:
        return None
    v1 = segment[1:-1] - segment[:-2]
    v2 = segment[2:] - segment[1:-1]
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    signs = np.sign(cross)
    # Tolerate zeros by carrying last non-zero sign forward
    last = 0.0
    clean = []
    for v in signs:
        if v == 0:
            clean.append(last)
        else:
            clean.append(v)
            last = v
    clean = np.array(clean)
    changes = np.where(np.diff(clean) != 0)[0]
    if len(changes) == 0:
        return None
    # Take the most central inflection
    mid = len(segment) // 2
    best = min(changes, key=lambda c: abs((c + 1) - mid))
    return int(best + 1)
