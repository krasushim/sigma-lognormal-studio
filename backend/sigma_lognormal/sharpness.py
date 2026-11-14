"""
Sharpness → Δt estimation — Berio 2015 Eq.20–21.

S(Z) = (A_s - A_Z) / (A_s - A_t)     if A_s > max(A_Z, A_t)
     = 0                             otherwise

where Z = region around dominant point p(z_j) with support l_sup:
  a = contour[z_j - l_sup]
  b = contour[z_j + l_sup]
  A_Z = actual signed area of [a, ..., p(z_j), ..., b] plus chord ab
  A_t = area of triangle (a, p(z_j), b)
  A_s = area of maximally-circular segment (circle passing through a, p(z_j), b)

Δt_j = Δt_min + (Δt_max - Δt_min) · min(S(Z)^α, 1),  α=3.5 default
"""
from __future__ import annotations

import math

import numpy as np


def _triangle_area(a: np.ndarray, p: np.ndarray, b: np.ndarray) -> float:
    return 0.5 * abs((p[0] - a[0]) * (b[1] - a[1]) - (b[0] - a[0]) * (p[1] - a[1]))


def _polygon_area_abs(poly: np.ndarray) -> float:
    closed = np.vstack([poly, poly[0:1]])
    return 0.5 * abs(np.sum(closed[:-1, 0] * closed[1:, 1] - closed[1:, 0] * closed[:-1, 1]))


def _circumscribed_arc_area(a: np.ndarray, p: np.ndarray, b: np.ndarray) -> float:
    """
    Maximally circular segment area A_s: the circular segment of the circle
    passing through (a, p, b), bounded by chord ab on the side of p.
    """
    # Circumradius from triangle sides
    A, B, C = a.astype(np.float64), b.astype(np.float64), p.astype(np.float64)
    a_len = np.linalg.norm(B - C)
    b_len = np.linalg.norm(A - C)
    c_len = np.linalg.norm(A - B)
    tri_area = _triangle_area(A, C, B)
    if tri_area < 1e-9:
        return 0.0
    R = a_len * b_len * c_len / (4.0 * tri_area)
    chord = c_len
    if chord >= 2.0 * R - 1e-9:
        theta = math.pi  # semicircle
    else:
        theta = 2.0 * math.asin(chord / (2.0 * R))
    # Check which side of chord p is on, to see if we want the "small" or "large" segment
    # A_s for the segment containing p.
    # Sagitta from chord to p:
    # Use perpendicular distance from p to line ab
    ab = B - A
    ab_len = np.linalg.norm(ab) + 1e-12
    u = ab / ab_len
    ap = C - A
    proj = ap @ u
    foot = A + proj * u
    h = np.linalg.norm(C - foot)
    # Small-side segment if h < R; large-side if h > R
    if h <= R:
        # small segment
        segment_area = 0.5 * R * R * (theta - math.sin(theta))
    else:
        # large segment = circle - small
        small = 0.5 * R * R * (theta - math.sin(theta))
        segment_area = math.pi * R * R - small
    return segment_area


def compute_sharpness(
    contour: np.ndarray,
    dom_idx: int,
    l_sup: int = 10,
) -> float:
    """
    Returns S(Z) for dominant point at `dom_idx` with ±l_sup support.
    """
    N = len(contour)
    lo = max(dom_idx - l_sup, 0)
    hi = min(dom_idx + l_sup, N - 1)
    if hi - lo < 2:
        return 0.0
    a = contour[lo]
    p = contour[dom_idx]
    b = contour[hi]

    A_t = _triangle_area(a, p, b)
    A_s = _circumscribed_arc_area(a, p, b)
    # A_Z = area of polygon [a, contour[lo..hi], b] (already starts at a and ends at b)
    poly = contour[lo: hi + 1]
    A_Z = _polygon_area_abs(poly)

    if A_s > max(A_Z, A_t):
        return float((A_s - A_Z) / (A_s - A_t + 1e-9))
    return 0.0


def estimate_delta_t(
    sharpness_values: np.ndarray,
    dt_min: float,
    dt_max: float,
    alpha: float = 3.5,
) -> np.ndarray:
    """
    Δt_j = Δt_min + (Δt_max - Δt_min) · min(S(Z)^α, 1)
    """
    clamped = np.clip(sharpness_values ** alpha, 0.0, 1.0)
    return dt_min + (dt_max - dt_min) * clamped
