"""
Dominant point detection — Berio 2015 §6.1.

Input: contour as [N, 2] (x, y), uniformly resampled with arclength step Δs = 1.
Output: list of dominant-point indices (includes 0 and N-1).

Algorithm:
  1. Compute discrete turning angle θ(i) between (p(i)-p(i-1)) and (p(i+1)-p(i)).
     Note: θ ∈ [0, π]. Sharp corners have large θ.
  2. Find local maxima of -cos(θ(i))  (== local maxima of |turning|).
  3. Keep only those with turn angle above `angle_thresh_deg`.
  4. For each pair of adjacent salient points, sum |θ(i)|; if > 180° add
     floor(sum/180°) equally-spaced dominant points between them.
  5. Optionally add inflection points (for SL* only).
"""
from __future__ import annotations

import math

import numpy as np


def _turning_angles(contour: np.ndarray) -> np.ndarray:
    """
    θ(i) for interior vertices (i=1..N-2), in radians, ∈ [0, π].
    """
    v1 = contour[1:-1] - contour[:-2]
    v2 = contour[2:] - contour[1:-1]
    n1 = np.linalg.norm(v1, axis=1)
    n2 = np.linalg.norm(v2, axis=1)
    denom = np.clip(n1 * n2, 1e-9, None)
    cos = np.clip(np.einsum("ij,ij->i", v1, v2) / denom, -1.0, 1.0)
    return np.arccos(cos)


def _signed_turn(contour: np.ndarray) -> np.ndarray:
    """
    Signed turning (cross-product sign). Used for inflection detection.
    """
    v1 = contour[1:-1] - contour[:-2]
    v2 = contour[2:] - contour[1:-1]
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    n1 = np.linalg.norm(v1, axis=1)
    n2 = np.linalg.norm(v2, axis=1)
    denom = np.clip(n1 * n2, 1e-9, None)
    return cross / denom


def find_inflection_indices(contour: np.ndarray, eps: float = 1e-2) -> list[int]:
    """Return interior indices where signed turning changes sign."""
    s = _signed_turn(contour)
    s_filt = np.where(np.abs(s) < eps, 0.0, s)
    signs = np.sign(s_filt)
    # carry last nonzero sign forward to avoid spurious flips on noise
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
    # indices in full contour = +1 (because signed_turn has length N-2, offset by 1)
    return (changes + 1).tolist()


def find_dominant_points(
    contour: np.ndarray,
    angle_thresh_deg: float = 35.0,
    include_inflections: bool = False,
    min_arc_spacing_px: float = 0.0,
    max_dominant_points: int = 0,
    cumulative_turn_insertion: bool = True,
) -> list[int]:
    """
    Returns sorted list of dominant point indices including 0 and N-1.

    Filters (適用順):
      1. 折れ角の局所極大 & angle_thresh_deg 以上
      2. cumulative_turn_insertion=True なら隣接 anchor 対の累積回転角が
         180° 超のとき等間隔で点を補間 (Berio 2015 §6.1)
      3. min_arc_spacing_px > 0 なら近すぎる優位点を間引き (回転角が大きい方を残す)
      4. max_dominant_points > 0 なら回転角の大きい順に絞り込む
    """
    N = len(contour)
    if N < 3:
        return list(range(N))
    theta = _turning_angles(contour)  # len N-2

    neg_cos = -np.cos(theta)
    local_max = np.zeros(len(neg_cos), dtype=bool)
    local_max[1:-1] = (neg_cos[1:-1] >= neg_cos[:-2]) & (neg_cos[1:-1] >= neg_cos[2:])

    deg = np.degrees(theta)
    salient = np.where(local_max & (deg > angle_thresh_deg))[0] + 1

    anchors = sorted(set([0] + salient.tolist() + [N - 1]))

    augmented: list[int] = []
    total_theta = np.concatenate([[0.0], theta, [0.0]])
    if cumulative_turn_insertion:
        for a, b in zip(anchors[:-1], anchors[1:]):
            augmented.append(a)
            if b <= a + 1:
                continue
            sum_theta_deg = float(np.sum(np.degrees(total_theta[a + 1:b])))
            n_insert = int(sum_theta_deg // 180)
            if n_insert > 0:
                for k in range(1, n_insert + 1):
                    idx = int(round(a + k * (b - a) / (n_insert + 1)))
                    idx = max(a + 1, min(b - 1, idx))
                    augmented.append(idx)
        augmented.append(anchors[-1])
        result = sorted(set(augmented))
    else:
        result = anchors[:]

    if include_inflections:
        infl = find_inflection_indices(contour, eps=1e-2)
        result = sorted(set(result) | set(infl))

    if result[0] != 0:
        result.insert(0, 0)
    if result[-1] != N - 1:
        result.append(N - 1)

    # 弧長間隔フィルタ
    if min_arc_spacing_px > 0 and len(result) > 2:
        result = _enforce_arc_spacing(contour, result, total_theta, min_arc_spacing_px)
    # 最大数キャップ
    if max_dominant_points > 0 and len(result) > max_dominant_points:
        result = _cap_by_priority(result, total_theta, max_dominant_points)
    return result


def _enforce_arc_spacing(
    contour: np.ndarray,
    indices: list[int],
    angle_at_idx: np.ndarray,
    min_spacing: float,
) -> list[int]:
    """
    隣接優位点間の弧長が min_spacing 未満なら統合する。
    端点 (0, N-1) は必ず保持。中間点は angle_at_idx の大きい方を残す。
    """
    if len(indices) <= 2:
        return indices
    seg = np.linalg.norm(np.diff(contour, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    last_idx = indices[-1]

    kept: list[int] = [indices[0]]
    for i in indices[1:-1]:
        if (cum[i] - cum[kept[-1]]) < min_spacing:
            # too close to previous: keep larger-angle one (but never replace the start)
            if kept[-1] != indices[0] and angle_at_idx[i] > angle_at_idx[kept[-1]]:
                kept[-1] = i
            # else drop i
        else:
            kept.append(i)

    # 末尾との距離チェック: 末尾近くの中間点を削除
    while len(kept) > 1 and (cum[last_idx] - cum[kept[-1]]) < min_spacing and kept[-1] != indices[0]:
        kept.pop()
    kept.append(last_idx)
    return kept


def _cap_by_priority(
    indices: list[int],
    angle_at_idx: np.ndarray,
    max_pts: int,
) -> list[int]:
    """端点を必ず残しつつ、回転角の大きい順に max_pts までに絞る。"""
    if len(indices) <= max_pts:
        return indices
    first, last = indices[0], indices[-1]
    middle = indices[1:-1]
    middle_sorted = sorted(middle, key=lambda i: -angle_at_idx[i])
    keep = sorted(middle_sorted[: max_pts - 2])
    return [first] + keep + [last]
