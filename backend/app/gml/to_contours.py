"""Helpers to feed GML strokes into the existing analyze pipeline."""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from sigma_lognormal.contour import resample_arclen, smooth_gaussian

from app.schemas.trajectory import GmlDocument


def strokes_to_contours(
    doc: GmlDocument,
    *,
    selected: Optional[list[int]] = None,
    resample_step_px: float = 1.0,
    smooth_sigma_px: float = 1.5,
) -> list[np.ndarray]:
    """Return per-stroke ``(N, 2) float32`` arrays in absolute pixel space.

    Mirrors the DrawPanel pre-processing: arc-length resample then a light
    Gaussian smooth. Strokes with <4 points after resampling are dropped.
    """
    contours: list[np.ndarray] = []
    wanted = set(selected) if selected is not None else None
    for s in doc.strokes:
        if wanted is not None and s.index not in wanted:
            continue
        if len(s.points) < 4:
            continue
        xy = np.array([[p[0], p[1]] for p in s.points], dtype=np.float32)
        xy = resample_arclen(xy, step=resample_step_px)
        if len(xy) < 4:
            continue
        xy = smooth_gaussian(xy, sigma_px=smooth_sigma_px)
        contours.append(xy.astype(np.float32))
    return contours


def rasterize_strokes(
    contours: list[np.ndarray],
    width: int,
    height: int,
    *,
    thickness: int = 10,
) -> np.ndarray:
    """Build a synthetic uint8 mask suitable for ``diff_refit``.

    Thickness is the *full* line width in pixels (spec §8.5: ``2 *
    diff_stroke_radius_px``).
    """
    mask = np.zeros((max(height, 1), max(width, 1)), dtype=np.uint8)
    for c in contours:
        if len(c) < 2:
            continue
        pts = c.round().astype(np.int32)
        cv2.polylines(
            mask, [pts], isClosed=False, color=255,
            thickness=max(int(thickness), 3),
        )
    return mask


def parse_indices_csv(spec: Optional[str]) -> Optional[list[int]]:
    """Parse ``"0,2,3"`` into ``[0, 2, 3]``. Returns None for empty/None."""
    if spec is None:
        return None
    spec = spec.strip()
    if not spec:
        return None
    out: list[int] = []
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            v = int(tok)
        except ValueError as e:
            raise ValueError(f"invalid stroke index: {tok!r}") from e
        if v < 0:
            raise ValueError(f"stroke index must be >= 0: {v}")
        out.append(v)
    return out or None
