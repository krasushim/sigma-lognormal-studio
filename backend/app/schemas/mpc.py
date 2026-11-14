"""Pydantic schemas for the MPC stroke estimation endpoint."""
from __future__ import annotations

from typing import Optional
from pydantic import BaseModel, Field


class MpcEllipse(BaseModel):
    """2-σ confidence ellipse for one Gaussian via-point (pixel coordinates)."""
    x: float
    y: float
    rx: float        # semi-axis along major direction (px)
    ry: float        # semi-axis along minor direction (px)
    angle_deg: float # rotation of major axis from +X axis (degrees, CCW)


class MpcStrokeResult(BaseModel):
    """MPC result for one GML stroke."""
    stroke_index: int
    n_via: int
    order: int
    ellipses: list[MpcEllipse]    # Gaussian via-point confidence ellipses
    traj_x: list[float]           # smooth trajectory x (pixel, downsampled)
    traj_y: list[float]           # smooth trajectory y (pixel, downsampled)
    duration_s: float
    reconstruction_error_px: float  # mean L2 distance to original (px)
    converged: bool                 # True if error ≤ tol_px
    error_history: list[float]      # per-iteration errors (auto_optimize mode)
    n_via_start: int                # first n_via tried (= min_via in auto mode)


class MpcResult(BaseModel):
    """MPC result for an entire GML document."""
    gml_id: Optional[int] = None
    screen_bounds: tuple[int, int]
    strokes: list[MpcStrokeResult]
    n_via: int          # final n_via (may differ per stroke in auto mode)
    order: int
    auto_optimize: bool
    tol_px: float


class MpcFromPointsRequest(BaseModel):
    """Request body for POST /api/mpc/from-points (raw stroke points, no GML file)."""
    strokes: list[list[list[float]]]  # [[[x_px, y_px, t_s], ...], ...]
    screen_w: int = Field(ge=1)
    screen_h: int = Field(ge=1)
    n_via: int = Field(5, ge=2, le=20)
    order: int = Field(4, ge=2, le=6)
    dt: float = Field(0.005, gt=0)
    auto_optimize: bool = False
    tol_px: float = Field(3.0, gt=0)
    min_via: int = Field(3, ge=2, le=20)
    max_via: int = Field(15, ge=2, le=30)
