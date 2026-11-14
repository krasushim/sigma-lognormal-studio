"""API request/response models."""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field

from .trajectory import TrajectoryPlan


class AnalyzeOptions(BaseModel):
    model: Literal["SL_STAR", "GL_STAR"] = "SL_STAR"
    max_dim: int = Field(1024, ge=128, le=4096)
    angle_thresh_deg: float = Field(25.0, ge=5.0, le=90.0)
    dt_min: float = Field(0.15, gt=0.0)
    dt_max: float = Field(0.60, gt=0.0)
    max_refit_iter: int = Field(10, ge=0, le=30)
    include_inflections: bool = Field(True, description="SL_STAR 時のみ有効")


class RefitRequest(BaseModel):
    plan: TrajectoryPlan
    contour: Optional[list[tuple[float, float]]] = None
    max_iter: int = Field(10, ge=1, le=50)
    tol_px: float = Field(1.0, gt=0.0)


class VariationPerturbations(BaseModel):
    target_offset_px: float = Field(5.0, ge=0.0)
    delta_t_scale: float = Field(0.15, ge=0.0, le=1.0)
    phi_scale: float = Field(0.10, ge=0.0, le=1.0)
    global_scale_xy: float = Field(0.05, ge=0.0, le=0.5)


class VariationRequest(BaseModel):
    plan: TrajectoryPlan
    n_variations: int = Field(6, ge=1, le=32)
    perturbations: VariationPerturbations = VariationPerturbations()
    seed: int = Field(42, ge=0)


class VariationResponse(BaseModel):
    variations: list[TrajectoryPlan]
