"""
Pydantic schemas for TrajectoryPlan and related API bodies.
v2.0: multi-stroke (stroke_groups) 対応。
v2.1: GML 入力用 SourceInfo / GmlDocument / GmlStroke を追加。
"""
from __future__ import annotations

import math
from typing import Literal, Optional

from pydantic import BaseModel, Field


class GlobalParams(BaseModel):
    mu: float = Field(math.log(0.2), description="Global log-delay")
    sigma: float = Field(math.log(1.4), description="Global log-response")
    delta_t_min: float = Field(0.15, gt=0.0)
    delta_t_max: float = Field(0.60, gt=0.0)
    use_min_time_reparam: bool = Field(
        True, description="Use Ac/T/Δt formulation (Berio 2025)"
    )
    lambda_smooth: float = Field(0.01, ge=0.0)
    seed: int = Field(42, ge=0)


class ActionPlan(BaseModel):
    p0: tuple[float, float]
    virtual_targets: list[tuple[float, float]]


class StrokeParams(BaseModel):
    index: int
    delta_t: float
    mu: float
    sigma: float
    phi: Optional[float] = None   # SL*
    phi1: Optional[float] = None  # GL*
    phi2: Optional[float] = None  # GL*


class TimeSeries(BaseModel):
    dt: float
    t:     list[float]
    x:     list[float]
    y:     list[float]
    vx:    list[float]
    vy:    list[float]
    speed: list[float]


class DerivedData(BaseModel):
    timeseries: TimeSeries
    t_end: float
    arc_length_px: float
    critical_points: list[tuple[float, float]] = []


class StrokeGroup(BaseModel):
    """
    1本の連続したストローク (1つの SL* / GL* 軌跡)。
    画像中に複数のストロークがある場合は TrajectoryPlan が複数の StrokeGroup を持つ。
    """
    group_index: int
    action_plan: ActionPlan
    strokes: list[StrokeParams]
    derived: DerivedData
    input_contour: list[tuple[float, float]] = []
    dominant_points: list[tuple[float, float]] = []
    quality: "Quality"


class Quality(BaseModel):
    reconstruction_mse_px: float = 0.0
    n_iter_used: int = 0


# Forward reference resolution (pydantic v2 uses model_rebuild; v1 uses update_forward_refs)
if hasattr(StrokeGroup, "model_rebuild"):
    StrokeGroup.model_rebuild()
else:
    StrokeGroup.update_forward_refs()


class SourceInfo(BaseModel):
    """どの入力から得られた Plan か (UI 側の分岐や表示に用いる)。"""
    kind: Literal["image", "drawing", "gml"]
    gml_id: Optional[int] = None
    gml_filename: Optional[str] = None
    gml_client: Optional[str] = None
    gml_screen_bounds: Optional[tuple[int, int]] = None
    gml_stroke_count: Optional[int] = None
    gml_selected_indices: Optional[list[int]] = None


class GmlStroke(BaseModel):
    """GML 内 1 本の <stroke> を px 座標に展開したもの (プレビュー用)。"""
    index: int
    n_points: int
    duration_s: float
    points: list[tuple[float, float, float]]  # (x_px, y_px, t_s)


class GmlDocument(BaseModel):
    """パース済みの GML ドキュメント。"""
    gml_id: Optional[int] = None
    filename: str
    spec: Optional[str] = None
    client_name: Optional[str] = None
    screen_bounds: tuple[int, int]
    strokes: list[GmlStroke]


class GmlIndexEntry(BaseModel):
    """コーパス一覧の 1 エントリ (軽量メタ)。"""
    gml_id: int
    filename: str
    client_name: Optional[str] = None
    n_strokes: int
    total_duration_s: float
    screen_bounds: tuple[int, int]


class GmlListResponse(BaseModel):
    total: int
    offset: int
    limit: int
    items: list[GmlIndexEntry]


class TrajectoryPlan(BaseModel):
    schema_version: str = "2.1"
    model: Literal["SL_STAR", "GL_STAR"] = "GL_STAR"
    image_width: int
    image_height: int
    global_params: GlobalParams
    stroke_groups: list[StrokeGroup]
    overall_quality: Quality
    # --- 解析過程の可視化用 (include_stages=true のとき設定) ---
    mask_png_b64: Optional[str] = None         # 前景インクマスク (PNG base64)
    skeleton_png_b64: Optional[str] = None     # Zhang-Suen 細線化 (PNG base64)
    # diff_refit 前の解析パイプライン出力。final stroke_groups と比較するための snapshot
    initial_stroke_groups: list[StrokeGroup] = []
    # --- 入力ソース情報 (v2.1) ---
    source: Optional[SourceInfo] = None
