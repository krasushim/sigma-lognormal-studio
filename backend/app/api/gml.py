"""GML corpus endpoints.

- GET /api/gml/list          : paged listing with optional ``q`` filter
- GET /api/gml/{gml_id}      : parsed GmlDocument (for preview)
- GET /api/gml/{gml_id}/raw  : original file as ``application/xml``
- POST /api/gml/reindex      : rebuild the corpus cache (dev / ops)
- POST /api/gml/preview      : parse a user-uploaded .gml without storing it
"""
from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter, File, HTTPException, Path, Query, UploadFile
from fastapi.responses import FileResponse

from app.gml.corpus import get_corpus
from app.gml.parser import GmlParseError, parse_gml
from app.schemas.trajectory import GmlDocument, GmlListResponse


logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/gml", tags=["gml"])


MAX_UPLOAD_BYTES = 2 * 1024 * 1024  # 2 MB per spec §8.6


@router.get("/list", response_model=GmlListResponse)
async def list_corpus(
    offset: int = Query(0, ge=0),
    limit: int = Query(50, ge=1, le=500),
    q: Optional[str] = Query(None, description="client_name 部分一致 or gml_id 完全一致"),
    random: bool = Query(False, description="true なら limit 件をランダムサンプル"),
    seed: Optional[int] = Query(None, description="random=true 時の再現用シード"),
) -> GmlListResponse:
    corpus = get_corpus()
    return corpus.list(
        offset=offset, limit=limit, q=q, shuffle=random, seed=seed,
    )


@router.post("/reindex")
async def reindex_corpus() -> dict:
    corpus = get_corpus()
    n = corpus.reindex()
    return {"indexed": n, "corpus_dir": str(corpus.corpus_dir)}


@router.post("/preview", response_model=GmlDocument)
async def preview_uploaded(gml_file: UploadFile = File(...)) -> GmlDocument:
    raw = await gml_file.read()
    if len(raw) == 0:
        raise HTTPException(400, "空ファイルです")
    if len(raw) > MAX_UPLOAD_BYTES:
        raise HTTPException(
            400, f"ファイルが大きすぎます (最大 {MAX_UPLOAD_BYTES // 1024} KB)",
        )
    try:
        return parse_gml(raw, filename=gml_file.filename or "<upload>")
    except GmlParseError as e:
        raise HTTPException(400, f"GML のパースに失敗: {e}")


@router.get("/{gml_id}", response_model=GmlDocument)
async def get_document(gml_id: int = Path(..., ge=0)) -> GmlDocument:
    corpus = get_corpus()
    try:
        return corpus.get(gml_id)
    except FileNotFoundError:
        raise HTTPException(404, f"gml_id {gml_id} がコーパスに見つかりません")
    except ValueError as e:
        raise HTTPException(400, str(e))
    except GmlParseError as e:
        raise HTTPException(500, f"GML のパースに失敗: {e}")


@router.get("/{gml_id}/raw")
async def get_raw(gml_id: int = Path(..., ge=0)) -> FileResponse:
    corpus = get_corpus()
    try:
        path = corpus.raw_path(gml_id)
    except FileNotFoundError:
        raise HTTPException(404, f"gml_id {gml_id} がコーパスに見つかりません")
    except ValueError as e:
        raise HTTPException(400, str(e))
    return FileResponse(
        path,
        media_type="application/xml",
        filename=path.name,
    )
