"""
Sigma-Lognormal Studio — FastAPI entrypoint.
"""
from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.analyze import router as analyze_router
from app.api.gml import router as gml_router
from app.api.mpc import router as mpc_router
from app.api.refit import router as refit_router
from app.api.variations import router as variations_router


app = FastAPI(
    title="Sigma-Lognormal Studio",
    version="1.0.0",
    description=(
        "Graffiti tag trajectory reconstruction & synthesis based on "
        "Berio et al. 2015 (CompModels) and 2025 (IGS/DSLM)."
    ),
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(analyze_router)
app.include_router(gml_router)
app.include_router(mpc_router)
app.include_router(refit_router)
app.include_router(variations_router)


@app.get("/health")
async def health() -> dict:
    return {"status": "ok", "version": "1.0.0"}
