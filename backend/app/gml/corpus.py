"""GML corpus index.

Scans ``gml_data/`` once, caches light metadata to ``.index.json``, and exposes
lookups used by ``GET /api/gml/*``. Path traversal is blocked by accepting only
integer IDs and resolving the final path under ``gml_data/``.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import random
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from app.gml.parser import GmlParseError, parse_gml
from app.schemas.trajectory import (
    GmlDocument, GmlIndexEntry, GmlListResponse,
)


logger = logging.getLogger(__name__)

INDEX_FILENAME = ".index.json"


def _default_corpus_dir() -> Path:
    """Resolve the corpus directory.

    Resolution order:
    1. ``GML_CORPUS_DIR`` env var (if set and exists)
    2. ``<repo_root>/gml_data`` — the file lives at
       ``backend/app/gml/corpus.py``, so ``parents[4]`` is the repo root.
    """
    env = os.environ.get("GML_CORPUS_DIR")
    if env:
        p = Path(env).expanduser().resolve()
        if p.is_dir():
            return p
    # backend/app/gml/corpus.py → parents[4] = repo root → /gml_data
    here = Path(__file__).resolve()
    candidate = here.parents[4] / "gml_data"
    return candidate.resolve()


@dataclass
class _IndexRow:
    gml_id: int
    filename: str
    client_name: Optional[str]
    n_strokes: int
    total_duration_s: float
    screen_bounds_w: int
    screen_bounds_h: int

    def to_entry(self) -> GmlIndexEntry:
        return GmlIndexEntry(
            gml_id=self.gml_id,
            filename=self.filename,
            client_name=self.client_name,
            n_strokes=self.n_strokes,
            total_duration_s=self.total_duration_s,
            screen_bounds=(self.screen_bounds_w, self.screen_bounds_h),
        )


class CorpusIndex:
    """In-memory index of the GML corpus, backed by ``.index.json``."""

    def __init__(self, corpus_dir: Path) -> None:
        self.corpus_dir = corpus_dir.resolve()
        self._rows: list[_IndexRow] = []
        self._by_id: dict[int, _IndexRow] = {}
        self._lock = threading.Lock()
        self._loaded = False

    # ------------------------------------------------------------------ paths
    @property
    def index_path(self) -> Path:
        return self.corpus_dir / INDEX_FILENAME

    def path_for(self, gml_id: int) -> Path:
        """Resolve the absolute path for a given ID with traversal protection."""
        if not isinstance(gml_id, int) or gml_id < 0:
            raise ValueError(f"invalid gml_id: {gml_id!r}")
        p = (self.corpus_dir / f"{gml_id}.gml").resolve()
        if not self._is_within_corpus(p):
            raise ValueError(f"resolved path escapes corpus: {p}")
        return p

    def _is_within_corpus(self, p: Path) -> bool:
        try:
            p.relative_to(self.corpus_dir)
            return True
        except ValueError:
            return False

    # ------------------------------------------------------------------ init
    def ensure_loaded(self) -> None:
        with self._lock:
            if self._loaded:
                return
            if self.index_path.exists():
                try:
                    self._load_index_file()
                    self._loaded = True
                    return
                except Exception:
                    logger.exception("failed to load %s, rebuilding", self.index_path)
            if self.corpus_dir.is_dir():
                self._build_index()
            self._loaded = True

    def reindex(self) -> int:
        """Force full rebuild of the index. Returns the number of entries."""
        with self._lock:
            self._build_index()
            self._loaded = True
        return len(self._rows)

    # Bump when the parser changes shape of cached fields. Mismatched cache → rebuild.
    INDEX_VERSION = 3

    def _load_index_file(self) -> None:
        data = json.loads(self.index_path.read_text(encoding="utf-8"))
        if int(data.get("version", 0)) != self.INDEX_VERSION:
            raise ValueError(
                f"index version mismatch (got {data.get('version')}, "
                f"expected {self.INDEX_VERSION}); rebuilding",
            )
        items = data.get("items", [])
        rows: list[_IndexRow] = []
        for it in items:
            try:
                rows.append(
                    _IndexRow(
                        gml_id=int(it["gml_id"]),
                        filename=str(it["filename"]),
                        client_name=it.get("client_name"),
                        n_strokes=int(it.get("n_strokes", 0)),
                        total_duration_s=float(it.get("total_duration_s", 0.0)),
                        screen_bounds_w=int(it.get("sw", 0)),
                        screen_bounds_h=int(it.get("sh", 0)),
                    ),
                )
            except (KeyError, TypeError, ValueError):
                continue
        rows.sort(key=lambda r: r.gml_id)
        self._rows = rows
        self._by_id = {r.gml_id: r for r in rows}

    def _build_index(self) -> None:
        if not self.corpus_dir.is_dir():
            logger.warning("corpus dir not found: %s", self.corpus_dir)
            self._rows = []
            self._by_id = {}
            return
        logger.info("indexing GML corpus at %s ...", self.corpus_dir)
        rows: list[_IndexRow] = []
        n_failed = 0
        for entry in os.scandir(self.corpus_dir):
            if not entry.is_file() or not entry.name.endswith(".gml"):
                continue
            stem = entry.name[:-4]
            try:
                gml_id = int(stem)
            except ValueError:
                continue
            try:
                with open(entry.path, "rb") as f:
                    raw = f.read()
                doc = parse_gml(raw, filename=entry.name, gml_id=gml_id)
            except (GmlParseError, OSError):
                n_failed += 1
                continue
            total = sum(s.duration_s for s in doc.strokes)
            rows.append(
                _IndexRow(
                    gml_id=gml_id,
                    filename=entry.name,
                    client_name=doc.client_name,
                    n_strokes=len(doc.strokes),
                    total_duration_s=round(total, 3),
                    screen_bounds_w=doc.screen_bounds[0],
                    screen_bounds_h=doc.screen_bounds[1],
                ),
            )
        rows.sort(key=lambda r: r.gml_id)
        self._rows = rows
        self._by_id = {r.gml_id: r for r in rows}
        self._write_index_file()
        logger.info(
            "indexed %d GML files (%d failed to parse)", len(rows), n_failed,
        )

    def _write_index_file(self) -> None:
        payload = {
            "version": self.INDEX_VERSION,
            "corpus_dir": str(self.corpus_dir),
            "items": [
                {
                    "gml_id": r.gml_id,
                    "filename": r.filename,
                    "client_name": r.client_name,
                    "n_strokes": r.n_strokes,
                    "total_duration_s": r.total_duration_s,
                    "sw": r.screen_bounds_w,
                    "sh": r.screen_bounds_h,
                }
                for r in self._rows
            ],
        }
        tmp = self.index_path.with_suffix(".json.part")
        # Explicit utf-8: client_name can contain non-ASCII (e.g. ‹ U+2039) that
        # crashes the Windows default cp932 codec.
        tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        tmp.replace(self.index_path)

    # ------------------------------------------------------------------ query
    def list(
        self,
        offset: int = 0,
        limit: int = 50,
        q: Optional[str] = None,
        *,
        shuffle: bool = False,
        seed: Optional[int] = None,
    ) -> GmlListResponse:
        self.ensure_loaded()
        rows: Iterable[_IndexRow] = self._rows
        if q:
            needle = q.casefold()
            rows = [
                r for r in self._rows
                if (r.client_name or "").casefold().find(needle) >= 0
                or str(r.gml_id) == q
            ]
        rows = list(rows)
        total = len(rows)
        offset = max(0, int(offset))
        limit = max(1, min(int(limit), 500))
        if shuffle:
            rng = random.Random(seed) if seed is not None else random.Random()
            sample_size = min(limit, total)
            # Random sampling (without replacement) from the entire filtered set.
            sampled = rng.sample(rows, sample_size) if sample_size > 0 else []
            items = [r.to_entry() for r in sampled]
        else:
            items = [r.to_entry() for r in rows[offset : offset + limit]]
        return GmlListResponse(
            total=total, offset=offset, limit=limit, items=items,
        )

    def get(self, gml_id: int) -> GmlDocument:
        """Parse and return the full document for the given corpus ID."""
        self.ensure_loaded()
        path = self.path_for(gml_id)
        if not path.exists():
            raise FileNotFoundError(f"gml_id {gml_id} not in corpus")
        raw = path.read_bytes()
        return parse_gml(raw, filename=path.name, gml_id=gml_id)

    def raw_path(self, gml_id: int) -> Path:
        """Return the raw file path (raising if absent). No parse."""
        path = self.path_for(gml_id)
        if not path.exists():
            raise FileNotFoundError(f"gml_id {gml_id} not in corpus")
        return path


_singleton: Optional[CorpusIndex] = None
_singleton_lock = threading.Lock()


def get_corpus() -> CorpusIndex:
    global _singleton
    with _singleton_lock:
        if _singleton is None:
            _singleton = CorpusIndex(_default_corpus_dir())
        return _singleton


def _cli() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    p = argparse.ArgumentParser(prog="app.gml.corpus")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("reindex", help="Rebuild the .index.json cache")
    sub.add_parser("list", help="Print a short summary of the current index")
    args = p.parse_args()
    corpus = get_corpus()
    if args.cmd == "reindex":
        n = corpus.reindex()
        print(f"indexed {n} GML files at {corpus.corpus_dir}")
    elif args.cmd == "list":
        corpus.ensure_loaded()
        resp = corpus.list(offset=0, limit=10)
        print(f"total={resp.total} corpus_dir={corpus.corpus_dir}")
        for item in resp.items:
            print(
                f"  {item.gml_id:>6}  strokes={item.n_strokes:>3}  "
                f"dur={item.total_duration_s:>6.2f}s  "
                f"bounds={item.screen_bounds}  "
                f"client={item.client_name or '-'}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(_cli())
