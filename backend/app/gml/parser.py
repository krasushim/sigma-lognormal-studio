"""Parse Graffiti Markup Language (.gml) files.

Handles both the GML 0.1b schema (``<environment>`` under ``<tag>``) and the
older temptTag variant (``<environment>`` under ``<header>``). Coordinates are
normalized to [0,1] via ``<screenBounds>``; we convert to absolute pixels and
remap them into the *photo frame* — the coordinate system the matching
``image_data/<id>.jpg`` uses (verified empirically: photos are stored at
``screenBounds`` resolution and the upright-content axis depends on ``<up>``).

For ``<up>=(±1,0,0)`` the device was held with its X axis vertical, so the
photo applies a 90° rotation + anisotropic resample to fit content back into
the original ``(w, h)`` rectangle. We mirror that here so spray.png always
shares its FOV (and pixel coords) with the matching photo.
"""
from __future__ import annotations

from typing import Iterable, Literal, Optional
from xml.etree import ElementTree as ET

from app.schemas.trajectory import GmlDocument, GmlStroke


DEFAULT_BOUNDS: tuple[int, int] = (1024, 768)
MIN_STROKE_POINTS: int = 4

# Orientation tag derived from the GML <up> vector. The parser rewrites all
# normalized point coords + ``screen_bounds`` into a single canonical
# Y-down screen frame, so downstream code never needs to look at <up>.
#
# The empirical convention (verified against 000000book corpus) is:
#   <up>=(0,1,0)  → already screen-aligned (Fat Tag stores coords in Y-down
#                   regardless of <up>; the vector just records device gravity)
#   <up>=(1,0,0)  → device rotated 90°, X-axis was up → rot_x_up
#   <up>=(-1,0,0) → device rotated -90°, -X was up   → rot_x_dn
#   <up>=(0,-1,0) → device upside-down               → rot180
#   missing/zero  → assume identity
Orientation = Literal["identity", "rot180", "rot_x_up", "rot_x_dn"]


class GmlParseError(ValueError):
    """Raised when the input bytes cannot be parsed as GML."""


def _first_child_text(node: Optional[ET.Element], *names: str) -> Optional[str]:
    if node is None:
        return None
    for name in names:
        found = node.find(name)
        if found is not None and found.text is not None:
            txt = found.text.strip()
            if txt:
                return txt
    return None


def _find_screen_bounds(tag: ET.Element) -> tuple[int, int]:
    # GML 0.1b: <tag>/<environment>/<screenBounds>
    # temptTag: <tag>/<header>/<environment>/<screenBounds>
    for path in (
        "./environment/screenBounds",
        "./header/environment/screenBounds",
    ):
        el = tag.find(path)
        if el is not None:
            x = _first_child_text(el, "x")
            y = _first_child_text(el, "y")
            if x is not None and y is not None:
                try:
                    w = int(round(float(x)))
                    h = int(round(float(y)))
                    if w > 0 and h > 0:
                        return w, h
                except ValueError:
                    pass
    return DEFAULT_BOUNDS


def _find_up(tag: ET.Element) -> Optional[tuple[float, float, float]]:
    """Return the ``<up>`` gravity vector as (x, y, z), or ``None`` if absent
    or unparseable. Both GML 0.1b (``<environment>``) and temptTag (legacy
    nested under ``<header>``) layouts are tried."""
    for path in ("./environment/up", "./header/environment/up"):
        el = tag.find(path)
        if el is None:
            continue
        try:
            x = _first_child_text(el, "x")
            y = _first_child_text(el, "y")
            z = _first_child_text(el, "z")
            if x is None or y is None:
                continue
            return float(x), float(y), float(z) if z is not None else 0.0
        except ValueError:
            continue
    return None


def _orientation_from_up(up: Optional[tuple[float, float, float]]) -> Orientation:
    """Map an ``<up>`` vector to the transform that re-expresses points in
    the matching photo's coordinate frame.

    Verified against the 000000book corpus (Fat Tag - Katsu Edition + temptTag)
    by re-rendering landmark GMLs (10103 ``FUCK ↓``, 49405 ``↑``) and
    overlaying parsed strokes on the matching ``image_data/<id>.jpg`` at
    *photo-native dimensions*.

    - ``up = (0, 1, 0)``       → ``identity`` (most common: phone held normally)
    - ``up = (1, 0, 0)``       → ``rot_x_up`` (90° device rotation; +X was up)
    - ``up = (-1, 0, 0)``      → ``rot_x_dn``
    - ``up = (0, -1, 0)``      → ``rot180``
    - missing / zero / unrecognised → ``identity``
    """
    if up is None:
        return "identity"
    x, y, _z = up
    EPS = 0.5  # GML values are 0/±1; tolerant snap covers float jitter
    if x > EPS and abs(y) < EPS:
        return "rot_x_up"
    if x < -EPS and abs(y) < EPS:
        return "rot_x_dn"
    if y < -EPS and abs(x) < EPS:
        return "rot180"
    # (0,1,0), (0,0,0), missing, or anything else → already screen-aligned.
    return "identity"


def _apply_orientation(
    pts: list[tuple[float, float, float]],
    *, w: int, h: int, orientation: Orientation,
) -> tuple[list[tuple[float, float, float]], int, int]:
    """Rewrite ``pts`` (in [0,w]×[0,h] device-local pixel coords) into the
    *photo frame* — same ``(w, h)`` rectangle as ``image_data/<id>.jpg`` —
    and return ``(new_pts, w, h)``.

    For 90° rotations the photo file keeps the original ``(w, h)`` aspect
    rather than swapping to ``(h, w)``, which means the rotation is paired
    with an anisotropic ``(h/w, w/h)`` resample. We replicate that here so
    spray.png pixel-aligns with the photo.
    """
    if orientation == "identity":
        return pts, w, h
    if orientation == "rot180":
        return [(w - x, h - y, t) for (x, y, t) in pts], w, h
    sx = float(w) / float(h)
    sy = float(h) / float(w)
    if orientation == "rot_x_up":
        # device +X was gravity-up. Rotate (x,y)→(y,w-x), then resample (h,w)→(w,h).
        return [(y * sx, (w - x) * sy, t) for (x, y, t) in pts], w, h
    if orientation == "rot_x_dn":
        # device +X was gravity-down. Rotate (x,y)→(h-y,x), then resample.
        return [((h - y) * sx, x * sy, t) for (x, y, t) in pts], w, h
    raise ValueError(f"unknown orientation: {orientation}")


def _find_client_name(tag: ET.Element) -> Optional[str]:
    for path in ("./header/client", "./client"):
        client = tag.find(path)
        if client is not None:
            name = _first_child_text(client, "name", "username")
            if name:
                return name
    return None


def _iter_strokes(tag: ET.Element) -> Iterable[ET.Element]:
    # 全 <drawing>/<stroke> をフラットに列挙 (複数 drawing 対応)。
    for drawing in tag.findall("./drawing"):
        for stroke in drawing.findall("./stroke"):
            yield stroke


def _parse_stroke(
    stroke_el: ET.Element, *, w: int, h: int,
) -> Optional[list[tuple[float, float, float]]]:
    pts: list[tuple[float, float, float]] = []
    for pt in stroke_el.findall("./pt"):
        xs = _first_child_text(pt, "x")
        ys = _first_child_text(pt, "y")
        ts = _first_child_text(pt, "time", "t")
        if xs is None or ys is None:
            continue
        try:
            x_norm = float(xs)
            y_norm = float(ys)
            t_s = float(ts) if ts is not None else 0.0
        except ValueError:
            continue
        if t_s < 0.0:
            t_s = 0.0
        pts.append((x_norm * w, y_norm * h, t_s))
    if len(pts) < MIN_STROKE_POINTS:
        return None
    return pts


def parse_gml(
    raw: bytes | str,
    *,
    filename: str = "<input>",
    gml_id: Optional[int] = None,
) -> GmlDocument:
    """Parse GML bytes/string into a ``GmlDocument``.

    Raises ``GmlParseError`` on malformed XML or total absence of strokes.
    """
    if isinstance(raw, bytes):
        # Tolerate BOM / leading whitespace.
        text = raw.lstrip(b"\xef\xbb\xbf").decode("utf-8", errors="replace")
    else:
        text = raw
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise GmlParseError(f"XML parse error: {e}") from e

    # <GML> or <tag> may appear as root depending on authoring tool.
    if root.tag == "GML":
        tag = root.find("./tag")
    elif root.tag == "tag":
        tag = root
    else:
        tag = root.find(".//tag")
    if tag is None:
        raise GmlParseError("no <tag> element found")

    spec = root.attrib.get("spec") if root.tag == "GML" else None
    w, h = _find_screen_bounds(tag)
    client_name = _find_client_name(tag)
    orientation = _orientation_from_up(_find_up(tag))
    # screen_bounds always reflects the photo file's (w, h) — no swap.
    strokes: list[GmlStroke] = []
    idx = 0
    for stroke_el in _iter_strokes(tag):
        pts = _parse_stroke(stroke_el, w=w, h=h)
        if pts is None:
            continue
        rotated_pts, _, _ = _apply_orientation(
            pts, w=w, h=h, orientation=orientation,
        )
        duration = max((p[2] for p in rotated_pts), default=0.0)
        strokes.append(
            GmlStroke(
                index=idx,
                n_points=len(rotated_pts),
                duration_s=float(duration),
                points=rotated_pts,
            ),
        )
        idx += 1

    if not strokes:
        raise GmlParseError("no valid strokes (each requires >=4 points)")

    return GmlDocument(
        gml_id=gml_id,
        filename=filename,
        spec=spec,
        client_name=client_name,
        screen_bounds=(w, h),
        strokes=strokes,
    )
