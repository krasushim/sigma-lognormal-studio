"""GML (Graffiti Markup Language) input support.

See sigma_lognormal_studio_spec.md §8 for the format and API.
"""
from app.gml.parser import parse_gml, GmlParseError
from app.gml.corpus import CorpusIndex, get_corpus
from app.gml.to_contours import strokes_to_contours, rasterize_strokes

__all__ = [
    "parse_gml",
    "GmlParseError",
    "CorpusIndex",
    "get_corpus",
    "strokes_to_contours",
    "rasterize_strokes",
]
