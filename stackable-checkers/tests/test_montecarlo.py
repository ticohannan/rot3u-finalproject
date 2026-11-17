"""SP-9, SP-13: seeded sampler, reference curve, metadata."""
import json

from helpers import REFERENCE, constraints
from stackpieces.geometry import PRESETS, make_piece
from stackpieces.montecarlo import Rng, crossing, metadata, reliable_offset, stacking_curve


def test_rng_sequence():
    r = Rng(1000)
    assert [round(r(), 12) for _ in range(3)] == [0.623620399972, 0.982332071755, 0.527806099737]


def test_reference_curve_reproduced():
    """Rerun 4 offsets of the reference printed-layer curve with the same seed; counts within 15 points."""
    ref = json.loads(REFERENCE.read_text(encoding="utf-8"))
    c = ref["constraints"]
    rows = ref["curves"]["S40"]["layered"]["rows"]
    idx = [0, 4, 7, 10]
    g = make_piece(PRESETS["S40"])
    cv = stacking_curve(g, c, "layered", only=idx)   # same seeds per offset index as the full run
    n = cv["n"]
    for k, i in enumerate(idx):
        a, b = rows[i]["counts"]["seated"], cv["rows"][k]["counts"]["seated"]
        assert abs(a - b) <= 0.15 * n, (rows[i]["e"], a, b)


def test_curve_metrics():
    cv = {"n": 10, "rows": [{"e": e, "counts": {"seated": s, "perched": 10 - s, "fell": 0, "knocked": 0}}
                            for e, s in ((0, 10), (1, 10), (2, 6), (3, 2), (4, 0))]}
    assert abs(crossing(cv) - 2.25) < 1e-9
    assert reliable_offset(cv, 0.8) == 1.0


def test_result_metadata():
    m = metadata(constraints(), engine="test")
    for k in ("version", "constraints_hash", "seed", "python", "date", "engine"):
        assert k in m
