"""SP-1: the constraints file loads, has every group, and notes its numbers."""
from helpers import constraints

GROUPS = ("board", "piece", "gripper", "goal", "printer", "physics", "analysis")


def test_reference_constraints_load():
    c = constraints()
    for g in GROUPS:
        assert g in c, g
    assert c["piece"]["diameter_mm"] == 25.0
    assert c["gripper"]["grip_band_min_mm"] == 10.0
    lo, hi = c["physics"]["friction_range"]
    assert 0 < lo < hi < 1


def test_numbers_have_notes():
    c = constraints()
    missing = []
    for g in ("board", "piece", "gripper", "printer"):
        for k, v in c[g].items():
            if not k.startswith("_") and isinstance(v, (int, float)) and not isinstance(v, bool) and "_" + k not in c[g]:
                missing.append(f"{g}.{k}")
    assert not missing, f"numbers without a note: {missing}"
