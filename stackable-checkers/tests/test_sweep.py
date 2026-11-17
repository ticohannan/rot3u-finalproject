"""SP-11: the search ranks candidates and explains rejections."""
from helpers import constraints
from stackpieces.sweep import QUICK_GRID, search


def test_quick_search():
    recs = search(constraints(), QUICK_GRID)
    assert recs[0]["passes"] and recs[0]["capture_smooth"] >= recs[1]["capture_smooth"] - 1e-9 or not recs[1]["passes"]
    assert any(r["name"] == "S40-b12-s2-t4" and r["passes"] for r in recs)
    for r in recs:
        if not r["passes"]:
            assert r["reasons"], r["name"]
