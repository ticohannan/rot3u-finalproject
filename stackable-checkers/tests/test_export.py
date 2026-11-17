"""SP-12: STL, partspec, report."""
import json
import os
import tempfile

from helpers import REFERENCE
from stackpieces.export import partspec, write_stl
from stackpieces.geometry import PRESETS, make_piece, volume_exact
from stackpieces.report import write_report


def test_stl_watertight_and_volume():
    g = make_piece(PRESETS["S40"])
    with tempfile.TemporaryDirectory() as d:
        s = write_stl(os.path.join(d, "s40.stl"), g["profile"])
        assert s["watertight"] and os.path.getsize(os.path.join(d, "s40.stl")) == 84 + 50 * s["triangles"]
    assert abs(s["volume_mm3"] - volume_exact(g["profile"])) / volume_exact(g["profile"]) < 0.002


def test_partspec_s40_matches_sw_parts():
    spec = partspec(make_piece(PRESETS["S40"]), "checker_s40")
    p = spec["params"]
    assert (p["D"], p["HC"], p["SH"], p["TIP_D"], p["CONE_H"], p["CLR"]) == (25.0, 12.0, 2.0, 4.0, 10.13, 0.5)
    assert [f["op"] for f in spec["features"]] == ["revolve", "revolve_cut"]


def test_report_from_reference():
    ref = json.loads(REFERENCE.read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory() as d:
        path = write_report(ref, os.path.join(d, "r.html"))
        html = open(path, encoding="utf-8").read()
    assert html.startswith("<!DOCTYPE html>") and '"chosen":"S40"' in html and "__DATA__" not in html
