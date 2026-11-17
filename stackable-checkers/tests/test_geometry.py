"""SP-2..SP-4: families, sizes, mass properties, checks."""
import math

from helpers import constraints
from stackpieces.geometry import PRESETS, GeometryError, checks, make_piece, volume_exact


def test_s40_matches_sw_parts_preview():
    g = make_piece(PRESETS["S40"])
    # volume of the SolidWorks part in cad/checker_s40 (same parameters): 5,659.463 mm3
    assert abs(volume_exact(g["profile"]) - 5659.463) < 0.01
    assert abs(g["V"] - 5659.463) / 5659.463 < 1e-3
    assert abs(g["H"] - 22.13) < 0.01 and g["seat"] == 12.0 and abs(g["king_h"] - 34.13) < 0.01
    assert abs(g["capture"] - 9.0) < 1e-9 and abs(g["Rm"] - 11.0) < 1e-9 and abs(g["bridge"] - 4.161) < 0.001


def test_grid_volume_close_to_exact():
    for k in PRESETS:
        g = make_piece(PRESETS[k])
        ex = volume_exact(g["profile"])
        assert abs(g["V"] - ex) / ex < 1e-3, k


def test_cylinder_analytic():
    g = make_piece(PRESETS["CYL"])
    R, H, ct, cb = 12.5, 15.0, 0.5, 0.4
    # cylinder minus two 45 deg chamfer rings (Pappus)
    ring = lambda a: 0.5 * a * a * 2 * math.pi * (R - a / 3)
    assert abs(volume_exact(g["profile"]) - (math.pi * R * R * H - ring(ct) - ring(cb))) < 1e-6


def test_n35_sizes():
    g = make_piece(PRESETS["N35"])
    assert abs(g["H"] - 28.709) < 0.01 and abs(g["seat"] - 13.428) < 0.01
    assert abs(g["capture"] - (11.5 - 0.8)) < 1e-9


def test_impossible_socket_raises():
    p = dict(PRESETS["S40"], alpha=25.0)          # socket deeper than the band
    try:
        make_piece(p)
    except GeometryError:
        return
    raise AssertionError("expected GeometryError")


def test_checks_s40():
    c = constraints()
    res = {x["id"]: x for x in checks(make_piece(PRESETS["S40"]), c)}
    for k in ("overhang", "bridge", "roof", "rim", "grip_band_min", "grip_band_max", "height"):
        assert res[k]["ok"], k
    assert not res["finger_room"]["ok"] and abs(res["finger_room"]["value"] - 10.921) < 0.001


def test_checks_n35_rim_fails():
    res = {x["id"]: x for x in checks(make_piece(PRESETS["N35"]), constraints())}
    assert not res["rim"]["ok"] and abs(res["rim"]["value"] - 0.6) < 1e-9
    assert not res["height"]["ok"]
