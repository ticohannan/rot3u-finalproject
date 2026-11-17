"""SP-6: the 2D section simulation gives the expected outcomes for S40 and the cylinder."""
from stackpieces.geometry import PRESETS, make_piece
from stackpieces.sim2d import Section2D
from stackpieces.surface import section


def test_s40_smooth_capture():
    g = make_piece(PRESETS["S40"])
    s = Section2D(g, section(g, "smooth"))
    assert s.simulate(0, d=1, mu=0.4)["code"] == "seated"
    r = s.simulate(2, d=1, mu=0.4)
    assert r["code"] == "seated" and abs(r["dz"] - 12.0) < 0.1
    assert s.simulate(8, d=1, mu=0.4)["code"] != "seated"


def test_s40_layered_seats_centered():
    g = make_piece(PRESETS["S40"])
    assert Section2D(g, section(g, "layered")).simulate(0.5, d=1, mu=0.3)["code"] == "seated"


def test_cylinder_stays_flat_off_center():
    g = make_piece(PRESETS["CYL"])
    r = Section2D(g, section(g, "smooth")).simulate(4, d=1, mu=0.4)
    assert r["code"] == "perched" and r["tilt"] < 1 and abs(r["dx"] - 4) < 0.5


def test_recording():
    g = make_piece(PRESETS["S40"])
    r = Section2D(g, section(g, "smooth")).simulate(3, d=1, mu=0.4, record=True)
    assert len(r["frames"]) > 10 and len(r["frames"][0]) == 7
