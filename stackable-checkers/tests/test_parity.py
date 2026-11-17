"""SP-7: web/sim2d.js and web/geometry.js agree with the Python package."""
import json

from helpers import NODE, constraints, node_eval
from stackpieces.geometry import PRESETS, checks, make_piece
from stackpieces.montecarlo import sample_releases
from stackpieces.sim2d import Section2D
from stackpieces.surface import section

SKIP = None if NODE else "Node.js not found; install Node to check the browser engine"

CASES = [  # (preset, variant, e, d, tilt, vx, mu)
    ("S40", "smooth", 0, 1, 0, 0, 0.4), ("S40", "smooth", 3, 1, 1.5, 5, 0.3), ("S40", "smooth", 8, 1, 0, 0, 0.4),
    ("S40", "layered", 2, 1.2, -1, 3, 0.25), ("S40", "hybrid", 4, 1, 0, 0, 0.3),
    ("N35", "smooth", 5, 1, 0, 0, 0.4), ("CYL", "smooth", 4, 1, 0, 0, 0.4),
]


def test_geometry_parity():
    for k in PRESETS:
        g = make_piece(PRESETS[k])
        js = node_eval(f"const g=G.makePiece({json.dumps(PRESETS[k])});"
                       f"console.log(JSON.stringify({{V:g.V,zc:g.zc,I:g.I,H:g.H,seat:g.seat,n:G.layers(g,0.2).length,L0:G.layers(g,0.2)[5]}}))")
        assert abs(js["V"] - g["V"]) < 1e-6 and abs(js["zc"] - g["zc"]) < 1e-9 and abs(js["I"] - g["I"]) < 1e-6, k
        from stackpieces.surface import layers
        L = layers(g, 0.2)
        assert js["n"] == len(L) and all(abs(a - b) < 1e-12 for p, q in zip(js["L0"], L[5]) for a, b in zip(p, q)), k


def test_checks_parity():
    c = constraints()
    for k in PRESETS:
        js = node_eval(f"console.log(JSON.stringify(G.checks(G.makePiece({json.dumps(PRESETS[k])}),{json.dumps(c)})))")
        py = checks(make_piece(PRESETS[k]), c)
        assert [(x["id"], x["ok"], x["value"]) for x in js] == [(x["id"], x["ok"], x["value"]) for x in py], k


def test_sampler_parity():
    ph = constraints()["physics"]
    js = node_eval(f"console.log(JSON.stringify(S.sampleReleases({json.dumps(ph)},1000,3,5)))")
    py = sample_releases(ph, 1000, 3, 5)
    for a, b in zip(js, py):
        for k in ("mu", "d", "tilt", "vx"):
            assert abs(a[k] - b[k]) < 1e-12


def test_simulation_parity():
    for k, v, e, d, tilt, vx, mu in CASES:
        g = make_piece(PRESETS[k])
        py = Section2D(g, section(g, v)).simulate(e, d=d, tilt=tilt, vx=vx, mu=mu)
        js = node_eval(f"const g=G.makePiece({json.dumps(PRESETS[k])});"
                       f"const r=S.simulate(g,G.section(g,'{v}'),{{e:{e},d:{d},tilt:{tilt},vx:{vx},mu:{mu}}});"
                       f"console.log(JSON.stringify({{code:r.code,up:r.up,t:r.t}}))")
        assert js["code"] == py["code"], (k, v, e, js["code"], py["code"])
        if py["code"] == "seated":
            assert abs(js["up"][0] - py["up"][0]) < 0.05 and abs(js["up"][1] - py["up"][1]) < 0.05
            assert abs(js["up"][2] - py["up"][2]) < 0.5 * 3.14159 / 180
