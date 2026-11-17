"""SP-5: printed-layer model."""
from stackpieces.geometry import PRESETS, make_piece, material_intervals
from stackpieces.surface import layers, ra_staircase, section


def test_layer_count_and_bounds():
    g = make_piece(PRESETS["S40"])
    L = layers(g, 0.2)
    zs = sorted({round(P[0][1], 6) for P in L})   # first vertex: bottom of the right cap = layer bottom
    # H = 22.13: layers start at 0.0, 0.2, ... 22.0 (mid-layer 22.1 < H)
    assert len(zs) == 111 and abs(zs[0]) < 1e-9 and abs(zs[-1] - 22.0) < 1e-9


def test_stadium_reaches_cad_radius_at_mid_layer():
    g = make_piece(PRESETS["S40"])
    for P in layers(g, 0.2):
        zm = P[2][1]                          # the caps' middle vertices are at mid-layer, at the CAD radii
        radii = [r for iv in material_intervals(g["profile"], zm) for r in iv]
        for tip in (P[2][0], P[7][0]):
            assert any(abs(abs(tip) - r) < 1e-9 for r in radii) or abs(tip) < 1e-9 and min(radii) < 1e-9


def test_hybrid_keeps_smooth_cone():
    g = make_piece(PRESETS["S40"])
    H = section(g, "hybrid")
    assert H[-1] == [list(v) for v in g["section"][3]] or H[-1] == g["section"][3]
    assert all(P[0][1] < g["cone_start"] for P in H[:-1])


def test_ra():
    assert abs(ra_staircase(0.2, 50) - 0.0321) < 1e-4
