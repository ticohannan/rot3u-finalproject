"""SP-8: the MuJoCo model seats S40 near center and not far off it."""
try:
    import mujoco  # noqa: F401
    SKIP = None
except ImportError:
    SKIP = "mujoco not installed (pip install mujoco)"

from stackpieces.geometry import PRESETS, make_piece


def test_s40_3d_outcomes():
    from stackpieces.sim3d import Stack3D
    st = Stack3D(make_piece(PRESETS["S40"]))
    r0 = st.simulate(0, mu=0.4)
    assert r0["code"] == "seated" and abs(r0["dz"] - 12.0) < 0.05
    assert st.simulate(3, mu=0.4)["code"] == "seated"
    assert st.simulate(8, mu=0.4)["code"] != "seated"


def test_cylinder_3d_flat_stack():
    from stackpieces.sim3d import Stack3D
    r = Stack3D(make_piece(PRESETS["CYL"])).simulate(4, mu=0.4)
    assert r["code"] == "perched" and r["tilt"] < 1
