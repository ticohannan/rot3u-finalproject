"""Tests for Robot Demo 4 (PC only: no Arduino, no camera).

    python -m pytest tests -q            or            python tests/test_demo4.py

Tests named test_fw_* and *_sketch run the REAL sketch (tests/sim/sketch_host.cpp is a copy of
servo_bridge.ino) compiled for the PC with an emulated PCA9685. They need g++ and are skipped
without it (typical on Windows). Everything else uses the Python stand-in, sim.PyArm.
Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
import json, math, os, shutil, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np
import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from rot3u_demo4 import camdiag, control, geometry, link, profiles, robot, robot_view, sim, vision   # noqa: E402

SIMDIR = HERE / "sim"
EXE = SIMDIR / ("sim_servo_bridge.exe" if os.name == "nt" else "sim_servo_bridge")
RED = vision.PRESETS["red"]


class Skip(Exception):
    pass


def skip(why):
    try:
        import pytest
        pytest.skip(why)
    except ImportError:
        raise Skip(why)


def exe():
    """Build the PC simulation of the sketch (once), from the sketch next to this package if it is there."""
    if not shutil.which("g++"):
        skip("g++ not found: tests of the real sketch are skipped")
    src = HERE.parent.parent / "arduino" / "servo_bridge" / "servo_bridge.ino"
    host = SIMDIR / "sketch_host.cpp"
    if src.exists() and src.read_text() != host.read_text():
        host.write_text(src.read_text())
        EXE.unlink(missing_ok=True)
    if not EXE.exists() or EXE.stat().st_mtime < host.stat().st_mtime:
        r = subprocess.run(["g++", "-O1", "-std=c++17", "-Wno-narrowing", f"-I{SIMDIR}", f"-I{SIMDIR / 'shim'}",
                            "-o", str(EXE), str(SIMDIR / "sim_main.cpp")], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr[-3000:]
    return EXE


def fw(lines, state=None):
    """Run the real sketch on a script. Returns (text lines, writes [(ch, counts, ms)])."""
    r = subprocess.run([str(exe())] + ([str(state)] if state else []), input="\n".join(lines) + "\n",
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    out = [l for l in r.stdout.splitlines() if not l.startswith("@t ")]
    text = [l for l in out if not l.startswith("W,")]
    writes = [tuple(int(v) for v in l.split(",")[1:]) for l in out if l.startswith("W,")]
    return text, writes


def us(counts):
    return counts * sim.US_PER_COUNT


def make(arm=None, pose="WALL", power=True, **kw):
    w = sim.SimWorld(arm=arm, **kw)
    L = link.BridgeLink(sim.SimTransport(w), threaded=False, clock=lambda: w.t)
    L.connect()
    if power:
        robot.power_up(L, robot.pose_us(pose), sleep=w.advance, say=lambda s: None)
    rig = control.Rig(L, sim.SimCamera(w), RED, sleep=w.advance, now=lambda: w.t)
    return w, L, rig


def run_tracker(trk, w, frames):
    recs = []
    for _ in range(frames):
        f, t, _ = trk.rig.camera.read()
        recs.append(trk.step(f, t, want_view=False)[0])
    return recs


def center_err(w):
    p = w.true_pixel()
    return math.hypot(p[0] - w.W / 2, p[1] - w.H / 2), p


# ================================================================================ vision
def synth(blobs, size=(480, 640), bg=(100, 20, 120)):
    hsv = np.empty(size + (3,), np.uint8)
    hsv[:] = bg
    for (cx, cy, half, col) in blobs:
        cv2.rectangle(hsv, (cx - half, cy - half), (cx + half, cy + half), col, -1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_matlab_threshold_conversion():
    c = vision.ColorRange.from_matlab("handout", 0.694, 0.000, 0.059, 1.000, 0.141, 0.882)
    assert c.ranges == [((125, 15, 36), (179, 255, 225)), ((0, 15, 36), (0, 255, 225))]
    c = vision.ColorRange.from_matlab("plain", 0.10, 0.20, 0.5, 1.0, 0.2, 1.0)
    assert c.ranges == [((18, 128, 51), (36, 255, 255))]


def test_blob_largest_component_centroid():
    img = synth([(200, 150, 30, (2, 220, 200)), (450, 300, 12, (2, 220, 200)), (100, 400, 40, (60, 220, 200))])
    mask, d = vision.detect(img, RED, "blob")
    assert d.found and abs(d.cx - 200) < 1.0 and abs(d.cy - 150) < 1.0 and d.n_blobs == 2
    assert abs(d.area - 61 * 61) < 200
    _, none = vision.detect(synth([(100, 400, 40, (60, 220, 200))]), RED, "blob")
    assert not none.found and math.isnan(none.cx)


def test_red_hue_wraps():
    img = synth([(150, 150, 30, (176, 220, 200)), (400, 150, 30, (4, 220, 200))])
    mask = vision.make_mask(img, RED)
    assert mask[150, 150] == 255 and mask[150, 400] == 255 and mask[400, 300] == 0


def test_blob_orientation_from_moments():
    hsv = np.full((480, 640, 3), (100, 20, 120), np.uint8)
    box = cv2.boxPoints(((320, 240), (200, 60), 30.0))            # long side 30 deg below the +x axis (y down)
    cv2.fillConvexPoly(hsv, np.int32(box), (2, 220, 200))
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    _, a, b = vision.detect(img, RED, both=True)
    assert abs(a.angle_deg - 30) < 2 and abs(b.angle_deg - 30) < 3
    assert abs(b.size_px[0] - 200) < 8 and abs(b.size_px[1] - 60) < 8


def test_edges_recover_block_under_glare():
    w, L, rig = make(glare=True, clutter=False)
    f, _, _ = rig.camera.read()
    _, a, b = vision.detect(f, RED, both=True)
    tx, ty, _ = w.true_pixel()
    ea, eb = math.hypot(a.cx - tx, a.cy - ty), math.hypot(b.cx - tx, b.cy - ty)
    assert b.note == "edges" and eb < 6 and ea > 12, (ea, eb)     # the centroid is pulled off; the rectangle is not


def _picker_file(d, name, h, sh, s_, ss, v, sv, n=200, exclude=False, orientation=None):
    rows = {"H": {"mean": h, "std": sh, "min": 0, "max": 0}, "S": {"mean": s_, "std": ss, "min": 0, "max": 0},
            "V": {"mean": v, "std": sv, "min": 0, "max": 0}}
    meta = {"tool": "camera_color_picker 0.2.0", "sampling": {"mode": "Cluster", "exclude": exclude, "k": 2.0},
            "stats": {"n": n, "kept": n, "rows": rows}, "orientation": orientation or {"rotate_cw_deg": 0, "flip_h": False, "flip_v": False}}
    p = os.path.join(d, name)
    json.dump(meta, open(p, "w"))
    return p


def test_color_model_mean_k_sigma():
    d = tempfile.mkdtemp()
    p = _picker_file(d, "red.json", 177.7, 4.1, 172, 30, 150, 34)
    m = vision.ColorModel.from_picker_json(p, "red", k=(2.5, 2.5, 3.0))
    assert m.range().ranges == [((167, 97, 48), (179, 247, 252)), ((0, 97, 48), (8, 247, 252))]   # 177.7 -/+ 10.25: H wraps through 0
    assert vision.ColorRange.from_picker_json(p, "red").ranges == m.range().ranges
    assert m.range(k=(30, 1, 1)).ranges == [((0, 142, 116), (179, 202, 184))]                   # k*sH over half a turn: all hues
    q = os.path.join(d, "colors.json")
    vision.save_colors(q, {"red": m, "plain": vision.PRESETS["green"]})
    back = vision.load_colors(q)
    assert isinstance(back["red"], vision.ColorModel) and back["red"].range().ranges == m.range().ranges
    assert back["plain"].ranges == vision.PRESETS["green"].ranges


def test_color_model_pools_captures_across_hue_zero():
    d = tempfile.mkdtemp()
    a = _picker_file(d, "lit.json", 176.0, 3.0, 180, 20, 190, 20, n=300)
    b = _picker_file(d, "shade.json", 4.0, 3.0, 160, 20, 90, 20, n=100)
    m = vision.ColorModel.from_picker_json([a, b], "red")
    assert abs(vision._hue_diff(m.mean[0], 178.0)) < 0.2          # 3/4 at 176 and 1/4 at 4 (= 184): 178, not 132
    sv = math.sqrt(0.75 * (20 ** 2 + 25 ** 2) + 0.25 * (20 ** 2 + 75 ** 2))   # spread of the means adds to the variance
    assert abs(m.mean[2] - 165) < 0.01 and abs(m.std[2] - sv) < 0.01 and m.n == 400 and m.sources == ["lit.json", "shade.json"]


def test_color_model_from_real_picker_statistics():
    try:
        sys.path.insert(0, str(HERE.parent.parent / "camera-color-picker"))
        from camera_color_picker.analysis import cluster_stats
    except ImportError:
        skip("camera-color-picker not next to this folder")
    rng = np.random.default_rng(3)
    h = rng.normal(178, 4, 400) % 180; s_ = np.clip(rng.normal(170, 30, 400), 0, 255); v = np.clip(rng.normal(150, 35, 400), 0, 255)
    bgr = cv2.cvtColor(np.stack([h, s_, v], 1).astype(np.uint8)[None], cv2.COLOR_HSV2BGR)[0]
    st = cluster_stats(bgr, exclude=False)
    d = tempfile.mkdtemp(); p = os.path.join(d, "cap.json")
    json.dump({"sampling": {"exclude": False}, "stats": {"n": st["n"], "kept": st["kept"], "rows": st["rows"]}}, open(p, "w"), default=float)
    m = vision.ColorModel.from_picker_json(p, "red")
    assert abs(vision._hue_diff(m.mean[0], 178)) < 1.5 and 2.5 < m.std[0] < 6 and abs(m.mean[1] - 170) < 6 and 20 < m.std[1] < 40
    lo_h = m.range().ranges[0][0][0]
    assert m.range().ranges[1][1][0] < 15 and lo_h > 160


def test_camera_opens_as_the_picker_had_it():
    calls = []

    class FakeCap:
        def __init__(self, i, backend=None): calls.append(("open", i)); self.p = {}
        def isOpened(self): return True
        def set(self, k, v): calls.append((k, v)); self.p[k] = v
        def get(self, k): return self.p.get(k, 0)
        def read(self): return True, np.zeros((720, 1280, 3), np.uint8)
        def release(self): pass

    real = cv2.VideoCapture
    cv2.VideoCapture = FakeCap
    try:
        d = tempfile.mkdtemp(); p = os.path.join(d, "cap.json")
        json.dump({"source": {"name": "HBVCAM camera", "index": 1}, "driver": {"w": 1280, "h": 720, "fps": 10.0, "fmt": "YUY2"},
                   "orientation": {"rotate_cw_deg": 90, "flip_h": False, "flip_v": False}}, open(p, "w"))
        cam = vision.Camera.from_picker_json(p, warmup=0)
        f, _, _ = cam.read()
        cam.close()
    finally:
        cv2.VideoCapture = real
    assert calls[0] == ("open", 1) and calls[1] == (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
    assert (cv2.CAP_PROP_FRAME_WIDTH, 1280) in calls and (cv2.CAP_PROP_FPS, 10.0) in calls
    assert f.shape == (1280, 720, 3) and (cam.width, cam.height) == (720, 1280)     # turned 90 deg like the picker


def test_orientation_matches_picker():
    img = np.zeros((4, 6, 3), np.uint8); img[0, 5] = 255                # top-right pixel
    o = vision.orient(img, 90, True, False)                               # mirror (-> top-left), then 90 cw (-> top-right of 6x4)
    assert o.shape == (6, 4, 3) and o[0, 3, 0] == 255
    d = tempfile.mkdtemp()
    p = _picker_file(d, "c.json", 10, 2, 200, 10, 200, 10, orientation={"rotate_cw_deg": 180, "flip_h": False, "flip_v": True})
    assert vision.orientation_from_picker_json(p) == (180, False, True)


def test_annotate_side_by_side():
    img = synth([(200, 150, 30, (2, 220, 200))])
    mask, a, b = vision.detect(img, RED, both=True)
    v = vision.annotate(img, mask, a, b, lines=["x"])
    assert v.shape == (480, 1280, 3)
    assert tuple(v[150, 200 + 9]) == vision.MAGENTA or (v[140:160, 190:210] == vision.MAGENTA).all(-1).any()
    assert (v[140:160, 640 + 190:640 + 210] == vision.MAGENTA).all(-1).any()     # marker on the thresholded image too


# ============================================================================== the sketch
def test_fw_boot_cold_outputs_off():
    text, writes = fw(["ST", "@adv 50"])
    assert text[-2] == "HELLO,servo_bridge,0.3,2"
    assert all(k == 4096 for _, k, _ in writes)                      # every write at boot is "output off"
    st = link.parse_state(text[-1])
    assert st.active == 0 and not st.moving


def test_fw_no_pulse_before_on():
    text, writes = fw(["T 1500 1500 1500 1000 1500 1500", "TC 3 1000", "@adv 500", "ST", "@adv 5"])
    assert sum("not started" in l for l in text) == 2
    assert all(k == 4096 for _, k, _ in writes)
    assert link.parse_state(text[-1]).active == 0


def test_fw_tagged_framing_and_fail():
    text, _ = fw(["@7 ON 0 1500", "@8 ON 0 1500", "@9 FOO", "@10 ST", "@adv 5"])
    assert "ACK,7" in text and "DONE,7,OK" in text and "DONE,8,FAIL" in text and "DONE,9,FAIL" in text
    i = text.index("ACK,10")
    assert text[i + 1].startswith("ST,") and text[i + 2] == "DONE,10,OK"


def test_fw_limits_clamp_targets():
    # v0.3: built-in limits are the servo's 500-2500 us; the robot's own limits arrive by LIM (robot.apply_limits)
    text, writes = fw(["ON 0 1500", "ON 3 1000", "ON 5 9999", "VA 3000 30000", "T 9999 - - 9999", "@adv 3000",
                       "T 0 - - 0", "@adv 3000", "LIM 3 600 1400", "TC 3 9999", "@adv 2000", "LIM 3 400 1400",
                       "LIM 0 530 2470", "TC 0 9999", "@adv 2000", "ST", "@adv 5"])
    hi = {c: max(k for ch, k, _ in writes if ch == c and k < 4096) for c in (0, 3, 5)}
    lo = {c: min(k for ch, k, _ in writes if ch == c and k < 4096) for c in (0, 3)}
    assert us(hi[0]) <= 2500 + 2.5 and us(hi[3]) <= 2500 + 2.5 and us(hi[5]) <= 2500 + 2.5
    assert us(lo[0]) >= 500 - 2.5 and us(lo[3]) >= 500 - 2.5
    st = link.parse_state(text[-1])
    assert st.pos[3] == 1400 and st.pos[0] == 2470                  # the limits sent by LIM hold
    assert any("inside the built-in limits" in l for l in text)      # outside 500-2500 is refused


def test_robot_config_read_and_limits_sent():
    root = HERE.parent.parent / "robots"
    c = robot.parse_config(root / "sir_pinch_a_lot_revB" / "robot_config.h")
    assert c["us0"][2] == 332.1 and c["uspd"][3] == -11.111 and c["max"] == [2470, 2470, 2020, 1470, 2500, 1920]
    assert (c["grip_open"], c["grip_hold"], c["d1"], c["tip"]) == (1150, 1580, 113.0, 174.0) and c["cal_valid"]
    try:
        robot.load_config(root / "template" / "robot_config.h"); raise AssertionError("template accepted")
    except ValueError as e:
        assert "CAL_VALID" in str(e)
    d = tempfile.mkdtemp()
    bad = os.path.join(d, "robot_config.h")
    with open(bad, "w") as f:
        f.write(open(root / "sir_pinch_a_lot_revB" / "robot_config.h").read().replace("{  1, J_ROT", "{  7, J_ROT"))
    try:
        robot.parse_config(bad); raise AssertionError("wrong channel accepted")
    except ValueError as e:
        assert "channels 0-4" in str(e)
    # another robot: limits and calibration follow the file, and power-up sends the limits first
    other = os.path.join(d, "other.h")
    with open(other, "w") as f:
        f.write(open(root / "sir_pinch_a_lot_revB" / "robot_config.h").read()
                .replace("1475.6,   10.901,  530, 2470", "1500.0,  -10.800,  600, 2400").replace('"Sir Pinch-a-lot', '"Test arm'))
    saved = (list(robot.US0), list(robot.USPD), list(robot.MIN_US), list(robot.MAX_US), robot.ROBOT_NAME, robot.CONFIG_PATH)
    try:
        robot.load_config(other)
        assert robot.USPD[0] == -10.8 and robot.MIN_US[0] == 600 and "Test arm" in robot.describe()
        w = sim.SimWorld(clutter=False)
        L = link.BridgeLink(sim.SimTransport(w), threaded=False, clock=lambda: w.t)
        L.connect()
        robot.power_up(L, robot.pose_us("WALL"), sleep=lambda s: w.advance(s), say=lambda s: None)
        L.cmd("TC 0 9999")
        robot.wait_done(L, sleep=lambda s: w.advance(s))
        assert L.get_state().pos[0] == 2400
    finally:
        robot.US0[:], robot.USPD[:], robot.MIN_US[:], robot.MAX_US[:] = saved[:4]
        robot.ROBOT_NAME, robot.CONFIG_PATH = saved[4], saved[5]


def test_fw_profile_speed_accel_no_overshoot():
    text, writes = fw(["ON 0 1000", "VA 400 1000", "TC 0 1800", "@adv 4000", "ST", "@adv 5"])
    w = [(ms, us(k)) for ch, k, ms in writes if ch == 0 and k < 4096]
    t = np.array([a for a, _ in w]) / 1000.0
    p = np.array([b for _, b in w])
    assert abs(p[-1] - 1800) < 2.5 and p.max() <= p[-1] + 0.01 and (np.diff(p) >= 0).all()   # monotonic, no overshoot
    tt = np.arange(t[1], t[-1], 0.5)
    pp = np.interp(tt, t, p)
    v = np.diff(pp) / 0.5
    assert 380 <= v.max() <= 400 * 1.04                               # speed limit (0.5 s windows; one count = 4.9 us)
    assert pp[1] - pp[0] < 0.9 * 400 * 0.5                            # the first half second includes the ramp up
    assert 800 / 400 + 0.15 <= t[-1] - t[1] <= 800 / 400 + 400 / 1000 + 0.1   # trapezoid: distance / v + v / a = 2.4 s


def test_fw_stop_holds_three_ways():
    for stopper in ("X", "", "GARBAGE 1 2"):
        text, writes = fw(["ON 0 1000", "VA 300 1200", "TC 0 2000", "@adv 1500", stopper, "@adv 5", "ST", "@adv 2000", "ST", "@adv 5"])
        s1, s2 = [link.parse_state(l) for l in text if l.startswith("ST,")]
        assert 1100 < s1.pos[0] < 1900 and s1.pos == s2.pos and not s2.moving and s2.active == 1, stopper
        last = max(ms for ch, k, ms in writes if ch == 0)
        assert last <= 205 + 1500 + 30                                # no pulse changes after the stop


def test_fw_stray_key_without_line_end_stops():
    text, _ = fw(["ON 0 1000", "TC 0 2000", "@adv 1000", "@raw q", "@adv 200", "ST", "@adv 1000", "ST", "@adv 5"])
    s1, s2 = [link.parse_state(l) for l in text if l.startswith("ST,")]
    assert s1.pos == s2.pos and not s2.moving


def test_fw_crlf_line_endings_do_not_stop():
    text, _ = fw(["ON 0 1000", "@crlf TC 0 1300", "@adv 3000", "ST", "@adv 5"])
    assert link.parse_state(text[-1]).pos[0] == 1300


def test_fw_line_too_long():
    text, _ = fw(["ON 0 1000", "TC 0 1100" + " " * 60 + "1", "@adv 500", "ST", "@adv 5"])
    assert any(l.startswith("ERR,") and l.endswith("LINE_TOO_LONG") for l in text)
    assert link.parse_state(text[-1]).pos[0] == 1000


def test_fw_adopts_after_reset_without_moving():
    st = Path(tempfile.mkdtemp()) / "pca.bin"
    fw(["ON 0 1500", "ON 1 1365", "ON 3 1019", "TC 0 1600", "@adv 3000"], state=st)
    text, writes = fw(["ST", "@adv 2000", "TC 0 1500", "@adv 3000", "ST", "@adv 5"], state=st)
    assert sum(l.startswith("# adopted ch") for l in text) == 3
    s = [link.parse_state(l) for l in text if l.startswith("ST,")]
    assert s[0].active == 0b1011 and abs(s[0].pos[0] - 1600) <= 3 and abs(s[0].pos[3] - 1019) <= 3
    first_write = min(ms for _, _, ms in writes)
    assert first_write > 2000                                         # nothing written until the first command
    steps = np.abs(np.diff([k for ch, k, _ in writes if ch == 0]))
    assert steps.max() <= 2 and abs(s[1].pos[0] - 1500) <= 1          # and then it moves smoothly, no jump


def test_fw_off_only_on_request():
    text, writes = fw(["ON 0 1500", "ON 3 1000", "@adv 100", "OFF 3", "@adv 100", "ST", "OFF *", "ST", "@adv 5"])
    s = [link.parse_state(l) for l in text if l.startswith("ST,")]
    assert s[0].active == 1 and s[1].active == 0


def test_pyarm_matches_sketch():
    script = ["ON 0 1500", "ON 3 1000", 50, "VA 300 1200", "T 1700 - - 1100", 400, "T 1400 - - 1004.9", 3000,
              "TC 0 1404.889", 500, "VA 800 6000 0", "TC 0 2000", 300, "X", 200, "TC 0 9999", 2500, "LIM 0 600 2000", 1500]

    def trace(arm):
        out = []
        for item in script:
            if isinstance(item, str):
                arm.feed(item)
            else:
                for _ in range(item // 5):
                    arm.advance(5)
                    out.append([v or 0 for v in arm.pulses()])
        return np.array(out), [l for l in arm.pop_lines() if not l.startswith("#")]

    a, la = trace(sim.PyArm())
    e = sim.ExeArm(exe())
    b, lb = trace(e)
    e.close()
    assert np.abs(a - b).max() < 0.01 and la == lb


# ================================================================================ control
def test_pid_converges_and_does_not_wind_up():
    pid = control.PID(kp=0.3, ki=3.0, kd=0.02)
    pid.reset(1000.0)
    pos, goal, dt = 1000.0, 1300.0, 1 / 30
    lead = []
    for i in range(300):                               # actuator limited to 100 us/s: far slower than the PID wants
        u = pid.update(goal - pos, dt, actual=pos)
        pos += max(-100 * dt, min(100 * dt, u - pos))
        lead.append(u - goal)
    assert abs(pos - goal) < 1.0 and max(lead) < 20     # arrives, and the target never ran far past the goal
    pid2 = control.PID(kp=0.0, ki=3.0, kd=0.0, out_min=900, out_max=1100)
    pid2.reset(1000.0)
    for i in range(300):
        u = pid2.update(500.0, dt)
    assert u == 1100 and pid2.i == 1100                 # clamped at the limit, integrator too


def test_jacobian_and_lost_motion_recovered():
    for roll in (0.0, 25.0, 180.0):
        w, L, rig = make(cam_roll_deg=roll, clutter=False)
        jac = control.measure_jacobian(rig, say=lambda s: None)
        truth = np.zeros((2, 2))
        p0 = np.array(w._pixel(w.y, w.target)[:2])
        for k, j in enumerate((0, 3)):
            y = list(w.y); y[j] += 20
            truth[:, k] = (np.array(w._pixel(y, w.target)[:2]) - p0) / 20
        assert np.abs(jac.J - truth).max() < 0.12 * np.abs(truth).max(), (roll, jac.J, truth)
        assert abs(jac.lost_us[0] - 11.6) < 3.0 and abs(jac.lost_us[1] - 3.6) < 3.0, jac.lost_us
        st = L.get_state()
        assert st.tgt[0] == 1476 and st.tgt[3] == 1019   # the arm is back where it started
        lo, hi = jac.deadband_px()
        assert 4 <= lo <= 14 and hi > lo


def test_jacobian_refuses_when_block_missing():
    w, L, rig = make(clutter=False)
    rig.color = vision.PRESETS["green"]
    try:
        control.measure_jacobian(rig, say=lambda s: None)
        assert False
    except RuntimeError as e:
        assert "not seen" in str(e)


def test_slop_staircase_measures_the_model():
    w, L, rig = make(clutter=False)
    jac = control.measure_jacobian(rig, say=lambda s: None)
    s0 = control.measure_slop(rig, jac, 0, say=lambda s: None)
    s1 = control.measure_slop(rig, jac, 1, say=lambda s: None)
    assert abs(s0["width_us"] - 11.6) < 2.5 and abs(s1["width_us"] - 3.6) < 2.5, (s0["width_us"], s1["width_us"])
    assert 0.85 < s0["gain"] < 1.15 and len(s0["cmd_us"]) == 40
    w2, L2, rig2 = make(clutter=False, servos=[sim.ServoModel(lash=24.0)] + [sim.ServoModel(loaded=True) for _ in range(5)])
    jac2 = control.measure_jacobian(rig2, say=lambda s: None)
    assert abs(control.measure_slop(rig2, jac2, 0, say=lambda s: None)["width_us"] - 27.6) < 3.5


def test_step_response_metrics():
    w, L, rig = make(clutter=False)
    jac = control.measure_jacobian(rig, say=lambda s: None)
    r = control.step_response(rig, jac, 0, 60, v=400, a=2000, say=lambda s: None)
    assert abs(r["final_us"] - 60) < 8 and 0.08 < r["dead_time"] < 0.45 and r["overshoot"] < 0.15
    assert len(r["t"]) == len(r["resp_us"]) == len(r["sketch_us"]) and np.nanmax(r["sketch_us"]) >= 58


def _track_scenario(arm=None, roll=0.0, mode="2AXIS", method="blob", **kw):
    w, L, rig = make(arm=arm, cam_roll_deg=roll, clutter=False, **kw)
    rig.method = method
    jac = control.measure_jacobian(rig, say=lambda s: None)
    cfg = control.TrackerConfig(mode=mode)
    cfg.deadband_in_px, cfg.deadband_out_px = jac.deadband_px()
    trk = control.Tracker(rig, jac.J, cfg)
    trk.start()
    return w, L, rig, trk, cfg


def _check_2axis(arm=None, roll=0.0, method="blob", **kw):
    w, L, rig, trk, cfg = _track_scenario(arm, roll, method=method, **kw)
    # placements in an order that keeps the block in view after each jump (47.9 deg lens, block about 260 mm away)
    for tg in [(450, 40, 280), (440, 60, 250), (455, 20, 260), (450, -30, 300), (430, -60, 330)]:
        w.target = np.array(tg, float)
        recs = run_tracker(trk, w, 150)                               # 5 s per placement
        states = [r["state"] for r in recs]
        first = states.index("CENTERED")
        assert first < 100, (tg, first)                               # centered within 3.3 s
        assert all(s == "CENTERED" for s in states[first + 8:]), tg   # and it stays: no hunting
        err, p = center_err(w)
        assert err <= cfg.deadband_out_px + 3, (tg, err, p)
        u = np.array([[r["u0"], r["u1"]] for r in recs])
        assert (u[:, 0] >= robot.MIN_US[0]).all() and (u[:, 1] <= robot.MAX_US[3]).all()
    return trk


def test_track_2axis():
    _check_2axis()


def test_track_2axis_real_sketch():
    e = sim.ExeArm(exe())
    try:
        _check_2axis(arm=e)
    finally:
        e.close()


def test_track_2axis_camera_rotated_and_upside_down():
    _check_2axis(roll=35.0)
    _check_2axis(roll=180.0)


def test_track_2axis_edges_method():
    _check_2axis(method="edges")


def test_track_with_more_latency_and_slop():
    _check_2axis(latency_s=0.22, servos=[sim.ServoModel(dead=8.0, lash=20.0)] + [sim.ServoModel(dead=8.0, loaded=True) for _ in range(5)])
    _check_2axis(latency_s=0.05, servos=[sim.ServoModel(dead=2.0, lash=0.0) for _ in range(6)])


def test_track_with_backlash_compensation():
    w, L, rig, trk, cfg = _track_scenario()
    cfg.lash_us = (10.0, 3.0)
    for tg in [(450, 40, 280), (430, -60, 330), (440, 60, 250), (450, -30, 300)]:
        w.target = np.array(tg, float)
        states = [r["state"] for r in run_tracker(trk, w, 150)]
        first = states.index("CENTERED")
        assert first < 100 and all(s == "CENTERED" for s in states[first + 8:]), tg
        assert center_err(w)[0] <= cfg.deadband_out_px + 3


def test_track_1axis_moves_only_the_base():
    w, L, rig, trk, cfg = _track_scenario(mode="1AXIS")
    wrist0 = L.get_state().tgt[3]
    for tg in [(450, -50, 300), (450, 70, 300), (450, 0, 320)]:      # 320: also off in height, which 1AXIS leaves alone
        w.target = np.array(tg, float)
        recs = run_tracker(trk, w, 150)
        assert recs[-1]["state"] == "CENTERED", tg
        p = w.true_pixel()
        assert abs(p[0] - w.W / 2) <= cfg.deadband_out_px + 3, (tg, p)
    assert L.get_state().tgt[3] == wrist0
    assert all(r["u1"] == recs[0]["u1"] for r in trk.log)


def test_track_holds_when_block_disappears():
    w, L, rig, trk, cfg = _track_scenario()
    w.target = np.array((440, -50, 320), float)
    run_tracker(trk, w, 12)                                           # under way
    w.target = np.array((-2000, 0, 300), float)                       # gone
    recs = run_tracker(trk, w, 90)
    assert recs[-1]["state"] == "LOST"
    u = [(r["u0"], r["u1"]) for r in recs[8:]]
    assert len(set(u)) == 1                                           # targets frozen
    assert not L.get_state().moving


def test_track_stops_at_joint_limit():
    w, L, rig = make(clutter=False)
    L.cmd("LIM 0 1400 1560")
    jac = control.measure_jacobian(rig, say=lambda s: None)
    trk = control.Tracker(rig, jac.J, control.TrackerConfig())
    trk.pid[0].out_min, trk.pid[0].out_max = 1410, 1550
    trk.start()
    w.target = np.array((430, 90, 285), float)                        # needs more base travel than allowed
    recs = run_tracker(trk, w, 150)
    assert recs[-1]["state"] == "AT_LIMIT" and max(r["u0"] for r in recs) <= 1550.01
    assert L.get_state().pos[0] <= 1560


def test_tracker_needs_jacobian_and_started_channels():
    w, L, rig = make(power=False)
    for J in (None, np.eye(2)):
        try:
            control.Tracker(rig, J).start()
            assert False
        except RuntimeError as e:
            assert "Jacobian" in str(e) or "not started" in str(e)


def test_run_loop_records_and_holds(tmp=None):
    w, L, rig, trk, cfg = _track_scenario()
    d = tempfile.mkdtemp()
    w.target = np.array((440, -50, 320), float)
    log = trk.run(seconds=3.0, show=None, record=os.path.join(d, "out.mp4"))
    trk.save_log(os.path.join(d, "log.csv"))
    assert os.path.getsize(os.path.join(d, "out.mp4")) > 5000 and len(open(os.path.join(d, "log.csv")).readlines()) == len(log) + 1
    assert log[-1]["state"] == "CENTERED" and not L.get_state().moving


def test_link_log_and_poses():
    d = tempfile.mkdtemp()
    w = sim.SimWorld()
    L = link.BridgeLink(sim.SimTransport(w), log_path=os.path.join(d, "s.csv"), threaded=False, clock=lambda: w.t)
    L.connect()
    assert not L.cmd("TC 0 1500").ok and L.cmd("ON 0 1500").ok
    L.close()
    rows = open(os.path.join(d, "s.csv")).read().splitlines()
    assert rows[0] == "t_host_s,dir,line" and any(",TX,@" in r for r in rows) and any(",RX,DONE," in r for r in rows)
    assert robot.pose_us("WALL") == [1476, 1365, 854, 1019, 500, 1580]
    try:
        robot.pose_us([0, 90, -45, -100, 90])                        # wrist past its limit
        assert False
    except ValueError:
        pass


def test_threaded_link_realtime_sim():
    w = sim.SimWorld(realtime=True)
    L = link.BridgeLink(sim.SimTransport(w))
    L.connect(timeout=3)
    import time
    robot.power_up(L, robot.pose_us("WALL"), pause=0.02, say=lambda s: None)
    L.cmd("STREAM 50")
    time.sleep(0.3)
    assert L.state is not None and L.state.active == 63
    cam = sim.SimCamera(w)
    f, t, n = cam.read()
    assert f.shape == (480, 640, 3)
    L.close()


def test_notebook_runs_on_the_simulator():
    """Runs demo4_tracking.ipynb with PORT = "SIM" in a copy of the demo4 folder (logs, videos and profiles
    written there, not here). The live k-slider cells (A.7) are left out."""
    try:
        import nbformat
        from nbclient import NotebookClient
    except ImportError:
        skip("nbformat / nbclient not installed")
    work = Path(tempfile.mkdtemp()) / "python"
    for d in ("rot3u_demo4", "captures", "profiles"):
        if (HERE.parent / d).exists():
            shutil.copytree(HERE.parent / d, work / d, ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(HERE.parent.parent / "robots", work.parent / "robots")
    nb = nbformat.read(HERE.parent / "demo4_tracking.ipynb", as_version=4)
    nb.cells = [c for c in nb.cells if not (c.cell_type == "code" and c.source.startswith(("# ===================== A.7", "# ---- A.7")))]
    for c in nb.cells:
        if c.cell_type == "code":
            c.source = c.source.replace('PORT = "COM3"', 'PORT = "SIM"').replace('SHOW = "window"', "SHOW = None")
    NotebookClient(nb, timeout=400, kernel_name="python3", resources={"metadata": {"path": str(work)}}).execute()
    outs = "".join(str(o.get("text", "")) for c in nb.cells if c.cell_type == "code" for o in c.get("outputs", []))
    assert "CENTERED" in outs and "px/us" in outs and "green wooden block #1, in the living room" in outs
    assert "without shaping" in outs and "Hz" in outs
    assert list((work / "logs").glob("*_session.json"))

# ======================================================================= v0.2: input shaping
def test_fw_shape_arguments():
    text, _ = fw(["@1 SHAPE 0 2.5 0.1", "@2 SHAPE 0 20 0.1", "@3 SHAPE 0 2 0.9", "@4 SHAPE 7 2 0.1", "@5 SHAPE * 0",
                  "@6 SHAPE 0 1.5 0.2", "SHAPE"])
    done = {l.split(",")[1]: l.split(",")[2] for l in text if l.startswith("DONE,")}
    assert done == {"1": "OK", "2": "FAIL", "3": "FAIL", "4": "FAIL", "5": "OK", "6": "OK"}, done
    assert any("1.50 Hz" in l for l in text)


def test_pyarm_matches_sketch_with_shaping():
    script = ["ON 0 1500", "ON 3 1000", 50, "SHAPE 0 2.4 0.1", "VA 600 4000", "T 1700 - - 1100", 600, "SHAPE * 1.5 0.2",
              "T 1400 - - 1004.9", 3000, "TC 0 1404.889", 500, "VA 800 6000 0", "TC 0 2000", 300, "X", 200,
              "TC 0 9999", 2500, "SHAPE 0 0", "LIM 0 600 2000", 1500, "SHAPE"]

    def trace(arm):
        out = []
        for item in script:
            if isinstance(item, str):
                arm.feed(item)
            else:
                for _ in range(item // 5):
                    arm.advance(5)
                    out.append([v or 0 for v in arm.pulses()])
        return np.array(out), [l for l in arm.pop_lines() if not l.startswith("#")]

    a, la = trace(sim.PyArm())
    e = sim.ExeArm(exe())
    b, lb = trace(e)
    e.close()
    assert np.abs(a - b).max() < 0.01 and la == lb


def test_fw_shaped_move_is_two_steps_and_x_holds_at_once():
    # f = 1 Hz, z = 0: half period 500 ms, amplitudes 1/2 and 1/2. A quick 100 us move (done in about 0.12 s)
    # goes halfway, waits, then finishes half a period later.
    text, writes = fw(["ON 0 1000", "@adv 100", "VA 3000 30000 0", "SHAPE 0 1 0", "TC 0 1100", "@adv 300", "STATE",
                       "@adv 400", "STATE", "TC 0 1000", "@adv 300", "X", "STATE", "@adv 600", "STATE"])
    st = [link.parse_state(l) for l in text if l.startswith("ST,")]
    assert 1045 < st[0].pos[0] < 1055 and st[0].moving, st[0]
    assert abs(st[1].pos[0] - 1100) < 0.6 and not st[1].moving, st[1]
    assert 1045 < st[2].pos[0] < 1055, st[2]
    assert abs(st[2].pos[0] - st[3].pos[0]) < 0.6 and not st[3].moving       # stopped where it was: no second copy


def test_fit_damped_sine():
    t = np.arange(0, 3, 1 / 9.0)                                   # 9 frames/s like the HBVCAM at 640 x 480
    y = 5 + 12 * np.exp(-0.3 * 2 * np.pi * 2.3 * t) * np.cos(2 * np.pi * 2.3 * np.sqrt(1 - 0.09) * t + 0.4)
    y += np.random.default_rng(0).normal(0, 0.3, len(t))
    f = control.fit_damped_sine(t, y)
    assert abs(f["f_hz"] - 2.3) < 0.15 and abs(f["zeta"] - 0.3) < 0.08 and abs(f["offset"] - 5) < 0.6


def test_ringdown_measured_and_shaping_cuts_it():
    st = {0: sim.StructureModel(2.4, 0.08)}
    w, L, rig = make(clutter=False, structure=st,
                     servos=[sim.ServoModel(dead=0.0, lash=0.0, rate=6000.0)] + [sim.ServoModel(loaded=True) for _ in range(5)])
    jac = control.measure_jacobian(rig, say=lambda s: None)
    off = control.measure_ringdown(rig, jac, 0, step_us=60, say=lambda s: None)
    assert abs(off["fit"]["f_hz"] - 2.4) < 0.3 and off["fit"]["zeta"] < 0.2, off["fit"]
    control.apply_shaping(L, [0], off["fit"]["f_hz"], off["fit"]["zeta"], say=lambda s: None)
    on = control.measure_ringdown(rig, jac, 0, step_us=60, say=lambda s: None)
    assert on["pp_us"] < 0.35 * off["pp_us"], (off["pp_us"], on["pp_us"])


# ================================================================== v0.2: geometry and views
def test_fk_matches_cad_home_frames_and_worked_pose():
    T = geometry.frames([0] * 5)
    home5 = np.array([[0, 0, 1, 371], [0, -1, 0, 0], [1, 0, 0, 113], [0, 0, 0, 1.0]])
    assert np.abs(T[5] - home5).max() < 1e-9
    T = geometry.frames([45, 45, -90, -90, -90])
    assert np.abs(T[5][:3, 3] - [15.5, 15.5, -2.26]).max() < 0.05     # worked pose of the class DH model (45, 45, -90, -90, -90 deg)


def test_gripper_axis_in_the_image():
    m = geometry.CameraMount()
    g = geometry.gripper_axis_point(m, 270.0, 640, 480)
    assert abs(g[0] - 320) < 1e-6 and abs(g[1] - (240 + 720 * 29.5 / 270)) < 1e-6
    far = geometry.gripper_axis_point(m, 1e6, 640, 480)
    assert abs(far[1] - 240) < 0.1                                    # vanishing point = image center
    img = np.zeros((480, 640, 3), np.uint8)
    det = vision.Detection(True, 330.0, 300.0, area=(720 * 40 / 270.0) ** 2)
    _, info = geometry.draw_overlay(img, m, det, robot.pose_us("WALL"))
    assert abs(info["Z_mm"] - 270) < 1
    dx5, dy5 = info["block_from_gripper_mm"]
    assert abs(dx5 - (318.67 - 300) * 270 / 720) < 0.2 and abs(dy5 - 10 * 270 / 720) < 0.2   # above: +x5, right: +y5
    assert abs(info["grasp_point_mm"][2] - 285.6) < 1 and img.any()


def test_sim_camera_agrees_with_the_mount():
    w = sim.SimWorld(clutter=False)
    T = geometry.frames(geometry.q_from_pulses(w.y))[5] @ geometry.CameraMount().T_5c()
    P = np.linalg.inv(T) @ np.append(w.target, 1.0)
    u, v, z = w._pixel(w.y, w.target)
    assert abs(P[2] - z) < 0.5 and abs(320 + 720 * P[0] / P[2] - u) < 1.5 and abs(240 + 720 * P[1] / P[2] - v) < 1.5


def test_depth_from_size_and_gripper_aim_tracking():
    w, L, rig, trk, cfg = _track_scenario()
    mount = geometry.CameraMount(block_mm=w.block_mm)
    cfg.aim_fn = geometry.gripper_aim(mount)
    trk.overlay = geometry.tracker_overlay(mount)
    w.target = np.array([450.0, 20.0, 290.0])
    recs = []
    for _ in range(200):
        f, t, _ = rig.camera.read()
        r, view = trk.step(f, t, want_view=True)
        recs.append(r)
    assert recs[-1]["state"] == "CENTERED" and view.shape[1] == 1280
    u, v, z = w.true_pixel()
    g = geometry.gripper_axis_point(mount, z, 640, 480)
    assert abs(u - g[0]) < cfg.deadband_out_px + 3 and abs(v - g[1]) < cfg.deadband_out_px + 3
    assert recs[-1]["aim_y"] > 240 + 40                               # aiming below the center: the gripper axis


def test_pose_figure_and_description():
    import matplotlib
    matplotlib.use("Agg")
    fig = robot_view.show_pose(robot.pose_us("WALL"), "WALL", block=(450, 40, 280))
    assert len(fig.axes) == 3
    txt = robot_view.describe_pose(robot.pose_us("WALL"))
    assert "straight up" in txt and "level" in txt and "image up is world up" in txt
    m = robot_view.pose_model(robot.pose_us([0, 0, 0, 0, 0], 1580) if False else [1475.6, 1365.1, 854.0, 1019.5, 500, 1580])
    assert len(m["boxes"]) == 7


def test_stl_reader_binary_and_ascii():
    d = tempfile.mkdtemp()
    tri = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], np.float32)
    b = os.path.join(d, "b.stl")
    with open(b, "wb") as f:
        f.write(b"\0" * 80 + np.uint32(1).tobytes() + np.zeros(3, np.float32).tobytes() + tri.tobytes() + b"\0\0")
    a = os.path.join(d, "a.stl")
    with open(a, "w") as f:
        f.write("solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid x\n")
    assert np.allclose(robot_view.read_stl(b), tri) and np.allclose(robot_view.read_stl(a), tri)


# ========================================================== v0.2: profiles and camera health
GREEN = sorted((HERE.parent / "captures").glob("20261115_22[2-3]*_cam1-HBVCAM-camera.json"))


def test_profile_from_captures_save_load_paste():
    if len(GREEN) < 6:
        skip("green captures of 2026-11-15 not found")
    d = tempfile.mkdtemp()
    p = profiles.ObjectProfile.from_captures([str(g) for g in GREEN], "green wooden block #1", "living room")
    assert p.tag == "green wooden block #1, in the living room"
    assert p.camera["index"] == 1 and p.camera["orientation"] == [0, True, True] and (p.camera["width"], p.camera["height"]) == (640, 480)
    assert 55 < p.model().mean[0] < 65 and len(p.captures) == 6
    path = p.save(os.path.join(d, "profiles"))
    assert path.endswith("green-wooden-block-1__living-room.json")
    q = profiles.load(path)
    assert q.to_dict() == p.to_dict()
    ns = {}
    exec(p.paste_block(), ns)
    r, where = profiles.resolve(ns["PROFILE"], "green wooden block #1", "living room", os.path.join(d, "profiles"))
    assert where.startswith("pasted") and r.to_dict() == p.to_dict()
    r, where = profiles.resolve(None, "Green Wooden Block #1", "Living Room", os.path.join(d, "profiles"))
    assert where.startswith("saved")
    r, where = profiles.resolve(ns["PROFILE"], "red cube", "lab", os.path.join(d, "profiles"), default=None)
    assert r is None and where == "none"


class _FakeCap:
    """A camera that gives `black` black frames, then normal ones; counts how often it was opened."""
    opened = 0

    def __init__(self, i, backend=None):
        _FakeCap.opened += 1
        self.p = {}
        self.k = 0

    def isOpened(self): return True
    def set(self, k, v): self.p[k] = v; return True
    def get(self, k): return self.p.get(k, 0)
    def release(self): pass

    def read(self):
        time.sleep(0.002)
        self.k += 1
        bad = _FakeCap.opened == 1                                    # only the first opening is broken
        return True, np.full((48, 64, 3), 0 if bad else 120, np.uint8)


def test_camera_watchdog_reopens_on_black_frames():
    real = cv2.VideoCapture
    cv2.VideoCapture = _FakeCap
    _FakeCap.opened = 0
    try:
        cam = vision.Camera(0, 64, 48, 30, warmup=0, black_limit=10)
        f, _, _ = cam.read(timeout=3)
        while f.mean() < 3:
            f, _, _ = cam.read(timeout=3)
        cam.close()
    finally:
        cv2.VideoCapture = real
    assert cam.reopens == 1 and _FakeCap.opened == 2 and f.mean() == 120 and "black" in cam.events[0][1]


def test_camdiag_probe_and_recommend():
    real = cv2.VideoCapture
    cv2.VideoCapture = _FakeCap
    _FakeCap.opened = 5                                               # every opening is good
    try:
        rows = camdiag.probe(0, sizes=[(64, 48)], formats=[None], seconds=0.3, show=False)
    finally:
        cv2.VideoCapture = real
    assert len(rows) == 1 and rows[0]["black_pct"] == 0 and rows[0]["fps"] > 20
    rows = [{"w": 640, "h": 480, "fmt_asked": None, "ok": True, "fps": 9.0}, {"w": 320, "h": 240, "fmt_asked": None, "ok": True, "fps": 25.0},
            {"w": 1280, "h": 720, "fmt_asked": None, "ok": False, "fps": 30.0}]
    assert camdiag.recommend(rows, 12)["w"] == 320 and camdiag.recommend(rows, 5)["w"] == 640
    rows = [{"w": 320, "h": 240, "fmt_asked": None, "ok": True, "fps": 10.0}, {"w": 640, "h": 480, "fmt_asked": None, "ok": True, "fps": 10.0},
            {"w": 640, "h": 480, "fmt_asked": "MJPG", "ok": True, "fps": 8.7}, {"w": 1280, "h": 720, "fmt_asked": None, "ok": True, "fps": 7.5}]
    r = camdiag.recommend(rows, 12)                                   # nothing reaches 12: equal speeds -> preferred size
    assert (r["w"], r["fmt_asked"]) == (640, None)


if __name__ == "__main__":
    ok = bad = sk = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); ok += 1; print("PASS", name)
            except Skip as e:
                sk += 1; print("SKIP", name, "-", e)
            except Exception as e:                                    # noqa: BLE001
                bad += 1; print("FAIL", name, "-", type(e).__name__, e)
    print(f"{ok} passed, {bad} failed, {sk} skipped")
    sys.exit(1 if bad else 0)
