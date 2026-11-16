"""Pictures of Sir Pinch-a-lot in a pose: side view, top view and a 3D view, to check the arm
against before powering it up.

What is drawn:
    - the six MG996R servos as boxes at their real places (from the CAD placements of the Rev B arm,
      ICP-fitted to the assembled model, carried along with the DH frames);
    - the links as bars between the joint origins (base plate -> shoulder -> elbow -> wrist -> grasp point);
    - the gripper jaws, the camera on its bracket and the camera's view direction;
    - the DH frame-5 axes at the grasp point (x5 red, y5 green, z5 blue).
The real part meshes are not part of this repository (the source CAD model has no license). If you
have the STL files (Addison Sears-Collins' 6 DOF robotic arm model, file names unchanged), pass
stl_dir and placements (the JSON in rot3u-robots studies/gravity-backlash/cad) to draw every part.

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
Servo placements derived from the Rev B placements file (source geometry: Addison Sears-Collins,
6 DOF robotic arm CAD, 2020, automaticaddison.com).
"""
from __future__ import annotations
import json, math, os, struct
import numpy as np
from . import geometry, robot

# MG996R box in the STL's own coordinates (mm)
SERVO_BOX = ((-9.85, -20.35, 0.0), (9.85, 20.35, 43.25))
# (link, 3x4 T: STL -> DH frame 0 at the home pose), from sir_pinch_a_lot_revB_placements.json
SERVOS_HOME = [
    (0, [[-1.0, 0.0, 0.0, -0.81], [0.0, -1.0, 0.0, 9.5], [0.0, 0.0, 1.0, 53.75]]),          # base (ch 0)
    (1, [[0.0, -1.0, 0.0, 10.24], [0.0, 0.0, -1.0, 18.52], [1.0, 0.0, 0.0, 113.03]]),       # shoulder (ch 1)
    (3, [[0.01, -1.0, 0.0, 113.96], [0.0, 0.0, -1.0, 20.0], [1.0, 0.01, 0.0, 112.37]]),     # elbow (ch 2)
    (4, [[1.0, 0.0, 0.0, 201.02], [0.0, 0.0, -1.0, 20.99], [0.0, 1.0, 0.0, 102.95]]),       # wrist pitch (ch 3)
    (4, [[0.0, 0.0, 1.0, 219.92], [0.0, -1.0, 0.0, 9.48], [1.0, 0.0, 0.0, 112.99]]),        # wrist roll (ch 4)
    (5, [[1.0, 0.01, 0.0, 288.33], [0.01, -1.0, 0.0, 25.21], [0.0, 0.0, -1.0, 154.01]]),    # gripper (ch 5)
]
SERVO_NAMES = ["base", "shoulder", "elbow", "wrist", "roll", "gripper"]
CAMERA_BOX = ((-16.0, -16.0, -12.0), (16.0, 16.0, 8.0))     # board + lens, camera frame (x right, y down, z forward), mm


def _h(T34):
    T = np.eye(4); T[:3, :] = np.asarray(T34, float)
    return T


def _corners(lo, hi):
    return np.array([[x, y, z, 1.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


_FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]


def pose_model(pulses, mount: geometry.CameraMount = None):
    """Everything to draw for these pulses (channels 0..5): boxes (8 x 3 corners, frame 0), polylines, camera."""
    mount = mount or geometry.CameraMount()
    q = geometry.q_from_pulses(pulses)
    T = geometry.frames(q)
    Th = geometry.frames([0.0] * 5)
    boxes = []
    for (link, T34), name in zip(SERVOS_HOME, SERVO_NAMES):
        M = T[link] @ np.linalg.inv(Th[link]) @ _h(T34)
        boxes.append((name, (M @ _corners(*SERVO_BOX).T).T[:, :3]))
    Tc = T[5] @ mount.T_5c()
    boxes.append(("camera", (Tc @ _corners(*CAMERA_BOX).T).T[:, :3]))
    o = [Ti[:3, 3] for Ti in T]
    skeleton = np.array([[0, 0, 0], o[1], o[2], o[3], o[5]])
    # gripper jaws: open width from the gripper pulse (closed 1580 us ~ 0 mm, open 1150 us ~ 60 mm)
    g = float(pulses[5]) if len(pulses) > 5 else robot.GRIP_HOLD_US
    half = 4.0 + max(0.0, min(60.0, (robot.GRIP_HOLD_US - g) / (robot.GRIP_HOLD_US - robot.GRIP_OPEN_US) * 60.0)) / 2
    R5, p5 = T[5][:3, :3], T[5][:3, 3]
    jaws = [np.array([p5 - 45 * R5[:, 2] + s * half * R5[:, 1], p5 + s * half * R5[:, 1], p5 + 4 * R5[:, 2] + s * half * R5[:, 1]])
            for s in (-1, 1)]
    L = 250.0
    if Tc[2, 2] < -1e-6:                     # stop the view line at the table (z0 = 0)
        L = min(L, -Tc[2, 3] / Tc[2, 2])
    view = np.array([Tc[:3, 3], Tc[:3, 3] + L * Tc[:3, 2]])
    axes5 = [(p5, p5 + 50 * R5[:, i]) for i in range(3)]
    return {"q": q, "T": T, "boxes": boxes, "skeleton": skeleton, "jaws": jaws, "view": view, "axes5": axes5,
            "camera": Tc}


def _hull2d(pts):
    pts = sorted(map(tuple, pts))
    if len(pts) <= 2:
        return np.array(pts)

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(reversed(pts))
    return np.array(lo[:-1] + up[:-1])


def read_stl(path):
    """Triangles (n x 3 x 3) from a binary or ASCII STL, with numpy only."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
            return rec["v"].astype(float)
    v = [list(map(float, l.split()[1:4])) for l in data.decode("ascii", "ignore").splitlines() if l.strip().startswith("vertex")]
    return np.array(v, float).reshape(-1, 3, 3)


def mesh_triangles(pulses, placements, stl_dir, max_tris_per_part=1500):
    """All parts as triangles in frame 0 for this pose (needs the STL files)."""
    with open(placements) as f:
        P = json.load(f)
    T = geometry.frames(geometry.q_from_pulses(pulses))
    Th = geometry.frames([0.0] * 5)
    cache, out = {}, []
    for ins in P["instances"]:
        src = os.path.join(stl_dir, ins["source"])
        if not os.path.exists(src):
            continue
        if src not in cache:
            tri = read_stl(src)
            if len(tri) > max_tris_per_part:
                tri = tri[np.linspace(0, len(tri) - 1, max_tris_per_part).astype(int)]
            cache[src] = tri
        M = T[ins["link"]] @ np.linalg.inv(Th[ins["link"]]) @ np.asarray(ins["T"], float)
        tri = cache[src]
        out.append((ins["link"], tri @ M[:3, :3].T + M[:3, 3]))
    return out


def show_pose(pulses, title="", mount=None, block=None, stl_dir=None, placements=None, figsize=(14, 4.8)):
    """Side view (looking along -y0: x0 forward to the right, z0 up), top view (x0 right, y0 up) and a 3D view.

    block: optional (x, y, z) mm of the block, drawn as a 40 mm cube. Returns the matplotlib figure."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    m = pose_model(pulses, mount)
    fig = plt.figure(figsize=figsize)
    axs = [fig.add_subplot(1, 3, 1), fig.add_subplot(1, 3, 2), fig.add_subplot(1, 3, 3, projection="3d")]
    meshes = mesh_triangles(pulses, placements, stl_dir) if (stl_dir and placements) else None
    colors = {"base": "#8c8c8c", "shoulder": "#4c72b0", "elbow": "#55a868", "wrist": "#c44e52", "roll": "#8172b2",
              "gripper": "#ccb974", "camera": "#222222"}
    for ax, (i, j), name in ((axs[0], (0, 2), "side (from the robot's right: x0 forward, z0 up)"),
                             (axs[1], (0, 1), "top (x0 forward, y0 to the robot's left)")):
        ax.set_title(name, fontsize=9)
        if i == 0 and j == 2:
            ax.axhline(0, color="k", lw=1)
            ax.fill_between([-150, 500], -30, 0, color="#eeeeee")
        if meshes:
            for _, tri in meshes:
                for t in tri[::2]:
                    ax.fill(t[:, i], t[:, j], color="#b0b0b0", alpha=0.25, lw=0)
        ax.plot(m["skeleton"][:, i], m["skeleton"][:, j], "-", color="#bbbbbb", lw=10, solid_capstyle="round", zorder=1)
        ax.plot(m["skeleton"][:, i], m["skeleton"][:, j], "o-", color="#555555", lw=1.2, ms=4, zorder=3)
        for nm, c in m["boxes"]:
            h = _hull2d(c[:, [i, j]])
            ax.fill(h[:, 0], h[:, 1], color=colors[nm], alpha=0.75, zorder=2)
            ax.text(*c[:, [i, j]].mean(axis=0), nm if nm != "camera" else "cam", fontsize=7, ha="center", va="center",
                    color="w", zorder=4)
        for jw in m["jaws"]:
            ax.plot(jw[:, i], jw[:, j], "-", color="#7a6a2a", lw=3, zorder=3)
        ax.plot(m["view"][:, i], m["view"][:, j], "--", color="#222222", lw=1, zorder=3)
        for (p0, p1), col in zip(m["axes5"], ("r", "g", "b")):
            ax.annotate("", p1[[i, j]], p0[[i, j]], arrowprops=dict(arrowstyle="->", color=col, lw=1.5), zorder=5)
        if block is not None:
            b = np.asarray(block, float)
            ax.add_patch(plt.Rectangle((b[i] - 20, b[j] - 20), 40, 40, color="#2ca02c", alpha=0.8))
        if j == 1:
            yl = max(120.0, float(np.abs(m["skeleton"][:, 1]).max()) + 60)
            ax.set_ylim(-yl, yl)
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel(f"{'xyz'[i]}0, mm"); ax.set_ylabel(f"{'xyz'[j]}0, mm")
    ax = axs[2]
    polys, fc = [], []
    for nm, c in m["boxes"]:
        for f in _FACES:
            polys.append(c[list(f)]); fc.append(colors[nm])
    ax.add_collection3d(Poly3DCollection(polys, facecolors=fc, edgecolors="k", linewidths=0.2, alpha=0.8))
    if meshes:
        ax.add_collection3d(Poly3DCollection([t for _, tri in meshes for t in tri[::3]], facecolors="#b0b0b0",
                                             alpha=0.2, linewidths=0))
    s = m["skeleton"]
    ax.plot(s[:, 0], s[:, 1], s[:, 2], "o-", color="#555555", lw=4)
    for jw in m["jaws"]:
        ax.plot(jw[:, 0], jw[:, 1], jw[:, 2], color="#7a6a2a", lw=3)
    ax.plot(*m["view"].T, "--", color="#222222", lw=1)
    for (p0, p1), col in zip(m["axes5"], ("r", "g", "b")):
        ax.plot(*np.array([p0, p1]).T, color=col, lw=2)
    if block is not None:
        ax.scatter(*block, color="#2ca02c", s=120, marker="s")
    pts = np.vstack([c for _, c in m["boxes"]] + [s, m["view"]] + ([np.reshape(block, (1, 3))] if block is not None else []))
    mid = 0.5 * (pts.min(axis=0) + pts.max(axis=0)); r = 0.55 * float((pts.max(axis=0) - pts.min(axis=0)).max())
    ax.set_xlim(mid[0] - r, mid[0] + r); ax.set_ylim(mid[1] - r, mid[1] + r); ax.set_zlim(max(0.0, mid[2] - r), max(0.0, mid[2] - r) + 2 * r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(22, -60)
    ax.set_xlabel("x0"); ax.set_ylabel("y0"); ax.set_zlabel("z0"); ax.set_title("3D", fontsize=9)
    deg = ", ".join(f"{v:.0f}" for v in m["q"])
    fig.suptitle((title + "  |  " if title else "") + f"joint angles (deg) {deg}  |  pulses {[int(round(p)) for p in pulses]}",
                 fontsize=10)
    fig.tight_layout()
    return fig


def describe_pose(pulses, mount=None):
    """Text description of a pose: what each joint does and where the camera looks."""
    m = pose_model(pulses, mount)
    q = m["q"]
    T5, Tc = m["T"][5], m["camera"]
    upper = q[1]; fore = q[1] + q[2]; grip = q[1] + q[2] + q[3]

    def tilt(a):
        a = (a + 180) % 360 - 180
        if abs(a) < 3: return "level, pointing forward"
        if abs(a - 90) < 3: return "straight up"
        if abs(a + 90) < 3: return "straight down"
        return f"{abs(a):.0f} deg {'above' if a > 0 else 'below'} level"
    look = Tc[:3, 2]
    el = math.degrees(math.asin(max(-1, min(1, look[2]))))
    lines = [
        f"base {q[0]:+.0f} deg: the arm faces " + ("straight ahead (x0)" if abs(q[0]) < 3 else f"{abs(q[0]):.0f} deg to the robot's {'left' if q[0] > 0 else 'right'}"),
        f"upper arm (shoulder to elbow): {tilt(upper)}",
        f"forearm (elbow to wrist): {tilt(fore)}",
        f"gripper (wrist to jaws): {tilt(grip)}; roll {q[4]:+.0f} deg " + ("(gripper servo and camera on top)" if abs(q[4]) < 3 else ""),
        f"grasp point at x {T5[0, 3]:.0f}, y {T5[1, 3]:.0f}, z {T5[2, 3]:.0f} mm (z from the base plate)",
        f"camera at height {Tc[2, 3]:.0f} mm, looking " + ("level" if abs(el) <= 1 else f"{abs(el):.0f} deg {'up' if el > 0 else 'down'}")
        + ("; image up is world up" if Tc[2, 1] < -0.9 else "; the top of the image is the side away from the robot" if el < -60 else ""),
    ]
    return "\n".join(lines)
