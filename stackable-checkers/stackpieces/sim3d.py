"""3D settling simulation in MuJoCo (DD-STACK SP-8).

Each piece is a free body made of convex parts: the socket wall split into angular sectors plus the
roof and cone frusta (shoulder family), the cone shell sectors plus the tip cap (nesting family), or
one hull (cylinder). MuJoCo takes the convex hull of each part's vertices. A piece rests on a plane;
an identical piece is released above it. Units mm, g, s (gravity 9810 mm/s^2).

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import math

import numpy as np

SCENARIOS = {
    "place": dict(label="Gentle place: released level and still, 1 mm above the cone tip", d=1.0, tilt=0.0, v=0.0),
    "drop10": dict(label="Drop: released level, 10 mm above the cone tip", d=10.0, tilt=0.0, v=0.0),
    "tilt10": dict(label="Tilted release: 10 deg, outer side down, 1 mm above", d=1.0, tilt=10.0, v=0.0),
    "slide": dict(label="Moving release: still moving 50 mm/s outward, 1 mm above", d=1.0, tilt=0.0, v=50.0),
}


def _ring(r, z, n=48):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for k in range(n)]


def convex_parts(g: dict, sectors: int = 32) -> list:
    """Point clouds (piece frame) whose convex hulls make up the piece."""
    p3 = g["parts3d"]
    if "hull" in p3:
        pts = []
        for r, z in p3["hull"]:
            pts += _ring(r, z) if r > 0 else [(0.0, 0.0, z)]
        return [pts]
    parts = []
    poly = p3["sectors"]
    for k in range(sectors):
        f0 = 2 * math.pi * k / sectors
        f1 = 2 * math.pi * (k + 1) / sectors
        parts.append([(r * math.cos(f), r * math.sin(f), z) for r, z in poly for f in (f0, (f0 + f1) / 2, f1)])
    for r1, z1, r2, z2 in p3["discs"]:
        parts.append(_ring(r1, z1) + _ring(r2, z2) + [(0.0, 0.0, z1), (0.0, 0.0, z2)])
    return parts


def model_xml(g: dict, mu: float, rest: float = 0.15, dt: float = 2e-4, sectors: int = 32) -> str:
    """MJCF for a board plane and two identical pieces ("low" and "up")."""
    parts = convex_parts(g, sectors)
    zc = g["zc"]
    assets = "".join(
        f'<mesh name="p{i}" vertex="{" ".join(f"{x:.5f} {y:.5f} {z - zc:.5f}" for x, y, z in pts)}"/>'
        for i, pts in enumerate(parts))
    # restitution: MuJoCo has none for soft contacts; damping ratio < 1 gives a small bounce
    damp = max(0.3, 1.0 - 2.5 * rest)
    geom_def = f'friction="{mu} 0.0 0.0" condim="3" solref="0.0008 {damp:.3f}" solimp="0.95 0.99 0.01"'

    def body(name):
        geoms = "".join(f'<geom type="mesh" mesh="p{i}" {geom_def} rgba="0.7 0.2 0.2 1"/>' for i in range(len(parts)))
        return (f'<body name="{name}" pos="0 0 {zc}"><freejoint name="{name}_j"/>'
                f'<inertial pos="0 0 0" mass="{g["m"]}" diaginertia="{g["I"]} {g["I"]} {g["Iz"]}"/>{geoms}</body>')

    return (f'<mujoco model="stack"><compiler angle="radian"/>'
            f'<option timestep="{dt}" gravity="0 0 -9810" cone="elliptic" impratio="1" iterations="100" '
            f'ls_iterations="50" integrator="implicitfast"><flag multiccd="enable"/></option>'
            f'<asset>{assets}</asset><worldbody>'
            f'<geom type="plane" size="300 300 1" {geom_def}/>{body("low")}{body("up")}</worldbody></mujoco>')


def _base_pose(qpos, zc):
    """Bottom-center position and tilt (deg) from a freejoint qpos (CoM position, quaternion w,x,y,z)."""
    w, x, y, z = qpos[3:7]
    # third column of the rotation matrix (body z axis in world)
    zx = 2 * (x * z + w * y)
    zy = 2 * (y * z - w * x)
    zz = 1 - 2 * (x * x + y * y)
    base = np.array(qpos[0:3]) - zc * np.array([zx, zy, zz])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, zz))))
    return base, tilt


class Stack3D:
    """MuJoCo model of two pieces, rebuilt per friction value."""

    def __init__(self, g: dict, sectors: int = 32, dt: float = 2e-4):
        import mujoco  # imported here so the rest of the package works without it
        self.mujoco = mujoco
        self.g, self.sectors, self.dt = g, sectors, dt
        self._models = {}

    def _model(self, mu, rest):
        key = (round(mu, 6), round(rest, 6))
        if key not in self._models:
            m = self.mujoco.MjModel.from_xml_string(model_xml(self.g, mu, rest, self.dt, self.sectors))
            self._models[key] = m
        return self._models[key]

    def simulate(self, e, d=1.0, tilt=0.0, v=0.0, mu=0.4, rest=0.15, T=1.5, record=False, fps=90):
        """Release the upper piece at offset e (mm along +x). Returns outcome as sim2d plus 3D poses."""
        mj = self.mujoco
        g = self.g
        m = self._model(mu, rest)
        data = mj.MjData(m)
        zc, R = g["zc"], g["R"]
        th = math.radians(tilt)
        # upper piece: rotate about +y by th lowers its +x (outer) side
        q = np.array([math.cos(th / 2), 0.0, math.sin(th / 2), 0.0])
        z0 = g["H"] + d + R * math.sin(th)
        rot_z = np.array([2 * q[0] * q[2], 0.0, 1 - 2 * q[2] * q[2]])
        com_up = np.array([e, 0.0, z0]) + zc * rot_z
        data.qpos[0:7] = [0, 0, zc, 1, 0, 0, 0]
        data.qpos[7:14] = [com_up[0], com_up[1], com_up[2], *q]
        data.qvel[6:9] = [v, 0, 0]
        mj.mj_forward(m, data)
        frames = []
        every = max(1, int(round((1 / fps) / self.dt)))
        calm = 0.0
        n = int(T / self.dt)
        i = 0
        for i in range(n):
            mj.mj_step(m, data)
            if record and i % every == 0:
                frames.append(self._frame(data))
            lin = max(np.linalg.norm(data.qvel[0:3]), np.linalg.norm(data.qvel[6:9]))
            ang = max(np.linalg.norm(data.qvel[3:6]), np.linalg.norm(data.qvel[9:12]))
            calm = calm + self.dt if (lin < 2 and ang < 0.05) else 0.0
            if calm > 0.1 and i * self.dt > 0.1:
                break
        if record:
            frames.append(self._frame(data))
        L, lt = _base_pose(data.qpos[0:7], zc)
        U, ut = _base_pose(data.qpos[7:14], zc)
        dxy = float(math.hypot(U[0] - L[0], U[1] - L[1]))
        dz = float(U[2] - L[2])
        if lt > 10:
            code = "knocked"
        elif U[2] < g["band_h"] * 0.6 or ut > 45:
            code = "fell"
        elif dxy < 1.0 and ut < 3 and abs(dz - g["seat"]) < 0.6:
            code = "seated"
        else:
            code = "perched"
        out = dict(code=code, dxy=dxy, dz=dz, tilt=float(ut), ltilt=float(lt), t=round(i * self.dt, 4))
        if record:
            out["frames"] = frames
        return out

    @staticmethod
    def _frame(data):
        """[low x y z qx qy qz qw, up ...] (CoM positions, quaternions in three.js order)."""
        f = []
        for k in (0, 7):
            qp = data.qpos[k:k + 7]
            f += [round(float(qp[0]), 3), round(float(qp[1]), 3), round(float(qp[2]), 3),
                  round(float(qp[4]), 4), round(float(qp[5]), 4), round(float(qp[6]), 4), round(float(qp[3]), 4)]
        return f


def outcome_map(g: dict, offsets, mus=(0.25, 0.4, 0.55), scenarios=None, record_mu=0.4, progress=None) -> dict:
    """Outcome for every scenario x friction x offset; trajectories recorded at record_mu."""
    st = Stack3D(g)
    scenarios = scenarios or list(SCENARIOS)
    res, frames = {}, {}
    for s in scenarios:
        sc = SCENARIOS[s]
        for mu in mus:
            row = []
            for e in offsets:
                rec = abs(mu - record_mu) < 1e-9
                r = st.simulate(e, d=sc["d"], tilt=sc["tilt"], v=sc["v"], mu=mu, record=rec)
                if rec:
                    frames[f"{s}|{e}"] = r.pop("frames")
                row.append({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
            res[f"{s}|{mu}"] = row
            if progress:
                progress(s, mu, "".join({"seated": "#", "perched": "p", "fell": ".", "knocked": "K"}[r["code"]] for r in row))
    return dict(results=res, frames=frames, mus=list(mus), offsets=list(offsets),
                scenarios={s: SCENARIOS[s]["label"] for s in scenarios})
