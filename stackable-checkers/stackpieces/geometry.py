"""Piece geometry: families, profiles, section parts, mass properties and checks (DD-STACK SP-2..SP-4).

Units mm and degrees. Piece frame: origin on the axis in the bottom face, z up.
Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import math

import numpy as np

FAMILIES = ("shoulder", "nesting", "cylinder")

PRESETS = {
    "S40": dict(family="shoulder", D=25.0, band_h=12.0, shoulder=2.0, tip_d=4.0, alpha=40.0,
                clr=0.5, vclr=0.5, ct=0.5, cb=0.4),
    "N35": dict(family="nesting", D=25.0, band_h=12.0, tip_d=1.6, alpha=35.0, land=1.0,
                vclr=0.6, ctip=0.4, cb=0.4),
    "CYL": dict(family="cylinder", D=25.0, band_h=15.0, ct=0.5, cb=0.4),
}


class GeometryError(ValueError):
    """The parameters do not describe a valid piece (e.g. the socket does not fit)."""


def make_piece(params: dict, density: float = 1.24e-3) -> dict:
    """Return a piece dict: profile, section parts, sizes, mass properties.

    ``params`` holds ``family`` plus the family's parameters (docs/design.md section 3).
    """
    p = dict(params)
    fam = p.get("family")
    if fam == "shoulder":
        g = _shoulder(p)
    elif fam == "nesting":
        g = _nesting(p)
    elif fam == "cylinder":
        g = _cylinder(p)
    else:
        raise GeometryError(f"unknown family {fam!r}; expected one of {FAMILIES}")
    g["params"] = p
    g["family"] = fam
    g.update(mass_props(g["profile"], density))
    g["king_h"] = g["seat"] + g["H"]
    return g


def _shoulder(p):
    R = p["D"] / 2
    Hc, sh, rt, ct, cb = p["band_h"], p["shoulder"], p["tip_d"] / 2, p["ct"], p["cb"]
    ta = math.tan(math.radians(p["alpha"]))
    Rb = R - sh
    if not (0 < rt < Rb < R - ct):
        raise GeometryError("need 0 < tip radius < cone base radius < D/2 - ct")
    h = (Rb - rt) / ta
    H = Hc + h
    Rm = Rb + p["clr"]
    hs = h + p["vclr"]
    rs = Rm - hs * ta
    if rs <= 0.05:
        raise GeometryError(f"socket roof radius {rs:.2f} <= 0: increase clr or tip_d")
    if hs >= Hc:
        raise GeometryError(f"socket depth {hs:.2f} >= band_h {Hc}: the socket does not fit in the band")
    prof = [(0, hs), (rs, hs), (Rm, 0), (R - cb, 0), (R, cb), (R, Hc - ct), (R - ct, Hc), (Rb, Hc), (rt, H), (0, H)]
    left = [(-R + cb, 0), (-Rm, 0), (-rs, hs), (-rs, Hc), (-R + ct, Hc), (-R, Hc - ct), (-R, cb)]
    right = [(-x, z) for x, z in reversed(left)]
    section = [left, right, [(-rs, hs), (rs, hs), (rs, Hc), (-rs, Hc)], [(-Rb, Hc), (Rb, Hc), (rt, H), (-rt, H)]]
    wall = [(rs, hs), (Rm, 0), (R - cb, 0), (R, cb), (R, Hc - ct), (R - ct, Hc), (rs, Hc)]
    return dict(R=R, H=H, band_h=Hc, h=h, Rb=Rb, rt=rt, Rm=Rm, rs=rs, hs=hs, seat=Hc,
                cone_start=Hc, capture=Rm - rt, profile=prof, section=section,
                parts3d=dict(sectors=wall, discs=[(rs, hs, rs, Hc), (Rb, Hc, rt, H)]),
                socket_overhang_deg=p["alpha"], bridge=2 * rs, rim=R - cb - Rm, roof=Hc - hs,
                straight_band=Hc - ct - cb)


def _nesting(p):
    R = p["D"] / 2
    Hb, rt, land, cb = p["band_h"], p["tip_d"] / 2, p["land"], p["cb"]
    a = math.radians(p["alpha"])
    ta = math.tan(a)
    H = Hb + (R - rt) / ta
    Rm = R - land
    seat = Hb + land / ta
    roof = H - seat + p["vclr"]
    rs = rt + p["ctip"]
    ts = (Rm - rs) / roof
    ro = R - (roof - Hb) * ta
    if not (roof > Hb and ro > rs):
        raise GeometryError(f"socket roof ({roof:.2f}) must be above band_h and inside the cone")
    prof = [(0, roof), (rs, roof), (Rm, 0), (R - cb, 0), (R, cb), (R, Hb), (rt, H), (0, H)]
    strip = [(rs, roof), (Rm, 0), (R - cb, 0), (R, cb), (R, Hb), (ro, roof)]
    left = [(-x, z) for x, z in reversed(strip)]
    section = [left, strip, [(-ro, roof), (ro, roof), (rt, H), (-rt, H)]]
    wall = min(ro - rs, R - (Rm - Hb * ts)) * math.cos(a)
    return dict(R=R, H=H, band_h=Hb, h=H - Hb, rt=rt, Rm=Rm, rs=rs, hs=roof, ro=ro, seat=seat,
                cone_start=Hb, capture=Rm - rt, profile=prof, section=section,
                parts3d=dict(sectors=strip, discs=[(ro, roof, rt, H)]),
                socket_overhang_deg=math.degrees(math.atan(ts)), bridge=2 * rs, rim=land - cb,
                roof=H - roof, shell_wall=wall, straight_band=Hb - cb)


def _cylinder(p):
    R = p["D"] / 2
    H, ct, cb = p["band_h"], p["ct"], p["cb"]
    prof = [(0, 0), (R - cb, 0), (R, cb), (R, H - ct), (R - ct, H), (0, H)]
    section = [[(-R + cb, 0), (R - cb, 0), (R, cb), (R, H - ct), (R - ct, H), (-R + ct, H), (-R, H - ct), (-R, cb)]]
    return dict(R=R, H=H, band_h=H, h=0.0, rt=0.0, Rm=0.0, seat=H, cone_start=H, capture=0.0,
                profile=prof, section=section, parts3d=dict(hull=prof),
                socket_overhang_deg=0.0, bridge=0.0, rim=R - cb, roof=H, straight_band=H - ct - cb)


def points_in_polygon(x, y, poly):
    """Vectorized even-odd test of points (x, y) against a polygon [(x, y), ...]."""
    inside = np.zeros(np.shape(x), dtype=bool)
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[i - 1]
        cond = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cond & (x < xc)
    return inside


def mass_props(profile, density=1.24e-3, d=0.05):
    """Volume (mm^3), CoM height zc, mass m (g), inertia about a horizontal axis through the CoM
    (I, g mm^2) and about the axis (Iz), from the half-section by midpoint integration on a d grid.
    The JavaScript version uses the same grid, so both give the same numbers."""
    rmax = max(r for r, _ in profile)
    zmax = max(z for _, z in profile)
    r = np.arange(d / 2, rmax, d)
    z = np.arange(d / 2, zmax, d)
    RR, ZZ = np.meshgrid(r, z)
    ins = points_in_polygon(RR, ZZ, profile)
    dv = 2 * np.pi * RR * d * d * ins
    V = dv.sum()
    zc = (dv * ZZ).sum() / V
    Srr = (dv * RR ** 2).sum()
    Szz = (dv * (ZZ - zc) ** 2).sum()
    return dict(V=float(V), zc=float(zc), m=float(V * density),
                I=float(density * (Srr / 2 + Szz)), Iz=float(density * Srr))


def volume_exact(profile):
    """Exact volume of the revolved half-section polygon (Pappus, per edge)."""
    V = 0.0
    n = len(profile)
    for i in range(n):
        r1, z1 = profile[i]
        r2, z2 = profile[(i + 1) % n]
        # volume swept by the trapezoid under each edge, signed (z decreasing = adds when CCW)
        V += math.pi * (z1 - z2) * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0
    return abs(V)


def radii_at(profile, z):
    """Sorted radii where the horizontal line at height z crosses the profile boundary (r >= 0)."""
    xs = []
    n = len(profile)
    for i in range(n):
        r1, z1 = profile[i]
        r2, z2 = profile[(i + 1) % n]
        if (z1 > z) != (z2 > z):
            xs.append(r1 + (r2 - r1) * (z - z1) / (z2 - z1))
    return sorted(xs)


def material_intervals(profile, z):
    """Material intervals [(r_in, r_out), ...] at height z (even-odd)."""
    xs = radii_at(profile, z)
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def checks(g: dict, c: dict) -> list:
    """Print and robot checks against the constraints (SP-4). Each item: id, ok, value, limit, note."""
    pr, gr, bd, goal = c["printer"], c["gripper"], c["board"], c["goal"]
    out = []

    def add(cid, ok, value, limit, note):
        out.append(dict(id=cid, ok=bool(ok), value=round(float(value), 3), limit=limit, note=note))

    fam = g["family"]
    if fam != "cylinder":
        add("overhang", g["socket_overhang_deg"] <= pr["overhang_max_deg"] + 1e-9, g["socket_overhang_deg"],
            f"<= {pr['overhang_max_deg']} deg from vertical", "socket wall printed upright without support")
        add("bridge", g["bridge"] <= pr["bridge_max_mm"], g["bridge"], f"<= {pr['bridge_max_mm']} mm",
            "socket roof diameter")
        add("roof", g["roof"] >= pr["min_wall_mm"], g["roof"], f">= {pr['min_wall_mm']} mm",
            "material above the socket roof")
    add("rim", g["rim"] >= pr["min_wall_mm"], g["rim"], f">= {pr['min_wall_mm']} mm",
        "flat ring on the bed / board (thinnest wall at the bottom)")
    if fam == "nesting":
        add("shell_wall", g["shell_wall"] >= pr["min_wall_mm"], g["shell_wall"], f">= {pr['min_wall_mm']} mm",
            "cone shell thickness")
    add("grip_band_min", g["band_h"] >= gr["grip_band_min_mm"], g["band_h"],
        f">= {gr['grip_band_min_mm']} mm", "cylindrical band the fingers grip (to the shoulder or cone start)")
    add("grip_band_max", g["band_h"] <= gr["grip_band_max_mm"] + 1e-9, g["band_h"],
        f"<= {gr['grip_band_max_mm']} mm", "Tico's band range")
    add("height", g["H"] <= goal["max_height_mm"] + 1e-9, g["H"], f"<= {goal['max_height_mm']} mm", "single piece height")
    gap = bd["square_mm"] * math.sqrt(2) - 2 * g["R"] if bd.get("pieces_on_diagonals_only", True) else bd["square_mm"] - 2 * g["R"]
    ft = gr.get("finger_thickness_mm")
    add("finger_room", ft is not None and gap >= ft, gap,
        f">= finger thickness ({'not measured' if ft is None else str(ft) + ' mm'})",
        "gap between pieces on diagonal neighbor squares")
    return out


def summary(g: dict) -> dict:
    """Key sizes of a piece, rounded, for tables."""
    keys = ["R", "H", "band_h", "h", "rt", "Rm", "rs", "hs", "seat", "king_h", "capture", "V", "m", "zc",
            "socket_overhang_deg", "bridge", "rim", "roof", "straight_band"]
    return {k: round(float(g[k]), 3) for k in keys if k in g}
