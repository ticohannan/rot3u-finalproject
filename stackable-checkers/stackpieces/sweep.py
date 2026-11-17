"""Design-space search from the constraints alone (DD-STACK SP-11).

1. Enumerate shoulder and nesting pieces (plus the cylinder control) on a parameter grid, with the
   diameter, clearances and chamfers taken from the constraints.
2. Build each; drop the ones whose geometry is impossible; run the print and robot checks.
3. Score every buildable candidate by its smooth-surface capture: the largest offset up to which a
   gentle release (level, still, 1 mm above the tip, mid friction) seats at every offset, 2D model.
4. Rank the candidates that pass every hard check by capture, then by height (shorter first).
Finalists are then confirmed with printed surfaces (montecarlo) and in 3D (sim3d) by the notebooks.

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import itertools

import numpy as np

from .geometry import GeometryError, checks, make_piece
from .sim2d import Section2D
from .surface import section

DEFAULT_GRID = {
    "shoulder": dict(alpha=[30, 35, 40, 45, 50, 55], shoulder=[1.2, 2.0, 3.0], tip_d=[1.6, 4.0], band_h=None),
    "nesting": dict(alpha=[30, 35, 40, 45, 50], tip_d=[1.6, 3.0], band_h=None),
}
QUICK_GRID = {
    "shoulder": dict(alpha=[35, 40, 45], shoulder=[2.0], tip_d=[4.0], band_h=[12.0]),
    "nesting": dict(alpha=[35, 45], tip_d=[1.6], band_h=[12.0]),
}
SOFT_CHECKS = {"finger_room"}   # reported, but not used to reject (finger thickness may be unmeasured)


def band_values(c):
    lo, hi = c["gripper"]["grip_band_min_mm"], c["gripper"]["grip_band_max_mm"]
    return sorted({lo, round((lo + hi) / 2 * 2) / 2, hi, 12.0} & set(np.arange(lo, hi + 1e-9, 0.5).round(1)))


def candidates(c: dict, grid: dict | None = None) -> list:
    """Parameter sets for every grid point, with constraint-derived sizes."""
    grid = grid or DEFAULT_GRID
    D = c["piece"]["diameter_mm"]
    clr = max(c["printer"]["hole_undersize_mm"])   # socket radial clearance covers the worst hole shrink
    out = []
    gs = grid.get("shoulder")
    if gs:
        for a, sh, td, bh in itertools.product(gs["alpha"], gs["shoulder"], gs["tip_d"], gs["band_h"] or band_values(c)):
            out.append(dict(family="shoulder", D=D, band_h=float(bh), shoulder=sh, tip_d=td, alpha=a,
                            clr=clr, vclr=0.5, ct=0.5, cb=0.4))
    gn = grid.get("nesting")
    if gn:
        for a, td, bh in itertools.product(gn["alpha"], gn["tip_d"], gn["band_h"] or band_values(c)):
            out.append(dict(family="nesting", D=D, band_h=float(bh), tip_d=td, alpha=a, land=1.0, vclr=0.6,
                            ctip=0.4, cb=0.4))
    out.append(dict(family="cylinder", D=D, band_h=max(c["gripper"]["grip_band_max_mm"], 15.0), ct=0.5, cb=0.4))
    return out


def name_of(p: dict) -> str:
    if p["family"] == "shoulder":
        return f"S{p['alpha']:g}-b{p['band_h']:g}-s{p['shoulder']:g}-t{p['tip_d']:g}"
    if p["family"] == "nesting":
        return f"N{p['alpha']:g}-b{p['band_h']:g}-t{p['tip_d']:g}"
    return f"CYL-h{p['band_h']:g}"


def smooth_capture(g: dict, mu: float, step: float = 0.5, emax: float = 12.5) -> float:
    """Largest offset with a seated gentle release at every offset up to it (smooth 2D model)."""
    sim = Section2D(g, section(g, "smooth"))
    best = -step
    e = 0.0
    while e <= emax + 1e-9:
        if sim.simulate(e, d=1.0, tilt=0.0, vx=0.0, mu=mu)["code"] != "seated":
            break
        best = e
        e += step
    return max(best, 0.0) if best >= 0 else -1.0


def search(c: dict, grid: dict | None = None, progress=None) -> list:
    """Evaluate every candidate; return records sorted best first (passing candidates first)."""
    mu = float(np.mean(c["physics"]["friction_range"]))
    recs = []
    cands = candidates(c, grid)
    for i, p in enumerate(cands):
        rec = dict(name=name_of(p), params=p)
        try:
            g = make_piece(p, c["physics"]["density_g_per_mm3"])
        except GeometryError as ex:
            rec.update(buildable=False, reasons=[f"geometry: {ex}"], passes=False)
            recs.append(rec)
            continue
        ch = checks(g, c)
        hard_fail = [f"{x['id']}: {x['value']} (limit {x['limit']})" for x in ch if not x["ok"] and x["id"] not in SOFT_CHECKS]
        rec.update(buildable=True, checks=ch, reasons=hard_fail, passes=not hard_fail,
                   H=round(g["H"], 2), band_h=g["band_h"], capture_geom=round(g["capture"], 2),
                   king_h=round(g["king_h"], 2), m=round(g["m"], 2))
        rec["capture_smooth"] = smooth_capture(g, mu) if g["family"] != "cylinder" else 0.0
        recs.append(rec)
        if progress:
            progress(i, len(cands), rec)
    recs.sort(key=lambda r: (not r.get("passes", False), -(r.get("capture_smooth") or -9), r.get("H", 99)))
    return recs
