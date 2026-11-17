"""Monte Carlo stacking-rate curves (DD-STACK SP-9, SP-13).

The release-condition generator is the same 32-bit LCG and Box-Muller transform as web/sim2d.js, so
Python and the browser draw identical releases for the same seed.

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import hashlib
import json
import math
import platform
import time

import numpy as np

from . import __version__
from .sim2d import Section2D
from .surface import section

CODES = ("seated", "perched", "fell", "knocked")


class Rng:
    """32-bit LCG (Numerical Recipes constants), uniform in [0, 1). Same sequence as web/sim2d.js rng()."""

    def __init__(self, seed: int):
        self.s = seed & 0xFFFFFFFF

    def __call__(self) -> float:
        self.s = (self.s * 1664525 + 1013904223) & 0xFFFFFFFF
        return self.s / 4294967296.0

    def randn(self) -> float:
        u = self()
        v = self()
        return math.sqrt(-2 * math.log(u + 1e-12)) * math.cos(2 * math.pi * v)


def sample_releases(phys: dict, seed: int, i: int, n: int) -> list:
    """n release conditions for offset index i (friction, height above the tip, tilt, sideways speed)."""
    r = Rng(seed + i * 7919)
    m0, m1 = phys["friction_range"]
    d0, d1 = phys["release"]["height_above_tip_mm"]
    out = []
    for _ in range(n):
        mu = m0 + (m1 - m0) * r()
        d = d0 + (d1 - d0) * r()
        tilt = phys["release"]["tilt_sigma_deg"] * r.randn()
        vx = phys["release"]["sideways_speed_sigma_mm_s"] * r.randn()
        out.append(dict(mu=mu, d=d, tilt=tilt, vx=vx, rest=phys["restitution"]))
    return out


def offsets_from(analysis: dict) -> list:
    o = analysis["offsets_mm"]
    n = int(round((o["stop"] - o["start"]) / o["step"])) + 1
    return [round(o["start"] + k * o["step"], 6) for k in range(n)]


def constraints_hash(c: dict) -> str:
    return hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()[:12]


def stacking_curve(g: dict, constraints: dict, variant: str = "layered", n: int | None = None,
                   offsets: list | None = None, record_first: bool = False, progress=None,
                   only: list | None = None) -> dict:
    """Seat/perch/fall counts against offset for one piece and surface variant.
    ``only``: run just these offset indices (same seeds as in the full run)."""
    an, ph = constraints["analysis"], constraints["physics"]
    n = n or an["releases_per_offset"]
    offs = offsets if offsets is not None else offsets_from(an)
    sim = Section2D(g, section(g, variant, constraints["printer"]["layer_mm"]))
    rows = []
    t0 = time.time()
    for i, e in enumerate(offs):
        if only is not None and i not in only:
            continue
        counts = {k: 0 for k in CODES}
        replay = None
        for k, rel in enumerate(sample_releases(ph, an["seed"], i, n)):
            r = sim.simulate(e, d=rel["d"], tilt=rel["tilt"], vx=rel["vx"], mu=rel["mu"], rest=rel["rest"],
                             record=record_first and k == 0)
            counts[r["code"]] += 1
            if record_first and k == 0:
                replay = dict(release=rel, code=r["code"], dx=r["dx"], tilt=r["tilt"],
                              frames=[[round(v, 4) for v in f] for f in r["frames"]])
        row = dict(e=e, counts=counts)
        if replay:
            row["replay"] = replay
        rows.append(row)
        if progress:
            progress(i, len(offs), e, counts)
    return dict(rows=rows, n=n, variant=variant, layer_mm=constraints["printer"]["layer_mm"],
                meta=metadata(constraints, engine="sim2d (numba)", seconds=time.time() - t0))


def metadata(constraints: dict, engine: str, **extra) -> dict:
    m = dict(package="stackpieces", version=__version__, engine=engine,
             constraints=constraints.get("name"), constraints_hash=constraints_hash(constraints),
             seed=constraints["analysis"]["seed"], python=platform.python_version(),
             date=time.strftime("%Y-%m-%d"))
    m.update({k: (round(v, 1) if isinstance(v, float) else v) for k, v in extra.items()})
    return m


def rates(curve: dict, codes=("seated",)) -> tuple:
    """(offsets, fraction) for the sum of the given outcome codes."""
    e = np.array([r["e"] for r in curve["rows"]])
    f = np.array([sum(r["counts"][c] for c in codes) for r in curve["rows"]]) / curve["n"]
    return e, f


def crossing(curve: dict, level: float = 0.5, codes=("seated",)) -> float | None:
    """First offset where the rate falls below ``level`` (linear interpolation)."""
    e, f = rates(curve, codes)
    for k in range(1, len(e)):
        if f[k - 1] >= level > f[k]:
            return float(e[k - 1] + (f[k - 1] - level) / (f[k - 1] - f[k]) * (e[k] - e[k - 1]))
    return None if f[0] < level else float(e[-1])


def reliable_offset(curve: dict, required: float) -> float:
    """Largest offset such that every offset up to it seats at least ``required`` of the time."""
    e, f = rates(curve)
    best = 0.0
    for k in range(len(e)):
        if f[k] < required:
            break
        best = float(e[k])
    return best
