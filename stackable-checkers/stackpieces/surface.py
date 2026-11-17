"""Printed-surface model (DD-STACK SP-5): each slicer layer is a convex stadium.

A layer of height t covering z0..z0+t is cut at mid-layer (as slicers do); each material interval
[r_in, r_out] at that height becomes a stadium whose rounded ends (radius t/2) reach exactly r_in and
r_out at mid-layer. Sloped surfaces become staircases of rounded ridges.

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import math

from .geometry import material_intervals

VARIANTS = ("smooth", "layered", "hybrid")


def stadium(x0, x1, z0, t):
    """Convex stadium from x0 to x1, z0 to z0 + t, ends rounded (5 points per end). CCW."""
    r = t / 2
    zc = z0 + r
    pts = []
    for k in range(-2, 3):
        a = k * math.pi / 4
        pts.append((x1 - r + r * math.cos(a), zc + r * math.sin(a)))
    for k in range(2, 7):
        a = k * math.pi / 4
        pts.append((x0 + r + r * math.cos(a), zc + r * math.sin(a)))
    return pts


def layers(g: dict, t: float) -> list:
    """Section of the piece as printed with layer height t: list of convex polygons."""
    out = []
    n = int(round(g["H"] / t))
    for i in range(n):
        z0 = i * t
        zm = z0 + t / 2
        if zm >= g["H"]:
            break
        for a, b in material_intervals(g["profile"], zm):
            if b - a < t * 0.5:
                continue
            if a < 1e-9:
                out.append(stadium(-b, b, z0, t))
            else:
                out.append(stadium(-b, -a, z0, t))
                out.append(stadium(a, b, z0, t))
    return out


def section(g: dict, variant: str = "smooth", t: float = 0.2) -> list:
    """Convex polygons of the section for a surface variant (smooth | layered | hybrid)."""
    if variant == "smooth":
        return [list(p) for p in g["section"]]
    if variant == "layered":
        return layers(g, t)
    if variant == "hybrid":
        if g["family"] == "nesting":
            raise ValueError("hybrid (sanded cone) is defined for the shoulder and cylinder families only")
        below = [P for P in layers(g, t) if P[0][1] < g["cone_start"] - 1e-6]
        if g["family"] == "shoulder":
            below.append(list(g["section"][3]))
        return below
    raise ValueError(f"unknown surface variant {variant!r}; expected one of {VARIANTS}")


def ra_staircase(layer_mm: float, inclination_deg: float) -> float:
    """Arithmetic roughness of an ideal layer staircase on a surface inclined at the given angle
    to the build plate: Ra = layer/4 * cos(inclination) (trigonometric staircase model)."""
    return layer_mm / 4 * math.cos(math.radians(inclination_deg))
