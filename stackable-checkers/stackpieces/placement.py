"""Overall stack rate for a robot's placement error (DD-STACK SP-10).

The released piece lands at a horizontal error (ex, ey) from the lower piece's axis. ``uniform``: each
axis uniform in +-resolution/2 (a robot that can only command positions on a grid of that pitch);
``gaussian``: each axis normal with sigma. The stack rate is the mean of the stacking-rate curve
S(r) over that error, with r = hypot(ex, ey) and S = 0 beyond the curve's last offset.

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import numpy as np


def error_grid(model: dict, n: int = 401):
    """Grid of radial errors and weights (sum 1) for the placement-error model."""
    if model["model"] == "uniform":
        h = model["resolution_mm"] / 2
        u = (np.arange(n) + 0.5) / n * 2 * h - h
        w1 = np.full(n, 1.0 / n)
    elif model["model"] == "gaussian":
        s = model["sigma_mm"]
        u = np.linspace(-5 * s, 5 * s, n)
        w1 = np.exp(-u ** 2 / (2 * s * s))
        w1 /= w1.sum()
    else:
        raise ValueError("placement_error.model must be 'uniform' or 'gaussian'")
    X, Y = np.meshgrid(u, u)
    return np.hypot(X, Y).ravel(), np.outer(w1, w1).ravel()


def stack_rate(offsets, rate, model: dict) -> float:
    """Expected fraction of placements that stack, for a curve rate(offsets)."""
    r, w = error_grid(model)
    s = np.interp(r, offsets, rate, right=0.0)
    return float((s * w).sum())


def radial_cdf(model: dict, r_values) -> list:
    """P(radial error <= r) for each r: how often the arm lands within r of the axis."""
    r, w = error_grid(model)
    return [float(w[r <= x].sum()) for x in r_values]
