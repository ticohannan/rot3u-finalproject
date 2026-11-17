"""The whole study from a constraints file: search, curves, 3D maps, placement, exports, report.

    python -m stackpieces run constraints/checkers_rot3u.json --out results/my_run [--quick]
    python -m stackpieces compare results/my_run/results.json results/reference/results.json

The notebooks call the same steps one at a time.
Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np

from . import __version__
from .export import write_partspec, write_stl
from .geometry import PRESETS, checks, make_piece, summary
from .montecarlo import crossing, metadata, rates, reliable_offset, stacking_curve
from .placement import radial_cdf, stack_rate
from .surface import section
from .sweep import QUICK_GRID, search


def load_constraints(path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def design_record(key: str, params: dict, c: dict, label: str | None = None) -> dict:
    g = make_piece(params, c["physics"]["density_g_per_mm3"])
    rec = summary(g)
    rec.update(family=g["family"], params=params, profile=[[round(r, 4), round(z, 4)] for r, z in g["profile"]],
               checks=checks(g, c), label=label or key)
    return rec


def curve_metrics(curve: dict, c: dict) -> dict:
    goal = c["goal"]
    D = c["piece"]["diameter_mm"]
    tgt = goal["target_offset_fraction"] * (D / 2 if goal["target_offset_of"] == "radius" else D)
    e, f = rates(curve)
    _, top = rates(curve, ("seated", "perched"))
    return dict(e50=crossing(curve), reliable=reliable_offset(curve, goal["seat_rate_required"]),
                rate_at_target=float(np.interp(tgt, e, f, right=0.0)),
                on_top_e50=crossing(curve, 0.5, ("seated", "perched")))


def slim(curve: dict) -> dict:
    return dict(rows=curve["rows"], n=curve["n"], variant=curve["variant"], layer_mm=curve["layer_mm"], meta=curve["meta"])


def run_all(constraints_path, out_dir, chosen: str = "S40", quick: bool = False, log=print,
            map3d_designs=("chosen", "best", "N35", "CYL")) -> dict:
    """Run every step and write results.json, report.html, STL and spec files into out_dir."""
    from .report import write_report
    from .sim3d import outcome_map

    t_start = time.time()
    c = load_constraints(constraints_path)
    os.makedirs(out_dir, exist_ok=True)
    say = (lambda *a: log(f"[{time.time() - t_start:6.0f} s]", *a)) if log else (lambda *a: None)

    # 1. search
    say("search: enumerating candidates from the constraints")
    recs = search(c, QUICK_GRID if quick else None)
    passing = [r for r in recs if r.get("passes")]
    say(f"search: {len(recs)} candidates, {len(passing)} pass the hard checks")

    # 2. printed-surface confirmation of the finalists (+ the chosen design)
    k_final = 3 if quick else 6
    n_final = 6 if quick else 20
    fin_offsets = [x * 0.5 for x in range(0, 21)]
    chosen_params = PRESETS[chosen] if chosen in PRESETS else next(r["params"] for r in recs if r["name"] == chosen)
    finals = [r for r in passing if r["params"]["family"] != "cylinder"][:k_final]
    for r in finals:
        g = make_piece(r["params"])
        cv = stacking_curve(g, c, "layered", n=n_final, offsets=fin_offsets)
        r["printed"] = dict(e50=crossing(cv), reliable=reliable_offset(cv, c["goal"]["seat_rate_required"]), n=n_final)
        say(f"finalist {r['name']}: printed 50 % at {r['printed']['e50']} mm")
    best_rec = max(finals, key=lambda r: ((r["printed"]["e50"] or 0), -r["H"])) if finals else None
    best = best_rec["name"] if best_rec else None
    if best_rec and all(abs(best_rec["params"].get(k, 0) - v) < 1e-9 if isinstance(v, (int, float)) else best_rec["params"].get(k) == v
                        for k, v in chosen_params.items()):
        best = chosen            # the search's best is the chosen design itself

    # 3. curves
    designs = {chosen: design_record(chosen, chosen_params, c, f"{chosen} (chosen)")}
    if best and best != chosen:
        designs[best] = design_record(best, best_rec["params"], c, f"{best} (search best)")
    n_curve = 8 if quick else None
    curves, metrics = {}, {}
    for key, d in designs.items():
        g = make_piece(d["params"])
        curves[key] = {}
        variants = ["smooth", "layered"] + (["hybrid"] if g["family"] != "nesting" else [])
        if key != chosen:
            variants = ["smooth", "layered"]
        for v in variants:
            cv = stacking_curve(g, c, v, n=n_curve, record_first=(key == chosen and v == "layered"))
            curves[key][v] = slim(cv)
            say(f"curve {key} {v}: 50 % at {crossing(cv)} mm ({cv['meta']['seconds']} s)")
        metrics[key] = {v: curve_metrics(cv, c) for v, cv in curves[key].items()}

    # 4. placement
    pe = c["gripper"]["placement_error"]
    placement = {k: {v: stack_rate(*rates(cv), pe) for v, cv in curves[k].items()} for k in curves}
    pl_label = (f"uniform ±{pe['resolution_mm'] / 2:g} mm on each axis ({pe['resolution_mm']:g} mm resolution)"
                if pe["model"] == "uniform" else f"Gaussian, σ {pe['sigma_mm']:g} mm on each axis")

    # 5. 3D outcome maps
    map3d = {}
    want = {"chosen": chosen, "best": best}
    keys3 = []
    for k in map3d_designs:
        k = want.get(k, k)
        if k and k not in keys3:
            keys3.append(k)
    if quick:
        keys3 = [chosen]
    for k in keys3:
        if k not in designs:
            designs[k] = design_record(k, PRESETS[k], c)
        g = make_piece(designs[k]["params"])
        offs = list(range(0, 13, 2)) if quick else list(range(0, 13))
        mus = (0.4,) if quick else (0.25, 0.4, 0.55)
        scen = ["place", "slide"] if quick else None
        t0 = time.time()
        om = outcome_map(g, offs, mus=mus, scenarios=scen)
        om.update(fps=90, record_mu=0.4)
        map3d[k] = om
        say(f"3D map {k}: {time.time() - t0:.0f} s")

    # 6. exports
    for k in [chosen] + ([best] if best and best != chosen else []):
        g = make_piece(designs[k]["params"])
        s = write_stl(os.path.join(out_dir, f"{k}.stl"), g["profile"], name=k)
        write_partspec(os.path.join(out_dir, f"{k}_partspec.json"), g, "piece_" + k.lower().replace("-", "_").replace(".", "p"))
        designs[k]["stl"] = dict(file=f"{k}.stl", volume_mm3=round(s["volume_mm3"], 2), watertight=s["watertight"])

    # 7. layers of the chosen design for the section replay
    gch = make_piece(chosen_params)
    layers = [[[round(x, 4), round(z, 4)] for x, z in P] for P in section(gch, "layered", c["printer"]["layer_mm"])]

    meta = metadata(c, engine="sim2d (numba) + MuJoCo", quick=quick, seconds=time.time() - t_start)
    try:
        import mujoco
        meta["mujoco"] = mujoco.__version__
    except ImportError:
        pass
    res = dict(title=f"Stackable pieces — {c['name']}", meta=meta,
               subtitle=f"Constraints {c['name']}, chosen design {chosen}{' (quick run)' if quick else ''}. "
                        f"Generated {meta['date']} by stackpieces {__version__}.",
               constraints=c, chosen=chosen, best=best, designs=designs, curves=curves, metrics=metrics,
               placement=placement, placement_label=pl_label, placement_cdf=radial_cdf(pe, [2.5, 5.0, 7.5]),
               map3d=map3d, layers=layers, highlight=[x for x in (chosen, best) if x],
               search=[{k: v for k, v in r.items() if k != "checks"} for r in recs])
    with open(os.path.join(out_dir, "results.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, separators=(",", ":"))
    write_report(res, os.path.join(out_dir, "report.html"))
    say(f"done: {out_dir}/report.html")
    return res


def compare(new: dict, ref: dict, tol_mm: float = 1.0) -> list:
    """Differences between two results bundles: curve 50 % points, reliable offsets, 3D maps.
    Returns rows (what, reference, new, difference, flag); flag is True beyond tol_mm or 1 cell."""
    rows = []
    for k in set(new["metrics"]) & set(ref["metrics"]):
        for v in set(new["metrics"][k]) & set(ref["metrics"][k]):
            for m in ("e50", "reliable"):
                a, b = ref["metrics"][k][v][m], new["metrics"][k][v][m]
                d = None if a is None or b is None else round(b - a, 2)
                rows.append((f"{k} {v} {m} (mm)", a, b, d, d is None or abs(d) > tol_mm))
    for k in set(new["map3d"]) & set(ref["map3d"]):
        for key, row in new["map3d"][k]["results"].items():
            if key not in ref["map3d"][k]["results"]:
                continue
            ro = ref["map3d"][k]["results"][key]
            offs_n, offs_r = new["map3d"][k]["offsets"], ref["map3d"][k]["offsets"]
            diff = sum(1 for i, e in enumerate(offs_n) if e in offs_r and row[i]["code"] != ro[offs_r.index(e)]["code"])
            rows.append((f"3D {k} {key} cells differing", 0, diff, diff, diff > 1))
    return rows
