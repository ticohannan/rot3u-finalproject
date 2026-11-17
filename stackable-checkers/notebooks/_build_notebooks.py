"""Writes notebooks 01-05 from the cells below (edit here, then run: python _build_notebooks.py).
Run them with: jupyter nbconvert --to notebook --execute --inplace 0*.ipynb (or open in Jupyter).
Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
import nbformat as nbf

SETUP = '''# Setup: works from this folder whether or not the package was pip-installed
import sys, os, json
HERE = os.path.abspath("")
ROOT = os.path.dirname(HERE) if os.path.basename(HERE) == "notebooks" else HERE
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
import numpy as np
import matplotlib.pyplot as plt
import stackpieces
print("stackpieces", stackpieces.__version__, "| folder", os.path.relpath(ROOT))
CONSTRAINTS = os.path.join(ROOT, "constraints", "checkers_rot3u.json")
REFERENCE = os.path.join(ROOT, "results", "reference", "results.json")
c = json.load(open(CONSTRAINTS))
QUICK = True   # True: a few minutes, coarse. False: the full reference run (tens of minutes).'''

NB = {}

NB["01_constraints_and_geometry"] = [
    ("md", """# 01 — Constraints and geometry

The study starts from the constraints file (`constraints/checkers_rot3u.json`): the board, the piece
diameter, what the gripper needs, the printer's rules and the physics ranges. Every number there has a
note saying where it came from. This notebook builds the three piece families from those numbers,
draws them, and runs the print and robot checks (DD-STACK SP-1 to SP-5)."""),
    ("code", SETUP),
    ("code", '''from stackpieces.geometry import PRESETS, make_piece, checks, summary, volume_exact
for group in ("board", "piece", "gripper", "goal", "printer"):
    print(group, {k: v for k, v in c[group].items() if not k.startswith("_")})'''),
    ("md", """## The three families

* **shoulder** (S40 is one): grip band, flat shoulder, cone, matching socket underneath. The piece above
  seats with its bottom rim flat on the shoulder.
* **nesting** (N35): the cone starts at the full diameter and the socket nests up inside the piece's own
  cone, like stacking cups. Steeper cones without a taller band.
* **cylinder**: the control. It stacks flat but never centers."""),
    ("code", '''pieces = {k: make_piece(p, c["physics"]["density_g_per_mm3"]) for k, p in PRESETS.items()}
fig, axs = plt.subplots(1, 3, figsize=(12, 4.5))
for ax, (k, g) in zip(axs, pieces.items()):
    for z0, col in ((0, "#b8322f"), (g["seat"], "#3a3a3a")):
        pr = g["profile"]
        pts = pr + [(-r, z) for r, z in reversed(pr)]
        ax.fill([x for x, _ in pts], [z + z0 for _, z in pts], color=col, alpha=0.8)
    ax.axhline(0, color="0.7"); ax.set_aspect("equal"); ax.set_title(f"{k}: two stacked")
    ax.set_xlabel("mm"); ax.set_xlim(-15, 15)
plt.tight_layout(); plt.show()'''),
    ("code", '''import pandas as pd  # WinPython includes pandas; only used for tables here
rows = {k: summary(g) for k, g in pieces.items()}
pd.DataFrame(rows).round(2)'''),
    ("md", """## Volume check

The mass properties come from integrating the half-section on a 0.05 mm grid (the browser tool uses the
same grid). The exact volume of the revolved polygon (Pappus) checks it; S40's exact volume also matches
the SolidWorks part in `cad/checker_s40`, 5,659.463 mm³."""),
    ("code", '''for k, g in pieces.items():
    ex = volume_exact(g["profile"])
    print(f"{k}: grid {g['V']:.2f} mm3, exact {ex:.3f} mm3, difference {100 * (g['V'] - ex) / ex:+.3f} %")'''),
    ("md", """## Print and robot checks

`finger_room` fails until the gripper's finger thickness is measured (it is `null` in the constraints): the gap
between pieces on diagonal neighbor squares is shown so it can be compared by hand. N35 fails the rim check:
its flat ring on the bed is 0.6 mm wide after the 0.4 mm chamfer, under the printer's 0.8 mm minimum wall."""),
    ("code", '''pd.DataFrame({k: {x["id"]: ("pass " if x["ok"] else "FAIL ") + str(x["value"]) for x in checks(g, c)}
              for k, g in pieces.items()})'''),
    ("md", """## Printed surfaces

With 0.2 mm layers, each layer's edge is a rounded bead that reaches the CAD surface at mid-layer, so the cone
and the socket become staircases of ridges (DD-STACK SP-5). The trigonometric staircase model gives the
roughness Ra = layer/4 · cos(inclination)."""),
    ("code", '''from stackpieces.surface import section, ra_staircase
g = pieces["S40"]
fig, axs = plt.subplots(1, 2, figsize=(12, 5))
for ax, (v, lim) in zip(axs, (("smooth", None), ("layered", (0, 8, 12, 18)))):
    for P in section(g, v, c["printer"]["layer_mm"]):
        ax.fill([x for x, _ in P], [z for _, z in P], color="#b8322f", ec="w", lw=0.3)
    ax.set_aspect("equal"); ax.set_title(f"S40 section, {v}")
    if lim: ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3]); ax.set_title("S40 printed layers, zoom on the cone")
plt.show()
print(f"Ra on the 40 deg cone (50 deg from the bed): {1000 * ra_staircase(0.2, 50):.0f} um")'''),
]

NB["02_design_space_search"] = [
    ("md", """# 02 — Design-space search

Can the constraints alone lead back to S40? `stackpieces.sweep.search` enumerates shoulder and nesting pieces
(cone angle, shoulder width, tip size, band height), builds each, drops the ones that fail a hard check, and
ranks the rest by **smooth capture**: the largest offset up to which a gentle release seats at every offset
(2D model, mid friction). Then the finalists are rerun with printed layers, which is the number that matters
(DD-STACK SP-11)."""),
    ("code", SETUP),
    ("code", '''import pandas as pd, time
from stackpieces.sweep import search, DEFAULT_GRID, QUICK_GRID
t = time.time()
recs = search(c, DEFAULT_GRID)     # about 25 s; the full grid is cheap
print(f"{len(recs)} candidates in {time.time() - t:.0f} s; {sum(r['passes'] for r in recs)} pass the hard checks")
from collections import Counter
print("rejected by:", Counter(r["reasons"][0].split(":")[0] for r in recs if not r["passes"]))'''),
    ("code", '''top = [r for r in recs if r["passes"]][:15]
pd.DataFrame([dict(name=r["name"], H=r["H"], band=r["band_h"], geometric_capture=r["capture_geom"],
                   smooth_capture=r["capture_smooth"]) for r in top])'''),
    ("code", '''ok = [r for r in recs if r.get("buildable") and r["params"]["family"] != "cylinder"]
fig, ax = plt.subplots(figsize=(8, 5))
for fam, mk in (("shoulder", "o"), ("nesting", "s")):
    pts = [r for r in ok if r["params"]["family"] == fam]
    ax.scatter([r["H"] for r in pts], [r["capture_smooth"] for r in pts], marker=mk, label=fam,
               c=["#2a78d6" if r["passes"] else "#c3c2b7" for r in pts])
ax.axvline(c["goal"]["max_height_mm"], ls=":", c="0.5"); ax.text(c["goal"]["max_height_mm"] + 0.2, 0.5, "max height")
ax.set_xlabel("piece height H (mm)"); ax.set_ylabel("smooth capture (mm)"); ax.legend(); ax.set_title("Blue: passes every hard check")
plt.show()'''),
    ("md", """## Printed-layer confirmation of the finalists

The smooth ranking is only a screen: printed ridges cut every design's range, by different amounts. Quick mode
uses 6 releases per offset (±40 points); the reference used 20."""),
    ("code", '''from stackpieces.geometry import make_piece
from stackpieces.montecarlo import stacking_curve, crossing, reliable_offset
k_final, n = (3, 6) if QUICK else (6, 20)
finals = [r for r in recs if r["passes"] and r["params"]["family"] != "cylinder"][:k_final]
s40 = next(r for r in recs if r["name"] == "S40-b12-s2-t4")
if s40 not in finals: finals.append(s40)
offs = [x * 0.5 for x in range(21)]
for r in finals:
    cv = stacking_curve(make_piece(r["params"]), c, "layered", n=n, offsets=offs)
    r["printed_e50"] = crossing(cv); r["printed_reliable"] = reliable_offset(cv, c["goal"]["seat_rate_required"])
    print(f"{r['name']:22s} H {r['H']:5.1f}  smooth {r['capture_smooth']:4.1f}  printed 50 % at {r['printed_e50']:.2f} mm")'''),
    ("md", """## What the reference run found

In the reference run (results/reference), S40 as specified (2 mm shoulder) is not the best shoulder piece: the
same cone angle with a **3 mm shoulder** (S40-b12-s3-t4, 20.9 mm tall) or a 35° cone with a 3 mm shoulder
(S35-b12.5-s3-t4, 23.2 mm) seat further off center when printed. See docs/method.md, "Search result"."""),
    ("code", '''import json, os
if os.path.exists(REFERENCE):
    ref = json.load(open(REFERENCE))
    fin = [r for r in ref["search"] if r.get("printed")]
    print("reference finalists (printed 50 % point, mm):")
    for r in fin: print(f"  {r['name']:22s} H {r['H']:5.1f}  printed 50 % {r['printed']['e50']:.2f}  reliable {r['printed']['reliable']}")
    print("search best:", ref["best"])'''),
]

NB["03_settling_3d"] = [
    ("md", """# 03 — 3D settling (MuJoCo)

A piece rests on the board; an identical piece is released above it, off center. MuJoCo simulates both as free
rigid bodies made of convex parts (DD-STACK SP-8). Four release scenarios, three friction values. MuJoCo
installs from a wheel on Windows (no compiler). An earlier run used PyBullet; docs/method.md compares them."""),
    ("code", SETUP),
    ("code", '''from stackpieces.geometry import make_piece, PRESETS
from stackpieces.sim3d import Stack3D, SCENARIOS, outcome_map
g = make_piece(PRESETS["S40"])
st = Stack3D(g)
for e in (0, 3, 6, 7, 9):
    r = st.simulate(e, d=1.0, mu=0.4)
    print(f"offset {e} mm: {r['code']:8s} final offset {r['dxy']:.2f} mm, tilt {r['tilt']:.1f} deg, height {r['dz']:.2f} mm")'''),
    ("code", '''offs = list(range(0, 13, 2)) if QUICK else list(range(13))
mus = (0.4,) if QUICK else (0.25, 0.4, 0.55)
scen = ["place", "slide"] if QUICK else list(SCENARIOS)
om = outcome_map(g, offs, mus=mus, scenarios=scen, progress=lambda s, mu, row: print(f"{s:7s} mu {mu}: {row}"))'''),
    ("code", '''from matplotlib.colors import ListedColormap
code = {"seated": 0, "perched": 1, "fell": 2, "knocked": 3}
keys = list(om["results"])
A = np.array([[code[r["code"]] for r in om["results"][k]] for k in keys])
fig, ax = plt.subplots(figsize=(10, 0.45 * len(keys) + 1))
ax.imshow(A, cmap=ListedColormap(["#0ca30c", "#fab219", "#d03b3b", "#ec835a"]), vmin=0, vmax=3, aspect="auto")
ax.set_yticks(range(len(keys)), keys); ax.set_xticks(range(len(offs)), offs); ax.set_xlabel("offset (mm)")
ax.set_title("S40 in 3D: green seated, yellow perched, red fell off")
plt.show()'''),
    ("md", """## Compare with the reference run

Counts how many cells differ from `results/reference` for the same scenario and friction. One cell is within the
normal spread of a model this sensitive at the edge of capture."""),
    ("code", '''import os
if os.path.exists(REFERENCE):
    ref = json.load(open(REFERENCE))["map3d"]["S40"]
    for k, row in om["results"].items():
        if k in ref["results"]:
            rr = ref["results"][k]
            diff = sum(row[i]["code"] != rr[ref["offsets"].index(e)]["code"] for i, e in enumerate(offs) if e in ref["offsets"])
            print(f"{k:12s} cells differing from the reference: {diff}")'''),
]

NB["04_printed_surfaces"] = [
    ("md", """# 04 — Printed surfaces and the stacking-rate curve

The answer to "what fraction of releases stack, at each offset?" for S40, with smooth CAD surfaces, as printed
(0.2 mm layers) and with the cone sanded. Each offset gets N releases with friction, release height, tilt and
sideways speed drawn from the constraints (DD-STACK SP-9). Then the curve is combined with the arm's
placement error to give the overall king-stacking rate (SP-10)."""),
    ("code", SETUP),
    ("code", '''from stackpieces.geometry import make_piece, PRESETS
from stackpieces.montecarlo import stacking_curve, crossing, reliable_offset, rates
g = make_piece(PRESETS["S40"])
n = 8 if QUICK else None                       # None: releases_per_offset from the constraints
offs = [x * 1.0 for x in range(13)] if QUICK else None
curves = {}
for v in ("smooth", "layered", "hybrid"):
    curves[v] = stacking_curve(g, c, v, n=n, offsets=offs)
    print(f"{v:8s} 50 % at {crossing(curves[v])} mm, reliable (>= {c['goal']['seat_rate_required']:.0%}) to "
          f"{reliable_offset(curves[v], c['goal']['seat_rate_required'])} mm, {curves[v]['meta']['seconds']} s")'''),
    ("code", '''D = c["piece"]["diameter_mm"]
fig, ax = plt.subplots(figsize=(10, 5))
for v, col, name in (("smooth", "#2a78d6", "Smooth (CAD)"), ("hybrid", "#1baf7a", "Printed, cone sanded"), ("layered", "#eb6834", "Printed layers")):
    e, f = rates(curves[v]); ax.plot(100 * e / D, 100 * f, "o-", c=col, label=name)
e, f = rates(curves["layered"], ("seated", "perched")); ax.plot(100 * e / D, 100 * f, "--", c="#eb6834", label="Printed: stays on top")
if os.path.exists(REFERENCE):
    ref = json.load(open(REFERENCE))["curves"]["S40"]["layered"]
    er = np.array([r["e"] for r in ref["rows"]]); fr = np.array([r["counts"]["seated"] for r in ref["rows"]]) / ref["n"]
    ax.plot(100 * er / D, 100 * fr, ":", c="0.4", label=f"Printed, reference ({ref['n']} per offset)")
ax.set_xlabel("offset, % of diameter"); ax.set_ylabel("% of releases that seat"); ax.set_xlim(0, 50); ax.legend(); ax.grid(alpha=.3)
sec = ax.secondary_xaxis("top", functions=(lambda p: p * D / 100, lambda mm: mm * 100 / D)); sec.set_xlabel("mm")
plt.show()'''),
    ("md", """## Overall stack rate for the robot

`placement_error` in the constraints: about 1 cm resolution, modeled as each horizontal axis uniform in ±5 mm."""),
    ("code", '''from stackpieces.placement import stack_rate, radial_cdf
pe = c["gripper"]["placement_error"]
print("arm lands within 2.5 / 5 / 7.5 mm of the axis:", [f"{p:.0%}" for p in radial_cdf(pe, [2.5, 5, 7.5])])
for v in curves:
    print(f"{v:8s}: {stack_rate(*rates(curves[v]), pe):.0%} of king stacks seat on the first release")
print("with 5 mm resolution (after compensation):",
      {v: f"{stack_rate(*rates(curves[v]), dict(model='uniform', resolution_mm=5)):.0%}" for v in curves})'''),
    ("md", """## One release, frame by frame

The same engine can record a single release; the browser tool (`web/index.html`) animates it."""),
    ("code", '''from stackpieces.sim2d import Section2D
from stackpieces.surface import section
sim = Section2D(g, section(g, "layered"))
r = sim.simulate(4.0, d=1.0, mu=0.3, record=True)
print(r["code"], f"final offset {r['dx']:.2f} mm, tilt {r['tilt']:.1f} deg, {len(r['frames'])} frames")
fr = np.array(r["frames"]); plt.figure(figsize=(8, 3)); plt.plot(fr[:, 0], fr[:, 4], label="released piece x (mm)")
plt.plot(fr[:, 0], np.degrees(fr[:, 6]), label="its tilt (deg)"); plt.xlabel("s"); plt.legend(); plt.show()'''),
]

NB["05_report_and_exports"] = [
    ("md", """# 05 — Report and exports

`stackpieces.pipeline.run_all` runs every step from the constraints and writes `results.json`, a self-contained
`report.html` (open it in a browser), STL files and parametric SolidWorks specs. Quick mode takes about 2 minutes; the
full run that produced `results/reference` takes about 15–20 minutes. `compare` lists what moved."""),
    ("code", SETUP),
    ("code", '''from stackpieces.pipeline import run_all, compare
OUT = os.path.relpath(os.path.join(ROOT, "results", "notebook_run"))
res = run_all(CONSTRAINTS, OUT, chosen="S40", quick=QUICK)
print("report:", os.path.join(OUT, "report.html"))'''),
    ("code", '''if os.path.exists(REFERENCE):
    ref = json.load(open(REFERENCE))
    for what, a, b, d, flag in compare(res, ref):
        print(("CHECK " if flag else "ok    ") + f"{what:45s} reference {a}  new {b}  diff {d}")
    print("(quick mode uses few releases, so CHECK lines on 50 % points are expected there)")'''),
    ("md", """## Files for printing and for SolidWorks

The STL is a watertight revolve in mm, ready for Bambu Studio (socket on the bed, no supports). The spec is the
partspec JSON format of Tico's SolidWorks build scripts (`revolve` + `revolve_cut`, every size a Global Variable).
For S40 it matches `cad/checker_s40/checker_s40_partspec.json`, which the SolidWorks part was built from."""),
    ("code", '''from stackpieces.geometry import make_piece, PRESETS
from stackpieces.export import write_stl, write_partspec
g = make_piece(PRESETS["S40"])
print(write_stl(os.path.join(OUT, "S40.stl"), g["profile"], name="S40"))
spec = write_partspec(os.path.join(OUT, "S40_partspec.json"), g, "checker_s40")
print(json.dumps(spec["params"], indent=1))'''),
]

for name, cells in NB.items():
    nb = nbf.v4.new_notebook()
    nb["metadata"]["kernelspec"] = {"name": "python3", "display_name": "Python 3", "language": "python"}
    nb["cells"] = [nbf.v4.new_markdown_cell(src) if kind == "md" else nbf.v4.new_code_cell(src) for kind, src in cells]
    nbf.write(nb, f"{name}.ipynb")
    print("wrote", name)
