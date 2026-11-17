# Design: Stackable pieces study (DD-STACK)

**Version:** 0.1.0
**Status:** IMPLEMENTED — tested on Linux (Python 3.12 and 3.13, Node 22); not yet run on the
author's WinPython 3.12 laptop; no printed piece tested yet
**Updated:** 2026-11-15
**Applies to:** `stackable-checkers/` in rot3u-finalproject (package `stackpieces` 0.1.0, `web/` 0.1.0)

Robot-placed game pieces (checkers for Sir Pinch-a-lot) that stack into kings even when the arm puts
the top piece off-center. This study turns the design work of 2026-11-13 into code that anyone can
rerun from a constraints file: piece geometry, print checks, 3D and 2D settling simulations, a
printed-surface (layer) model, stacking-rate curves, the robot's overall stack rate, a report, STL and
a SolidWorks spec.

The folder is self-contained (no imports from the rest of the repository, relative paths only).

## 1. Requirements

| ID | Requirement |
| --- | --- |
| SP-1 | All inputs that were constraints at the start of the design live in one JSON file (`constraints/*.json`): board, piece diameter, gripper grip-band limits and placement error, printer rules, friction range, release conditions, goal. Every number carries a note or a source |
| SP-2 | Three piece families, all bodies of revolution: **shoulder** (grip band, flat shoulder, cone, matching socket; S40 is one), **nesting** (grip band, cone from the full diameter, socket nesting up into the cone; N35 is one), **cylinder** (control). Parameters in mm and degrees |
| SP-3 | Geometry gives the half-section profile, the 2D section as convex polygons, 3D convex parts, mass, center of mass, inertia, geometric capture (socket mouth radius − tip radius), stack pitch and king height |
| SP-4 | Print and robot checks against the constraints: overhang ≤ limit, bridge ≤ limit, minimum wall, grip band ≥ minimum and ≤ maximum, socket fits inside the piece, finger room between diagonal neighbors, height ≤ maximum. Each check returns pass/fail with the value and the limit |
| SP-5 | Printed-surface model: each slicer layer of height `t` is a convex stadium whose rounded ends reach the CAD radius at mid-layer. Variants: `smooth` (CAD), `layered` (as printed), `hybrid` (as printed, cone sanded smooth; shoulder family) |
| SP-6 | 2D section simulation (rigid bodies, gravity, Coulomb friction, restitution) of a piece released over another piece resting on the board. Inputs: offset, release height above the cone tip, tilt (outer side down), sideways speed, μ, restitution, surface variant. Output: `seated` (center within 1 mm, tilt < 3°, height within 0.6 mm of the seat), `perched`, `fell`, `knocked`, final pose, optional trajectory |
| SP-7 | The 2D engine exists in JavaScript (`web/sim2d.js`, browser and Node) and Python (`stackpieces/sim2d.py`, Numba). Both give the same outcome on the parity cases and final poses within 0.05 mm / 0.5° on non-chaotic cases |
| SP-8 | 3D simulation in MuJoCo (pip wheel; no compiler needed on Windows), pieces built from convex parts, same scenarios and classification as SP-6. PyBullet is optional and only used for cross-checks where it installs |
| SP-9 | Monte Carlo stacking-rate curves: for each offset, N releases with randomized μ, release height, tilt and sideways speed from the constraints, a seeded generator identical in Python and JavaScript |
| SP-10 | Overall stack rate for a robot: combine a stacking-rate curve with the placement-error model (per-axis uniform ± resolution/2, or Gaussian σ) |
| SP-11 | Design-space search from the constraints alone: enumerate family parameters, drop candidates that fail SP-4, rank the rest by the 50 % offset of the smooth 2D curve, confirm the finalists with printed surfaces and 3D |
| SP-12 | Exports: STL of a design (watertight revolve, mm), parametric SolidWorks spec (`revolve` + `revolve_cut`, Global Variables) for the shoulder family, and a self-contained HTML report (constraints, candidates, curves, outcome map, replays, geometry, print checks, method) |
| SP-13 | Reproducibility: every result file records the package version, constraints file hash, seeds and engine; `results/reference/` holds the data behind the published report |
| SP-14 | Runs on WinPython 3.12 with `pip install` of wheels only (numpy, numba, mujoco, matplotlib, jupyter); the web tool runs from `web/index.html` in a browser with no install (three.js from a CDN for the 3D view only) |

## 2. Units and frames

mm, g, s; gravity 9810 mm/s². Piece frame: origin on the axis in the bottom face, z up along the axis.
Offset `e` is the horizontal distance from the lower piece's axis to the released piece's axis. Tilt
is about the horizontal axis perpendicular to the offset, positive when the released piece's outer
side (away from the lower piece's axis) is lower.

## 3. Families and parameters

| Param | Shoulder | Nesting | Cylinder |
| --- | --- | --- | --- |
| `D` diameter | yes | yes | yes |
| `band_h` | height of the shoulder plane | height where the cone starts at full diameter | total height |
| `shoulder` | radial width of the flat shoulder | — | — |
| `tip_d` | flat tip diameter | flat tip diameter | — |
| `alpha` | cone half-angle from the axis, deg | same | — |
| `clr` | socket radial clearance at the mouth | — | — |
| `vclr` | gap from the lower tip to the socket roof when seated | same | — |
| `land` | — | flat ring width at the socket mouth | — |
| `ctip` | — | radial clearance at the socket roof | — |
| `ct`, `cb` | top and bottom outer 45° chamfers | bottom chamfer | both |

Presets: `S40` (shoulder, D 25, band 12, shoulder 2, tip 4, 40°, clr 0.5, vclr 0.5, ct 0.5, cb 0.4),
`N35` (nesting, D 25, band 12, tip 1.6, 35°, land 1, vclr 0.6, ctip 0.4, cb 0.4), `CYL` (D 25, H 15).

## 4. Tests

| Requirement | Test |
| --- | --- |
| SP-1 | `test_constraints.py::test_reference_constraints_load` (schema, notes) |
| SP-2, SP-3 | `test_geometry.py` (S40 volume vs the SolidWorks part, 5,659.463 mm³ within 0.1 %; N35 and CYL vs analytic; capture, pitch) |
| SP-4 | `test_geometry.py::test_checks_s40` (S40 passes printer checks; finger room flags 10.9 mm) |
| SP-5 | `test_surface.py` (layer count, stadium reaches the CAD radius at mid-layer) |
| SP-6 | `test_sim2d.py` (S40 smooth: seated at 0 and 2 mm, not seated at 8 mm; cylinder stays flat) |
| SP-7 | `test_parity.py` (Node runs `web/sim2d.js` on the parity cases; skipped with a message if Node is missing) |
| SP-8 | `test_sim3d.py` (S40: seated at 0 and 3 mm, not seated at 7 mm, μ 0.4) |
| SP-9 | `test_montecarlo.py` (seeded sampler equals the JavaScript sequence; reference curve reproduced within 15 points at each offset) |
| SP-10 | `test_placement.py` (step curve and uniform error give the analytic area fraction) |
| SP-11 | `test_sweep.py` (quick grid returns ranked candidates, failing candidates carry reasons) |
| SP-12 | `test_export.py` (STL watertight and volume, spec values, report HTML contains the data) |
| SP-13 | `test_montecarlo.py::test_result_metadata` |
| SP-14 | `README.md` install steps, run in a fresh Python 3.12 virtual environment from the zip |
