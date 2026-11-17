# Stackable checkers study

**Group members: to reproduce the results, follow [`REPRODUCE.md`](REPRODUCE.md).**

Design and settling analysis of 3D-printed game pieces that a robot arm can stack even when it places
the top piece off center: checkers for Sir Pinch-a-lot (ROT3U arm), kings made of two pieces. Each
piece has a cylindrical grip band, a centering cone on top and a matching socket underneath.

Everything is computed from a **constraints file** (`constraints/checkers_rot3u.json`): board
squares, piece diameter, what the gripper needs, the arm's placement error, the printer's rules,
friction and release conditions. Change the file and rerun to redo the analysis for another
board, gripper or printer.

| Folder | What it holds |
| --- | --- |
| `constraints/` | The starting constraints (`checkers_rot3u.json`, every number with a note) and `template.json` |
| `stackpieces/` | Python package: geometry and checks, printed-layer model, 2D and 3D settling simulations, Monte Carlo curves, placement error, search, exports, report |
| `notebooks/` | Jupyter notebooks 01–05 that walk through the study step by step |
| `web/` | Browser tool (`index.html`): edit a design, check it, run the 2D simulation, export an STL. No install |
| `results/reference/` | The full run behind the published report: `results.json`, `report.html`, STL and SolidWorks specs |
| `cad/checker_s40/` | The S40 piece as built in SolidWorks: `.SLDPRT`, `.step`, print files (`.3mf`) and the spec it was built from |
| `docs/` | `design.md` (requirements DD-STACK SP-1 to SP-14 and their tests), `method.md` (physics, validation, sources) |
| `tests/` | `python tests/run_tests.py` |

**Status:** tested on Linux (Python 3.12 and 3.13, Node 22); **not yet run on WinPython**; no printed
piece tested yet. See `docs/method.md` for what the models leave out.

## Results in one paragraph (reference run, 2026-11-15)

S40 (Ø25, 12 mm grip band, 40° cone on a 2 mm shoulder, 22.1 mm tall), released level and still about
1 mm above the piece below, seats at least 80 % of the time up to about 2.5 mm off center when printed
with 0.2 mm layers, and half the time at about 3.4 mm (14 % of the diameter); smooth CAD surfaces
would give 5.5 mm. The printed layer ridges on the cone and in the socket hook each other. With the
arm's ±5 mm placement error, about 38 % of king stacks seat on the first release as printed, 61 % with
the cone lightly sanded. The search from the constraints alone finds a better shoulder piece: a 35°
cone on a 3 mm shoulder (S35-b12.5-s3-t4, 23.2 mm tall) seats half the time at about 5 mm when
printed, and 69 % of king stacks seat on the first release.
Open `results/reference/report.html` in a browser for the curves, replays and tables.

## Install (Windows, WinPython 3.12)

Everything installs from wheels; no Visual C++ build tools are needed.

1. Unzip the folder (or take `stackable-checkers` from the repository) somewhere without
   spaces in the path is safest, for example `C:\work\stackable-checkers`.
2. Open the **WinPython Command Prompt** (in the WinPython folder) and go to the folder:
   ```
   cd C:\work\stackable-checkers
   ```
3. Install the package and its dependencies (numpy, numba, mujoco, matplotlib, pandas):
   ```
   pip install -e .
   ```
   `-e` keeps the package pointing at this folder, so edits take effect without reinstalling.
4. Check it:
   ```
   python -c "import stackpieces, numba, mujoco; print(stackpieces.__version__, numba.__version__, mujoco.__version__)"
   ```

On Linux or macOS the same commands work in a virtual environment (`python -m venv .venv`, then
activate it). Node.js is optional: it is only used by the tests that check the browser engine.

## Check that it works (about 3 minutes)

```
python tests/run_tests.py
```
All tests should pass (`test_parity` is skipped without Node.js). The first run is slower because
Numba compiles the simulation and caches it.

## Revalidate the published results (about 20 minutes)

```
python -m stackpieces run constraints/checkers_rot3u.json --out results/my_run
python -m stackpieces compare results/my_run/results.json results/reference/results.json
```
`compare` lists every 50 % point and reliable offset (tolerance 1 mm) and every 3D outcome map
(tolerance one cell). On the same platform the 2D results repeat exactly (same seeds). On another
CPU or operating system a few releases near the edge of capture can come out differently, because
the dynamics there are chaotic; differences within the tolerances are expected.
`results/my_run/report.html` is the new report.

For a quick smoke test (about 2 minutes, coarse): add `--quick`.

## Notebooks

```
jupyter lab
```
Open `notebooks/01_constraints_and_geometry.ipynb` and work through 01 to 05. Each has a
`QUICK = True` line in its first cell (minutes); set it to `False` to rerun at full size. The
notebooks are generated from `notebooks/_build_notebooks.py`; edit that file and run it to change them.

| Notebook | Covers |
| --- | --- |
| 01 | Constraints, the three families, volume check, print and robot checks, printed layers |
| 02 | Design-space search from the constraints, printed-layer confirmation of the finalists |
| 03 | 3D settling in MuJoCo, outcome maps, comparison with the reference |
| 04 | Stacking-rate curves (smooth, printed, cone sanded), overall stack rate for the arm, one release frame by frame |
| 05 | The whole pipeline, comparison with the reference, STL and SolidWorks spec |

## Browser tool

Open `web/index.html` in Chrome, Edge or Firefox (double-click works). It needs no install; the 3D
view loads three.js from the internet and is the only part that needs a connection. It runs the same
2D engine as the Python package (checked by `tests/test_parity.py`), more slowly: keep releases per
offset around 10 there and use the notebooks for full runs.

## Your own constraints

Copy `constraints/template.json`, change the values and their notes, then:
```
python -m stackpieces run constraints/my_pieces.json --out results/my_pieces --chosen S40
```
`--chosen` is a preset (S40, N35, CYL) or a candidate name from the search table (for example
`S35-b12.5-s3-t4`).

## Exports

`run` writes, for the chosen design and the search's best: `<name>.stl` (watertight, mm, socket on
the bed when printed) and `<name>_partspec.json`, a parametric spec for Tico's SolidWorks build scripts (`revolve` +
`revolve_cut`, every size a SolidWorks Global Variable). For S40 the spec matches
`cad/checker_s40/checker_s40_partspec.json`, which the SolidWorks part in `cad/checker_s40/` was
built from.

## Hand it to someone

```
python make_zip.py
```
writes `stackable-pieces-<version>.zip` with everything except local run outputs. The recipient
follows **Install**, **Check that it works** and **Revalidate** above.

## Where this folder comes from

A copy of `studies/stackable-pieces` in `ticohannan/rot3u-robots` at commit 078b568, with the S40
SolidWorks files added in `cad/checker_s40/`. The folder imports nothing from the rest of the
repository and uses relative paths only.

## License

Author: Tico Hannan. BSD-3-Clause, see `LICENSE`.
