# Reproduce the stackable-checkers results

Step by step, for a Windows laptop with WinPython 3.12 (any Python 3.10 or newer works). Stop after
whichever part gives you what you need. Typed commands are in boxes; type them exactly.

## Part A: look at the results, no install (2 minutes)

1. Get the files, either way:
   - **ZIP:** open https://github.com/ticohannan/rot3u-finalproject, click the green **Code** button,
     then **Download ZIP**. Right-click the downloaded `rot3u-finalproject-main.zip`, choose
     **Extract All…**, and extract to `C:\work`. The folder you need is then
     `C:\work\rot3u-finalproject-main\stackable-checkers`.
   - **git:** in a command prompt,
     ```
     cd C:\work
     git clone https://github.com/ticohannan/rot3u-finalproject
     ```
     The folder is then `C:\work\rot3u-finalproject\stackable-checkers`.
   The steps below use the ZIP path; with git, leave out `-main`.
2. Double-click `stackable-checkers\results\reference\report.html`. It opens in your browser: the
   stacking rate against offset, the robot's overall stack rate, replays of single releases (section
   view with the printed layers, and 3D), the outcome maps, the design search, geometry and checks.
   The 3D view needs an internet connection; the rest works offline.
3. Optional: double-click `stackable-checkers\web\index.html` for the interactive tool. Click **S40**,
   set **releases per offset** to `5` and **offset step** to `1`, then click **Run** (about a minute).
4. The SolidWorks part and print files are in `stackable-checkers\cad\checker_s40` (see the README
   there).

## Part B: install and run it yourself, quick mode (about 10 minutes)

5. Open the WinPython folder and double-click **WinPython Command Prompt.exe**. (A plain `cmd`
   window does not find WinPython's Python.)
6. Go to the folder:
   ```
   cd C:\work\rot3u-finalproject-main\stackable-checkers
   ```
7. Install the package and what it needs (numpy, numba, mujoco, matplotlib, pandas). Everything
   comes as ready-built wheels; no compiler is needed:
   ```
   pip install -e .
   ```
   That is a lowercase `e`, a space and a period. It takes a minute or two. The last line should start with
   `Successfully installed` and include `stackpieces-0.1.0`.
8. Check the install:
   ```
   python -c "import stackpieces, numba, mujoco; print(stackpieces.__version__, numba.__version__, mujoco.__version__)"
   ```
   You should see `0.1.0` and two version numbers.
9. Run the tests (1–3 minutes; the first run is slower while Numba compiles the simulation):
   ```
   python tests\run_tests.py
   ```
   The last line should read `33 passed, 0 failed, 0 skipped`, or `29 passed, 0 failed, 4 skipped`
   if Node.js is not installed (those four check the browser engine).
10. Run the whole study in quick mode (2–4 minutes):
    ```
    python -m stackpieces run constraints\checkers_rot3u.json --out results\quick --quick
    ```
11. Open the report it wrote:
    ```
    start results\quick\report.html
    ```
    Quick mode uses few releases, so its curves are coarse and differ from the reference.

## Part C: reproduce the reference numbers (about 20 minutes)

12. In the same window:
    ```
    python -m stackpieces run constraints\checkers_rot3u.json --out results\full
    python -m stackpieces compare results\full\results.json results\reference\results.json
    ```
13. The last line of `compare` should read `0 of 58 items differ by more than the tolerance`
    (1 mm for the 50 % points and reliable offsets, one cell for the 3D outcome maps). A few `CHECK`
    lines can still appear on another computer: near the edge of capture a release is chaotic, so a
    different CPU or operating system can turn one release or two. Send any `CHECK` lines to Tico.
14. Open the new report:
    ```
    start results\full\report.html
    ```

## Part D: notebooks (optional)

15. In the same window:
    ```
    jupyter lab
    ```
16. In the browser tab that opens, go to `notebooks` and open `04_printed_surfaces.ipynb`, then
    **Run → Run All Cells**. It draws the S40 stacking-rate curve next to the reference curve.
    Notebooks 01–05 walk through the whole study; each starts with `QUICK = True` (minutes), set it to
    `False` for the full size.

## If something fails

| Symptom | Fix |
| --- | --- |
| `'pip' is not recognized` or `'python' is not recognized` | You are in a plain `cmd` window: use **WinPython Command Prompt.exe** (step 5) |
| `ERROR: You must give at least one requirement to install` | The command was typed with `-i`; it is `pip install -e .` |
| SSL or connection errors during `pip install` | The network blocks PyPI (common on work or campus networks): try another network |
| `ModuleNotFoundError: No module named 'stackpieces'` | Step 7 was run in another folder: `cd` into `stackable-checkers` (step 6) and repeat step 7 |
| A test fails | Send the whole output of step 9 to Tico |

On Linux or macOS use `/` instead of `\`, `open` (macOS) or `xdg-open` (Linux) instead of `start`, and
a virtual environment: `python3 -m venv .venv`, then `source .venv/bin/activate`, then step 7.
