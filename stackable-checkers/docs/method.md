# Method, validation and limits

Updated 2026-11-15.

## 1. Where this came from

The design was worked out on 2026-11-13: first 3D simulations in PyBullet (smooth surfaces, four
release scenarios), then a 2D section model with printed layers in JavaScript for the stacking-rate
curve, published as the page "Stackable Checkers — Geometry and Settling Simulations". This folder
redoes all of it as code that runs from the constraints file. Two engines changed on the way:

* **3D: PyBullet → MuJoCo.** PyBullet has no Windows wheel for Python 3.12 and would need the Visual
  C++ build tools; MuJoCo installs from a wheel everywhere.
* **2D: one engine, two languages.** The JavaScript engine (`web/sim2d.js`) is the reference; the
  Python package runs a Numba port of it (`stackpieces/sim2d.py`), about ten times faster.
  `tests/test_parity.py` runs both on fixed cases: outcomes agree, and final poses agree to about
  1e-13 mm on the same machine.

## 2. Models

**Geometry.** Bodies of revolution described by a closed half-section polygon (`profile`). Mass,
center of mass and inertia come from midpoint integration on a 0.05 mm grid (the browser uses the
same grid); the exact revolved volume (Pappus) checks it to better than 0.1 %. S40's exact volume
equals the volume of the SolidWorks part built from the same parameters (`cad/checker_s40`, 5,659.463 mm³).

**Printed layers.** The slicer cuts at mid-layer; each material interval at that height becomes a
convex stadium 0.2 mm tall whose rounded ends (radius 0.1 mm) reach the CAD radius at mid-layer. The
cone and the socket become staircases of rounded ridges. The ideal staircase roughness,
Ra = layer/4 · cos(inclination), is about 32 µm on S40's cone (50° from the bed), within the 9–40 µm
measured on FDM parts. This is a geometric model of the ridges; the material friction μ acts on top.

**2D section simulation.** The two pieces move in the vertical plane through both axes. For two
coaxial bodies of revolution offset along x, the first contact lies in that plane, so the section
captures how the released piece slides and tips. Rigid bodies with the pieces' 3D mass and
inertia, separating-axis polygon contacts with reference-face clipping, 16 sequential-impulse
iterations, Coulomb friction, Baumgarte correction, restitution 0.15 above 60 mm/s, 0.2 ms steps.
A run stops when both pieces are still for 80 ms, or at 1.2 s.

**3D simulation.** MuJoCo 3, each piece a free body of convex parts (32 angular sectors of the socket
wall or cone shell, plus frusta for the roof and the cone), elliptic friction cones, stiff contacts
(solref time constant 0.8 ms, seated penetration under 0.001 mm), 0.2 ms steps, `multiccd` on.

**Outcomes** (both models): *seated* = centers within 1 mm, tilt < 3°, height within 0.6 mm of the
stack pitch; *perched* = still on top otherwise; *fell* = off the lower piece; *knocked* = the lower
piece tipped more than 10°.

**Monte Carlo.** For each offset, N releases with friction uniform in the constraints' range,
release height uniform 0.5–2 mm above the cone tip, tilt and sideways speed Gaussian (2°, 10 mm/s
1σ). A 32-bit LCG with Box-Muller, identical in Python and JavaScript, seeded per offset index.

**Placement.** The arm's error on each horizontal axis is uniform in ±resolution/2 (or Gaussian);
the overall stack rate is the curve averaged over the radial error.

## 3. Revalidation, 2026-11-15

### 3D: MuJoCo against the earlier PyBullet run

Same geometry, scenarios, friction values and classification. Each row gives the largest offset that
seated at every offset up to it (gentle place = released level and still, 1 mm above the tip).

| Design, scenario | PyBullet (2026-11-13) | MuJoCo | 2D section model |
| --- | --- | --- | --- |
| S40 gentle place, μ 0.25 / 0.4 / 0.55 | 5 / 5 / 4 mm | 6 / 6 / 6 mm | 6 / 4 / 3 mm |
| S40 drop 10 mm | 5 / 5 / 4 | 7 / 7 / 7 | 6 / 5 / 3 |
| S40 tilted 10° | 3 / 3 / 3 | 3 / 2 / 2 | 5 / 4 / 3 |
| S40 still moving 50 mm/s | 1 / 1 / 1 | 4 / 4 / 2 | 3 / 2 / 1 |
| N35 gentle place | 7 / 7 / 7 | 9 / 7 / 6 | 8 / 6 / 5 |
| Cylinder, gentle place (stacks flat, never centers) | seated ≤ 1, off-center stack to 10–11 | same | — |

**What differs noticeably.** MuJoCo seats S40 about 1 mm further off center on a gentle place and
2–3 mm further after a 10 mm drop, where PyBullet showed no benefit from dropping. The largest
difference is the moving release: PyBullet said a gripper still moving at 50 mm/s leaves only 1 mm
of capture, MuJoCo 2–4 mm, the 2D model 1–3 mm. The earlier advice ("stop before opening") still
holds, but the penalty is smaller and less certain than first reported. Read all 3D capture figures
as ±1–2 mm: the models differ in how they treat contact stiffness, restitution and friction.

### 2D printed-layer curve: Python against the earlier JavaScript run

Same model and the same release draws (the earlier run's tilt sign is mirrored; the draws are
symmetric). 30 releases per offset.

| S40 | Earlier (JavaScript) | Now (Python, reference) |
| --- | --- | --- |
| Printed: 50 % seat point | 3.7 mm | 3.4 mm |
| Printed: seats ≥ 80 % up to | 3.0 mm | 2.5 mm |
| Cone sanded: 50 % point | 4.6 mm | 4.5 mm |
| Smooth: 50 % point | 5.6 mm | 5.5 mm |

Within the Monte Carlo spread (±18 points at 50 % with 30 releases, 95 %), so not a real change.

## 4. Search result (new)

The search enumerates 185 candidates from the constraints; 23 pass every hard check. It ranks them by
smooth capture, then reruns the top six with printed layers (20 releases per offset):

| Candidate | Height | Smooth capture | Printed 50 % point |
| --- | --- | --- | --- |
| S35-b12.5-s3-t4 (35°, 12.5 band, 3 mm shoulder, Ø4 tip) | 23.2 mm | 7.5 mm | 5.1 mm |
| S40-b12.5-s3-t4 | 21.4 mm | 6.0 mm | 4.9 mm |
| S40-b15-s3-t4 | 23.9 mm | 6.0 mm | 3.9 mm |
| S40-b12-s3-t1.6 | 22.4 mm | 7.0 mm | 3.9 mm |
| S40-b12.5-s3-t1.6 | 22.9 mm | 7.0 mm | 3.9 mm |
| S45-b15-s3-t1.6 | 23.7 mm | 6.0 mm | 2.8 mm |
| S40 as chosen (2 mm shoulder) | 22.1 mm | 5.5 mm | 3.4 mm (30 releases) |

Two findings: a **3 mm shoulder** helps every cone angle tried (probably because the socket mouth is narrower, so the upper
piece's rim stays on the lower piece's flat top longer before the socket can straddle its outer edge),
and the smooth ranking is only a screen (S45-b15 ranks fifth smooth and last printed). N35 fails the
printer's minimum wall: its bed ring is 0.6 mm after the 0.4 mm chamfer; a 1.2 mm land fixes that.
None of these were built; only S40 with the 2 mm shoulder exists in SolidWorks (`cad/checker_s40`).

## 5. Limits

* The gripper is not modeled: the fingers may drag or tilt the piece as they open.
* Printed details not modeled: droop of the socket's overhang, the seam, elephant's foot,
  print-to-print size scatter, rounding of ridges by wear.
* The 2D model sees only the plane of the offset; out-of-plane rocking is ignored.
* Friction is one coefficient per release; real printed PLA is anisotropic and has stick-slip.
* No printed piece has been tested yet. The first bench test (README of the checkers project and the
  report) is what decides which of these matter.

## 6. Sources

* Staircase roughness model, Ra = ¼ · layer thickness · cos(α): euspen knowledge base AM21119,
  https://www.euspen.eu/knowledge-base/AM21119.pdf
* FDM surface roughness 9–40 µm (Campbell et al., ABS, 0.253 mm layers), as cited in T. Elattar,
  *Surface roughness of FDM parts (horizontal, vertical, inclined)*,
  https://beng.stafpu.bu.edu.eg/Mechanical%20Engineering/7045/publications/Tarek%20Ahmed%20Mohamed%20Salah%20Eldin%20Elattar_paper.pdf
* Static friction between printed PLA parts 0.12–0.38, higher when filaments of equal angle fit into
  each other: *Effect of the angle of the laid-up filaments on the surfaces on the coefficient of static
  friction between parts manufactured by extrusion-based 3D printing*, listed at
  https://ouci.dntb.gov.ua/works/4Mp1rdv7
* MuJoCo: https://mujoco.org; Numba: https://numba.pydata.org
