"""Self-contained HTML report from a results bundle (DD-STACK SP-12).

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import json
from importlib import resources

from .surface import ra_staircase


def method_html(c: dict) -> str:
    t = c["printer"]["layer_mm"]
    return (
        "<p><b>Geometry.</b> Bodies of revolution from the family parameters; mass properties by integrating the "
        "half-section (density cancels out of the settling). Checks against the printer and gripper constraints.</p>"
        "<p><b>2D section model.</b> Rigid bodies in the vertical plane through both axes (contact between two coaxial "
        "rings lies in that plane), the pieces' 3D mass and inertia, separating-axis polygon contacts, sequential "
        "impulses with Coulomb friction, 5 kHz, 16 iterations. The same engine runs in Python (Numba) and in the "
        "browser (JavaScript); a test checks they agree.</p>"
        f"<p><b>Printed surfaces.</b> Each {t} mm layer is a convex stadium whose rounded ends reach the CAD radius at "
        f"mid-layer, so sloped faces are ridged staircases (Ra ≈ layer/4 · cos(inclination): about "
        f"{1000 * ra_staircase(t, 50):.0f} µm on a 50° face, inside the 9–40 µm reported for FDM parts). Ridges on the cone "
        "and in the socket can hook each other, which is what cuts the printed curve below the smooth one. "
        "<i>Cone sanded</i>: printed socket, smooth cone.</p>"
        "<p><b>3D model.</b> MuJoCo, pieces as convex parts (32 angular sectors of the socket wall or cone shell, plus "
        "frusta), stiff soft contacts (time constant 0.8 ms), elliptic friction cones, 0.2 ms steps. Outcomes as in 2D. "
        "On 2026-11-15 these maps were compared with an earlier PyBullet run (docs/method.md): gentle place agrees "
        "within 1–2 mm of offset; a release with the arm still moving differs most (MuJoCo seats 2–3 mm further out).</p>"
        "<p><b>Outcomes.</b> <i>Seated</i>: centers within 1 mm, tilt under 3°, height within 0.6 mm of the stack pitch. "
        "<i>Perched</i>: still on top but not seated (tilted on the cone, or flat and off-center for the cylinder). "
        "<i>Fell</i>: fell off. <i>Knocked</i>: the lower piece tipped more than 10°.</p>"
        "<p><b>Not modeled.</b> The gripper fingers dragging the piece as they open, overhang droop inside the socket, "
        "the seam, elephant's foot, print-to-print size scatter, wear. Treat offsets as ±1 mm between models.</p>"
        "<p><b>Sources.</b> Staircase roughness model Ra = ¼·layer·cos(α): euspen AM21119 (trigonometric staircase "
        "model). FDM Ra 9–40 µm: Campbell et al., as cited in Elattar, <i>Surface roughness of FDM parts (RSM)</i>. "
        "Static friction between printed PLA parts 0.12–0.38, higher when filaments of equal angle fit into each other: "
        "<i>Effect of the angle of the laid-up filaments on the surfaces on the coefficient of static friction between "
        "parts manufactured by extrusion-based 3D printing</i>.</p>")


def summary_text(res: dict) -> str:
    ch = res["chosen"]
    m = res["metrics"][ch]
    best = res.get("best")
    goal = res["constraints"]["goal"]
    D = res["constraints"]["piece"]["diameter_mm"]
    tgt = goal["target_offset_fraction"] * (D / 2 if goal["target_offset_of"] == "radius" else D)
    s = (f"As printed, it seats on every release up to about {m['layered']['reliable']:.1f} mm off center "
         f"(at least {goal['seat_rate_required']:.0%} at each offset) and half the time at "
         f"{(m['layered']['e50'] or 0):.1f} mm. The goal offset of {tgt:.1f} mm "
         f"({goal['target_offset_fraction']:.0%} of the {goal['target_offset_of']}) seats "
         f"{m['layered']['rate_at_target']:.0%} of the time. ")
    if "hybrid" in m:
        s += f"Sanding the cone moves the 50 % point to {(m['hybrid']['e50'] or 0):.1f} mm. "
    if best and best != ch:
        mb = res["metrics"][best]
        s += (f"The search from the constraints ranks <b>{best}</b> best: printed 50 % at "
              f"{(mb['layered']['e50'] or 0):.1f} mm, height {res['designs'][best]['H']:.1f} mm.")
    return s


def write_report(res: dict, path) -> str:
    """Write the HTML report for a results bundle (see pipeline.run_all) and return its path."""
    tpl = resources.files("stackpieces").joinpath("report_template.html").read_text(encoding="utf-8")
    data = dict(res)
    data["summary_text"] = summary_text(res)
    data["method_html"] = method_html(res["constraints"])
    html = tpl.replace("__TITLE__", res["title"]).replace("__DATA__", json.dumps(data, separators=(",", ":")))
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    return str(path)
