"""Exports (DD-STACK SP-12): STL of a piece and a parametric SolidWorks spec (partspec JSON).

Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import json
import math
import struct

import numpy as np


def revolve_mesh(profile, segments: int = 128):
    """Triangle mesh (vertices, faces) of a half-section polygon revolved 360 deg about z.
    Profile vertices on the axis (r == 0) become single vertices, so the mesh is closed."""
    V, F, idx = [], [], []
    for r, z in profile:
        idx.append(len(V))
        if r < 1e-9:
            V.append((0.0, 0.0, z))
        else:
            V += [(r * math.cos(2 * math.pi * j / segments), r * math.sin(2 * math.pi * j / segments), z)
                  for j in range(segments)]
    n = len(profile)
    for i in range(n):
        a, b = i, (i + 1) % n
        ra, rb = profile[a][0], profile[b][0]
        if ra < 1e-9 and rb < 1e-9:
            continue
        for j in range(segments):
            j2 = (j + 1) % segments
            if ra < 1e-9:
                F.append((idx[a], idx[b] + j2, idx[b] + j))
            elif rb < 1e-9:
                F.append((idx[a] + j, idx[a] + j2, idx[b]))
            else:
                F.append((idx[a] + j, idx[a] + j2, idx[b] + j2))
                F.append((idx[a] + j, idx[b] + j2, idx[b] + j))
    V = np.array(V)
    F = np.array(F)
    if mesh_volume(V, F) < 0:      # make the normals point outward
        F = F[:, ::-1]
    return V, F


def mesh_volume(V, F) -> float:
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def is_watertight(F) -> bool:
    """Every edge used exactly twice, once in each direction."""
    edges = {}
    for f in F:
        for k in range(3):
            e = (int(f[k]), int(f[(k + 1) % 3]))
            edges[e] = edges.get(e, 0) + 1
    return all(cnt == 1 and edges.get((b, a), 0) == 1 for (a, b), cnt in edges.items())


def write_stl(path, profile, segments: int = 128, name: str = "piece") -> dict:
    """Binary STL in mm. Returns volume and watertightness."""
    V, F = revolve_mesh(profile, segments)
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(F)))
        for t in F:
            a, b, c = V[t[0]], V[t[1]], V[t[2]]
            nrm = np.cross(b - a, c - a)
            ln = np.linalg.norm(nrm)
            nrm = nrm / ln if ln > 0 else nrm
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))
    return dict(path=str(path), triangles=len(F), volume_mm3=mesh_volume(V, F), watertight=is_watertight(F))


def partspec(g: dict, name: str) -> dict:
    """Parametric SolidWorks spec (partspec JSON): one revolve (body) and one revolve_cut (socket), Global Variables.
    Angles become cone heights (the kit's expressions have no tan): CONE_H sets the cone angle."""
    p = g["params"]
    fam = g["family"]
    ta = math.tan(math.radians(p.get("alpha", 45)))
    if fam == "shoulder":
        params = {
            "D": p["D"], "HC": p["band_h"], "SH": p["shoulder"], "TIP_D": p["tip_d"],
            "CONE_H": round((p["D"] / 2 - p["shoulder"] - p["tip_d"] / 2) / ta, 3),
            "CLR": p["clr"], "VCLR": p["vclr"], "CT": p["ct"], "CB": p["cb"],
            "SLOPE": "(D/2 - SH - TIP_D/2) / CONE_H", "MOUTH_D": "D - 2*SH + 2*CLR",
            "SOCK_H": "CONE_H + VCLR", "ROOF_D": "MOUTH_D - 2*SOCK_H*SLOPE"}
        body = [{"z": 0, "d": 0}, {"z": 0, "d": "D - 2*CB"}, {"z": "CB", "d": "D"}, {"z": "HC - CT", "d": "D"},
                {"z": "HC", "d": "D - 2*CT"}, {"z": "HC", "d": "D - 2*SH"}, {"z": "HC + CONE_H", "d": "TIP_D"},
                {"z": "HC + CONE_H", "d": 0}]
        sock = [{"z": -0.5, "d": 0}, {"z": -0.5, "d": "MOUTH_D + 2*0.5*SLOPE"}, {"z": "SOCK_H", "d": "ROOF_D"},
                {"z": "SOCK_H", "d": 0}]
        comment = "shoulder family: grip band, flat shoulder, cone, matching socket underneath"
    elif fam == "nesting":
        params = {
            "D": p["D"], "HB": p["band_h"], "TIP_D": p["tip_d"],
            "CONE_H": round((p["D"] / 2 - p["tip_d"] / 2) / ta, 3),
            "LAND": p["land"], "VCLR": p["vclr"], "CTIP": p["ctip"], "CB": p["cb"],
            "MOUTH_D": "D - 2*LAND", "SEAT": "HB + LAND*CONE_H/(D/2 - TIP_D/2)",
            "ROOF_Z": "HB + CONE_H - SEAT + VCLR", "ROOF_D": "TIP_D + 2*CTIP",
            "SLOPE_S": "(MOUTH_D/2 - ROOF_D/2) / ROOF_Z"}
        body = [{"z": 0, "d": 0}, {"z": 0, "d": "D - 2*CB"}, {"z": "CB", "d": "D"}, {"z": "HB", "d": "D"},
                {"z": "HB + CONE_H", "d": "TIP_D"}, {"z": "HB + CONE_H", "d": 0}]
        sock = [{"z": -0.5, "d": 0}, {"z": -0.5, "d": "MOUTH_D + 2*0.5*SLOPE_S"}, {"z": "ROOF_Z", "d": "ROOF_D"},
                {"z": "ROOF_Z", "d": 0}]
        comment = "nesting family: grip band, cone from the full diameter, socket nesting up into the cone"
    else:
        params = {"D": p["D"], "H": p["band_h"], "CT": p["ct"], "CB": p["cb"]}
        body = [{"z": 0, "d": 0}, {"z": 0, "d": "D - 2*CB"}, {"z": "CB", "d": "D"}, {"z": "H - CT", "d": "D"},
                {"z": "H", "d": "D - 2*CT"}, {"z": "H", "d": 0}]
        sock = None
        comment = "cylinder (control)"
    feats = [{"id": "body", "op": "revolve", "axis": "z", "angle": 360, "profile": {"turned": body}}]
    if sock:
        feats.append({"id": "socket", "op": "revolve_cut", "axis": "z", "angle": 360, "profile": {"turned": sock}})
    return {"name": name, "units": "mm",
            "_comment": f"Stackable piece ({comment}). Spec Z = piece axis, z = 0 on the bottom face. "
                        f"Generated by stackpieces.export.partspec.",
            "params": params, "features": feats}


def write_partspec(path, g: dict, name: str) -> dict:
    spec = partspec(g, name)
    with open(path, "w") as f:
        json.dump(spec, f, indent=1)
    return spec
