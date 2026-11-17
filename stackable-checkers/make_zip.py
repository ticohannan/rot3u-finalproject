"""Make a zip of this folder to hand to someone: python make_zip.py  ->  stackable-pieces-<version>.zip

Leaves out local run outputs (results/ except results/reference), caches and earlier zips.
Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
import os
import re
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
version = re.search(r'__version__ = "([^"]+)"', open(os.path.join(HERE, "stackpieces", "__init__.py")).read()).group(1)
name = f"stackable-pieces-{version}"
out = os.path.join(HERE, name + ".zip")
SKIP_DIRS = {"__pycache__", ".ipynb_checkpoints", ".pytest_cache", "build", "dist"}
n = 0
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for root, dirs, files in os.walk(HERE):
        rel = os.path.relpath(root, HERE)
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS and not d.endswith(".egg-info")
                   and not (rel == "results" and d != "reference")]
        for f in files:
            if f.endswith((".zip", ".pyc")):
                continue
            p = os.path.join(root, f)
            z.write(p, os.path.join(name, os.path.relpath(p, HERE)))
            n += 1
print(f"wrote {out} ({n} files)")
