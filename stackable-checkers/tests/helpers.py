"""Shared test helpers."""
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONSTRAINTS = ROOT / "constraints" / "checkers_rot3u.json"
REFERENCE = ROOT / "results" / "reference" / "results.json"
NODE = shutil.which("node")


def constraints():
    return json.loads(CONSTRAINTS.read_text(encoding="utf-8"))


def node_eval(js: str):
    """Run JavaScript with web/geometry.js as G and web/sim2d.js as S; the script prints JSON."""
    pre = (f"const G=require({json.dumps(str(ROOT / 'web' / 'geometry.js'))});"
           f"const S=require({json.dumps(str(ROOT / 'web' / 'sim2d.js'))});")
    out = subprocess.run([NODE, "-e", pre + js], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)
