"""Command line: python -m stackpieces run|compare|stl|spec ... (see README.md)."""
import argparse
import json
import sys

from .pipeline import compare, load_constraints, run_all


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m stackpieces")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run the whole study from a constraints file")
    r.add_argument("constraints")
    r.add_argument("--out", required=True)
    r.add_argument("--chosen", default="S40", help="preset name (S40, N35, CYL) or a search candidate name")
    r.add_argument("--quick", action="store_true", help="small grids and few releases (about 2 minutes)")
    cmp_ = sub.add_parser("compare", help="compare two results.json files")
    cmp_.add_argument("new")
    cmp_.add_argument("reference")
    cmp_.add_argument("--tol", type=float, default=1.0)
    a = ap.parse_args(argv)
    if a.cmd == "run":
        run_all(a.constraints, a.out, chosen=a.chosen, quick=a.quick)
    else:
        with open(a.new) as f:
            new = json.load(f)
        with open(a.reference) as f:
            ref = json.load(f)
        rows = compare(new, ref, a.tol)
        bad = 0
        for what, x, y, d, flag in rows:
            bad += flag
            print(f"{'CHECK' if flag else 'ok   '}  {what:45s} reference {x}  new {y}  diff {d}")
        print(f"{bad} of {len(rows)} items differ by more than the tolerance")
        return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
