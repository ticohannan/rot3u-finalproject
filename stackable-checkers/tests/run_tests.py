"""Run every test in this folder without pytest (pytest also works: `pytest tests`).

    python tests/run_tests.py            all tests
    python tests/run_tests.py parity     only files whose name contains 'parity'

A test module may set SKIP = "reason" (for example when Node or MuJoCo is missing).
Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
import importlib
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))


def main(filt=""):
    passed = failed = skipped = 0
    for f in sorted(HERE.glob("test_*.py")):
        if filt not in f.stem:
            continue
        mod = importlib.import_module(f.stem)
        skip = getattr(mod, "SKIP", None)
        for name in [n for n in dir(mod) if n.startswith("test_")]:
            label = f"{f.stem}::{name}"
            if skip:
                print(f"SKIP {label} ({skip})")
                skipped += 1
                continue
            t0 = time.time()
            try:
                getattr(mod, name)()
                print(f"PASS {label}  ({time.time() - t0:.1f} s)")
                passed += 1
            except Exception:
                print(f"FAIL {label}")
                traceback.print_exc()
                failed += 1
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else ""))
