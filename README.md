# rot3u-finalproject

Tools and tests on the way to the ECE 6311 final project, for ROT3U-type 6-servo arms driven by an
Arduino Mega 2560 and a PCA9685 servo board, with a USB camera on the gripper.

Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see `LICENSE`).

**New here? Start with [`docs/GETTING-STARTED.md`](docs/GETTING-STARTED.md):** it goes from an empty laptop
to an arm that tracks a colored block with its camera, step by step.

## What is here

| Folder | What | Status |
| --- | --- | --- |
| `python/demo4_tracking.ipynb`, `python/rot3u_demo4/` | Robot Demo 4: keep a colored block centered in the gripper camera, with the camera as the only sensor. Color picker set-up, object profiles, camera diagnostics, start-pose pictures, image-Jacobian and slop measurements, input shaping against the arm's wobble, PID tracking with the gripper's axis drawn on the picture | ran on Sir Pinch-a-lot 2026-11-15 (demo recorded); v0.3 (any arm, from its `robot_config.h`) tested on the PC simulator |
| `firmware/servo_bridge/` | Minimal sketch the notebook drives: smooth, limited moves to pulse targets; input shaping | v0.1 ran on the arm; v0.3 tested on a PC simulation |
| `firmware/servo_tester/` | Joint mapping: measure each joint's pulse-to-angle line and safe range, export a `robot_config.h` block | v3.0, from `ticohannan/rot3u-robots`; mapping commands not yet run on hardware |
| `robots/` | One `robot_config.h` per arm: `sir_pinch_a_lot_revB/` (Tico's arm A), `template/` for a new arm | |
| `docs/` | `GETTING-STARTED.md`, `joint-mapping.md` (full mapping reference), `rot3u-conventions.md` (DH model and angle conventions) | |

Related repositories: [`ticohannan/rot3u-robots`](https://github.com/ticohannan/rot3u-robots) (firmware,
`ik_demo`, the host link and the design documents these tools come from) and
[`ticohannan/camera-color-picker`](https://github.com/ticohannan/camera-color-picker) (the color picker used in part A
of the notebook).

## Tests (PC only, no hardware)

```
cd python
python -m pytest tests -q
```
Some tests build the sketch for the PC and need `g++`; on Windows they are skipped without it.

Prepared with assistance from AI.
