# Robot Demo 4, Python side: `rot3u_demo4` v0.3.0

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.

Camera tracking for a ROT3U arm from a Jupyter notebook. The Arduino runs `firmware/servo_bridge`, which
only moves servos to pulse targets; everything else is here. Set-up from scratch:
`../docs/GETTING-STARTED.md`.

| File | What it is |
| --- | --- |
| `demo4_tracking.ipynb` | The notebook. Every section says what it does and whether to run it |
| `rot3u_demo4/robot.py` | The arm's numbers, read from its `robots/<name>/robot_config.h`; poses; power-up |
| `rot3u_demo4/link.py` | Serial link to the sketch (default port COM3), session log |
| `rot3u_demo4/vision.py` | Camera (newest frame only, watchdog), HSV color model, blob and edge detectors, display, recording |
| `rot3u_demo4/camdiag.py` | Camera diagnostics: list, probe sizes and formats, watch, driver settings |
| `rot3u_demo4/profiles.py` | Object profiles: object + room + camera settings + color statistics from color-picker captures |
| `rot3u_demo4/control.py` | PID tracker (1 or 2 axes) and the measurements: noise, image Jacobian, slop, step response, ringing |
| `rot3u_demo4/geometry.py` | DH forward kinematics, the camera mount on the gripper, the gripper-axis overlay |
| `rot3u_demo4/robot_view.py` | Pictures of the arm in a pose (side, top, 3D) |
| `rot3u_demo4/sim.py` | Simulator: the sketch's behavior, servos with dead band and backlash, a springy frame, a camera |
| `profiles/`, `captures/` | Sir Pinch-a-lot's green block profile and the captures it was built from (examples) |
| `tests/` | `python -m pytest tests -q` |

Install (WinPython command prompt):
```
pip install pyserial opencv-python numpy ipywidgets matplotlib
pip install git+https://github.com/ticohannan/camera-color-picker
```
