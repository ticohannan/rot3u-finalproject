"""Robot Demo 4 (camera tracking) for Sir Pinch-a-lot: Python side of servo_bridge.ino.

    link     serial link to the sketch (targets in microseconds, state stream)
    robot    Sir Pinch-a-lot numbers: pulse <-> angle, limits, named poses, power-up
    vision   camera, color ranges, the two detectors (blob as in class, color-gated edges), display
    control  PID, tracker (1-axis / 2-axis), and the measurement routines (noise, image Jacobian,
             slop staircase, step response, ringdown and input shaping)
    sim      simulator: the sketch's behavior, servos with dead band and backlash, a springy frame, a fake camera
    camdiag  camera diagnostics: list cameras, probe sizes and formats, watch brightness and frame rate
    profiles object profiles: object + room + camera settings + color statistics from color-picker captures
    geometry DH forward kinematics, the camera mount on the gripper, the gripper-axis overlay
    robot_view  pictures of the arm in a pose (side, top, 3D) with the servos at their CAD positions

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
__version__ = "0.3.0"
