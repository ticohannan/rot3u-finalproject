"""Where the camera is on the gripper, where the gripper's z axis falls in the picture, and forward
kinematics (DH, Rev B) for Sir Pinch-a-lot.

DH (Spong, robotica): A_i = Rot(z, theta_i) Trans(z, d_i) Trans(x, r_i) Rot(x, alpha_i)
    link  alpha   r     d     theta
    1     +90     0     d1    theta1*
    2     0       r2    0     theta2*
    3     0       r3    0     theta3*
    4     +90     0     0     theta4* + 90
    5     0       0     d5    theta5*           (o5 = grasp surface, jaws closed, pads fitted)
Sir Pinch-a-lot: d1 113, r2 104, r3 97, d5 170 mm. Lengths and the pulse <-> angle calibration come from
the loaded robot_config.h (robot.load_config).

Camera mount (gripper_camera_bracket rev 2.1, on the arm since 2026-11-15), in DH frame 5:
    optical center      (29.5, 0, -55) mm: 29.5 mm toward +x5 (above the gripper at roll 0),
                        55 mm back from the grasp point toward the wrist (the bracket design puts the
                        camera's mass at z5 = -65.5; the lens is about 10 mm in front of that)
    optical axis        parallel to +z5 (the approach direction)
    image up            +x5, image right +y5 (after the flip/rotate set in the color picker)
    focal length        f_px at ref_width pixels wide (scaled for other sizes)
These are user settings (CameraMount); measure and change them if the bracket or camera changes.

What follows for the picture, with dx = 29.5 mm (camera above the gripper axis) and the block Z mm
in front of the camera:
    - the gripper's z axis is parallel to the optical axis, so in the image it is a vertical line
      that starts at the image center (its vanishing point) and runs down;
    - at the block's depth the gripper axis is f * dx / Z pixels below the image center
      (about 80 px at Z = 270 mm, f = 720 px): that is where the gripper would hit if it moved
      straight ahead;
    - x5 points up the image and y5 to the right, at every pose (the camera turns with frame 5).

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field, asdict
import numpy as np
import cv2
from . import robot

ALPHA = [90.0, 0.0, 0.0, 90.0, 0.0]
THETA_OFFSET = [0.0, 0.0, 0.0, 90.0, 0.0]


def dh_table():
    """(r, d) per link from the loaded robot (robot.load_config)."""
    return [0.0, robot.R2, robot.R3, 0.0, 0.0], [robot.D1, 0.0, 0.0, 0.0, robot.D5]


def dh(theta_deg, d, r, alpha_deg):
    t, a = math.radians(theta_deg), math.radians(alpha_deg)
    ct, st, ca, sa = math.cos(t), math.sin(t), math.cos(a), math.sin(a)
    return np.array([[ct, -st * ca, st * sa, r * ct],
                     [st, ct * ca, -ct * sa, r * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def frames(q_deg):
    """[T0_0, T0_1, ..., T0_5] for joint variables theta1*..theta5* in degrees (the +90 on link 4 is added here)."""
    R, D = dh_table()
    T = [np.eye(4)]
    for i in range(5):
        T.append(T[-1] @ dh(q_deg[i] + THETA_OFFSET[i], D[i], R[i], ALPHA[i]))
    return T


def q_from_pulses(pulses):
    """Joint variables (deg) from pulses on channels 0..4 (calibration of robot.py)."""
    return [robot.us_to_deg(j, float(pulses[j])) for j in range(5)]


@dataclass
class CameraMount:
    """The camera on the gripper, in DH frame 5. Defaults: gripper_camera_bracket rev 2.1 and the HBVCAM."""
    pos_5: tuple = (29.5, 0.0, -55.0)       # optical center, mm, DH frame 5
    up_5: tuple = (1.0, 0.0, 0.0)           # image-up direction, frame 5
    right_5: tuple = (0.0, 1.0, 0.0)        # image-right direction, frame 5
    f_px: float = 720.0                     # focal length in pixels at ref_width (estimate; measure with the picker's lens tool)
    ref_width: int = 640
    block_mm: float = 40.0                  # side of the tracked block, for the depth estimate from its size

    def f_at(self, width):
        return self.f_px * float(width) / self.ref_width

    def R_5c(self):
        """Columns: camera x (right), y (down), z (forward) in frame 5."""
        r = np.asarray(self.right_5, float); u = np.asarray(self.up_5, float)
        r, u = r / np.linalg.norm(r), u / np.linalg.norm(u)
        fwd = np.cross(r, -u)
        return np.column_stack([r, -u, fwd])

    def T_5c(self):
        T = np.eye(4); T[:3, :3] = self.R_5c(); T[:3, 3] = self.pos_5
        return T

    def to_dict(self):
        return asdict(self)

    @staticmethod
    def from_dict(d):
        return CameraMount(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in d.items()})


def project_cam(p_c, f, cx, cy):
    """Camera-frame point (x right, y down, z forward, mm) to pixels; None if behind the camera."""
    if p_c[2] <= 1e-6:
        return None
    return (cx + f * p_c[0] / p_c[2], cy + f * p_c[1] / p_c[2])


def gripper_axis_point(mount: CameraMount, Z, width, height):
    """Pixel where the gripper z axis is at depth Z (mm in front of the camera, along the optical axis)."""
    f = mount.f_at(width); cx, cy = width / 2, height / 2
    Tc5 = np.linalg.inv(mount.T_5c())
    o = Tc5 @ np.array([0, 0, 0, 1.0]); a = Tc5[:3, :3] @ np.array([0, 0, 1.0])
    if abs(a[2]) < 1e-9:
        return None
    s = (Z - o[2]) / a[2]                    # point on the axis whose camera depth is Z
    return project_cam(o[:3] + s * a, f, cx, cy)


def depth_from_size(mount: CameraMount, size_px, width):
    """Block depth (mm) from its apparent side length in pixels (sqrt of the blob area works for a square face)."""
    return mount.f_at(width) * mount.block_mm / max(float(size_px), 1e-6)


def depth_from_jacobian(jac, pulses, mount: CameraMount, width, k_base=0):
    """Depth (mm) from the base joint's image gain and the camera's distance from the base axis.

    Turning the base by d theta moves a point straight ahead of the camera by f (Z + Rc) d theta / Z
    pixels, Rc = horizontal distance of the camera from the base axis. So Z = f Rc / (g - f), with
    g = pixels per radian. Rough (the base calibration and slop enter): use it as a check."""
    j = jac.joints[k_base]
    g = jac.gain(k_base) * abs(robot.USPD[j]) * 180.0 / math.pi     # px per radian
    T = frames(q_from_pulses(pulses))[5] @ mount.T_5c()
    Rc = math.hypot(T[0, 3], T[1, 3])
    f = mount.f_at(width)
    return f * Rc / (g - f) if g > f * 1.02 else float("nan")


@dataclass
class OverlayConfig:
    show_camera_center: bool = True
    show_gripper_axis: bool = True
    show_axes: bool = True                  # x5 (red) and y5 (green) arrows at the gripper-axis point
    show_world: bool = True                 # small compass: world up (z0) and forward (x0) as seen by the camera
    arrow_px: int = 45
    depth: str = "size"                     # "size": from the block's area; or a number (mm) for a fixed depth


def draw_overlay(img, mount: CameraMount, det=None, pulses=None, cfg: OverlayConfig = None, Z=None):
    """Draw the camera center, the gripper z axis (and where it meets the block's depth), the x5/y5
    arrows there, and the block's offset from the gripper axis in mm. Returns (img, info dict)."""
    cfg = cfg or OverlayConfig()
    H, W = img.shape[:2]
    cx, cy = W / 2, H / 2
    f = mount.f_at(W)
    info = {}
    if Z is None:
        if isinstance(cfg.depth, (int, float)):
            Z = float(cfg.depth)
        elif det is not None and getattr(det, "found", False) and det.area > 0:
            Z = depth_from_size(mount, math.sqrt(det.area), W)
    info["Z_mm"] = Z
    if cfg.show_camera_center:
        cv2.drawMarker(img, (int(cx), int(cy)), (255, 255, 255), cv2.MARKER_CROSS, 22, 1)
        cv2.putText(img, "camera", (int(cx) + 6, int(cy) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    Tc5 = np.linalg.inv(mount.T_5c())
    if cfg.show_gripper_axis:
        a = Tc5[:3, :3] @ np.array([0, 0, 1.0])
        if a[2] > 1e-6:                      # vanishing point of the gripper axis, then down to the edge
            vp = project_cam(a, f, cx, cy)
            o = (Tc5 @ np.array([0, 0, 0, 1.0]))[:3]
            near = project_cam(o + a * (60.0 - o[2]) / a[2], f, cx, cy)
            if vp and near:
                cv2.line(img, (int(vp[0]), int(vp[1])), (int(near[0]), int(near[1])), (0, 200, 255), 1, cv2.LINE_AA)
    g = gripper_axis_point(mount, Z, W, H) if Z else None
    if g is not None:
        gp = (int(round(g[0])), int(round(g[1])))
        info["gripper_px"] = g
        cv2.circle(img, gp, 7, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"gripper z at {Z:.0f} mm", (gp[0] + 10, gp[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if cfg.show_axes:
            for axis, col, name in ((0, (0, 0, 255), "x5"), (1, (0, 220, 0), "y5")):
                d = Tc5[:3, axis]
                # direction in the image of a small step along the axis, at the gripper-axis point
                base = np.array([g[0] - cx, g[1] - cy, f]) * Z / f
                q = project_cam(base + 5.0 * d, f, cx, cy)
                if q is None:
                    continue
                v = np.array([q[0] - g[0], q[1] - g[1]])
                if np.linalg.norm(v) < 1e-6:
                    continue
                v = v / np.linalg.norm(v) * cfg.arrow_px
                tip = (int(gp[0] + v[0]), int(gp[1] + v[1]))
                cv2.arrowedLine(img, gp, tip, col, 2, cv2.LINE_AA, tipLength=0.25)
                cv2.putText(img, name, (tip[0] + 4, tip[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        if det is not None and getattr(det, "found", False):
            du, dv = det.cx - g[0], det.cy - g[1]
            # pixel offset at depth Z -> mm in the camera frame -> frame 5
            off_c = np.array([du * Z / f, dv * Z / f, 0.0])
            off_5 = mount.R_5c() @ off_c
            info["block_from_gripper_mm"] = (float(off_5[0]), float(off_5[1]))
            cv2.line(img, gp, (int(det.cx), int(det.cy)), (0, 200, 255), 1, cv2.LINE_AA)
            cv2.putText(img, f"block from gripper axis: x5 {off_5[0]:+.0f} mm, y5 {off_5[1]:+.0f} mm",
                        (8, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    if cfg.show_world and pulses is not None:
        T05 = frames(q_from_pulses(pulses))[5]
        R0c = T05[:3, :3] @ mount.R_5c()
        info["grasp_point_mm"] = tuple(float(v) for v in T05[:3, 3])
        c0 = np.array([W - 60, 60])
        cv2.circle(img, tuple(int(v) for v in c0), 40, (90, 90, 90), 1, cv2.LINE_AA)
        for vec, col, name in ((np.array([0, 0, 1.0]), (255, 120, 0), "up"), (np.array([1.0, 0, 0]), (200, 200, 200), "fwd")):
            v_c = R0c.T @ vec                # world direction in camera axes
            v2 = np.array([v_c[0], v_c[1]])
            n = np.linalg.norm(v2)
            if n < 0.15:                     # nearly along the optical axis: draw a dot
                cv2.circle(img, tuple(int(v) for v in c0), 4, col, -1)
                cv2.putText(img, name + (" (toward)" if v_c[2] < 0 else " (away)"), (int(c0[0]) - 55, int(c0[1]) + 56),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
                continue
            tip = c0 + 34 * v2 / n * min(1.0, n)
            cv2.arrowedLine(img, tuple(int(v) for v in c0), tuple(int(v) for v in tip), col, 2, cv2.LINE_AA, tipLength=0.3)
            cv2.putText(img, name, (int(tip[0]) - 10, int(tip[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        x, y, z = info["grasp_point_mm"]
        cv2.putText(img, f"grasp point x {x:.0f} y {y:.0f} z {z:.0f} mm (model)", (8, H - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return img, info


def gripper_aim(mount: CameraMount):
    """Aim function for TrackerConfig.aim_fn: put the block on the gripper's z axis instead of the image center."""
    def aim(det, W, H):
        if not getattr(det, "found", False) or det.area <= 0:
            return (W / 2, H / 2)
        g = gripper_axis_point(mount, depth_from_size(mount, math.sqrt(det.area), W), W, H)
        return g if g is not None else (W / 2, H / 2)
    return aim


def tracker_overlay(mount: CameraMount, cfg: OverlayConfig = None):
    """Overlay function for Tracker.overlay: draws draw_overlay() on every frame, with the pose from the sketch's state."""
    def overlay(frame, det, state):
        pulses = None
        if state is not None and all(state.is_on(c) for c in range(5)):
            pulses = [float(state.pos[c]) for c in range(5)]
        draw_overlay(frame, mount, det, pulses, cfg)
    return overlay
