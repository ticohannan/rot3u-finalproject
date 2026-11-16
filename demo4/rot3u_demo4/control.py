"""Control for Robot Demo 4: keep the block at the image center using only the camera.

How the loop works (image-based, no joint sensors):
    e      = (cx, cy) - aim point                 pixel error from the detector
    J      = image Jacobian, pixels per microsecond of pulse, measured by measure_jacobian():
             column 0 = how the image point moves when the pan joint (base) gets +1 us,
             column 1 = the same for the tilt joint (wrist pitch)
    eu     = -J^-1 e                              pulse change each joint still needs (us)
    target = PID(eu) per joint                    sent to the sketch as an absolute pulse target
The sketch then moves the servo to the target with its speed and acceleration limits.

Why each PID term is there:
    I  the main term. It accumulates until the joint really moves, which is what gets a servo out
       of its dead band (small steps that do nothing).
    P  a quick first push toward the goal.
    D  slows the approach when the error is already shrinking (less overshoot, less jerk).
Anti-windup: the integrator is corrected by how far the target is ahead of the sketch's moving
position (back-calculation), so a slow, smooth profile does not make it overshoot.
A dead band with hysteresis (in pixels) stops the loop from hunting around the servo's resolution.

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import csv, math, os, time
from dataclasses import dataclass, field
import numpy as np
import cv2
from . import robot, vision

US_PER_COUNT = 1e6 * 132 / 27e6        # one PCA9685 count = 4.889 us: the finest step the board can make


# ============================================================================================ rig
@dataclass
class Rig:
    """Everything a routine needs: the link, the camera, the color, and how to wait."""
    link: object
    camera: object
    color: vision.ColorRange
    method: str = "blob"               # "blob" (class method) or "edges"
    min_area: int = 150
    sleep: object = time.sleep
    now: object = time.perf_counter

    def poll(self):
        if not self.link.threaded:
            self.link.poll()

    def detect(self, frame, both=False):
        return vision.detect(frame, vision.as_range(self.color), self.method, self.min_area, both=both)

    def centroid(self, n=5):
        """Median aim point over n new frames, or None if the block was missing in more than half."""
        pts = []
        for _ in range(n):
            frame, _, _ = self.camera.read()
            _, d = self.detect(frame)
            if d.found:
                pts.append((d.cx, d.cy))
        self.poll()
        if len(pts) * 2 <= n:
            return None
        return np.median(np.array(pts), axis=0)

    def settle(self, extra=0.30):
        """Wait until the sketch has finished moving, then a little longer for the servo and the camera."""
        robot.wait_done(self.link, self.sleep)
        self.sleep(extra)
        self.camera.read()
        self.camera.read()

    def move(self, joint, us, extra=0.30):
        r = self.link.cmd(f"TC {joint} {us:.1f}")
        if not r.ok:
            raise RuntimeError(f"joint {joint} target refused: " + " | ".join(r.lines))
        self.settle(extra)


# ============================================================================================ PID
class PID:
    """PID on an error, output = an absolute command. Integrator with back-calculation anti-windup.

        u = I + kp * e + kd * de/dt          I += (ki * e + kt * (actual - u_last)) * dt
    `actual` is what the actuator has really been told so far (here: the sketch's moving position).
    de/dt is low-pass filtered with time constant d_tau so camera noise is not amplified."""

    def __init__(self, kp=0.3, ki=3.0, kd=0.02, d_tau=0.10, out_min=-math.inf, out_max=math.inf):
        self.kp, self.ki, self.kd, self.d_tau = kp, ki, kd, d_tau
        self.out_min, self.out_max = out_min, out_max
        self.reset(0.0)

    def reset(self, output):
        """Start (or re-start) holding `output` with no stored error."""
        self.i = float(output)
        self.u = float(output)
        self.e_prev = None
        self.de = 0.0

    def update(self, e, dt, actual=None):
        if self.e_prev is not None and dt > 0:
            raw = (e - self.e_prev) / dt
            a = dt / (self.d_tau + dt)
            self.de += a * (raw - self.de)
        self.e_prev = e
        back = (actual - self.u) if actual is not None else 0.0     # negative while the target is ahead
        self.i += (self.ki * e + self.ki * back) * dt
        self.i = min(self.out_max, max(self.out_min, self.i))
        self.u = min(self.out_max, max(self.out_min, self.i + self.kp * e + self.kd * self.de))
        return self.u


# ====================================================================================== tracker
@dataclass
class TrackerConfig:
    joints: tuple = (0, 3)             # pan joint (base), tilt joint (wrist pitch): PCA9685 channels
    mode: str = "2AXIS"                # "1AXIS": only joints[0] moves. "2AXIS": both
    kp: tuple = (0.3, 0.3)
    ki: tuple = (3.0, 3.0)             # 1/s
    kd: tuple = (0.02, 0.02)           # s
    deadband_in_px: float = 6.0        # inside this for center_frames frames -> CENTERED, targets frozen
    deadband_out_px: float = 11.0      # outside this -> tracking again
    center_frames: int = 4
    aim: tuple = (0.5, 0.5)            # where the block should sit, as fractions of width and height
    max_lead_us: float = 150.0         # the target may be at most this far ahead of the arm (limits a bad detection)
    min_step_us: float = 2.5           # do not send target changes smaller than this (half a PCA9685 count)
    lash_us: tuple = (0.0, 0.0)        # optional backlash compensation per joint (experimental; 0 = off)
    track_v: float = 500.0             # sketch speed limit while tracking, us/s
    track_a: float = 2500.0            # sketch acceleration limit while tracking, us/s^2
    lost_frames: int = 6               # frames without the block before the state reads LOST
    compare: bool = True               # also run the other detector and log where it would aim
    margin_us: float = 10.0            # stay this far inside the pulse limits
    aim_fn: object = None              # optional f(detection, W, H) -> (x, y) aim point, e.g. geometry.gripper_aim(mount):
                                       # put the block on the gripper's z axis instead of at `aim`


class Tracker:
    """tracker.step(frame, t) does one cycle. tracker.run(...) loops with a display until stopped."""

    def __init__(self, rig: Rig, J, config: TrackerConfig = None):
        self.rig, self.cfg = rig, config or TrackerConfig()
        self.J = None if J is None else np.asarray(J, float).reshape(2, 2)
        c = self.cfg
        self.pid = [PID(c.kp[k], c.ki[k], c.kd[k],
                        out_min=robot.MIN_US[j] + c.margin_us, out_max=robot.MAX_US[j] - c.margin_us)
                    for k, j in enumerate(c.joints)]
        self.u = None                  # targets being commanded, per tracked joint (us)
        self.sent = None
        self.state = "IDLE"
        self.centered = False
        self._count = 0
        self._lost = 0
        self._t_prev = None
        self._dir = [0.0, 0.0]
        self.paused = False
        self.overlay = None            # optional f(frame, detection, state) drawn on the camera view (geometry overlay)
        self.log = []

    # ---- setup
    def start(self):
        if self.J is None:
            raise RuntimeError("no image Jacobian: run measure_jacobian(rig) first and pass its J")
        if abs(np.linalg.det(self.J)) < 1e-9 and self.cfg.mode == "2AXIS":
            raise RuntimeError("the image Jacobian is singular: both joints move the image the same way")
        link, c = self.rig.link, self.cfg
        st = link.get_state()
        for j in c.joints:
            if not st.is_on(j):
                raise RuntimeError(f"channel {j} ({robot.JOINT_NAMES[j]}) is not started: power up the arm first")
        for j in c.joints:
            link.cmd(f"VA {c.track_v:.0f} {c.track_a:.0f} {j}")
        link.cmd("STREAM 50")
        self.u = [float(st.tgt[j]) for j in c.joints]
        self.sent = list(self.u)
        for k in range(2):
            self.pid[k].reset(self.u[k])
        self.centered, self._count, self._lost, self._t_prev = False, 0, 0, None
        self.state = "SEARCH"

    def set_gains(self, kp=None, ki=None, kd=None):
        for k, p in enumerate(self.pid):
            if kp is not None: p.kp = kp if np.isscalar(kp) else kp[k]
            if ki is not None: p.ki = ki if np.isscalar(ki) else ki[k]
            if kd is not None: p.kd = kd if np.isscalar(kd) else kd[k]

    # ---- one cycle
    def step(self, frame, t, want_view=True):
        c, rig = self.cfg, self.rig
        if self.u is None:
            self.start()
        rig.poll()
        dt = 0.033 if self._t_prev is None else min(0.25, max(0.005, t - self._t_prev))
        self._t_prev = t
        H, W = frame.shape[:2]
        if c.compare:
            mask, a, b = rig.detect(frame, both=True)
            det, alt = (a, b) if rig.method == "blob" else (b, a)
        else:
            mask, det = rig.detect(frame)
            alt = None
        aim = (c.aim[0] * W, c.aim[1] * H)
        if c.aim_fn is not None and det.found:
            aim = tuple(float(v) for v in c.aim_fn(det, W, H))
        st = rig.link.state
        pos = [float(st.pos[j]) for j in c.joints] if st is not None else list(self.sent)
        e = np.array([float("nan")] * 2)
        e_px = float("nan")
        eu = [0.0, 0.0]
        active = [True, c.mode == "2AXIS"]
        if self.paused:
            self.state = "PAUSED"
        elif not det.found:
            self._lost += 1
            self._count = 0
            self.state = "LOST" if self._lost >= c.lost_frames else self.state
            for p in self.pid:
                p.e_prev = None
        else:
            self._lost = 0
            e = np.array([det.cx - aim[0], det.cy - aim[1]])
            if c.mode == "1AXIS":          # only the part of the error the pan joint can remove
                j0 = self.J[:, 0]
                eu = [-float(j0 @ e) / float(j0 @ j0), 0.0]
                e_px = abs(float(j0 @ e)) / float(np.linalg.norm(j0))
            else:
                eu = [float(v) for v in -np.linalg.solve(self.J, e)]
                e_px = float(np.linalg.norm(e))
            if self.centered:
                if e_px > c.deadband_out_px:
                    self.centered, self._count = False, 0
            else:
                self._count = self._count + 1 if e_px < c.deadband_in_px else 0
                if self._count >= c.center_frames:
                    self.centered = True
                    for k in range(2):     # freeze where we are; leaving the dead band later is bumpless
                        self.pid[k].reset(self.sent[k])
                        self.u[k] = self.sent[k]
            if self.centered:
                self.state = "CENTERED"
            else:
                self.state = "TRACK"
                out = [None] * robot.NUM_CH
                change = False
                for k, j in enumerate(c.joints):
                    if not active[k]:
                        continue
                    u = self.pid[k].update(eu[k], dt, actual=pos[k])
                    u = min(pos[k] + c.max_lead_us, max(pos[k] - c.max_lead_us, u))
                    if (u <= self.pid[k].out_min + 0.5 and eu[k] < 0) or (u >= self.pid[k].out_max - 0.5 and eu[k] > 0):
                        self.state = "AT_LIMIT"
                    self.pid[k].u = u
                    self.u[k] = u
                    if c.lash_us[k] > 0:   # take up the free play in the direction of travel
                        if abs(u - self.sent[k]) > 1.0:
                            self._dir[k] = math.copysign(1.0, eu[k])
                        u = u + 0.5 * c.lash_us[k] * self._dir[k]
                        u = min(self.pid[k].out_max, max(self.pid[k].out_min, u))
                    if abs(u - self.sent[k]) >= c.min_step_us:
                        out[j] = u
                        self.sent[k] = u
                        change = True
                if change:
                    rig.link.targets(out)
        rec = {"t": t, "state": self.state, "mode": c.mode, "method": rig.method, "found": int(det.found),
               "cx": det.cx, "cy": det.cy, "area": det.area, "ex": float(e[0]), "ey": float(e[1]), "e_px": e_px,
               "alt_cx": alt.cx if alt is not None else float("nan"), "alt_cy": alt.cy if alt is not None else float("nan"),
               "eu0": eu[0], "eu1": eu[1], "u0": self.sent[0], "u1": self.sent[1], "pos0": pos[0], "pos1": pos[1],
               "aim_x": aim[0], "aim_y": aim[1]}
        self.log.append(rec)
        view = None
        if want_view:
            lines = [f"{c.mode}  {self.state}  method {rig.method}" + ("  [" + det.note + "]" if det.note else ""),
                     (f"error {e[0]:+.0f}, {e[1]:+.0f} px   |e| {e_px:.1f}" if det.found else "no block in the color range"),
                     f"{robot.JOINT_NAMES[c.joints[0]]} {self.sent[0]:.0f} us   {robot.JOINT_NAMES[c.joints[1]]} {self.sent[1]:.0f} us"]
            shown = frame
            if self.overlay is not None:
                shown = frame.copy()
                try:
                    self.overlay(shown, det, st)
                except Exception as ex:            # a drawing error must never stop the arm's loop
                    lines.append(f"overlay error: {ex}")
            view = vision.annotate(shown, mask, det, alt, aim, lines, centered=self.centered)
        return rec, view

    # ---- loop
    def run(self, seconds=None, show="window", record=None, title="Robot Demo 4 - q quits, 1/2 axes, m method, space pause"):
        """Track until `seconds` have passed, q/Esc is pressed in the window, or the kernel is interrupted.

        show: "window" (OpenCV window; best for recording the screen), "inline" (in the notebook), or None.
        record: path of an MP4 of the annotated view. The arm always ends holding (X is sent)."""
        rig = self.rig
        self.start()
        fps = getattr(rig.camera, "fps_measured", 0.0)
        rec = vision.Recorder(record, fps=round(fps) if fps > 5 else 20.0) if record else None
        widget = None
        if show == "window":               # opencv-python-headless has no windows: fall back to the notebook
            try:
                cv2.namedWindow(title)
            except cv2.error:
                print("this OpenCV build cannot open windows (opencv-python-headless?): showing in the notebook instead")
                show = "inline"
        if show == "inline":
            import ipywidgets
            from IPython.display import display
            widget = ipywidgets.Image(format="jpeg", width=960)
            display(widget)
        t0 = None
        n = 0
        try:
            while True:
                frame, t, _ = rig.camera.read()
                t0 = t if t0 is None else t0
                _, view = self.step(frame, t, want_view=bool(show or rec))
                n += 1
                if rec:
                    rec.write(view)
                if show == "window":
                    cv2.imshow(title, view)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("1"): self.cfg.mode = "1AXIS"; self.centered = False
                    if key == ord("2"): self.cfg.mode = "2AXIS"; self.centered = False
                    if key == ord("m"): rig.method = "edges" if rig.method == "blob" else "blob"
                    if key == ord(" "): self.paused = not self.paused
                elif widget is not None and n % 3 == 0:
                    widget.value = vision.to_jpeg(view, 70)
                if seconds is not None and t - t0 >= seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            rig.link.stop()
            self.state = "IDLE"
            if rec:
                rec.close()
            if show == "window":
                cv2.destroyAllWindows()
                cv2.waitKey(1)
        return self.log

    def save_log(self, path):
        if not self.log:
            return
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.log[0].keys()))
            w.writeheader()
            w.writerows(self.log)


# ================================================================================ measurements
def measure_noise(rig: Rig, n=30):
    """Arm still, block in view: how much the aim point jitters from frame to frame (pixels)."""
    pts, areas = [], []
    for _ in range(n):
        frame, _, _ = rig.camera.read()
        _, d = rig.detect(frame)
        if d.found:
            pts.append((d.cx, d.cy)); areas.append(d.area)
    rig.poll()
    if len(pts) < 3:
        return {"found_frac": len(pts) / n, "ok": False}
    p = np.array(pts)
    return {"found_frac": len(pts) / n, "ok": len(pts) >= 0.9 * n, "cx": float(p[:, 0].mean()), "cy": float(p[:, 1].mean()),
            "std_x": float(p[:, 0].std()), "std_y": float(p[:, 1].std()), "area": float(np.mean(areas))}


@dataclass
class JacobianResult:
    J: np.ndarray                      # 2x2, pixels per us: columns = joints
    joints: tuple
    lost_us: list                      # same command reached from above and from below: difference, us
    repeat_px: list                    # same command, same direction, twice: difference, pixels
    noise_px: float = 0.0
    notes: list = field(default_factory=list)

    def gain(self, k):                 # pixels per us for tracked joint k
        return float(np.linalg.norm(self.J[:, k]))

    def direction(self, k):            # unit vector in the image for +1 us on tracked joint k
        return self.J[:, k] / self.gain(k)

    def deadband_px(self):
        """Suggested dead band: the loop cannot place the image finer than the servo steps."""
        res = max((0.5 * US_PER_COUNT + 0.25 * abs(l)) * self.gain(k) for k, l in enumerate(self.lost_us))
        inner = max(4.0, 2.5 * self.noise_px, 1.2 * res)
        return round(inner, 1), round(1.8 * inner, 1)

    def report(self):
        out = []
        for k, j in enumerate(self.joints):
            d = self.direction(k)
            out.append(f"{robot.JOINT_NAMES[j]} (ch {j}): {self.gain(k):.3f} px/us = {self.gain(k) * abs(robot.USPD[j]):.1f} px/deg, "
                       f"+1 us moves the block ({d[0]:+.2f}, {d[1]:+.2f}) in the image; lost motion {self.lost_us[k]:+.1f} us; "
                       f"repeat {self.repeat_px[k]:.1f} px")
        cond = np.linalg.cond(self.J)
        out.append(f"condition number {cond:.1f}" + ("  (over 10: the two joints move the image almost the same way)" if cond > 10 else ""))
        lo, hi = self.deadband_px()
        out.append(f"aim-point noise {self.noise_px:.2f} px; suggested dead band: in {lo} px, out {hi} px")
        return "\n".join(out + self.notes)


def measure_jacobian(rig: Rig, joints=(0, 3), step_us=40.0, n=5, say=print) -> JacobianResult:
    """Jog each joint a little and watch the block move in the image. The block must hold still.

    Per joint, around its present target u0 (all moves are step_us):
        down, up to u0 (p0), up (p1), down to u0 (p2), down (p3), up to u0 (p4)
    column = average of (p1 - p0) and (p2 - p3), both pairs taken while moving one way, so free
    play does not shrink it. p2 - p0 = the same command reached from above and from below = lost
    motion (dead band + backlash). p4 - p0 = repeatability."""
    link = rig.link
    st = link.get_state()
    noise = measure_noise(rig, 15)
    if not noise.get("ok"):
        raise RuntimeError("the block is not seen steadily: fix the color range or the lighting first")
    J = np.zeros((2, 2)); lost, rep, notes = [], [], []
    for k, j in enumerate(joints):
        u0 = float(st.tgt[j])
        lo, hi = robot.MIN_US[j] + 5, robot.MAX_US[j] - 5
        if u0 - step_us < lo or u0 + step_us > hi:
            raise RuntimeError(f"{robot.JOINT_NAMES[j]} is too close to a limit for a {step_us:.0f} us jog")
        pts = []
        try:
            for u in (u0 - step_us, u0, u0 + step_us, u0, u0 - step_us, u0):
                rig.move(j, u)
                p = rig.centroid(n)
                if p is None:
                    raise RuntimeError(f"lost the block while jogging {robot.JOINT_NAMES[j]}: use a smaller step_us "
                                       "or start with the block nearer the image center")
                pts.append(p)
        finally:
            link.cmd(f"TC {j} {u0:.1f}")
        _, p0, p1, p2, p3, p4 = pts
        col = 0.5 * ((p1 - p0) + (p2 - p3)) / step_us
        g = float(np.linalg.norm(col))
        if g * step_us < 4 * max(0.5, noise["std_x"], noise["std_y"]):
            notes.append(f"NOTE {robot.JOINT_NAMES[j]}: the block moved only {g * step_us:.1f} px; use a larger step_us")
            g = max(g, 1e-6)
        J[:, k] = col
        lost.append(float((p2 - p0) @ col) / (g * g) if g > 1e-5 else 0.0)
        rep.append(float(np.linalg.norm(p4 - p0)))
        say(f"{robot.JOINT_NAMES[j]}: +{step_us:.0f} us moved the block by ({col[0] * step_us:+.1f}, {col[1] * step_us:+.1f}) px")
    rig.settle()
    return JacobianResult(J, tuple(joints), lost, rep, float(max(noise["std_x"], noise["std_y"])), notes)


def _project(rig, p_ref, direction, gain, n=3):
    p = rig.centroid(n)
    return float("nan") if p is None else float((p - p_ref) @ direction) / gain


def measure_slop(rig: Rig, jac: JacobianResult, k=0, step_us=US_PER_COUNT, n_steps=10, settle=0.25, say=print):
    """Staircase test of one tracked joint (k = 0 pan, 1 tilt), with the camera as the sensor.

    The joint climbs n_steps small steps, comes down 2 n_steps, and climbs back: a closed loop of
    commands. The block's shift in the image, divided by the Jacobian gain, is the joint's real
    movement in pulse units. Returns the loop (for plotting) and:
        width_us     the loop's width: how far the joint lags when the direction reverses
                     (dead band + backlash). This is the 'slop'.
        stuck_frac   fraction of steps where the joint did not move (under 30% of a step)
        max_jump     biggest single response, in steps (over 1.5 = stick-slip: stores up, then jumps)
        gain         real movement per commanded us while moving one way (1.0 = Jacobian still right)"""
    j = jac.joints[k]
    d, g = jac.direction(k), jac.gain(k)
    st = rig.link.get_state()
    u0 = float(st.tgt[j])
    span = n_steps * step_us
    if u0 - span - 30 < robot.MIN_US[j] or u0 + span > robot.MAX_US[j]:
        raise RuntimeError("too close to a limit for this staircase: fewer steps or move the joint")
    cmds = [u0 + i * step_us for i in range(1, n_steps + 1)]
    cmds += [u0 + i * step_us for i in range(n_steps - 1, -n_steps - 1, -1)]
    cmds += [u0 + i * step_us for i in range(-n_steps + 1, 1)]
    resp = []
    try:
        rig.move(j, u0 - 30)                           # arrive at u0 from below, like the first branch
        rig.move(j, u0)
        p_ref = rig.centroid(5)
        if p_ref is None:
            raise RuntimeError("the block is not in view")
        for u in cmds:
            rig.move(j, u, extra=settle)
            resp.append(_project(rig, p_ref, d, g))
    finally:
        rig.link.cmd(f"TC {j} {u0:.1f}")
    c = np.array(cmds) - u0
    r = np.array(resp)
    up1, down, up2 = slice(0, n_steps), slice(n_steps, 3 * n_steps), slice(3 * n_steps, 4 * n_steps)
    lookup = {}
    for ci, ri in list(zip(c[up1], r[up1])) + list(zip(c[up2], r[up2])):
        lookup.setdefault(round(float(ci), 2), []).append(ri)
    widths = [ri - np.mean(lookup[round(float(ci), 2)]) for ci, ri in zip(c[down], r[down])
              if round(float(ci), 2) in lookup and abs(ci) < 0.7 * span]
    dr = np.diff(np.concatenate([[0.0], r])) * np.sign(np.diff(np.concatenate([[0.0], c])))
    out = {"joint": j, "cmd_us": c, "resp_us": r, "step_us": step_us,
           "width_us": float(np.nanmean(widths)) if widths else float("nan"),
           "stuck_frac": float(np.nanmean(np.abs(dr) < 0.3 * step_us)),
           "max_jump": float(np.nanmax(dr) / step_us),
           "gain": float(np.polyfit(c[down][n_steps // 2:], r[down][n_steps // 2:], 1)[0])}
    say(f"{robot.JOINT_NAMES[j]}: slop (loop width) {out['width_us']:.1f} us = {out['width_us'] / abs(robot.USPD[j]):.2f} deg "
        f"= {out['width_us'] * g:.1f} px; {100 * out['stuck_frac']:.0f}% of {step_us:.1f} us steps did nothing; "
        f"biggest jump {out['max_jump']:.1f} steps; gain {out['gain']:.2f}")
    rig.settle()
    return out


def step_response(rig: Rig, jac: JacobianResult, k=0, step_us=60.0, seconds=1.5, v=None, a=None, say=print):
    """Command one step on a tracked joint and film the block: latency, rise time, overshoot.

    v, a: sketch speed and acceleration limits for this test (None = leave as they are).
    Use it to choose TrackerConfig.track_v / track_a: raise them until the motion looks jerky or
    the overshoot grows, then back off."""
    j = jac.joints[k]
    d, g = jac.direction(k), jac.gain(k)
    link = rig.link
    if v is not None and a is not None:
        link.cmd(f"VA {v:.0f} {a:.0f} {j}")
    link.cmd("STREAM 20")
    u0 = float(link.get_state().tgt[j])
    rig.move(j, u0 - 30); rig.move(j, u0)
    p_ref = rig.centroid(5)
    if p_ref is None:
        raise RuntimeError("the block is not in view")
    ts, ys, fw = [], [], []
    frame, t0, _ = rig.camera.read()
    link.send(f"TC {j} {u0 + step_us:.1f}")
    t = t0
    try:
        while t - t0 < seconds:
            frame, t, _ = rig.camera.read()
            rig.poll()
            _, det = rig.detect(frame)
            ts.append(t - t0)
            ys.append(float((np.array([det.cx, det.cy]) - p_ref) @ d) / g if det.found else float("nan"))
            fw.append(float(link.state.pos[j]) - u0 if link.state else float("nan"))
    finally:
        link.cmd(f"TC {j} {u0:.1f}")
        link.cmd("STREAM 50")
    ts, ys, fw = np.array(ts), np.array(ys), np.array(fw)
    final = float(np.nanmedian(ys[int(0.8 * len(ys)):]))
    out = {"joint": j, "t": ts, "resp_us": ys, "sketch_us": fw, "step_us": step_us, "final_us": final}
    if abs(final) > 0.2 * step_us:
        s = ys / final
        i10 = np.argmax(s > 0.1); i90 = np.argmax(s > 0.9)
        out.update(dead_time=float(ts[i10]), rise_time=float(ts[i90] - ts[i10]),
                   overshoot=float(max(0.0, np.nanmax(s) - 1.0)))
        say(f"{robot.JOINT_NAMES[j]}: step {step_us:.0f} us -> moved {final:.1f} us; first motion seen after {out['dead_time'] * 1000:.0f} ms "
            f"(camera delay + servo start); 10-90% in {out['rise_time'] * 1000:.0f} ms; overshoot {100 * out['overshoot']:.0f}%")
    else:
        say(f"{robot.JOINT_NAMES[j]}: the block hardly moved ({final:.1f} us for a {step_us:.0f} us step)")
    rig.settle()
    return out


# ============================================================================ ringing (v0.2)
def fit_damped_sine(t, y, f_range=(0.5, 8.0), z_range=(0.0, 0.6), nf=300, nz=61):
    """Fit y(t) = e^(-z w t) (a cos(wd t) + b sin(wd t)) + c by a grid search over f and z.

    For each (f, z) on the grid, a, b and c come from linear least squares; the pair with the
    smallest error wins. No SciPy needed. w = 2 pi f (natural), wd = w sqrt(1 - z^2) (what the
    camera sees). Returns f_hz, zeta, amp (first peak, same units as y), offset, rms_resid, fitted."""
    t = np.asarray(t, float); y = np.asarray(y, float)
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok] - t[ok][0], y[ok]
    if len(t) < 6:
        raise ValueError("too few samples to fit the ringing (need 6 or more)")
    fmax = min(f_range[1], 0.45 * (len(t) - 1) / max(t[-1], 1e-6))      # stay under half the frame rate
    best = None
    for f in np.linspace(f_range[0], fmax, nf):
        for z in np.linspace(z_range[0], z_range[1], nz):
            w = 2 * math.pi * f; wd = w * math.sqrt(1 - z * z); e = np.exp(-z * w * t)
            A = np.column_stack([e * np.cos(wd * t), e * np.sin(wd * t), np.ones_like(t)])
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            r = float(np.sum((A @ coef - y) ** 2))
            if best is None or r < best[0]:
                best = (r, f, z, coef)
    r, f, z, (a, b, c) = best
    w = 2 * math.pi * f; wd = w * math.sqrt(1 - z * z)
    fitted = np.exp(-z * w * t) * (a * np.cos(wd * t) + b * np.sin(wd * t)) + c
    return {"f_hz": float(f), "zeta": float(z), "amp": float(math.hypot(a, b)), "offset": float(c),
            "rms_resid": float(math.sqrt(r / len(t))), "t": t, "y": y, "fitted": fitted}


def measure_ringdown(rig: Rig, jac: JacobianResult, k=0, step_us=60.0, seconds=3.0, v=3000.0, a=30000.0,
                     skip_s=0.10, say=print):
    """Shake one tracked joint with a quick step and film the swing that follows.

    The joint steps by step_us as fast as the sketch allows (v, a), then holds. The camera keeps
    watching for `seconds`; the block's motion along that joint's image direction, in pulse units,
    is the frame's ringing. A damped sine is fitted to the part after the sketch has stopped
    (plus skip_s): f_hz and zeta are what SHAPE needs (apply_shaping).

    Frame rate limits it: ringing faster than half the camera's frame rate cannot be measured
    (about 4.5 Hz at 9 frames/s). Returns the record and the fit; joint returns to where it was."""
    j = jac.joints[k]
    d, g = jac.direction(k), jac.gain(k)
    link = rig.link
    st = link.get_state()
    u0 = float(st.tgt[j])
    if not (robot.MIN_US[j] + 10 < u0 + step_us < robot.MAX_US[j] - 10):
        raise RuntimeError(f"{robot.JOINT_NAMES[j]} is too close to a limit for a {step_us:.0f} us step")
    link.cmd("STREAM 20")
    p_ref = wait_still(rig)                          # the frame may still swing from the last move
    if p_ref is None:
        raise RuntimeError("the block is not in view")
    link.cmd(f"VA {v:.0f} {a:.0f} {j}")
    ts, ys, mv = [], [], []
    frame, t0, _ = rig.camera.read()
    link.send(f"TC {j} {u0 + step_us:.1f}")
    t = t0
    t_done = None
    try:
        while t - t0 < seconds:
            frame, t, _ = rig.camera.read()
            rig.poll()
            _, det = rig.detect(frame)
            ts.append(t - t0)
            ys.append(float((np.array([det.cx, det.cy]) - p_ref) @ d) / g if det.found else float("nan"))
            s = link.state
            moving = bool(s and (abs(float(s.pos[j]) - (u0 + step_us)) > 0.5))
            mv.append(moving)
            if t_done is None and not moving:
                t_done = t - t0
    finally:
        link.cmd("STREAM 50")
    ts, ys = np.array(ts), np.array(ys)
    out = {"joint": j, "t": ts, "resp_us": ys, "step_us": step_us, "t_done": t_done}
    if t_done is None:
        say("the sketch never finished the step inside the time: use a longer `seconds`")
    else:
        sel = ts >= t_done + skip_s
        try:
            fit = fit_damped_sine(ts[sel], ys[sel])
            out["fit"] = fit
            out["t_fit0"] = float(ts[sel][0])
            tail = ys[sel] - fit["offset"]
            out["pp_us"] = float(np.nanmax(tail) - np.nanmin(tail))
            out["pp_px"] = out["pp_us"] * g
            say(f"{robot.JOINT_NAMES[j]}: after the servo stopped, the block swung {out['pp_px']:.1f} px peak to peak "
                f"({out['pp_us']:.1f} us); fit: {fit['f_hz']:.2f} Hz, damping ratio {fit['zeta']:.2f} "
                f"(fit error {fit['rms_resid']:.2f} us rms)")
        except ValueError as e:
            say(str(e))
    # back to the start, gently
    link.cmd(f"VA 400 2000 {j}")
    rig.move(j, u0)
    return out


def wait_still(rig: Rig, tol_px=1.5, n=5, timeout=5.0):
    """Wait until the block holds still in the picture (n frames within tol_px), up to timeout seconds of camera
    time. Returns the mean aim point, or None if the block is not in view."""
    pts = []
    t0 = None
    while True:
        frame, t, _ = rig.camera.read()
        t0 = t if t0 is None else t0
        _, d = rig.detect(frame)
        rig.poll()
        if d.found:
            pts = (pts + [(d.cx, d.cy)])[-n:]
        if len(pts) == n:
            p = np.array(pts)
            if np.ptp(p, axis=0).max() <= tol_px or t - t0 > timeout:
                return p.mean(axis=0)
        elif t - t0 > timeout:
            return None


def apply_shaping(link, joints, f_hz, zeta, say=print):
    """Turn on the sketch's input shaping (SHAPE) for these channels. f_hz = 0 turns it off.

    The sketch splits every move into two copies, the second delayed by half a ringing period, so
    the swing the first copy starts is cancelled by the second. Moves take that much longer
    (half a period: about 0.2 s at 2.4 Hz). Shaping needs f within about 20% of the real frequency."""
    for j in (joints if np.iterable(joints) else [joints]):
        r = link.cmd(f"SHAPE {j} {f_hz:.2f} {zeta:.2f}")
        if not r.ok:
            raise RuntimeError(f"SHAPE refused for ch {j}: " + " | ".join(r.lines) + " (sketch older than v0.2?)")
    if f_hz:
        say(f"input shaping on for ch {list(joints) if np.iterable(joints) else joints}: {f_hz:.2f} Hz, damping {zeta:.2f} "
            f"(moves are {500 / f_hz:.0f} ms longer)")
    else:
        say("input shaping off")


def compare_shaping(rig: Rig, jac: JacobianResult, f_hz, zeta, k=0, step_us=60.0, seconds=3.0, say=print):
    """Ringdown with shaping off, then on, for one joint. Returns (off, on); says how much the swing dropped."""
    j = jac.joints[k]
    apply_shaping(rig.link, [j], 0, 0, say=lambda s: None)
    off = measure_ringdown(rig, jac, k, step_us, seconds, say=say)
    apply_shaping(rig.link, [j], f_hz, zeta, say=say)
    on = measure_ringdown(rig, jac, k, step_us, seconds, say=say)
    if "pp_us" in off and "pp_us" in on and off["pp_us"] > 0:
        say(f"swing after the stop: {off['pp_px']:.1f} px without shaping, {on['pp_px']:.1f} px with it "
            f"({100 * (1 - on['pp_us'] / off['pp_us']):.0f}% less)")
    return off, on


def suggest_deadband(jac: JacobianResult, slops=()):
    """Dead band (in, out) in pixels from the Jacobian and the slop staircases.

    A servo that sticks, then jumps max_jump steps, cannot place the block finer than that jump:
    a dead band smaller than the jump makes the loop hunt."""
    lo, hi = jac.deadband_px()
    for s in slops:
        k = list(jac.joints).index(s["joint"])
        jump_px = s.get("max_jump", 1.0) * s.get("step_us", US_PER_COUNT) * jac.gain(k)
        lo = max(lo, round(0.6 * jump_px, 1))
    return lo, max(hi, round(1.8 * lo, 1))


def make_rig(link, camera, color, method="blob", min_area=150):
    """A fresh Rig from what is open now. Call it at the top of each cell instead of keeping an old rig:
    after the camera cell is run again, an old rig would still point at the closed camera."""
    return Rig(link, camera, color, method=method, min_area=min_area)
