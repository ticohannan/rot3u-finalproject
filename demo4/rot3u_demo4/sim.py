"""Simulator for Demo 4: runs the notebook and the tests with no Arduino and no camera.

    PyArm      what servo_bridge.ino does, in Python (same commands, same motion profile)
    ExeArm     the real sketch compiled for the PC (tests only; needs tests/sim/sim_servo_bridge)
    ServoModel a hobby servo: dead band (small steps do nothing), backlash, limited speed
    SimWorld   arm + servos + a pinhole camera on the gripper looking at a colored block
    SimTransport / SimCamera   stand-ins for SerialTransport and vision.Camera

The numbers (dead band, backlash, field of view, latency) are placeholders, not measurements of
Sir Pinch-a-lot. The simulator is for checking that the code runs and the loop converges.
Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import collections, math, subprocess, threading, time
import numpy as np
import cv2
from . import robot

US_PER_COUNT = 1e6 * 132 / 27e6        # PCA9685 at prescale 131: 4.889 us per count


# ======================================================================== the sketch, in Python
class PyArm:
    """Line-for-line stand-in for servo_bridge.ino v0.3 (commands, replies, motion profile, input shaping)."""
    STEP_MS = 20
    HARD_MIN = [500] * 6                   # v0.3: the servo's absolute range; robot limits come by LIM
    HARD_MAX = [2500] * 6
    HIST = 64

    def __init__(self, counts=None):
        n = robot.NUM_CH
        self.ms = 200
        self.pos = [1500.0] * n; self.tgt = [1500.0] * n; self.vel = [0.0] * n
        self.raw = [1500.0] * n; self.hist = [[1500.0] * self.HIST for _ in range(n)]; self.hat = 0
        self.shA2 = [0.0] * n; self.shLag = [0.0] * n; self.shF = [0.0] * n; self.shZ = [0.0] * n
        self.vmax = [300.0] * n; self.amax = [1200.0] * n
        self.lo = list(self.HARD_MIN); self.hi = list(self.HARD_MAX)
        self.active = [False] * n
        self.counts = [0] * n                      # 0 = output off
        self.stream = 0; self.last_step = 200; self.last_stream = 200
        self.out = collections.deque()
        self.out.append("# READY servo_bridge v0.3")
        if counts:                                 # the servo board kept running through a reset: adopt
            for c, k in enumerate(counts):
                us = k * US_PER_COUNT
                if k and 495 <= us <= 2505:
                    self._hold(c, us); self.active[c] = True; self.counts[c] = k
                    self.out.append(f"# adopted ch {c} at {int(us + 0.5)}")
        self.out.append("HELLO,servo_bridge,0.3,2")

    def _hold(self, c, us):
        self.pos[c] = self.raw[c] = self.tgt[c] = float(us); self.vel[c] = 0.0
        self.hist[c] = [float(us)] * self.HIST

    def _set_shape(self, c, f, z):
        if f <= 0:
            self.shA2[c] = self.shF[c] = self.shZ[c] = 0.0; return
        wd = math.sqrt(1 - z * z); K = math.exp(-z * math.pi / wd)
        self.shA2[c] = K / (1 + K)
        self.shLag[c] = min(self.HIST - 2, (0.5 / (f * wd)) / (self.STEP_MS / 1000.0))
        self.shF[c], self.shZ[c] = f, z

    def _hist_back(self, c, lag):
        n = int(lag); fr = lag - n
        a = self.hist[c][(self.hat - n) % self.HIST]; b = self.hist[c][(self.hat - n - 1) % self.HIST]
        return a + (b - a) * fr

    # -- outputs
    def pulses(self):
        return [k * US_PER_COUNT if a else None for k, a in zip(self.counts, self.active)]

    def pop_lines(self):
        o = list(self.out); self.out.clear(); return o

    def _write(self, c):
        self.counts[c] = int(min(2500.0, max(500.0, self.pos[c])) / US_PER_COUNT + 0.5)

    def _state(self):
        bits = sum(1 << c for c in range(6) if self.active[c])
        mv = any(self.active[c] and (self.pos[c] != self.tgt[c] or self.raw[c] != self.tgt[c] or self.vel[c] != 0) for c in range(6))
        return "ST," + ",".join(str(v) for v in [self.ms] + [int(p + 0.5) for p in self.pos] +
                                [int(t + 0.5) for t in self.tgt] + [bits, int(mv)])

    def _stop(self):
        for c in range(6):
            self._hold(c, self.pos[c])

    # -- commands
    @staticmethod
    def _num(t):
        try:
            float(t); return t not in ("nan", "inf") and not any(ch.isalpha() for ch in t)
        except (TypeError, ValueError):
            return False

    def _exec(self, s, tagged):
        tk = s.replace(",", " ").split()
        say = self.out.append
        ok = lambda: (None if tagged else say("# ok"), True)[1]
        fail = lambda why: (say("# " + why), False)[1]
        if not tk:
            self._stop(); return (None if tagged else say("# stopped - holding"), True)[1]
        cmd, a = tk[0].upper(), tk[1:]
        ch = lambda t: int(t) if (t is not None and len(t) == 1 and t.isdigit() and int(t) < 6) else -1
        if cmd == "T":
            if not a or len(a) > 6 and False:
                return fail("T a b c d e f : numbers in us, or - to leave a channel")
            vals = {}
            for c, t in enumerate(a[:6]):
                if t == "-":
                    continue
                if not self._num(t):
                    return fail("T a b c d e f : numbers in us, or - to leave a channel")
                vals[c] = float(t)
            if any(not self.active[c] for c in vals):
                return fail("a channel in that list is not started - ON c us first; nothing changed")
            for c, v in vals.items():
                self.tgt[c] = min(self.hi[c], max(self.lo[c], v))
            return ok()
        if cmd in ("ST", "STATE"):
            say(self._state()); return True
        if cmd == "X":
            self._stop(); return (None if tagged else say("# stopped - holding"), True)[1]
        if cmd == "TC":
            c = ch(a[0] if a else None)
            if c < 0 or len(a) < 2 or not self._num(a[1]): return fail("TC c us")
            if not self.active[c]: return fail("that channel is not started - ON c us first")
            self.tgt[c] = min(self.hi[c], max(self.lo[c], float(a[1]))); return ok()
        if cmd == "ON":
            c = ch(a[0] if a else None)
            if c < 0 or len(a) < 2 or not self._num(a[1]): return fail("ON c us")
            if self.active[c]: return fail("that channel is already on - use TC or T to move it")
            self._hold(c, min(self.hi[c], max(self.lo[c], float(a[1])))); self.active[c] = True; self._write(c)
            say(f"# ch {c} on at {int(self.pos[c] + 0.5)}"); return True
        if cmd == "OFF":
            if a and a[0][0] == "*":
                for c in range(6): self.active[c] = False; self.vel[c] = 0.0; self.counts[c] = 0
                say("# all channels released"); return True
            c = ch(a[0] if a else None)
            if c < 0: return fail("OFF c | OFF *")
            self.active[c] = False; self.vel[c] = 0.0; self.counts[c] = 0; return ok()
        if cmd == "VA":
            if len(a) < 2 or not self._num(a[0]) or not self._num(a[1]): return fail("VA v a [c]")
            v, acc = float(a[0]), float(a[1])
            if not (10 <= v <= 3000 and 50 <= acc <= 30000): return fail("VA v a [c]")
            if len(a) > 2 and ch(a[2]) < 0: return fail("VA v a [c]   c is 0-5")
            for c in range(6):
                if len(a) < 3 or c == int(a[2]): self.vmax[c] = v; self.amax[c] = acc
            return ok()
        if cmd == "LIM":
            if not a:
                for c in range(6): say(f"# ch {c}: {self.lo[c]} - {self.hi[c]} us")
                return True
            c = ch(a[0])
            if c < 0 or len(a) < 3 or not self._num(a[1]) or not self._num(a[2]): return fail("LIM c lo hi")
            lo, hi = int(float(a[1])), int(float(a[2]))
            if lo < self.HARD_MIN[c] or hi > self.HARD_MAX[c] or lo >= hi: return fail("LIM c lo hi : inside the built-in limits")
            self.lo[c], self.hi[c] = lo, hi
            if self.active[c]: self.tgt[c] = min(hi, max(lo, self.tgt[c]))
            return ok()
        if cmd == "STREAM":
            ms = int(float(a[0])) if a and self._num(a[0]) else -1
            if ms != 0 and not (20 <= ms <= 5000): return fail("STREAM ms : 20-5000, or 0 to stop")
            self.stream = ms; self.last_stream = self.ms; return ok()
        if cmd == "SHAPE":
            if not a:
                for c in range(6): say(f"# ch {c}: shaped {self.shF[c]:.2f} Hz z {self.shZ[c]:.2f}")
                return True
            allc = a[0] == "*"
            if not allc and ch(a[0]) < 0: return fail("SHAPE c f z   c 0-5 or *")
            if len(a) < 2 or not self._num(a[1]): return fail("SHAPE c f z   f 0.5-10 Hz (0 = off), z 0-0.7")
            f = float(a[1]); z = float(a[2]) if len(a) > 2 and self._num(a[2]) else 0.0
            if f != 0 and not (0.5 <= f <= 10 and 0 <= z <= 0.7): return fail("SHAPE c f z   f 0.5-10 Hz (0 = off), z 0-0.7")
            for c in range(6):
                if allc or c == int(a[0]):
                    if self.active[c]: self._hold(c, self.pos[c])
                    self._set_shape(c, f, z)
            return ok()
        if cmd == "HELLO":
            say("HELLO,servo_bridge,0.3,2"); return True
        if cmd in ("?", "HELP"):
            say("# ON c us | T a b c d e f | TC c us | VA v a [c] | LIM | ST | STREAM ms | X | OFF c | OFF *"); return True
        self._stop(); say("# unknown command - stopped, holding: " + cmd); return False

    def feed(self, line: str):
        s = line.strip()
        if len(s) > 63:
            self._stop(); self.out.append(f"ERR,{self.ms},LINE_TOO_LONG"); return
        if s.startswith("@"):
            head, _, rest = s[1:].partition(" ")
            i = int(head) if head.isdigit() else 0
            self.out.append(f"ACK,{i}")
            ok = self._exec(rest, True)
            self.out.append(f"DONE,{i},{'OK' if ok else 'FAIL'}")
        else:
            self._exec(s, False)

    # -- time
    def _step(self, dt):
        self.hat = (self.hat + 1) % self.HIST
        for c in range(6):
            if not self.active[c]:
                self.hist[c][self.hat] = self.raw[c]; continue
            err = self.tgt[c] - self.raw[c]
            dvmax = self.amax[c] * dt
            d, h = abs(err), 0.5 * dvmax
            vdes = min(math.sqrt(2 * self.amax[c] * d + h * h) - h, self.vmax[c], d / dt)
            if err < 0: vdes = -vdes
            self.vel[c] += max(-dvmax, min(dvmax, vdes - self.vel[c]))
            np_ = self.raw[c] + self.vel[c] * dt
            if abs(self.tgt[c] - np_) < 0.02 and abs(self.vel[c]) <= dvmax:
                np_ = self.tgt[c]; self.vel[c] = 0.0
            if self.vel[c] > 0 and np_ > self.hi[c]:
                np_ = self.raw[c] if self.raw[c] > self.hi[c] else float(self.hi[c]); self.vel[c] = 0.0
            if self.vel[c] < 0 and np_ < self.lo[c]:
                np_ = self.raw[c] if self.raw[c] < self.lo[c] else float(self.lo[c]); self.vel[c] = 0.0
            self.raw[c] = np_
            self.hist[c][self.hat] = np_
            if self.shA2[c] > 0:
                out = (1 - self.shA2[c]) * np_ + self.shA2[c] * self._hist_back(c, self.shLag[c])
                self.pos[c] = self.tgt[c] if (abs(out - self.tgt[c]) < 0.02 and np_ == self.tgt[c] and self.vel[c] == 0) else out
            else:
                self.pos[c] = np_
            self._write(c)

    def advance(self, ms: int):
        for _ in range(int(ms)):
            if self.ms - self.last_step >= self.STEP_MS:
                dt = min(0.05, (self.ms - self.last_step) / 1000.0)
                self.last_step = self.ms
                self._step(dt)
            if self.stream and self.ms - self.last_stream >= self.stream:
                self.last_stream = self.ms
                self.out.append(self._state())
            self.ms += 1


class ExeArm:
    """The real sketch, compiled for the PC (tests/sim/sim_main.cpp). Same interface as PyArm."""

    def __init__(self, exe, state_file=None):
        args = [str(exe)] + ([str(state_file)] if state_file else [])
        self.p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.counts = [0] * 6; self.ms = 0
        self.out = collections.deque()
        self._until_mark()

    def _until_mark(self):
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError("simulated sketch ended")
            line = line.rstrip("\n")
            if line.startswith("@t "):
                self.ms = int(line[3:]); return
            if line.startswith("W,"):
                _, c, k, _ = line.split(",")
                self.counts[int(c)] = 0 if int(k) >= 4096 else int(k)
            else:
                self.out.append(line)
                if line.startswith("# adopted ch "):
                    f = line.split()
                    self.counts[int(f[3])] = int(float(f[5]) / US_PER_COUNT + 0.5)

    def _send(self, s):
        self.p.stdin.write(s + "\n"); self.p.stdin.flush(); self._until_mark()

    def feed(self, line): self._send(line)
    def advance(self, ms):
        if ms > 0: self._send(f"@adv {int(ms)}")
    def pop_lines(self):
        o = list(self.out); self.out.clear(); return o
    def pulses(self): return [k * US_PER_COUNT if k else None for k in self.counts]
    def close(self):
        try:
            self.p.stdin.close(); self.p.wait(2)
        except Exception:
            self.p.kill()


# ============================================================================== servo physics
class ServoModel:
    """Commanded pulse -> where the joint really is (in pulse units).

    dead: the servo only starts moving when the command is more than `dead` us from where the
          motor sits, and stops `stop_frac * dead` short of the command.
    lash: free play between motor and link, us. loaded=True means gravity holds the link against
          one side of the play, so the play does not show."""

    def __init__(self, dead=6.0, lash=8.0, rate=3500.0, stop_frac=0.3, loaded=False):
        self.dead, self.lash, self.rate, self.stop_frac, self.loaded = dead, lash, rate, stop_frac, loaded
        self.m = None; self.y = None; self.running = False

    def step(self, u, dt):
        if u is None:
            return self.y
        if self.m is None:
            self.m = u; self.y = u - (self.lash / 2 if self.loaded else 0.0)
            return self.y
        e = u - self.m
        if not self.running and abs(e) > self.dead:
            self.running = True
        if self.running:
            goal = u - math.copysign(self.dead * self.stop_frac, e)
            d = goal - self.m
            stepmax = self.rate * dt
            if abs(d) <= stepmax or d * e <= 0:
                self.m = goal if d * e > 0 else self.m
                self.running = False
            else:
                self.m += math.copysign(stepmax, d)
        if self.loaded:
            self.y = self.m - self.lash / 2
        elif self.m - self.y > self.lash / 2:
            self.y = self.m - self.lash / 2
        elif self.y - self.m > self.lash / 2:
            self.y = self.m + self.lash / 2
        return self.y


class StructureModel:
    """The arm's frame between a servo and the camera: a lightly damped spring-mass (second order).

    Input: where the servo really is (pulse units). Output: where the link really points.
    f_hz, zeta: natural frequency and damping ratio of the swing (the camera sees it as ringing)."""

    def __init__(self, f_hz=2.4, zeta=0.08):
        self.w, self.z = 2 * math.pi * f_hz, zeta
        self.y = None; self.v = 0.0

    def step(self, u, dt):
        if u is None:
            return self.y
        if self.y is None:
            self.y = u
            return self.y
        n = max(1, int(math.ceil(dt / 0.001)))      # small steps for a stable integration
        h = dt / n
        for _ in range(n):
            a = self.w * self.w * (u - self.y) - 2 * self.z * self.w * self.v
            self.v += a * h
            self.y += self.v * h
        return self.y


# ===================================================================================== the world
class SimWorld:
    """Arm, servos and a camera on the gripper looking at one colored block (plus optional clutter).

    Camera defaults match the real mount (geometry.CameraMount): 115 mm ahead of the wrist-pitch axis
    (170 - 55), 29.5 mm above the gripper axis, 47.9 deg horizontal view (f = 720 px at 640 wide).

    realtime=True: simulated time follows the wall clock (notebook with PORT = "SIM").
    realtime=False: time moves only when advance() is called (tests; reading a camera frame
    advances time by one frame period)."""

    def __init__(self, arm=None, realtime=False, pose="WALL", target=(450.0, 40.0, 280.0), block_mm=40.0,
                 color_hsv=(4, 210, 200), fov_deg=47.9, size=(640, 480), cam_roll_deg=0.0,
                 cam_fwd_mm=115.0, cam_up_mm=29.5, latency_s=0.10, fps=30.0, noise=4.0, seed=1,
                 servos=None, clutter=True, glare=False, structure=None):
        self.arm = arm or PyArm()
        self.realtime = realtime
        self.t = self.arm.ms / 1000.0
        self._wall0 = time.perf_counter() - self.t
        self.lock = threading.RLock()
        self.W, self.H = size
        self.fpx = (self.W / 2) / math.tan(math.radians(fov_deg) / 2)
        self.cam_roll = math.radians(cam_roll_deg)
        self.cam_fwd, self.cam_up = cam_fwd_mm, cam_up_mm
        self.target = np.array(target, float)      # block center, mm, robot base frame (x forward, z up)
        self.block_mm, self.color_hsv = block_mm, color_hsv
        self.block_spin = 20.0                     # deg, in-image rotation of the block's outline
        self.latency, self.fps, self.noise = latency_s, fps, noise
        self.clutter, self.glare = clutter, glare
        self.rng = np.random.default_rng(seed)
        self.servos = servos or [ServoModel(lash=8.0), ServoModel(loaded=True), ServoModel(loaded=True),
                                 ServoModel(lash=6.0, loaded=True), ServoModel(), ServoModel()]
        # structure: None (rigid), or {channel: StructureModel(...)} for joints whose frame swings
        self.structure = structure or {}
        self.pose_us = robot.pose_us(pose)
        self.y = [float(u) for u in self.pose_us]  # true joint positions, pulse units
        self.hist = collections.deque(maxlen=400)  # (t, y) for the camera's latency
        self.lines = collections.deque()
        bg = self.rng.normal(0, 1, (self.H // 8, self.W // 8)).astype(np.float32)
        self._bg = cv2.resize(bg, (self.W, self.H), interpolation=cv2.INTER_CUBIC)
        self._collect()

    # -- time
    def _collect(self):
        self.lines.extend(self.arm.pop_lines())

    def advance(self, dt: float):
        with self.lock:
            n = max(0, int(round(dt * 1000)))
            for _ in range(n // 5 + (1 if n % 5 else 0)):
                step = min(5, n); n -= step
                self.arm.advance(step)
                for c, u in enumerate(self.arm.pulses()):
                    y = self.servos[c].step(u, step / 1000.0)
                    if y is not None and c in self.structure:
                        y = self.structure[c].step(y, step / 1000.0)
                    if y is not None:
                        self.y[c] = y
                self.t += step / 1000.0
                self.hist.append((self.t, list(self.y)))
            self._collect()

    def sync(self):
        if self.realtime:
            lag = (time.perf_counter() - self._wall0) - self.t
            if lag > 0.004:
                self.advance(min(lag, 0.5))

    def feed(self, line: str):
        with self.lock:
            self.sync()
            self.arm.feed(line)
            self._collect()

    # -- camera
    def _pixel(self, y, point):
        d = [robot.us_to_deg(j, y[j]) for j in range(5)]
        t1, t2 = math.radians(d[0]), math.radians(d[1])
        t23 = t2 + math.radians(d[2]); ph = t23 + math.radians(d[3])
        rw = robot.R2 * math.cos(t2) + robot.R3 * math.cos(t23)
        zw = robot.D1 + robot.R2 * math.sin(t2) + robot.R3 * math.sin(t23)
        rc = rw + self.cam_fwd * math.cos(ph) - self.cam_up * math.sin(ph)
        zc = zw + self.cam_fwd * math.sin(ph) + self.cam_up * math.cos(ph)
        cam = np.array([rc * math.cos(t1), rc * math.sin(t1), zc])
        ax = np.array([math.cos(ph) * math.cos(t1), math.cos(ph) * math.sin(t1), math.sin(ph)])
        up = np.array([-math.sin(ph) * math.cos(t1), -math.sin(ph) * math.sin(t1), math.cos(ph)])
        right = np.array([math.sin(t1), -math.cos(t1), 0.0])
        cr, sr = math.cos(self.cam_roll), math.sin(self.cam_roll)
        right2, up2 = cr * right + sr * up, -sr * right + cr * up
        v = np.asarray(point, float) - cam
        z = float(v @ ax)
        if z < 20:
            return None
        return (self.W / 2 + self.fpx * float(v @ right2) / z, self.H / 2 - self.fpx * float(v @ up2) / z, z)

    def true_pixel(self, delayed=False):
        y = self.y
        if delayed and self.hist:
            tq = self.t - self.latency
            for th, yh in reversed(self.hist):
                y = yh
                if th <= tq:
                    break
        return self._pixel(y, self.target)

    def _blob(self, img_hsv, center, half, hsv, spin_deg):
        a = math.radians(spin_deg)
        pts = [(center[0] + half * (math.cos(a) * sx - math.sin(a) * sy),
                center[1] + half * (math.sin(a) * sx + math.cos(a) * sy)) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        cv2.fillConvexPoly(img_hsv, np.array(pts, np.int32), tuple(int(v) for v in hsv), lineType=cv2.LINE_AA)

    def render(self):
        with self.lock:
            hsv = np.empty((self.H, self.W, 3), np.uint8)
            hsv[..., 0] = 100; hsv[..., 1] = 25
            hsv[..., 2] = np.clip(120 + 18 * self._bg, 0, 255).astype(np.uint8)
            p = self.true_pixel(delayed=True)
            if self.clutter and p:                 # a smaller block of the same color, and one of another color
                q = self._pixel(self.hist[-1][1] if self.hist else self.y, self.target + np.array([0, -160.0, 60.0]))
                r = self._pixel(self.hist[-1][1] if self.hist else self.y, self.target + np.array([0, 130.0, -70.0]))
                if q: self._blob(hsv, q[:2], 0.25 * self.fpx * self.block_mm / q[2], self.color_hsv, 0)
                if r: self._blob(hsv, r[:2], 0.6 * self.fpx * self.block_mm / r[2], ((int(self.color_hsv[0]) + 90) % 180, 200, 180), 10)
            if p:
                half = 0.5 * self.fpx * self.block_mm / p[2]
                self._blob(hsv, p[:2], half, self.color_hsv, self.block_spin)
                if self.glare:                     # a washed-out strip across one side of the block
                    a = math.radians(self.block_spin)
                    c2 = (p[0] + 0.55 * half * math.cos(a), p[1] + 0.55 * half * math.sin(a))
                    pts = [(c2[0] + half * (math.cos(a) * sx * 0.45 - math.sin(a) * sy),
                            c2[1] + half * (math.sin(a) * sx * 0.45 + math.cos(a) * sy)) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                    cv2.fillConvexPoly(hsv, np.array(pts, np.int32), (int(self.color_hsv[0]), 40, 250), lineType=cv2.LINE_AA)
            bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(np.int16)
            bgr += self.rng.normal(0, self.noise, bgr.shape).astype(np.int16)
            return np.clip(bgr, 0, 255).astype(np.uint8)


class SimTransport:
    def __init__(self, world: SimWorld):
        self.w = world

    def write_line(self, s: str):
        self.w.feed(s)

    def read_line(self, timeout: float = 0.0):
        with self.w.lock:
            self.w.sync()
            if self.w.lines:
                return self.w.lines.popleft()
        if self.w.realtime and timeout > 0:
            time.sleep(min(timeout, 0.01))
        return None

    def close(self):
        pass


class SimCamera:
    """Same read() as vision.Camera. With realtime=False each read advances time by one frame."""

    def __init__(self, world: SimWorld):
        self.w = world
        self.n = 0
        self.width, self.height = world.W, world.H
        self.name = "simulated camera"
        self.fps_measured = world.fps
        self.reopens = 0
        self.brightness = float("nan")
        self.events = []
        self.cap = None

    def read(self, timeout: float = 1.0):
        if self.w.realtime:
            time.sleep(1.0 / self.w.fps)
            self.w.sync()
        else:
            self.w.advance(1.0 / self.w.fps)
        self.n += 1
        img = self.w.render()
        self.brightness = float(img[::8, ::8].mean())
        return img, self.w.t, self.n

    def format(self):
        return "SIM"

    def status(self):
        return f"{self.name}: {self.width} x {self.height} | {self.fps_measured:.0f} frames/s | brightness {self.brightness:.0f}"

    def reopen(self):
        self.reopens += 1

    def settings_dialog(self):
        print("the simulated camera has no settings dialog")

    def close(self):
        pass
