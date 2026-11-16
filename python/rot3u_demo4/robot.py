"""The robot's numbers: pulse <-> angle calibration, pulse limits, gripper pulses, link lengths, poses.
The sketch only knows pulses.

The numbers come from the robot's robot_config.h, the file servo_tester's EXPORT fills in
(robots/<name>/robot_config.h; format: robots/template/robot_config.h). load_config(path) reads it.
Until then the module holds Sir Pinch-a-lot's numbers (calibration of 2026-10-11).

Angle conventions are the ROT3U DH conventions (docs/rot3u-conventions.md): theta1 base, theta2 the upper
arm's angle above horizontal, theta3 and theta4* relative to the link before, theta5 roll.
The camera adds weight at the wrist, so treat the angles as approximate. Tracking does not depend on
them: it uses the image Jacobian measured with the camera (control.measure_jacobian).

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import os, re, time

NUM_CH = 6
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist", "roll", "gripper"]
# pulse = US0[j] + USPD[j] * angle_deg   (joints 0..4)
ROBOT_NAME = "Sir Pinch-a-lot (ROT3U arm A, Rev B)"
CONFIG_PATH = None                                  # None = the built-in Sir Pinch-a-lot numbers below
US0 = [1475.6, 408.8, 332.1, 519.5, 500.0]
USPD = [10.901, 10.625, -11.587, -11.111, 11.111]
MIN_US = [530, 530, 530, 530, 500, 1124]
MAX_US = [2470, 2470, 2020, 1470, 2500, 1920]     # ch 3: the wrist brackets collide above 1500
GRIP_OPEN_US, GRIP_HOLD_US = 1150, 1580
D1, R2, R3, D5 = 113.0, 104.0, 97.0, 170.0        # mm; D5 = wrist axis to the grasp surface (jaw tips - 4 mm)
PAD_MM = 4.0                                       # jaw tips to the grasp surface (rubber pads)

# Named poses: joint angles in degrees (base, shoulder, elbow, wrist, roll) and the gripper pulse.
# WALL: upper arm vertical, forearm 45 deg up, gripper level: the camera looks straight ahead.
# TABLE: the ik_demo check pose, gripper 10 deg outward of straight down: the camera looks at the table.
# Roll 0 puts the gripper servo, and the camera on its bracket (gripper_camera_bracket rev 2.1), on top
# of the gripper: the picture is upright.
POSES = {
    "WALL":  ([0.0, 90.0, -45.0, -45.0, 0.0], None),
    "TABLE": ([0.0, 70.0, -70.0, -80.0, 0.0], None),
}


# ============================================================================ robot_config.h
def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return "\n".join(l.split("//", 1)[0] for l in text.splitlines())


def parse_config(path):
    """Read a robot_config.h (or a pasted servo_tester EXPORT block). Returns a dict; raises ValueError
    with a plain explanation when something needed is missing or does not fit this tool."""
    with open(path, encoding="utf-8", errors="replace") as f:
        raw = f.read()
    text = _strip_comments(raw)
    num = r"([-+]?\d+(?:\.\d*)?)"
    rows = re.findall(r"\{\s*(\d+)\s*,\s*J_(ROT|LIN)\s*,\s*" + r"\s*,\s*".join([num] * 2) + r"\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*"
                      + num + r"\s*,\s*(\d)\s*,\s*\"([^\"]*)\"\s*\}", text)
    g = re.search(r"GripCal\s+GRIP\s*=\s*\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}", text)
    geo = {k: float(v) for k, v in re.findall(r"const\s+float\s+(ARM_\w+)\s*=\s*" + num, text)}
    name = re.search(r'#define\s+ROBOT_NAME\s+"([^"]*)"', text)
    valid = re.search(r"#define\s+CAL_VALID\s+(true|false)", text)
    where = os.path.basename(os.path.dirname(os.path.abspath(path))) + "/" + os.path.basename(path)
    if len(rows) < 5:
        raise ValueError(f"{where}: found {len(rows)} joint rows, need 5 (base, shoulder, elbow, wrist, roll). "
                         "Map every joint with servo_tester and paste its EXPORT block (docs/GETTING-STARTED.md, part 4)")
    rows = rows[:5]
    for i, r in enumerate(rows):
        if int(r[0]) != i:
            raise ValueError(f"{where}: row {i + 1} is channel {r[0]}; this tool needs the joints on PCA9685 channels 0-4 in "
                             "kinematic order (base 0, shoulder 1, elbow 2, wrist 3, roll 4, gripper 5). Rewire, or tell Tico")
        if r[1] != "ROT":
            raise ValueError(f"{where}: channel {i} is J_{r[1]}; all five joints must be rotary (J_ROT)")
        if not (int(r[4]) < int(r[5])):
            raise ValueError(f"{where}: channel {i} has MIN {r[4]} >= MAX {r[5]}")
        if not (6.0 <= abs(float(r[3])) <= 16.0):
            raise ValueError(f"{where}: channel {i} USPD {r[3]} us/deg is not a hobby-servo slope (about 10-11.5): check the fit")
    if not g:
        raise ValueError(f"{where}: no 'const GripCal GRIP = {{ ch, open, hold, min, max }};' line (map the gripper: JT GRIP, SETP OPEN, SETP HOLD)")
    if int(g.group(1)) != 5:
        raise ValueError(f"{where}: the gripper is on channel {g.group(1)}; this tool needs it on channel 5")
    for k in ("ARM_D1", "ARM_R2", "ARM_R3", "ARM_TIP"):
        if k not in geo:
            raise ValueError(f"{where}: missing 'const float {k} = ...;' (measure the arm: docs/rot3u-conventions.md)")
    return {
        "path": os.path.abspath(path), "name": name.group(1) if name else where, "cal_valid": bool(valid and valid.group(1) == "true"),
        "us0": [float(r[2]) for r in rows], "uspd": [float(r[3]) for r in rows],
        "min": [int(r[4]) for r in rows] + [int(g.group(4))], "max": [int(r[5]) for r in rows] + [int(g.group(5))],
        "names": [r[7] for r in rows] + ["gripper"], "grip_open": int(g.group(2)), "grip_hold": int(g.group(3)),
        "d1": geo["ARM_D1"], "r2": geo["ARM_R2"], "r3": geo["ARM_R3"], "tip": geo["ARM_TIP"],
    }


def load_config(path, allow_placeholder=False):
    """Use the numbers of this robot_config.h from now on (every module reads them from here).
    Returns the parsed dict. A config with CAL_VALID false (the template) is refused unless allow_placeholder."""
    global ROBOT_NAME, CONFIG_PATH, US0, USPD, MIN_US, MAX_US, GRIP_OPEN_US, GRIP_HOLD_US, D1, R2, R3, D5
    c = parse_config(path)
    if not c["cal_valid"] and not allow_placeholder:
        raise ValueError(f"{c['name']}: CAL_VALID is false in {path}. That is the template's placeholder state: "
                         "paste your servo_tester EXPORT block and measured ARM_* lengths, then set CAL_VALID to true")
    ROBOT_NAME, CONFIG_PATH = c["name"], c["path"]
    US0[:], USPD[:] = c["us0"], c["uspd"]
    MIN_US[:], MAX_US[:] = c["min"], c["max"]
    GRIP_OPEN_US, GRIP_HOLD_US = c["grip_open"], c["grip_hold"]
    D1, R2, R3, D5 = c["d1"], c["r2"], c["r3"], c["tip"] - PAD_MM
    return c


def describe():
    """The robot's numbers, one line per joint."""
    lines = [f"robot: {ROBOT_NAME}" + (f"  ({CONFIG_PATH})" if CONFIG_PATH else "  (built-in numbers)"),
             f"links: d1 {D1:g}, r2 {R2:g}, r3 {R3:g}, d5 (to the grasp surface) {D5:g} mm"]
    for j in range(5):
        lines.append(f"  ch {j} {JOINT_NAMES[j]:<8} pulse = {US0[j]:7.1f} + {USPD[j]:+7.3f} x deg   limits {MIN_US[j]}-{MAX_US[j]} us "
                     f"= {us_to_deg(j, MIN_US[j]):+.0f} to {us_to_deg(j, MAX_US[j]):+.0f} deg")
    lines.append(f"  ch 5 gripper  open {GRIP_OPEN_US}, hold {GRIP_HOLD_US}, limits {MIN_US[5]}-{MAX_US[5]} us")
    return "\n".join(lines)


def apply_limits(link, say=print):
    """Send this robot's pulse limits to the sketch (LIM per channel). The sketch itself only knows the
    servo's absolute range (500-2500 us), so this must run after every connect and before anything moves."""
    for c in range(NUM_CH):
        r = link.cmd(f"LIM {c} {MIN_US[c]} {MAX_US[c]}")
        if not r.ok:
            raise RuntimeError(f"the sketch refused the limits of ch {c} ({MIN_US[c]}-{MAX_US[c]}): " + " | ".join(r.lines))
    link.limits_applied = True
    say(f"pulse limits of {ROBOT_NAME} sent to the sketch")


def deg_to_us(joint: int, deg: float) -> float:
    return US0[joint] + USPD[joint] * deg


def us_to_deg(joint: int, us: float) -> float:
    return (us - US0[joint]) / USPD[joint]


def pose_us(name_or_deg, grip_us=None) -> list:
    """Pulses for channels 0..5 from a pose name or a list of 5 angles. Raises if outside the limits."""
    if isinstance(name_or_deg, str):
        deg, g = POSES[name_or_deg]
        grip_us = g if grip_us is None else grip_us
    else:
        deg = list(name_or_deg)
    us = [deg_to_us(j, d) for j, d in enumerate(deg)] + [GRIP_HOLD_US if grip_us is None else grip_us]
    for c, u in enumerate(us):
        if not (MIN_US[c] <= u <= MAX_US[c]):
            raise ValueError(f"{JOINT_NAMES[c]} (ch {c}) would need {u:.0f} us, outside its limits {MIN_US[c]}-{MAX_US[c]} us "
                             f"(robot_config.h). Choose another POSE, or check that joint's mapping")
    return [round(u) for u in us]


def wait_done(link, sleep=time.sleep, timeout=30.0, poll=0.05):
    """Wait until the sketch reports that nothing is moving. Returns the final state."""
    t = 0.0
    st = link.get_state()
    while st.moving and t < timeout:
        sleep(poll)
        t += poll
        st = link.get_state()
    return st


def goto(link, target_us, v=200.0, a=600.0, sleep=time.sleep, wait=True):
    """Smooth move of every started channel to target_us (list of 6; None leaves a channel alone)."""
    st = link.get_state()
    tg = [u if (u is not None and st.is_on(c)) else None for c, u in enumerate(target_us)]
    link.cmd(f"VA {v:.0f} {a:.0f}")
    r = link.cmd(link.targets_text(tg))
    if not r.ok:
        raise RuntimeError("target refused: " + " | ".join(r.lines))
    return wait_done(link, sleep) if wait else link.get_state()


def power_up(link, target_us, pause=0.5, sleep=time.sleep, say=print):
    """Start the channels that are off, base outward, one at a time, then move the rest smoothly.

    HOLD THE ARM near the pose first: a channel that is off jumps to its pulse at full servo speed.
    Channels already holding (adopted after a reset) are not jumped: they are moved with goto().
    The robot's pulse limits are sent first (apply_limits)."""
    if not getattr(link, "limits_applied", False):
        apply_limits(link, say=say)
    st = link.get_state()
    for c in range(NUM_CH):
        if st.is_on(c):
            continue
        r = link.cmd(f"ON {c} {target_us[c]}")
        say(f"ch {c} ({JOINT_NAMES[c]}): " + ("started at %d us" % target_us[c] if r.ok else "REFUSED " + " | ".join(r.lines)))
        sleep(pause)
    return goto(link, target_us, sleep=sleep)
