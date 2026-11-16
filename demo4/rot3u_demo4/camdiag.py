"""Camera diagnostics for fussy USB cameras: which camera is which, which modes really work, and
whether the picture stays good over time.

    cameras()                     list the cameras the system reports (names from the color picker's helper)
    probe(index, ...)             try sizes and formats; measure the real size, format, frame rate and black frames
    recommend(rows, min_fps)      the best mode from a probe
    watch(cam, seconds)           brightness and frame rate once a second; counts black frames and reopens
    properties(cam)               exposure, gain, white balance ... as the driver reports them
    compare_properties(cam, prof) differences from the settings saved with an object profile
    TROUBLESHOOTING               what to try, in order

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import sys, time
import numpy as np
import cv2
from . import vision

SIZES = [(320, 240), (640, 480), (800, 600), (1280, 720)]
FORMATS = [None, "MJPG", "YUY2"]

PROPS = {"exposure": cv2.CAP_PROP_EXPOSURE, "auto exposure": cv2.CAP_PROP_AUTO_EXPOSURE, "gain": cv2.CAP_PROP_GAIN,
         "brightness": cv2.CAP_PROP_BRIGHTNESS, "contrast": cv2.CAP_PROP_CONTRAST, "saturation": cv2.CAP_PROP_SATURATION,
         "white balance (K)": cv2.CAP_PROP_WB_TEMPERATURE, "auto white balance": cv2.CAP_PROP_AUTO_WB,
         "sharpness": cv2.CAP_PROP_SHARPNESS, "backlight comp.": cv2.CAP_PROP_BACKLIGHT,
         "focus": cv2.CAP_PROP_FOCUS, "autofocus": cv2.CAP_PROP_AUTOFOCUS}

TROUBLESHOOTING = """\
Camera troubleshooting, in order:
 1. Only one program can use a camera. Stop the color picker (picker.close() in its cell) and close
    the Windows Camera app, Teams, Zoom and browser tabs that use the camera.
 2. Wrong camera: indexes change when cameras are plugged in or out. Run cameras() and pick by name.
 3. Black frames while frames keep arriving: cam.reopen() (the watchdog also does this after
    20 black frames). If it keeps happening, unplug the camera, wait 5 s, plug it back in, and run
    the camera cell again.
 4. Stripes, garbage or black only at one size: that size/format pair does not work on this camera.
    Run probe() and use a mode with 0% black frames; leave the format as None (driver default).
 5. Slow (under 10 frames/s): a smaller size is faster on the HBVCAM (YUY2 at 1280 x 720 gave
    about 7.5 frames/s, 640 x 480 about 9). Automatic exposure also slows the camera down in dim light:
    more light helps more than any setting.
 6. Colors change while tracking: automatic white balance or exposure is on. cam.settings_dialog()
    opens the Windows dialog: turn both to manual at the values the color picker used.
 7. Nothing works: try another USB port (not a hub), then reboot the laptop.
"""


def cameras(show=True):
    """Cameras the system reports, as [{index, name, vid, pid}]. Uses the color picker's helper when installed."""
    rows = []
    try:
        from camera_color_picker import camera as cpc
        for e in cpc.list_cameras():
            rows.append({"index": e.index, "name": e.name, "vid": e.vid, "pid": e.pid})
    except Exception:
        rows = vision.list_cameras()
    if show:
        if not rows:
            print("no cameras found (is the camera plugged in? is another program using it?)")
        for r in rows:
            vp = f"  ({r['vid']:04X}:{r['pid']:04X})" if r.get("vid") and r.get("pid") else ""
            print(f"  index {r['index']}: {r.get('name', '?')}{vp}")
    return rows


def _measure(cam, seconds):
    """Read frames for `seconds`: (frames, black frames, mean brightness, frames/s, stripe score)."""
    t0 = time.perf_counter(); n = black = 0; br = []; stripe = []
    first = None
    while time.perf_counter() - t0 < seconds or first is None:
        try:                                       # the first frame of a large mode can take several seconds
            f, t, _ = cam.read(timeout=1.5 if first is not None else 6.0)
        except TimeoutError:
            break
        if first is None:
            first = t; t0 = time.perf_counter()        # measure from the first frame, not from opening
        first = t if first is None else first
        n += 1
        m = float(f[::8, ::8].mean()); br.append(m)
        black += m < 3.0
        rows = f[::4, ::8].mean(axis=(1, 2))                 # garbled frames show strong row-to-row jumps
        stripe.append(float(np.median(np.abs(np.diff(rows)))))
        last = t
    fps = (n - 1) / (last - first) if n >= 2 and last > first else 0.0
    return n, black, (float(np.mean(br)) if br else float("nan")), fps, (float(np.median(stripe)) if stripe else float("nan"))


def probe(index, sizes=SIZES, formats=FORMATS, seconds=2.0, orientation=(0, False, False), show=True):
    """Open the camera in each size and format; report what the driver really delivered.

    Takes about (seconds + 1.5) x len(sizes) x len(formats): 3.5 s x 12 = about 40 s by default (longer when a
    mode is slow to start). The frame rate is often set by the exposure, not the size: in dim light every size
    gives the same rate, and more light is what makes the camera faster.
    A row is good when black is 0% and stripes is small (under about 8). Close other camera users first."""
    rows = []
    for (w, h) in sizes:
        for fmt in formats:
            row = {"ask": f"{w}x{h} {fmt or 'default'}", "w": w, "h": h, "fmt_asked": fmt}
            try:
                cam = vision.Camera(index, w, h, 30, fmt, orientation, warmup=5, watchdog=False)
            except RuntimeError as e:
                row.update(ok=False, note=str(e)); rows.append(row)
                continue
            try:
                n, black, br, fps, stripe = _measure(cam, seconds)
                row.update(got=f"{cam.width}x{cam.height}", fmt=cam.format(), frames=n, fps=round(fps, 1),
                           black_pct=round(100 * black / n, 0) if n else 100.0, brightness=round(br, 1),
                           stripes=round(stripe, 1),
                           ok=bool(n >= 3 and black == 0 and (cam.width, cam.height) == (w, h) and not stripe > 8))
                row["note"] = "" if row["ok"] else ("no frames" if n < 3 else "black frames" if black else
                                                     "size not kept" if (cam.width, cam.height) != (w, h) else "stripes")
            finally:
                cam.close()
                time.sleep(0.5)
            rows.append(row)
            if show:
                print(f"  ask {row['ask']:<22} got {row.get('got', '-'):>9} {row.get('fmt', '-'):>4}  "
                      f"{row.get('fps', 0):5.1f} frames/s  black {row.get('black_pct', 100):3.0f}%  "
                      f"brightness {row.get('brightness', float('nan')):5.1f}  stripes {row.get('stripes', float('nan')):4.1f}  "
                      f"{'OK' if row.get('ok') else 'BAD: ' + row.get('note', '')}")
    return rows


def recommend(rows, min_fps=12.0, prefer=(640, 480), tie=0.10):
    """Best good mode. Modes within `tie` (10%) of the fastest good mode count as equally fast.
    Among the modes that reach min_fps, or else among the equally fast ones: `prefer` if it is there,
    otherwise the largest. (When exposure sets the frame rate, every size is about equally fast:
    then the preferred size wins rather than the smallest.)"""
    good = [r for r in rows if r.get("ok")]
    if not good:
        return None
    pool = [r for r in good if r["fps"] >= min_fps]
    if not pool:
        top = max(r["fps"] for r in good)
        pool = [r for r in good if r["fps"] >= (1 - tie) * top]
    pick = [r for r in pool if (r["w"], r["h"]) == tuple(prefer)]
    if pick:
        return max(pick, key=lambda r: (r["fmt_asked"] is None, r["fps"]))
    return max(pool, key=lambda r: (r["w"] * r["h"], r["fps"]))


def watch(cam, seconds=30.0, every=1.0, show=True):
    """Brightness and frame rate every `every` seconds for `seconds`. Arm still, block in view.

    Steady brightness = good. A drop to 0 = the black-frame fault (the watchdog reopens the camera).
    Slow drift (say 106 to 96) = automatic exposure working; fine unless the color check fails."""
    if cam is None:
        raise RuntimeError("no camera open: run A.4 first (the probe in A.5b closes the camera)")
    out = []
    t0 = time.perf_counter(); r0 = cam.reopens
    while time.perf_counter() - t0 < seconds:
        t1 = time.perf_counter(); n = black = 0; br = []
        while time.perf_counter() - t1 < every:
            try:
                f, _, _ = cam.read(timeout=1.5)
            except TimeoutError:
                break
            n += 1; m = float(f[::8, ::8].mean()); br.append(m); black += m < 3.0
        row = {"t": round(time.perf_counter() - t0, 1), "frames": n, "fps": round(n / every, 1),
               "brightness": round(float(np.mean(br)), 1) if br else float("nan"), "black": black,
               "reopens": cam.reopens - r0}
        out.append(row)
        if show:
            print(f"  t {row['t']:5.1f} s  {row['fps']:5.1f} frames/s  brightness {row['brightness']:6.1f}  "
                  f"black {black}  reopened {row['reopens']}")
    return out


def properties(cam, show=True):
    """What the driver reports for exposure, gain, white balance and so on (None = not supported)."""
    out = {}
    for name, pid in PROPS.items():
        try:
            v = cam.cap.get(pid)
        except Exception:
            v = None
        out[name] = None if v is None or v == -1 else float(v)
    if show:
        print("  " + ", ".join(f"{k} {v:g}" for k, v in out.items() if v is not None))
    return out


def compare_properties(cam, saved: dict, show=True):
    """Settings that differ from the ones saved with the color captures (they change the colors)."""
    now = properties(cam, show=False)
    diff = {k: (saved.get(k), now.get(k)) for k in PROPS
            if saved.get(k) is not None and now.get(k) is not None and abs(float(saved[k]) - float(now[k])) > 1e-6}
    if show:
        if diff:
            print("camera settings differ from when the colors were captured (colors may not match):")
            for k, (a, b) in diff.items():
                print(f"  {k}: captured {a:g}, now {b:g}")
        else:
            print("camera settings match the ones saved with the colors (where the driver reports them)")
    return diff


def test_card(cam, width=640):
    """One frame with a center crosshair and thirds grid: is it upright, focused, the right camera?"""
    f, _, _ = cam.read()
    H, W = f.shape[:2]
    for i in (1, 2):
        cv2.line(f, (W * i // 3, 0), (W * i // 3, H), (200, 200, 200), 1)
        cv2.line(f, (0, H * i // 3), (W, H * i // 3), (200, 200, 200), 1)
    cv2.drawMarker(f, (W // 2, H // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
    cv2.putText(f, "UP", (W // 2 - 18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(f, cam.status(), (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    vision.show_inline(f, width)
    return f
