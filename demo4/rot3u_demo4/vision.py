"""Vision for Robot Demo 4: camera, color ranges, two ways to find the block, and the display.

Method A, "blob" (the one taught in class; the handout's MATLAB code):
    threshold in HSV -> connected components (bwconncomp) -> areas and centroids (regionprops)
    -> the largest component (max(areas)) -> aim at its centroid.
Method B, "edges" (alternative): the color range only decides which region is eligible; then Canny
    edges near that region plus the region's outline -> convex hull -> the smallest rectangle around
    it (minAreaRect) -> aim at the rectangle's center. Also gives the block's size and angle.

HSV units here are OpenCV's: H 0-179 (degrees / 2), S and V 0-255. MATLAB's rgb2hsv gives 0-1 for all
three: H_opencv = H_matlab * 180, S and V * 255.
Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import json, math, os, sys, threading, time
from dataclasses import dataclass, field
import numpy as np
import cv2


# ============================================================================ color ranges
@dataclass
class ColorRange:
    """One named color = one or two HSV boxes (two when the hue wraps through red, 179 -> 0)."""
    name: str
    ranges: list                       # [((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi)), ...]

    @staticmethod
    def from_hsv(name, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi):
        """h_lo > h_hi means the hue range wraps through 0 (red): two boxes."""
        c = lambda v, top: int(min(top, max(0, round(v))))
        h_lo, h_hi = c(h_lo, 179), c(h_hi, 179)
        s_lo, s_hi, v_lo, v_hi = c(s_lo, 255), c(s_hi, 255), c(v_lo, 255), c(v_hi, 255)
        if h_lo <= h_hi:
            return ColorRange(name, [((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))])
        return ColorRange(name, [((h_lo, s_lo, v_lo), (179, s_hi, v_hi)), ((0, s_lo, v_lo), (h_hi, s_hi, v_hi))])

    @staticmethod
    def from_matlab(name, h_min, h_max, s_min, s_max, v_min, v_max):
        """colorThresholder numbers (all 0-1). h_min > h_max is MATLAB's (H >= min) | (H <= max)."""
        return ColorRange.from_hsv(name, h_min * 180, h_max * 180, s_min * 255, s_max * 255, v_min * 255, v_max * 255)

    @staticmethod
    def from_picker_json(paths, name=None, k=(2.5, 2.5, 3.0)):
        """Range = mean +/- k*sigma from one or more color-picker captures (see ColorModel)."""
        return ColorModel.from_picker_json(paths, name, k).range()

    def widen(self, dh=0, ds=0, dv=0):
        out = [((max(0, lo[0] - dh), max(0, lo[1] - ds), max(0, lo[2] - dv)),
                (min(179, hi[0] + dh), min(255, hi[1] + ds), min(255, hi[2] + dv))) for lo, hi in self.ranges]
        return ColorRange(self.name, out)

    def to_dict(self):
        return {"name": self.name, "ranges": [[list(lo), list(hi)] for lo, hi in self.ranges]}

    @staticmethod
    def from_dict(d):
        return ColorRange(d["name"], [(tuple(lo), tuple(hi)) for lo, hi in d["ranges"]])

    def __str__(self):
        return self.name + ": " + " OR ".join(f"H {lo[0]}-{hi[0]} S {lo[1]}-{hi[1]} V {lo[2]}-{hi[2]}" for lo, hi in self.ranges)


def _hue_diff(h, m):
    """Signed hue difference in OpenCV units (-90 .. 90): hue is an angle, 180 units per turn."""
    return (h - m + 90.0) % 180.0 - 90.0


@dataclass
class ColorModel:
    """A block color as mean and standard deviation per HSV channel; the range is mean +/- k*sigma.

    Hue is circular (OpenCV units, 0-179). k is set per channel by watching the mask (ThresholdTuner)."""
    name: str
    mean: tuple                        # (H, S, V)
    std: tuple                         # (sH, sS, sV)
    k: tuple = (2.5, 2.5, 3.0)
    n: int = 0                         # sample points behind the statistics
    sources: list = field(default_factory=list)

    def range(self, k=None) -> ColorRange:
        kh, ks, kv = self.k if k is None else k
        (h, s_, v), (sh, ss, sv) = self.mean, self.std
        if kh * sh >= 89.5:            # more than the whole circle: every hue
            h_lo, h_hi = 0, 179
        else:
            h_lo, h_hi = round(h - kh * sh) % 180, round(h + kh * sh) % 180
        return ColorRange.from_hsv(self.name, h_lo, h_hi, s_ - ks * ss, s_ + ks * ss, v - kv * sv, v + kv * sv)

    def to_dict(self):
        return {"name": self.name, "mean": list(self.mean), "std": list(self.std), "k": list(self.k),
                "n": self.n, "sources": self.sources}

    @staticmethod
    def from_dict(d):
        return ColorModel(d["name"], tuple(d["mean"]), tuple(d["std"]), tuple(d.get("k", (2.5, 2.5, 3.0))),
                          d.get("n", 0), list(d.get("sources", [])))

    @staticmethod
    def from_picker_json(paths, name=None, k=(2.5, 2.5, 3.0)):
        """Pool one or more color-picker captures (Cluster mode) into one model.

        Each file gives n, and mean and std for H, S and V (stats.rows). Pooling: weighted mean, and
        variance = weighted mean of (std^2 + (mean_i - pooled mean)^2). For hue the pooled mean is the
        weighted circular mean: class Atan2(x = sum w cos a, y = sum w sin a), Python math.atan2(y, x),
        with a = 2 h in degrees; differences are taken around the circle."""
        if isinstance(paths, (str, os.PathLike)):
            paths = [paths]
        rows, ns = [], []
        for p in paths:
            with open(p) as f:
                data = json.load(f)
            r = _find_key(data, "rows")
            if not (isinstance(r, dict) and all(c in r for c in "HSV") and "std" in r["H"]):
                raise ValueError(f"{p}: no cluster statistics (stats.rows with mean and std). Save the capture in Cluster mode")
            st = _find_key(data, "stats") or {}
            samp = _find_key(data, "sampling") or {}
            if isinstance(samp, dict) and samp.get("exclude"):
                print(f"note: {os.path.basename(str(p))} was saved with 'Exclude outliers' on: its std is smaller than the real spread")
            rows.append(r)
            ns.append(float(st.get("kept") or st.get("n") or 1) if isinstance(st, dict) else 1.0)
        w = np.array(ns) / sum(ns)
        a = np.radians([2.0 * float(r["H"]["mean"]) for r in rows])
        hm = (math.degrees(math.atan2(float(w @ np.sin(a)), float(w @ np.cos(a)))) / 2.0) % 180.0
        dh = np.array([_hue_diff(float(r["H"]["mean"]), hm) for r in rows])
        sh = math.sqrt(float(w @ (np.array([float(r["H"]["std"]) for r in rows]) ** 2 + dh ** 2)))
        mean, std = [hm], [sh]
        for c in "SV":
            m = np.array([float(r[c]["mean"]) for r in rows]); sd = np.array([float(r[c]["std"]) for r in rows])
            mc = float(w @ m)
            mean.append(mc); std.append(math.sqrt(float(w @ (sd ** 2 + (m - mc) ** 2))))
        return ColorModel(name or os.path.basename(str(paths[0])), tuple(round(v, 2) for v in mean),
                          tuple(round(max(v, 1.0), 2) for v in std), tuple(k), int(sum(ns)),
                          [os.path.basename(str(p)) for p in paths])

    def __str__(self):
        (h, s_, v), (sh, ss, sv) = self.mean, self.std
        return (f"{self.name}: H {h:.1f}±{sh:.1f} S {s_:.0f}±{ss:.0f} V {v:.0f}±{sv:.0f}, "
                f"k = {self.k[0]:g}, {self.k[1]:g}, {self.k[2]:g} -> {str(self.range()).split(': ', 1)[1]}")


def _find_key(obj, key):
    if isinstance(obj, dict):
        if key in obj and obj[key]:
            return obj[key]
        for v in obj.values():
            r = _find_key(v, key)
            if r:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find_key(v, key)
            if r:
                return r
    return None


# Starting guesses only. Replace each one with a color-picker capture taken under the demo's lighting.
PRESETS = {
    "red":    ColorRange.from_hsv("red", 170, 10, 110, 255, 60, 255),
    "yellow": ColorRange.from_hsv("yellow", 20, 35, 100, 255, 90, 255),
    "green":  ColorRange.from_hsv("green", 40, 85, 80, 255, 50, 255),
    "wood":   ColorRange.from_hsv("wood", 10, 28, 40, 170, 90, 255),   # low saturation: expect trouble on beige backgrounds
    # The handout's MATLAB numbers: H >= 0.694 or <= 0.000, S 0.059-1.000, V 0.141-0.882 (reaches from blue-purple to red).
    "handout": ColorRange.from_matlab("handout", 0.694, 0.000, 0.059, 1.000, 0.141, 0.882),
}


def as_range(c) -> "ColorRange":
    return c.range() if isinstance(c, ColorModel) else c


def save_colors(path, colors: dict):
    """colors.json: a ColorModel is saved with its mean, std and k; a plain ColorRange as its boxes."""
    with open(path, "w") as f:
        json.dump({k: v.to_dict() for k, v in colors.items()}, f, indent=1)


def load_colors(path) -> dict:
    with open(path) as f:
        d = json.load(f)
    return {k: (ColorModel.from_dict(v) if "mean" in v else ColorRange.from_dict(v)) for k, v in d.items()}


# ================================================================================= detection
@dataclass
class Detection:
    found: bool = False
    cx: float = float("nan")
    cy: float = float("nan")
    area: float = 0.0                  # pixels in the blob (A) or inside the rectangle (B)
    bbox: tuple = (0, 0, 0, 0)         # x, y, w, h
    angle_deg: float = float("nan")    # A: major axis from the second moments. B: rectangle's long side. Image axes, y down
    corners: object = None             # B: 4x2 array of the rectangle's corners
    size_px: tuple = (0.0, 0.0)        # B: rectangle sides, long then short
    n_blobs: int = 0                   # components above min_area
    method: str = ""
    note: str = ""


def make_mask(bgr, color: ColorRange, blur=5, open_k=5, close_k=7):
    """Thresholded image (255 = inside the color range), cleaned with an opening then a closing."""
    color = as_range(color)
    src = cv2.GaussianBlur(bgr, (blur, blur), 0) if blur and blur > 1 else bgr
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in color.ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    if open_k and open_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k)))
    if close_k and close_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k)))
    return mask


def detect_blob(mask, min_area=150) -> Detection:
    """Method A (class method): largest connected component, its centroid.

    MATLAB                               here
    CC = bwconncomp(BW)                  connectedComponentsWithStats(mask, connectivity=8)
    s = regionprops(CC,'Centroid','Area')  stats[:, CC_STAT_AREA], centroids
    [m, ind] = max(areas)                argmax over the components (label 0 is the background)
    """
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return Detection(method="blob")
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = 1 + int(np.argmax(areas))
    if areas[i - 1] < min_area:
        return Detection(method="blob")
    x, y, w, h = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    m = cv2.moments((labels[y:y + h, x:x + w] == i).astype(np.uint8), binaryImage=True)
    # Orientation of the major axis from the central second moments:
    #   theta = 1/2 * Atan2(x = mu20 - mu02, y = 2 mu11)   (class argument order: x first)
    #   Python: 0.5 * math.atan2(2 * mu11, mu20 - mu02)    (y first)
    ang = 0.5 * math.degrees(math.atan2(2 * m["mu11"], m["mu20"] - m["mu02"]))
    return Detection(True, float(cents[i, 0]), float(cents[i, 1]), float(areas[i - 1]), (x, y, w, h), ang,
                     n_blobs=int((areas >= min_area).sum()), method="blob")


def detect_edges(bgr, mask, min_area=150, pad_frac=0.5, max_grow=2.5) -> Detection:
    """Method B: the color mask picks the eligible region; edges outline the whole block.

    1. largest outline (contour) of the color mask = the candidate
    2. Canny edges in a padded box around it
    3. the candidate plus every edge connected to it = one region; its convex hull
    4. smallest rectangle around the hull (minAreaRect): its center is the aim point
    A part of the block the color test missed (glare, shadow) is recovered if its border shows as an
    edge. If the edges run off into the background (the region reaches the box border or grows past
    max_grow times the candidate), the rectangle of the candidate alone is used; note says which."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    big = [c for c in cnts if cv2.contourArea(c) >= min_area]
    if not big:
        return Detection(method="edges")
    c = max(big, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    rect = cv2.minAreaRect(c)
    note = "color outline only"
    pad = int(pad_frac * max(w, h)) + 6
    H, W = mask.shape[:2]
    x0, y0, x1, y1 = max(0, x - pad), max(0, y - pad), min(W, x + w + pad), min(H, y + h + pad)
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), (5, 5), 0)
    med = float(np.median(gray))
    edges = cv2.Canny(gray, int(max(10, 0.5 * med)), int(max(30, 1.2 * med)))
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    cand = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.drawContours(cand, [c - np.array([[x0, y0]])], -1, 255, -1)
    n, labels = cv2.connectedComponents(cv2.bitwise_or(edges, cand), connectivity=8)
    lab = int(np.bincount(labels[cand > 0]).argmax())
    region = (labels == lab).astype(np.uint8)
    touches = ((x0 > 0 and region[:, 0].any()) or (y0 > 0 and region[0, :].any()) or
               (x1 < W and region[:, -1].any()) or (y1 < H and region[-1, :].any()))
    rc, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if rc and not touches:
        hull = cv2.convexHull(max(rc, key=cv2.contourArea) + np.array([[x0, y0]]))
        r1 = cv2.minAreaRect(hull)
        a0, a1 = rect[1][0] * rect[1][1], r1[1][0] * r1[1][1]
        if a0 > 0 and 0.9 * a0 <= a1 <= max_grow * a0:
            rect, note = r1, "edges"
    (cx, cy), (rw, rh), ang = rect
    if rw < rh:                                    # report the long side's direction
        rw, rh, ang = rh, rw, ang + 90.0
    ang = (ang + 90.0) % 180.0 - 90.0
    return Detection(True, float(cx), float(cy), float(rw * rh), (x, y, w, h), float(ang),
                     corners=cv2.boxPoints(((cx, cy), (rw, rh), ang)), size_px=(float(rw), float(rh)),
                     n_blobs=len(big), method="edges", note=note)


def detect(bgr, color: ColorRange, method="blob", min_area=150, both=False):
    """Returns (mask, detection) or, with both=True, (mask, blob detection, edges detection)."""
    mask = make_mask(bgr, as_range(color))
    if both:
        return mask, detect_blob(mask, min_area), detect_edges(bgr, mask, min_area)
    return mask, (detect_blob(mask, min_area) if method == "blob" else detect_edges(bgr, mask, min_area))


# =================================================================================== display
MAGENTA, CYAN, GREEN, WHITE, YELLOW = (255, 0, 255), (255, 255, 0), (0, 255, 0), (255, 255, 255), (0, 255, 255)


def _star(img, x, y, color, r=14):
    cv2.drawMarker(img, (int(round(x)), int(round(y))), color, cv2.MARKER_STAR, r * 2, 2, cv2.LINE_AA)


def annotate(bgr, mask, det: Detection, alt: Detection = None, aim=None, lines=(), centered=False):
    """Camera image and thresholded image side by side, each with the centroid marker (rubric: 30 pts).

    Magenta star: the aim point from the active method (the handout's 'm*'). Cyan: the other method.
    Crosshair: where the object should end up. Yellow line: the error."""
    H, W = bgr.shape[:2]
    ax, ay = aim if aim else (W / 2, H / 2)
    left = bgr.copy()
    right = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    for img in (left, right):
        col = GREEN if centered else WHITE
        cv2.line(img, (int(ax) - 22, int(ay)), (int(ax) + 22, int(ay)), col, 1, cv2.LINE_AA)
        cv2.line(img, (int(ax), int(ay) - 22), (int(ax), int(ay) + 22), col, 1, cv2.LINE_AA)
        if alt is not None and alt.found:
            if alt.corners is not None:
                cv2.polylines(img, [np.int32(alt.corners)], True, CYAN, 1, cv2.LINE_AA)
            cv2.circle(img, (int(round(alt.cx)), int(round(alt.cy))), 5, CYAN, 1, cv2.LINE_AA)
        if det.found:
            if det.corners is not None:
                cv2.polylines(img, [np.int32(det.corners)], True, MAGENTA, 2, cv2.LINE_AA)
            cv2.line(img, (int(ax), int(ay)), (int(round(det.cx)), int(round(det.cy))), YELLOW, 1, cv2.LINE_AA)
            _star(img, det.cx, det.cy, MAGENTA)
    out = np.hstack([left, right])
    for i, s in enumerate(lines):
        cv2.putText(out, s, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, s, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
    return out


def to_jpeg(img, quality=80) -> bytes:
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def show_inline(img, width=None):
    """Show an image in the notebook."""
    from IPython.display import display, Image
    display(Image(data=to_jpeg(img), width=width))


# =============================================================================== orientation
_CV_ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def orient(img, rot=0, flip_h=False, flip_v=False):
    """Turn a camera frame as the color picker does: flips first, then a clockwise rotation (0/90/180/270)."""
    if flip_h and flip_v:
        img = cv2.flip(img, -1)
    elif flip_h:
        img = cv2.flip(img, 1)
    elif flip_v:
        img = cv2.flip(img, 0)
    rot = int(rot) % 360
    return cv2.rotate(img, _CV_ROT[rot]) if rot else img


def orientation_from_picker_json(path):
    """(rotate_cw_deg, flip_h, flip_v) saved in a color-picker capture; (0, False, False) if absent."""
    with open(path) as f:
        o = _find_key(json.load(f), "orientation") or {}
    return int(o.get("rotate_cw_deg", 0)), bool(o.get("flip_h", False)), bool(o.get("flip_v", False))


def focal_from_picker_json(path):
    """The picker's focal-length record ('focal'), or None if the lens tool was not used."""
    with open(path) as f:
        return _find_key(json.load(f), "focal") or None


# ==================================================================================== camera
def list_cameras(max_index=6):
    """Cameras Windows reports. Uses cv2_enumerate_cameras for names when it is installed."""
    out = []
    try:
        from cv2_enumerate_cameras import enumerate_cameras
        for c in enumerate_cameras(cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY):
            out.append({"index": c.index, "name": c.name, "vid": getattr(c, "vid", None), "pid": getattr(c, "pid", None)})
        if out:
            return out
    except Exception:
        pass
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(i)
        if cap.isOpened():
            ok, f = cap.read()
            if ok:
                out.append({"index": i, "name": f"Camera {i}", "size": f.shape[1::-1]})
        cap.release()
    return out


class Camera:
    """USB camera with a background thread that always holds the newest frame.

    OpenCV keeps a few frames in a buffer; reading them one per control cycle would feed the controller
    old pictures. read() waits for a frame newer than the last one it returned.

    Watchdog (v0.2): a camera that keeps delivering black frames (mean brightness under black_level
    for black_limit frames in a row), or stops delivering frames for stall_s seconds, is closed and
    opened again with the same settings. `reopens` counts how often that happened; `brightness`
    is the mean of the newest frame (0-255). cam.reopen() does it by hand."""

    def __init__(self, index=0, width=640, height=480, fps=30, fourcc=None, orientation=(0, False, False), warmup=5,
                 black_level=3.0, black_limit=20, stall_s=3.0, watchdog=True):
        """fourcc None = the camera's default format (safest). "MJPG" or "YUY2" only if that mode is known to work:
        a format the camera does not support at that size gives black or garbled frames."""
        self.args = dict(index=int(index), width=int(width), height=int(height), fps=fps, fourcc=fourcc)
        self.index = int(index)
        self.name = f"camera {index}"
        self.orientation = tuple(orientation)      # (rotate_cw_deg, flip_h, flip_v), as in the color picker
        self.black_level, self.black_limit, self.stall_s, self.watchdog = black_level, int(black_limit), stall_s, watchdog
        self.reopens = 0
        self.brightness = float("nan")
        self.events = []                           # (time, text) of watchdog actions
        self._warmup0 = int(warmup)
        self.cap = None
        self._open()
        self._frame = None; self._t = 0.0; self.n = 0; self._last = 0
        self._times = []
        self._cv = threading.Condition()
        self._stop = False
        self._want_reopen = False
        self.fps_measured = 0.0
        self._thread = threading.Thread(target=self._grab, daemon=True)
        self._thread.start()

    def _open(self):
        a = self.args
        self.cap = cv2.VideoCapture(a["index"], cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(a["index"])
        if not self.cap.isOpened():
            raise RuntimeError(f"camera {a['index']} did not open (is another program or the color picker using it?)")
        fourcc = a["fourcc"]
        # Same steps as the color picker (camera_color_picker.camera): format before the size, and if the
        # driver did not keep the format, size first and format after.
        for order in ("fourcc_first", "size_first"):
            if fourcc and order == "fourcc_first":
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, a["width"])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a["height"])
            if fourcc and order == "size_first":
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            if not fourcc or self.format() == fourcc:
                break
        if a["fps"]:
            self.cap.set(cv2.CAP_PROP_FPS, float(a["fps"]))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)); self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if int(self.orientation[0]) % 180 == 90:
            self.width, self.height = self.height, self.width
        self.warmup = self._warmup0                    # first frames are often black while exposure settles
        self._black = 0
        self._t_ok = time.perf_counter()

    def _reopen_now(self, why):
        self.events.append((time.perf_counter(), why))
        try:
            self.cap.release()
        except Exception:
            pass
        time.sleep(0.3)
        try:
            self._open()
            self.reopens += 1
        except RuntimeError as e:
            self.events.append((time.perf_counter(), f"reopen failed: {e}"))
            time.sleep(1.0)

    def reopen(self):
        """Close and open the camera again with the same settings (what fixed black frames on 2026-11-15)."""
        self._want_reopen = True

    def _grab(self):
        while not self._stop:
            if self._want_reopen:
                self._want_reopen = False
                self._reopen_now("reopen requested")
                continue
            ok, f = self.cap.read()
            now = time.perf_counter()
            if not ok:
                if self.watchdog and now - self._t_ok > self.stall_s:
                    self._reopen_now(f"no frames for {self.stall_s:.0f} s")
                time.sleep(0.01)
                continue
            self._t_ok = now
            if self.orientation != (0, False, False):
                f = orient(f, *self.orientation)
            if self.warmup > 0:
                self.warmup -= 1
                continue
            self.brightness = float(f[::8, ::8].mean())
            self._black = self._black + 1 if self.brightness < self.black_level else 0
            if self.watchdog and self._black >= self.black_limit:
                self._reopen_now(f"{self._black} black frames in a row")
                continue
            self._times = (self._times + [now])[-30:]
            if len(self._times) >= 2:                  # frames per second over the last 30 frames
                self.fps_measured = (len(self._times) - 1) / (self._times[-1] - self._times[0])
            with self._cv:
                self._frame, self._t = f, now
                self.n += 1
                self._cv.notify_all()

    @staticmethod
    def from_picker_json(path, warmup=5, size=None):
        """Open the camera exactly as the color picker had it when it saved this capture: the same camera
        index, the mode the driver actually ran (width, height, frames per second, format), and the same
        flip/rotate. size=(640, 480) keeps the camera and the flip/rotate but asks for that size instead
        (faster: the HBVCAM gives about 7.5 frames/s at 1280 x 720 in YUY2). Prints what it found. Camera indexes can change when cameras are plugged in or out:
        if the name printed is not the gripper camera, take a new capture or pass the index by hand."""
        with open(path) as f:
            d = json.load(f)
        src = _find_key(d, "source") or {}
        drv = _find_key(d, "driver") or {}
        if not isinstance(src, dict) or src.get("index") is None:
            raise ValueError(f"{path}: no camera index in 'source' (was it saved from a still image or the test pattern?)")
        o = orientation_from_picker_json(path)
        fmt = drv.get("fmt") if isinstance(drv, dict) and drv.get("fmt") not in (None, "", "?") else None
        w, h = int(drv.get("w", 640) or 640), int(drv.get("h", 480) or 480)
        fps = drv.get("fps") or 30
        if size:                                       # a different size than the capture: let the driver pick its format
            w, h = int(size[0]), int(size[1])
            fmt, fps = None, 30
        print(f"from {os.path.basename(str(path))}: camera {src.get('index')} '{src.get('name', '')}', "
              f"{w} x {h} {fmt or 'driver default'} @ {fps}, orientation (rotate cw, flip h, flip v) {o}")
        cam = Camera(int(src["index"]), w, h, fps, fmt, o, warmup)
        cam.name = f"camera {src['index']} ({src.get('name', '')})"
        return cam

    def format(self):
        """The format the driver reports, e.g. 'MJPG' or 'YUY2'."""
        v = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        s = "".join(chr((v >> 8 * i) & 0xFF) for i in range(4)) if v else ""
        return s if len(s) == 4 and all(c.isalnum() or c == " " for c in s) else "?"   # some drivers report junk

    def read(self, timeout=2.0):
        """Newest frame as (image, time it arrived in seconds, frame number). Raises if the camera stalls."""
        with self._cv:
            if not self._cv.wait_for(lambda: self.n > self._last, timeout):
                raise TimeoutError("no new camera frame (is another program, such as the color picker, using the camera? "
                                   "Stop the picker with picker.close(), or try cam.reopen())")
            self._last = self.n
            return self._frame.copy(), self._t, self.n

    def settings_dialog(self):
        """Windows camera dialog: turn off automatic exposure and white balance before picking colors."""
        self.cap.set(cv2.CAP_PROP_SETTINGS, 1)

    def status(self):
        """One line: size, format, frame rate, brightness, watchdog reopen count."""
        return (f"{self.name}: {self.width} x {self.height} {self.format()} | {self.fps_measured:.1f} frames/s | "
                f"brightness {self.brightness:.0f} | reopened {self.reopens} times")

    def close(self):
        self._stop = True
        self._thread.join(1.0)
        self.cap.release()


class Recorder:
    """MP4 of the annotated view."""

    def __init__(self, path, fps=20.0):
        self.path, self.fps, self.vw = path, fps, None

    def write(self, img):
        if self.vw is None:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            self.vw = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (img.shape[1], img.shape[0]))
        self.vw.write(img)

    def close(self):
        if self.vw is not None:
            self.vw.release()
            self.vw = None


# ============================================================================ threshold tuner
class ThresholdTuner:
    """Live mask with sliders, like MATLAB's colorThresholder. tuner.color is the result.

    Given a ColorModel: three sliders kH, kS, kV (range = mean +/- k*sigma).
    Given a ColorRange: six sliders (H low/high, S low/high, V low/high); H low above H high wraps through red.
        tuner = ThresholdTuner(cam, colors["red"]); tuner.show()      # one notebook cell
        colors["red"] = tuner.color; tuner.stop()                       # when it looks right"""

    def __init__(self, camera, color, method="blob", width=900):
        import ipywidgets as W
        self.cam, self.method = camera, method
        self.model = color if isinstance(color, ColorModel) else None
        lay = W.Layout(width="420px")
        if self.model:
            mk = lambda d, v: W.FloatSlider(description=d, value=float(v), min=0.5, max=6.0, step=0.1,
                                            continuous_update=True, layout=lay)
            self.s = [mk("k H", self.model.k[0]), mk("k S", self.model.k[1]), mk("k V", self.model.k[2])]
            rows = [W.HBox(self.s)]
        else:
            self.name = color.name
            lo, hi = color.ranges[0][0], color.ranges[-1][1]
            h_lo, h_hi = color.ranges[0][0][0], color.ranges[-1][1][0]
            mk = lambda d, v, top: W.IntSlider(description=d, value=int(v), min=0, max=top, continuous_update=True, layout=lay)
            self.s = [mk("H low", h_lo, 179), mk("H high", h_hi, 179), mk("S low", lo[1], 255), mk("S high", hi[1], 255),
                      mk("V low", lo[2], 255), mk("V high", hi[2], 255)]
            rows = [W.HBox([W.VBox(self.s[0:2]), W.VBox(self.s[2:4]), W.VBox(self.s[4:6])])]
        self.img = W.Image(format="jpeg", width=width)
        self.info = W.Label()
        self.box = W.VBox(rows + [self.info, self.img])
        self._stop = True
        self._thread = None

    @property
    def color(self):
        v = [w.value for w in self.s]
        if self.model:
            return ColorModel(self.model.name, self.model.mean, self.model.std, tuple(round(x, 2) for x in v),
                              self.model.n, self.model.sources)
        return ColorRange.from_hsv(self.name, *v)

    def step(self):
        frame, _, _ = self.cam.read()
        c = self.color
        mask, a, b = detect(frame, as_range(c), both=True)
        det, alt = (a, b) if self.method == "blob" else (b, a)
        self.img.value = to_jpeg(annotate(frame, mask, det, alt, lines=[str(as_range(c))]))
        self.info.value = (f"blob: {a.area:.0f} px at ({a.cx:.0f}, {a.cy:.0f}), {a.n_blobs} blob(s)   "
                           f"edges: ({b.cx:.0f}, {b.cy:.0f})" if a.found else "nothing inside the range")

    def _loop(self):
        while not self._stop:
            try:
                self.step()
            except Exception as e:
                self.info.value = f"stopped: {e}"
                return
            time.sleep(0.07)

    def show(self):
        from IPython.display import display
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        display(self.box)

    def stop(self):
        self._stop = True
