"""Object profiles: one object, one room, one camera setup, and the color statistics that go with them.

A profile is what the color picker session produces, packed for the robot side:
    tag       "green wooden block #1, in the living room" (object + room)
    camera    index, name, size, format, frame rate, flip/rotate, driver properties (exposure, white balance ...)
    captures  the picker files it was built from (Cluster mode, random "shotgun" pattern)
    color     HSV mean and standard deviation pooled over the captures, and the k (sigma) per channel

Profiles are saved as profiles/<object>__<room>.json and can be printed as a paste block: a few lines
of Python that go into the notebook's settings cell (PROFILE = {...}), so a later session for the
same object and room starts from them without the picker.

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import glob, json, os, re, time
from dataclasses import dataclass, field, asdict
from . import vision

DEFAULT_K = (2.5, 2.5, 3.0)


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", str(text).lower()).strip("-") or "unnamed"


def tag_of(obj, room):
    return f"{obj}, in the {room}" if room else str(obj)


def read_capture(path):
    with open(path) as f:
        return json.load(f)


def camera_from_capture(path, size=None):
    """Camera settings as the picker had them when it saved this capture (size overrides width x height)."""
    d = read_capture(path)
    f = vision._find_key
    src, drv = f(d, "source") or {}, f(d, "driver") or {}
    o = vision.orientation_from_picker_json(path)
    cam = {"index": src.get("index"), "name": src.get("name", ""), "vid": src.get("vid"), "pid": src.get("pid"),
           "width": int(drv.get("w", 640) or 640), "height": int(drv.get("h", 480) or 480),
           "fps": float(drv.get("fps", 30) or 30), "fmt": drv.get("fmt") if drv.get("fmt") not in (None, "", "?") else None,
           "orientation": list(o), "properties": f(d, "camera_properties") or {},
           "measured_fps": d.get("measured_capture_fps")}
    if size:
        cam.update(width=int(size[0]), height=int(size[1]), fmt=None, fps=30.0)
    return cam


@dataclass
class ObjectProfile:
    object: str                                   # "green wooden block #1"
    room: str                                     # "living room"
    camera: dict
    color: dict                                   # vision.ColorModel.to_dict()
    captures: list = field(default_factory=list)
    created: str = ""
    notes: str = ""
    size_mm: float = 0.0                          # block side in mm (0 = not measured): the depth estimate uses it

    @property
    def tag(self):
        return tag_of(self.object, self.room)

    # ---- build
    @staticmethod
    def from_captures(paths, obj, room, size=(640, 480), k=DEFAULT_K, notes="", size_mm=0.0):
        """Pool picker captures of one object into a profile. Camera settings come from the newest capture;
        size=(640, 480) keeps that camera and its flip/rotate but asks the driver for 640 x 480 (faster)."""
        paths = sorted(paths)
        if not paths:
            raise ValueError("no capture files given")
        model = vision.ColorModel.from_picker_json(paths, name=obj, k=tuple(k))
        prof = ObjectProfile(obj, room, camera_from_capture(paths[-1], size), model.to_dict(),
                             [os.path.basename(p) for p in paths], time.strftime("%Y-%m-%d %H:%M"), notes, float(size_mm))
        prof.check_captures(paths)
        return prof

    def check_captures(self, paths):
        """Warn about captures that do not match: another camera, another orientation, outliers excluded,
        Pin-point mode, or a pattern other than random (shotgun)."""
        ref = camera_from_capture(sorted(paths)[-1])
        for p in paths:
            d = read_capture(p)
            c = camera_from_capture(p)
            s = vision._find_key(d, "sampling") or {}
            name = os.path.basename(p)
            if (c["index"], c["name"]) != (ref["index"], ref["name"]):
                print(f"warning {name}: camera {c['index']} '{c['name']}', not {ref['index']} '{ref['name']}'")
            if c["orientation"] != ref["orientation"]:
                print(f"warning {name}: flip/rotate {c['orientation']}, not {ref['orientation']}")
            if s.get("mode") not in (None, "Cluster"):
                print(f"warning {name}: sampled in {s.get('mode')} mode, not Cluster")
            if s.get("exclude"):
                print(f"warning {name}: 'exclude outliers' was on: the spread is underestimated")
            if s.get("pattern") not in (None, "random"):
                print(f"note {name}: pattern {s.get('pattern')} (the shotgun pattern is 'random')")

    # ---- use
    def model(self) -> vision.ColorModel:
        return vision.ColorModel.from_dict(self.color)

    def color_range(self) -> vision.ColorRange:
        return self.model().range()

    def set_k(self, k):
        self.color["k"] = list(k)

    def open_camera(self, warmup=5, **kw) -> vision.Camera:
        c = self.camera
        print(f"camera {c['index']} '{c['name']}', {c['width']} x {c['height']} {c['fmt'] or 'driver default'} @ {c['fps']:g}, "
              f"orientation (rotate cw, flip h, flip v) {tuple(c['orientation'])}")
        cam = vision.Camera(int(c["index"]), c["width"], c["height"], c["fps"], c["fmt"], tuple(c["orientation"]), warmup, **kw)
        cam.name = f"camera {c['index']} ({c['name']})"
        return cam

    # ---- save, load, paste
    def to_dict(self):
        d = asdict(self)
        d["tag"] = self.tag
        return d

    @staticmethod
    def from_dict(d):
        d = dict(d)
        d.pop("tag", None)
        return ObjectProfile(**d)

    def path(self, folder="profiles"):
        return os.path.join(folder, f"{slug(self.object)}__{slug(self.room)}.json")

    def save(self, folder="profiles"):
        os.makedirs(folder, exist_ok=True)
        p = self.path(folder)
        with open(p, "w") as f:
            json.dump(self.to_dict(), f, indent=1)
        return p

    def paste_block(self):
        """Python text for the notebook's settings cell. Paste it over the PROFILE = None line."""
        d = self.to_dict()
        d.pop("tag", None)
        body = ",\n    ".join(f"{k}={v!r}" for k, v in d.items())
        return (f"# ---- object profile: {self.tag} (made {self.created}) ----\n"
                f"PROFILE = dict(\n    {body})\n")

    def __str__(self):
        c = self.camera
        return (f"{self.tag}: camera {c['index']} '{c['name']}' {c['width']} x {c['height']}, "
                f"{len(self.captures)} captures | {self.model()}")


def load(path):
    with open(path) as f:
        return ObjectProfile.from_dict(json.load(f))


def find(obj, room, folder="profiles"):
    """Saved profile for this object and room, or None."""
    p = os.path.join(folder, f"{slug(obj)}__{slug(room)}.json")
    return load(p) if os.path.exists(p) else None


def list_profiles(folder="profiles", show=True):
    out = []
    for p in sorted(glob.glob(os.path.join(folder, "*.json"))):
        try:
            out.append(load(p))
        except Exception as e:                    # noqa: BLE001
            print(f"skipped {p}: {e}")
    if show:
        for pr in out:
            print(" ", pr)
    return out


def resolve(pasted, obj, room, folder="profiles", default=None):
    """The profile this session uses, first match wins:
        1. a profile pasted into the settings cell (PROFILE = {...}) for this object and room
        2. the saved file profiles/<object>__<room>.json
        3. `default` (the notebook's built-in starting profile)
    Returns (profile or None, where it came from)."""
    if pasted:
        p = ObjectProfile.from_dict(pasted)
        if (slug(p.object), slug(p.room)) == (slug(obj), slug(room)):
            return p, "pasted in the settings cell"
        print(f"the pasted PROFILE is for '{p.tag}', not '{tag_of(obj, room)}': ignored")
    p = find(obj, room, folder)
    if p:
        return p, f"saved file {p.path(folder)}"
    if default is not None:
        return default, "built-in default"
    return None, "none"


def model_from_preset(name, k=DEFAULT_K):
    """A rough ColorModel from one of vision.PRESETS (the range's middle and half-width / k). Only a start."""
    r = vision.PRESETS[name]
    (lo, hi) = r.ranges[0] if len(r.ranges) == 1 else (r.ranges[0][0], r.ranges[1][1])   # red: wraps through 0
    span_h = (hi[0] - lo[0]) % 180
    h = (lo[0] + span_h / 2) % 180
    mean = (h, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
    std = (max(1.0, span_h / 2 / k[0]), max(1.0, (hi[1] - lo[1]) / 2 / k[1]), max(1.0, (hi[2] - lo[2]) / 2 / k[2]))
    return vision.ColorModel(name, tuple(round(v, 2) for v in mean), tuple(round(v, 2) for v in std), tuple(k), 0,
                             [f"preset {name}"])


def default_profile(obj, room, camera: dict, color=None):
    """Profile from the settings cell's defaults when nothing was captured for this object and room:
    the default camera, and a preset color guessed from the object's name (or `color`)."""
    name = color or next((c for c in ("green", "red", "yellow", "wood") if c in str(obj).lower()), None)
    if name is None:
        raise ValueError(f"no profile for '{tag_of(obj, room)}' and no color word in the name: "
                         "run the color picker part (or set DEFAULT_COLOR)")
    cam = {"index": 0, "name": "", "vid": None, "pid": None, "width": 640, "height": 480, "fps": 30.0, "fmt": None,
           "orientation": [0, False, False], "properties": {}, "measured_fps": None}
    cam.update(camera)
    cam["orientation"] = list(cam["orientation"])
    return ObjectProfile(obj, room, cam, model_from_preset(name).to_dict(), [], time.strftime("%Y-%m-%d %H:%M"),
                         f"defaults: preset color '{name}', camera from DEFAULT_CAMERA")


# ============================================================================ capture picker
def captures_since(folder="captures", since=None):
    """Capture JSON files in `folder`, newest last; only those saved after `since` (time.time() value) if given."""
    files = sorted(glob.glob(os.path.join(folder, "*.json")))
    if since is not None:
        files = [f for f in files if os.path.getmtime(f) >= since]
    return files


class CaptureSelector:
    """Thumbnails of picker captures with a check box each. selector.selected = the checked files.

    Uncheck any capture whose circle took in a finger, the background or a highlight."""

    def __init__(self, files, thumb=200):
        import ipywidgets as W
        self.files = list(files)
        self.boxes = []
        cards = []
        for p in self.files:
            d = read_capture(p)
            img = None
            fl = d.get("files") or {}
            for key in ("annotated", "raw"):
                q = os.path.join(os.path.dirname(p), fl.get(key, "")) if fl.get(key) else None
                if q and os.path.exists(q):
                    with open(q, "rb") as f:
                        img = f.read()
                    break
            st = (vision._find_key(d, "rows") or {})
            hsv = "  ".join(f"{c} {st[c]['mean']:.0f}±{st[c]['std']:.0f}" for c in "HSV" if c in st)
            cb = W.Checkbox(value=True, description=os.path.basename(p)[:22], indent=False)
            self.boxes.append(cb)
            parts = [W.Image(value=img, format="png", width=thumb)] if img else [W.HTML("<i>(image not found)</i>")]
            cards.append(W.VBox(parts + [cb, W.HTML(f"<small>{hsv}</small>")], layout=W.Layout(width=f"{thumb + 20}px")))
        self.widget = W.GridBox(cards, layout=W.Layout(grid_template_columns=f"repeat(auto-fill, {thumb + 24}px)"))

    @property
    def selected(self):
        return [p for p, b in zip(self.files, self.boxes) if b.value]

    def show(self):
        from IPython.display import display
        display(self.widget)
        return self


def capture_image(path, kind="raw"):
    """The picture the picker saved with a capture (kind "raw" or "annotated"), or None if it is not next to the JSON."""
    import cv2
    fl = read_capture(path).get("files") or {}
    if not fl.get(kind):
        return None
    q = os.path.join(os.path.dirname(path), fl[kind])
    return cv2.imread(q) if os.path.exists(q) else None


def mask_check(prof: ObjectProfile, files, width=320):
    """Each capture's own picture with this profile's range applied: (side-by-side images, fraction of each
    picture in range). A large fraction means the background or skin is in range too."""
    import cv2, numpy as np
    rng = prof.color_range()
    tiles, fr = [], []
    for p in files:
        img = capture_image(p)
        if img is None:
            continue
        m = vision.make_mask(img, rng)
        fr.append(float((m > 0).mean()))
        over = img.copy()
        over[m > 0] = (0.5 * over[m > 0] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
        h = int(img.shape[0] * width / img.shape[1])
        t = cv2.resize(over, (width, h))
        cv2.putText(t, f"{100 * fr[-1]:.1f}% in range", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        tiles.append(t)
    if not tiles:
        return None, []
    rows = [np.hstack(tiles[i:i + 3] + [np.zeros_like(tiles[0])] * (3 - len(tiles[i:i + 3]))) for i in range(0, len(tiles), 3)]
    return np.vstack(rows), fr
