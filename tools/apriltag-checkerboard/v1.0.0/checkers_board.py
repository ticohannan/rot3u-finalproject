# Printable checkers board with four AprilTags (tag36h11, IDs 0-3) and orientation arrows.
# Version 1.0.0
# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see the LICENSE file of the repository).
# Works as a Jupyter notebook cell (WinPython 3.12) or as a command-line script.
# Install once:  pip install numpy opencv-python reportlab
# Command line:  python checkers_board.py --paper a4 --tag-size 2.2 --no-border \
#                    --color-1 "#202020" --color-2 "#F0E0B0"

import math
import sys
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.colors import HexColor
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas

# ================================================================= SETTINGS
# The ONE place to change defaults. Used by the notebook cell, by the function
# defaults, and as the command-line defaults (command-line options override them).
SETTINGS = {
    "paper": "letter",        # "letter" or "a4"
    "color_1": "#000000",     # dark squares
    "color_2": "#FFFFFF",     # light squares (the top-left square)
    "output": "checkers_board.pdf",
    "tag_cm": 2.4,            # AprilTag side length, 2.0 - 2.4 cm
    "border": True,           # False = no border around the tags
    "border_w_cm": 3.6,       # border width  (used when border is True)  [assumption]
    "border_h_cm": 4.0,       # border height (used when border is True)  [assumption]
}

# ----------------------------------------------------------------- fixed spec
CM = 72.0 / 2.54                 # points per cm
INCH = 72.0
PAPERS = {                       # portrait size in points (width, height)
    "letter": (8.5 * INCH, 11.0 * INCH),
    "a4": (210.0 / 25.4 * INCH, 297.0 / 25.4 * INCH),
}
NUM_SQUARES = 8
SQUARE_CM = 2.5
TAG_MIN_CM, TAG_MAX_CM = 2.0, 2.4   # allowed tag side lengths
ARROW_CM = 2.0                   # orientation arrow footprint, 2 x 2 cm
ARROW_GAP_CM = 1.0               # gap between arrow bottom and top edge of the tag
MARGIN_IN = 0.5                  # printer cannot print inside this strip
FOLD_GRAY = "#696969"            # dim gray
FOLD_LINE_PT = 0.75
TAG_IDS = {"top_right": 0, "top_left": 1, "bottom_right": 2, "bottom_left": 3}


# ----------------------------------------------------------------- AprilTags
def tag_matrix(tag_id):
    """8 x 8 boolean array (True = black) for a tag36h11 tag, upright."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, "generateImageMarker"):
        img = aruco.generateImageMarker(dictionary, tag_id, 8, borderBits=1)
    else:  # older OpenCV
        img = aruco.drawMarker(dictionary, tag_id, 8, borderBits=1)
    return img < 128


# ----------------------------------------------------------------- page layout
def split_axis(total, printable, grid=(), forbidden=()):
    """Seam positions along one axis.
    Two pages: the seam snaps to the board grid line nearest the middle that does
    not cut a tag or arrow (fallback: the middle). Three or more pages: middle
    pages use the full printable size (fold on both sides), end pages share the rest."""
    n = max(1, math.ceil(total / printable - 1e-9))
    if n == 1:
        return [0.0, total]
    if n == 2:
        lo, hi, eps = total - printable, printable, 1e-6
        cands = [g for g in grid if lo - eps <= g <= hi + eps
                 and not any(a + eps < g < b - eps for a, b in forbidden)]
        seam = min(cands, key=lambda g: abs(g - total / 2)) if cands else total / 2
        return [0.0, seam, total]
    end = (total - (n - 2) * printable) / 2
    return [0.0, end] + [end + k * printable for k in range(1, n - 1)] + [total]


def choose_orientation(paper, total_w, total_h, grid_x, grid_y, forb_x, forb_y):
    pw, ph = PAPERS[paper]
    best = None
    for name, (w, h) in (("portrait", (pw, ph)), ("landscape", (ph, pw))):
        pr_w, pr_h = w - 2 * MARGIN_IN * INCH, h - 2 * MARGIN_IN * INCH
        xs = split_axis(total_w, pr_w, grid_x, forb_x)
        ys = split_axis(total_h, pr_h, grid_y, forb_y)
        pages = (len(xs) - 1) * (len(ys) - 1)
        if best is None or pages < best[0]:        # tie -> keep portrait
            best = (pages, name, w, h, xs, ys)
    return best[1:]


def hex_color(value):
    value = value.strip()
    return HexColor(value if value.startswith("#") else "#" + value)


# ----------------------------------------------------------------- main routine
def make_board_pdf(paper=None, color_1=None, color_2=None, output=None, tag_cm=None,
                   border=None, border_w_cm=None, border_h_cm=None,
                   draw_border_outline=False):
    """Any argument left as None takes its value from SETTINGS."""
    given = dict(paper=paper, color_1=color_1, color_2=color_2, output=output, tag_cm=tag_cm,
                 border=border, border_w_cm=border_w_cm, border_h_cm=border_h_cm)
    cfg = {**SETTINGS, **{k: v for k, v in given.items() if v is not None}}
    paper, color_1, color_2, output = cfg["paper"], cfg["color_1"], cfg["color_2"], cfg["output"]
    tag_cm, border = cfg["tag_cm"], cfg["border"]
    border_w_cm, border_h_cm = cfg["border_w_cm"], cfg["border_h_cm"]
    paper = paper.lower().replace(" ", "").replace("-", "").replace("_", "")
    paper = {"usletter": "letter"}.get(paper, paper)
    if paper not in PAPERS:
        raise ValueError("paper must be 'letter' or 'a4'")
    if not TAG_MIN_CM - 1e-9 <= tag_cm <= TAG_MAX_CM + 1e-9:
        raise ValueError(f"tag size must be {TAG_MIN_CM}-{TAG_MAX_CM} cm")
    if border and (border_w_cm < tag_cm or border_h_cm < tag_cm):
        raise ValueError("border must be at least as large as the tag (or use border=False)")
    c1, c2 = hex_color(color_1), hex_color(color_2)

    sq = SQUARE_CM * CM
    B = NUM_SQUARES * sq                     # board side
    L = tag_cm * CM                          # tag side
    # Border box: the given size, or exactly the tag when there is no border.
    bw, bh = (border_w_cm * CM, border_h_cm * CM) if border else (L, L)
    arrow, gap = ARROW_CM * CM, ARROW_GAP_CM * CM

    # Build in board coordinates (board top-left = 0,0; y down), then shift so the
    # bounding box of everything (board, borders, arrows) starts at (0,0).
    # Each border box has one corner touching the matching board corner.
    corner_box = {
        "top_left": (-bw, -bh),        # its bottom-right = board top-left
        "top_right": (B, -bh),         # its bottom-left  = board top-right
        "bottom_left": (-bw, B),       # its top-right    = board bottom-left
        "bottom_right": (B, B),        # its top-left     = board bottom-right
    }
    raw = {}
    for pos, (x, y) in corner_box.items():
        tx, ty = x + (bw - L) / 2, y + (bh - L) / 2                  # tag centered in its border
        ax, ay = tx + L / 2 - arrow / 2, ty - gap - arrow           # arrow footprint, above the tag
        raw[pos] = dict(border=(x, y), tag=(tx, ty), arrow=(ax, ay))
    xs_all = [0, B]
    ys_all = [0, B]
    for d in raw.values():
        xs_all += [d["border"][0], d["border"][0] + bw, d["arrow"][0], d["arrow"][0] + arrow]
        ys_all += [d["border"][1], d["border"][1] + bh, d["arrow"][1], d["arrow"][1] + arrow]
    sx, sy = -min(xs_all), -min(ys_all)
    total_w, total_h = max(xs_all) + sx, max(ys_all) + sy
    bx, by = sx, sy                                                  # board top-left, assembly coords
    borders = {p: (d["border"][0] + sx, d["border"][1] + sy) for p, d in raw.items()}
    tags = {p: (d["tag"][0] + sx, d["tag"][1] + sy, tag_matrix(TAG_IDS[p])) for p, d in raw.items()}
    arrows = {p: (d["arrow"][0] + sx, d["arrow"][1] + sy) for p, d in raw.items()}

    # Things a fold should not cut (tags, arrows) and preferred seams (board grid lines).
    forb_x = [(t[0], t[0] + L) for t in tags.values()] + [(a[0], a[0] + arrow) for a in arrows.values()]
    forb_y = [(t[1], t[1] + L) for t in tags.values()] + [(a[1], a[1] + arrow) for a in arrows.values()]
    grid_x = [bx + k * sq for k in range(NUM_SQUARES + 1)]
    grid_y = [by + k * sq for k in range(NUM_SQUARES + 1)]
    orient, page_w, page_h, xs, ys = choose_orientation(
        paper, total_w, total_h, grid_x, grid_y, forb_x, forb_y)
    m = MARGIN_IN * INCH
    ncols, nrows = len(xs) - 1, len(ys) - 1
    npages = ncols * nrows

    for seam in xs[1:-1]:
        if any(a + 1e-6 < seam < b + -1e-6 for a, b in forb_x):
            print("WARNING: a vertical fold crosses a tag or arrow")
    for seam in ys[1:-1]:
        if any(a + 1e-6 < seam < b + -1e-6 for a, b in forb_y):
            print("WARNING: a horizontal fold crosses a tag or arrow")

    cv = canvas.Canvas(str(output), pagesize=(page_w, page_h))
    cv.setTitle("Checkers board with AprilTags")
    lw = FOLD_LINE_PT
    black, cell = HexColor("#000000"), L / 8

    def align(join_far, join_near, size, page):
        # content ends exactly on the fold line of a joining edge
        if join_far:
            return page - m - size
        if join_near:
            return m
        return (page - size) / 2

    page_no = 0
    for r in range(nrows):
        for col in range(ncols):
            page_no += 1
            cx0, cx1, cy0, cy1 = xs[col], xs[col + 1], ys[r], ys[r + 1]
            w, h = cx1 - cx0, cy1 - cy0
            jl, jr, jt, jb = col > 0, col < ncols - 1, r > 0, r < nrows - 1
            px = align(jr, jl, w, page_w)
            py = align(jb, jt, h, page_h)
            ox, oy = px - cx0, py - cy0               # assembly origin on this page

            def rect(x, y, rw, rh, color):
                cv.setFillColor(color)
                cv.rect(ox + x, page_h - (oy + y + rh), rw, rh, stroke=0, fill=1)

            cv.saveState()
            clip = cv.beginPath()
            clip.rect(px, page_h - (py + h), w, h)
            cv.clipPath(clip, stroke=0, fill=0)

            # board (top-left square = color 2)
            for i in range(NUM_SQUARES):
                for j in range(NUM_SQUARES):
                    rect(bx + j * sq, by + i * sq, sq, sq, c2 if (i + j) % 2 == 0 else c1)

            # tags (all upright, black and white)
            for pos, (tx, ty, mat) in tags.items():
                if draw_border_outline and border:
                    bxx, byy = borders[pos]
                    cv.setStrokeColor(HexColor(FOLD_GRAY)); cv.setLineWidth(0.25)
                    cv.rect(ox + bxx, page_h - (oy + byy + bh), bw, bh, stroke=1, fill=0)
                for i in range(8):
                    j = 0
                    while j < 8:
                        if mat[i, j]:
                            k = j
                            while k < 8 and mat[i, k]:
                                k += 1
                            rect(tx + j * cell, ty + i * cell, (k - j) * cell, cell, black)
                            j = k
                        else:
                            j += 1

            # orientation arrows: point up, 2 x 2 cm footprint, 1 cm above each tag
            cv.setFillColor(black)
            for ax, ay in arrows.values():
                cx = ax + arrow / 2
                hw, hh, sw = arrow / 2, arrow / 2, 0.35 * CM      # head half-width / height, shaft half-width
                pts = [(cx, ay), (cx + hw, ay + hh), (cx + sw, ay + hh), (cx + sw, ay + arrow),
                       (cx - sw, ay + arrow), (cx - sw, ay + hh), (cx - hw, ay + hh)]
                p = cv.beginPath()
                for k, (qx, qy) in enumerate(pts):
                    X, Y = ox + qx, page_h - (oy + qy)
                    if k == 0:
                        p.moveTo(X, Y)
                    else:
                        p.lineTo(X, Y)
                p.close()
                cv.drawPath(p, stroke=0, fill=1)

            # page label in an outer corner, only if it hits no board/tag/arrow
            if not ((jl and jr) or (jt and jb)):
                label = f"Page {page_no}/{npages}  row {r + 1}, col {col + 1}"
                fs, pad = 6, 0.15 * CM
                tw, th = stringWidth(label, "Helvetica", fs), fs
                lx = cx0 + pad if not jl else cx1 - pad - tw
                ly = cy0 + pad if not jt else cy1 - pad - th
                boxes = ([(bx, by, B, B)] + [(t[0], t[1], L, L) for t in tags.values()]
                         + [(a[0], a[1], arrow, arrow) for a in arrows.values()])
                hit = any(lx < bxn + bwn and lx + tw > bxn and ly < byn + bhn and ly + th > byn
                          for bxn, byn, bwn, bhn in boxes)
                if not hit:
                    cv.setFillColor(HexColor(FOLD_GRAY)); cv.setFont("Helvetica", fs)
                    cv.drawString(ox + lx, page_h - (oy + ly + th * 0.8), label)
            cv.restoreState()

            # fold lines: inside the printable area, 0.5 in from each joining edge
            cv.setStrokeColor(HexColor(FOLD_GRAY)); cv.setLineWidth(lw); cv.setDash(4, 3)
            if jl:
                cv.line(m + lw / 2, m, m + lw / 2, page_h - m)
            if jr:
                cv.line(page_w - m - lw / 2, m, page_w - m - lw / 2, page_h - m)
            if jt:
                cv.line(m, page_h - (m + lw / 2), page_w - m, page_h - (m + lw / 2))
            if jb:
                cv.line(m, m + lw / 2, page_w - m, m + lw / 2)
            cv.setDash()
            cv.showPage()
    cv.save()

    print(f"Wrote {output}: {npages} page(s), {orient} {paper.upper()}, "
          f"{ncols} column(s) x {nrows} row(s).")
    print(f"Assembled size (incl. tags and arrows): {total_w / CM:.1f} x {total_h / CM:.1f} cm; "
          f"board {B / CM:.1f} x {B / CM:.1f} cm; tags {tag_cm:.2f} cm; "
          + (f"border {bw / CM:.1f} x {bh / CM:.1f} cm." if border else "no border."))
    print("Print at 100% / 'Actual size' (not 'Fit to page').")
    print("Assembly: on every page, fold the strip between each dashed fold line and the "
          "page edge to the BACK, butt the folded edges of neighbouring pages together "
          "(page labels show row/column), and tape them on the back.")
    return Path(output)


# ----------------------------------------------------------------- run
if "ipykernel" in sys.modules:
    # Notebook: edit SETTINGS at the top of this cell, then run it.
    pdf_path = make_board_pdf()
    try:
        from IPython.display import FileLink, display
        display(FileLink(str(pdf_path)))
    except Exception:
        pass
elif __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Defaults come from SETTINGS at the top of the file.")
    ap.add_argument("--paper", choices=["letter", "a4"], default=SETTINGS["paper"])
    ap.add_argument("--color-1", default=SETTINGS["color_1"], help="dark square color (hex)")
    ap.add_argument("--color-2", default=SETTINGS["color_2"], help="light square color (hex)")
    ap.add_argument("--output", default=SETTINGS["output"])
    ap.add_argument("--tag-size", type=float, default=SETTINGS["tag_cm"], help="tag side in cm, 2.0-2.4")
    ap.add_argument("--border", action=argparse.BooleanOptionalAction, default=SETTINGS["border"],
                    help="--border / --no-border")
    ap.add_argument("--border-w", type=float, default=SETTINGS["border_w_cm"])
    ap.add_argument("--border-h", type=float, default=SETTINGS["border_h_cm"])
    a = ap.parse_args()
    make_board_pdf(a.paper, a.color_1, a.color_2, a.output, tag_cm=a.tag_size,
                   border=a.border, border_w_cm=a.border_w, border_h_cm=a.border_h)
