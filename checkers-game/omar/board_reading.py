# Suggested additions to utils.py of https://github.com/fresh4/checkers-bot
# Same license terms as that repository.
"""Suggested additions to the group vision code (fresh4/checkers-bot, utils.py and main.py).

Drop this file next to utils.py. It reuses that code's functions unchanged and adds:
  1. board_to_fen_fixed(): red is the side on squares 1-12 that moves first (WCDF rule 1.13), so
     it is written 'B' in the FEN. The current board_to_fen() writes red as 'W', and py-draughts
     rejects the result ("man on promotion square 1").
  2. board_to_occupancy(): the 32-character string the game logic reads ('r', 'b', '.', '?').
     The camera reports colors only: a stacked king looks like one checker from above, so kings
     are tracked by the game logic, not by the camera.
  3. A "not sure" result: a square is '?' when both colors show, or when the color fraction is
     close to the threshold. The robot then asks instead of guessing.
  4. A light-square check: piece color on a light square means a hand or a displaced checker.
  5. read_board(frame): works on a camera frame (a numpy image), not only a file, and returns a
     result instead of raising when a tag is hidden.
"""
from __future__ import annotations

SQUARE_ORDER = [(row, col) for row in range(8) for col in range(8) if (row + col) % 2 == 1]


# ---------------------------------------------------------------- pure functions (no OpenCV)
def board_to_occupancy(board_state):
    """8 x 8 matrix from detect_pieces() -> 32 characters, square 1 first ('R'/'B' -> 'r'/'b')."""
    out = []
    for row, col in SQUARE_ORDER:
        cell = str(board_state[row][col]).lower()
        out.append(cell if cell in ("r", "b", ".", "?") else "?")
    return "".join(out)


def board_to_fen_fixed(board_state, red_to_move=True):
    """FEN for the solver. Red = 'B' side (squares 1-12, moves first); blue = 'W' side."""
    red, blue = [], []
    for number, (row, col) in enumerate(SQUARE_ORDER, start=1):
        piece = board_state[row][col]
        if piece in ("r", "R"):
            red.append(("K" if piece == "R" else "") + str(number))
        elif piece in ("b", "B"):
            blue.append(("K" if piece == "B" else "") + str(number))
    side = "B" if red_to_move else "W"
    return f"{side}:W{','.join(blue)}:B{','.join(red)}"


def classify_fractions(red_fraction, blue_fraction, minimum=0.05, doubt_band=0.5, both_ratio=0.5):
    """'r', 'b', '.' or '?' from the color fractions classify_square() already computes.

    '?' when both colors are present (the weaker one above both_ratio of the stronger and above
    the minimum), or when the stronger one is between doubt_band x minimum and minimum.
    """
    hi, lo = max(red_fraction, blue_fraction), min(red_fraction, blue_fraction)
    color = "r" if red_fraction >= blue_fraction else "b"
    if hi >= minimum:
        if lo >= minimum and lo >= both_ratio * hi:
            return "?"
        return color
    if hi >= doubt_band * minimum:
        return "?"
    return "."


# ------------------------------------------------------------------ camera image functions
def square_fractions(image, row, col, grid_data, red_ranges, blue_ranges, padding_fraction=0.15):
    """Red and blue pixel fractions in the center of one square (same method as classify_square)."""
    import cv2
    import numpy as np
    from utils import create_color_mask

    x1 = grid_data["grid_start_x"] + col * grid_data["square_width"]
    y1 = grid_data["grid_start_y"] + row * grid_data["square_height"]
    px = grid_data["square_width"] * padding_fraction
    py = grid_data["square_height"] * padding_fraction
    roi = image[int(y1 + py):int(y1 + grid_data["square_height"] - py),
                int(x1 + px):int(x1 + grid_data["square_width"] - px)]
    if roi.size == 0:
        return 0.0, 0.0
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    fr = []
    for ranges in (red_ranges, blue_ranges):
        mask = cv2.morphologyEx(create_color_mask(hsv, ranges), cv2.MORPH_OPEN, kernel)
        fr.append(np.count_nonzero(mask) / (roi.shape[0] * roi.shape[1]))
    return fr[0], fr[1]


def read_board(frame, tag_positions=None, board_size=800, margin_fraction=4.0 / (20.32 + 8.0),
               offset_x=-6, offset_y=8, minimum=0.05, color_ranges=None):
    """Read one camera frame. Returns a dict:

        ok            False if the board could not be found (missing tag); then only 'error' is set
        occupancy     32 characters 'r', 'b', '.', '?'
        unsure        squares 1-32 marked '?'
        light_hits    (row, col) of light squares showing piece color (hand or displaced checker)
        fen           debug FEN, red to move (the game logic keeps the real side to move)
        fractions     (red, blue) per square 1-32, for tuning thresholds

    Defaults match the current main.py (8 in board, 4 cm tag offset, grid offsets -6, +8).
    """
    from utils import construct_grid, detect_tags, load_color_calibration, warp_board

    tag_positions = tag_positions or {0: "top-right", 1: "top-left", 2: "bottom-right", 3: "bottom-left"}
    red_ranges, blue_ranges = color_ranges or load_color_calibration()
    try:
        detections = detect_tags(frame, tag_positions)
    except ValueError as err:                       # missing or duplicate tag: ask, don't guess
        return {"ok": False, "error": str(err)}
    warped, _ = warp_board(frame, detections, board_size)
    grid = construct_grid(board_size, 8, margin_fraction, offset_x, offset_y)

    state = [["." for _ in range(8)] for _ in range(8)]
    fractions, light_hits = [], []
    for row in range(8):
        for col in range(8):
            r, b = square_fractions(warped, row, col, grid, red_ranges, blue_ranges)
            if (row + col) % 2 == 1:
                state[row][col] = classify_fractions(r, b, minimum)
                fractions.append((round(r, 3), round(b, 3)))
            elif max(r, b) >= minimum:
                light_hits.append((row, col))
    occupancy = board_to_occupancy(state)
    return {
        "ok": True,
        "occupancy": occupancy,
        "unsure": [i + 1 for i, c in enumerate(occupancy) if c == "?"],
        "light_hits": light_hits,
        "fen": board_to_fen_fixed(state),
        "fractions": fractions,
    }


def grab_frame(camera_index=0, warmup=5):
    """One frame from a USB camera (a few frames are dropped while exposure settles)."""
    import cv2

    cap = cv2.VideoCapture(camera_index)
    try:
        frame = None
        for _ in range(warmup + 1):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"camera {camera_index} returned no frame")
        return frame
    finally:
        cap.release()
