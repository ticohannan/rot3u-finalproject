# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""The suggested vision functions. The photo test runs only when CHECKERS_VISION_REPO points to a
checkout of the vision code (it uses that code's utils.py, sample photos and color calibration):

    set CHECKERS_VISION_REPO=C:\\path\\to\\checkers-bot        (Windows)
    export CHECKERS_VISION_REPO=/path/to/checkers-bot         (macOS, Linux)
"""
import os
import sys

import pytest

from board_reading import board_to_fen_fixed, board_to_occupancy, classify_fractions
from checkers_game import Game
from checkers_game.rules import START_BOARD, from_fen

START_MATRIX = [
    [".", "r", ".", "r", ".", "r", ".", "r"],
    ["r", ".", "r", ".", "r", ".", "r", "."],
    [".", "r", ".", "r", ".", "r", ".", "r"],
    ["."] * 8,
    ["."] * 8,
    ["b", ".", "b", ".", "b", ".", "b", "."],
    [".", "b", ".", "b", ".", "b", ".", "b"],
    ["b", ".", "b", ".", "b", ".", "b", "."],
]


def test_matrix_to_occupancy_matches_the_start_position():
    assert board_to_occupancy(START_MATRIX) == START_BOARD
    assert Game().check_board(board_to_occupancy(START_MATRIX)).ok


def test_fixed_fen_puts_red_on_1_to_12_and_red_to_move():
    fen = board_to_fen_fixed(START_MATRIX)
    assert fen.startswith("B:W21,") and fen.endswith(":B1,2,3,4,5,6,7,8,9,10,11,12")
    assert from_fen(fen) == (START_BOARD, "red")


def test_fixed_fen_is_accepted_by_the_solver_library():
    draughts = pytest.importorskip("draughts")
    board = draughts.AmericanBoard.from_fen(board_to_fen_fixed(START_MATRIX))
    assert len(board.legal_moves) == 7


def test_kings_from_the_matrix_are_reported_as_plain_colors():
    m = [row[:] for row in START_MATRIX]
    m[0][1] = "R"
    assert board_to_occupancy(m)[0] == "r"


@pytest.mark.parametrize("red, blue, expected", [
    (0.30, 0.00, "r"), (0.00, 0.35, "b"), (0.00, 0.00, "."),
    (0.03, 0.00, "?"),            # close to the 0.05 threshold
    (0.30, 0.20, "?"),            # both colors (hand, or a piece knocked half onto a square)
    (0.30, 0.06, "r"),            # a little of the other color is not enough to doubt
])
def test_classify_fractions(red, blue, expected):
    assert classify_fractions(red, blue) == expected


@pytest.mark.skipif(not os.environ.get("CHECKERS_VISION_REPO"), reason="CHECKERS_VISION_REPO not set")
def test_sample_photo_reads_as_the_start_position(monkeypatch):
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("pupil_apriltags")
    repo = os.environ["CHECKERS_VISION_REPO"]
    monkeypatch.chdir(repo)                                     # utils loads output/color_calibration.json
    monkeypatch.syspath_prepend(repo)
    sys.modules.pop("utils", None)
    from board_reading import read_board
    result = read_board(cv2.imread(os.path.join(repo, "images", "board_with_pieces.jpg")))
    assert result["ok"] and result["unsure"] == [] and result["light_hits"] == []
    assert Game().check_board(result["occupancy"]).ok
