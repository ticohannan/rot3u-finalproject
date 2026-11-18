# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
for sub in ("", "justin", "omar"):
    p = str(HERE / sub)
    if p not in sys.path:
        sys.path.insert(0, p)

import pytest  # noqa: E402

from checkers_game import Game, from_fen  # noqa: E402


def game_at(fen, robot="blue"):
    """A Game set to a position given in FEN (red = 'B' side, squares 1-12, moves first)."""
    board, to_move = from_fen(fen)
    return Game(robot=robot, board=board, to_move=to_move)


@pytest.fixture
def at():
    return game_at
