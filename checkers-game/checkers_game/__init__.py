# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""Game logic for the ROT3U checkers robot: rules, turn reading, arm steps."""
from .game import Game, Reply, Step, announce_result, steps_for
from .rules import BLUE, RED, START_BOARD, Move, apply, from_fen, legal_moves, name, to_fen

__version__ = "0.1.0"
__all__ = ["Game", "Reply", "Step", "Move", "RED", "BLUE", "START_BOARD", "apply", "legal_moves",
           "name", "to_fen", "from_fen", "steps_for", "announce_result"]
