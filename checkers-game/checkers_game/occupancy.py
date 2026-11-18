# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""What the camera reports, and how it is compared with the game.

Occupancy string (the one contract between vision and game logic)
    32 characters, character i = square i + 1 (numbering in rules.py):
        'r' red checker or red king (the camera does not tell them apart)
        'b' blue checker or blue king
        '.' empty
        '?' the camera is not sure about this square
"""
from __future__ import annotations

from .rules import row_col

VALID = set("rb.?")


def from_board(board: str) -> str:
    """Game board ('rRbB.') -> what a camera would see ('rb.')."""
    return board.lower()


def from_matrix(matrix: list[list[str]]) -> str:
    """The vision code's 8 x 8 matrix (rows from the top of the flattened image) -> occupancy.

    Only dark squares are read. 'R' and 'B' are accepted and reported as plain 'r' and 'b'.
    """
    out = []
    for sq in range(1, 33):
        row, col = row_col(sq)
        cell = str(matrix[row][col])
        out.append(cell.lower() if cell.lower() in VALID else "?")
    return "".join(out)


def check(occupancy: str) -> str:
    if len(occupancy) != 32 or set(occupancy) - VALID:
        raise ValueError(f"occupancy must be 32 characters from 'rb.?': {occupancy!r}")
    return occupancy


def differences(expected: str, seen: str) -> list[int]:
    """Squares (1-32) where the camera disagrees with the expected occupancy, '?' excluded."""
    return [i + 1 for i, (a, b) in enumerate(zip(expected, seen)) if b != "?" and a != b]
