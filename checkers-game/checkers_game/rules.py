# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""American checkers rules (WCDF "Rules of Draughts (Checkers)"), no third-party libraries.

Board and numbering
    The 32 dark squares are numbered 1-32 (WCDF rule 1.5). Square 1 is in the top row of the
    picture, second column; numbers run left to right, top to bottom:

         .  1  .  2  .  3  .  4        row 0   (a8 is the top-left corner, h1 the bottom-right)
         5  .  6  .  7  .  8  .        row 1
         ...
        29  . 30  . 31  . 32  .        row 7

    This matches the vision code (row 0 = top of the flattened image) and py-draughts.

Colors
    'r' = red man, 'R' = red king: Red moves first (WCDF 1.13), starts on squares 1-12 and moves
          toward 29-32.
    'b' = blue man, 'B' = blue king: starts on 21-32 and moves toward 1-4.
    '.' = empty.
    A position is a 32-character string, character i = square i + 1, plus the side to move.
"""
from __future__ import annotations

from dataclasses import dataclass

RED, BLUE = "red", "blue"
START_BOARD = "r" * 12 + "." * 8 + "b" * 12

FILES = "abcdefgh"


def row_col(sq: int) -> tuple[int, int]:
    """Square 1-32 -> (row 0-7 from the top, column 0-7 from the left)."""
    i = sq - 1
    row = i // 4
    return row, 2 * (i % 4) + (1 if row % 2 == 0 else 0)


def square_at(row: int, col: int) -> int | None:
    """(row, col) -> square 1-32, or None for a light square or off the board."""
    if not (0 <= row < 8 and 0 <= col < 8) or (row + col) % 2 == 0:
        return None
    return row * 4 + col // 2 + 1


def name(sq: int) -> str:
    """Square 1-32 -> algebraic name, e.g. 1 -> 'b8', 22 -> 'c3'."""
    row, col = row_col(sq)
    return f"{FILES[col]}{8 - row}"


def color_of(piece: str) -> str | None:
    return RED if piece in "rR" else BLUE if piece in "bB" else None


def other(color: str) -> str:
    return BLUE if color == RED else RED


@dataclass(frozen=True)
class Move:
    """One complete turn: every square the piece visits, and every square it jumps over."""

    path: tuple[int, ...]
    captures: tuple[int, ...] = ()
    crowns: bool = False

    def __str__(self) -> str:
        sep = "x" if self.captures else "-"
        return sep.join(str(s) for s in self.path)

    def spoken(self) -> str:
        text = " to ".join(name(s) for s in self.path)
        if self.captures:
            text += ", capturing " + " and ".join(name(s) for s in self.captures)
        return text


def _directions(piece: str) -> tuple[tuple[int, int], ...]:
    if piece in "RB":
        return ((1, -1), (1, 1), (-1, -1), (-1, 1))
    return ((1, -1), (1, 1)) if piece == "r" else ((-1, -1), (-1, 1))


def _crown_row(piece: str) -> int:
    return 7 if piece == "r" else 0


def _jumps(board: str, start: int, piece: str) -> list[Move]:
    """Every complete capture chain for the piece on `start` (WCDF 1.19, 1.20)."""
    enemy = other(color_of(piece))
    out: list[Move] = []

    def dfs(sq: int, path: list[int], caps: list[int]) -> None:
        row, col = row_col(sq)
        extended = False
        for dr, dc in _directions(piece):
            over = square_at(row + dr, col + dc)
            land = square_at(row + 2 * dr, col + 2 * dc)
            if over is None or land is None or over in caps:
                continue                                   # a piece is jumped only once (1.20)
            if color_of(board[over - 1]) != enemy:
                continue
            if board[land - 1] != "." and land != start:   # the mover has left its start square
                continue
            extended = True
            path.append(land)
            caps.append(over)
            crowned = piece in "rb" and row_col(land)[0] == _crown_row(piece)
            if crowned:                                    # crowning ends the turn (1.16, 1.19)
                out.append(Move(tuple(path), tuple(caps), True))
            else:
                dfs(land, path, caps)
            path.pop()
            caps.pop()
        if not extended and caps:
            out.append(Move(tuple(path), tuple(caps), False))

    dfs(start, [start], [])
    unique: dict[tuple, Move] = {}
    for m in out:                                          # same result reached by two routes
        unique.setdefault((m.path[0], m.path[-1], frozenset(m.captures)), m)
    return list(unique.values())


def quiet_moves(board: str, to_move: str) -> list[Move]:
    """One-square moves (1.15, 1.17), whether or not a capture is available."""
    out: list[Move] = []
    for sq in range(1, 33):
        piece = board[sq - 1]
        if color_of(piece) != to_move:
            continue
        row, col = row_col(sq)
        for dr, dc in _directions(piece):
            dest = square_at(row + dr, col + dc)
            if dest is not None and board[dest - 1] == ".":
                crowns = piece in "rb" and row + dr == _crown_row(piece)
                out.append(Move((sq, dest), (), crowns))
    return out


def capture_moves(board: str, to_move: str) -> list[Move]:
    """Every complete capture chain for the side to move."""
    out: list[Move] = []
    for sq in range(1, 33):
        piece = board[sq - 1]
        if color_of(piece) == to_move:
            out += _jumps(board, sq, piece)
    return out


def legal_moves(board: str, to_move: str) -> list[Move]:
    """All legal turns. Captures are compulsory (1.20); any capture chain may be chosen."""
    return capture_moves(board, to_move) or quiet_moves(board, to_move)


def apply(board: str, move: Move) -> str:
    """Board after `move` (captured pieces removed, a man reaching the far row crowned)."""
    cells = list(board)
    piece = cells[move.path[0] - 1]
    cells[move.path[0] - 1] = "."
    for c in move.captures:
        cells[c - 1] = "."
    cells[move.path[-1] - 1] = piece.upper() if move.crowns else piece
    return "".join(cells)


def to_fen(board: str, to_move: str) -> str:
    """Standard checkers FEN. Red is written as 'B' (the side on 1-12 that moves first) so the
    string is valid for py-draughts and most checkers software; blue is written as 'W'."""
    red = [("K" if p == "R" else "") + str(i + 1) for i, p in enumerate(board) if p in "rR"]
    blue = [("K" if p == "B" else "") + str(i + 1) for i, p in enumerate(board) if p in "bB"]
    side = "B" if to_move == RED else "W"
    return f"{side}:W{','.join(blue)}:B{','.join(red)}"


def from_fen(fen: str) -> tuple[str, str]:
    """Inverse of to_fen. Accepts ranges ('1-12') and K-prefixed kings. Returns (board, to_move)."""
    fen = fen.strip().strip('[]').replace('FEN', '').strip().strip('"').upper()
    side, *groups = fen.split(":")
    cells = ["."] * 32
    for group in groups[:2]:
        if not group:
            continue
        who, body = group[0], group[1:]
        for token in filter(None, body.split(",")):
            king = token.startswith("K")
            token = token[1:] if king else token
            lo, _, hi = token.partition("-")
            for sq in range(int(lo), int(hi or lo) + 1):
                if who == "B":
                    cells[sq - 1] = "R" if king else "r"
                else:
                    cells[sq - 1] = "B" if king else "b"
    return "".join(cells), RED if side == "B" else BLUE
