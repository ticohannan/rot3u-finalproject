# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""A small stand-in engine: alpha-beta search on material and advancement.

Any engine can replace it: the game only needs a function (board, to_move) -> Move that
returns one of rules.legal_moves(board, to_move).
"""
from __future__ import annotations

import random

from .rules import BLUE, RED, Move, apply, legal_moves, other, row_col

WIN = 10_000


def evaluate(board: str, side: str) -> float:
    score = 0.0
    for i, p in enumerate(board):
        if p == ".":
            continue
        row = row_col(i + 1)[0]
        value = 1.6 if p in "RB" else 1.0 + 0.04 * (row if p == "r" else 7 - row)
        score += value if (p in "rR") == (side == RED) else -value
    return score


def _search(board: str, side: str, depth: int, alpha: float, beta: float) -> float:
    moves = legal_moves(board, side)
    if not moves:
        return -WIN - depth                                # no move = loss (WCDF 1.30)
    if depth == 0:
        return evaluate(board, side)
    best = -float("inf")
    for m in moves:
        val = -_search(apply(board, m), other(side), depth - 1, -beta, -alpha)
        best = max(best, val)
        alpha = max(alpha, val)
        if alpha >= beta:
            break
    return best


def best_move(board: str, side: str, depth: int = 4, rng: random.Random | None = None) -> Move:
    moves = legal_moves(board, side)
    if not moves:
        raise ValueError("no legal moves: the game is over")
    rng = rng or random.Random(0)
    scored = []
    for m in moves:
        scored.append((-_search(apply(board, m), other(side), depth - 1, -float("inf"), float("inf")), m))
    top = max(s for s, _ in scored)
    return rng.choice([m for s, m in scored if s == top])


__all__ = ["best_move", "evaluate", "RED", "BLUE"]
