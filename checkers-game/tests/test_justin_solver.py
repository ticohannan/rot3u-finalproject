# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""The group solver (py-draughts) against the shared rules. Skipped when py-draughts is missing."""
import random

import pytest

draughts = pytest.importorskip("draughts")

from draughts import AmericanBoard  # noqa: E402

from checkers_game import Game  # noqa: E402
from checkers_game.rules import START_BOARD, RED, apply, legal_moves, to_fen  # noqa: E402
from draughts_engine import solver_engine  # noqa: E402
from main import CompulsoryAmericanBoard, main  # noqa: E402


def as_set(moves):
    return {(tuple(m.path), frozenset(m.captures)) for m in moves}


def solver_set(board_cls, board, side):
    b = board_cls.from_fen(to_fen(board, side))
    return {(tuple(s + 1 for s in m.square_list), frozenset(c + 1 for c in m.captured_list))
            for m in b.legal_moves}


def random_positions(n, seed=3):
    rng = random.Random(seed)
    board, side = START_BOARD, RED
    out = []
    while len(out) < n:
        moves = legal_moves(board, side)
        if not moves:
            board, side = START_BOARD, RED
            continue
        out.append((board, side))
        board = apply(board, rng.choice(moves))
        side = "blue" if side == RED else RED
    return out


def test_compulsory_board_agrees_with_shared_rules_in_2000_positions():
    for board, side in random_positions(2000):
        assert solver_set(CompulsoryAmericanBoard, board, side) == as_set(legal_moves(board, side)), to_fen(board, side)


def test_stock_american_board_lets_a_capture_be_skipped():
    """Documents the gap: py-draughts' AmericanBoard lists ordinary moves beside a capture."""
    stock = solver_set(AmericanBoard, *("." * 13 + "r" + "." * 3 + "b" + "." * 14, "blue"))
    assert any(not caps for _, caps in stock) and any(caps for _, caps in stock)


def test_solver_main_runs_on_start_and_game_over(capsys):
    assert main("B:W21-32:B1-12") == 0
    assert main("W:W29:B25,22") == 1
    assert "Game over" in capsys.readouterr().out


def test_solver_as_the_robot_engine_plays_only_legal_moves():
    g = Game(robot="blue", engine=solver_engine(depth=4))
    rng = random.Random(5)
    for _ in range(60):
        if g.result():
            break
        if g.to_move == g.robot:
            g.robot_turn_done(g.plan_robot_turn())
        else:
            g._play(rng.choice(g.legal()))
