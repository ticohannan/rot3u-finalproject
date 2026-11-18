# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""Whole games through the Game interface, with the camera simulated from colors only."""
import random

from checkers_game import Game
from checkers_game import occupancy as occ
from checkers_game.rules import apply


def play(seed, robot="blue", noise=0.0, max_turns=200):
    """Robot (engine) against a random human. The human's moves reach the game only as camera
    occupancy strings; with `noise`, some readings get a '?' and must be retaken."""
    rng = random.Random(seed)
    g = Game(robot=robot)
    counts = {"turns": 0, "captures": 0, "multi": 0, "crowns": 0, "retakes": 0}
    while g.result() is None and counts["turns"] < max_turns:
        if g.to_move == g.robot:
            plan = g.plan_robot_turn()
            g.robot_turn_done(plan)
            assert g.check_board(g.occupancy()).ok
            move = plan.move
        else:
            move = rng.choice(g.legal())
            view = occ.from_board(apply(g.board, move))
            if rng.random() < noise:
                blurred = list(view)
                blurred[rng.randrange(32)] = "?"
                r = g.read_human_turn("".join(blurred))
                assert r.status == "unsure"
                counts["retakes"] += 1
            r = g.read_human_turn(view)
            assert r.ok and r.move == move, (r.status, r.say)
        counts["turns"] += 1
        counts["captures"] += bool(move.captures)
        counts["multi"] += len(move.captures) > 1
        counts["crowns"] += move.crowns
    return g, counts


def test_twenty_games_every_human_move_read_from_colors_alone():
    total = {"turns": 0, "captures": 0, "multi": 0, "crowns": 0, "retakes": 0}
    results = []
    for seed in range(20):
        g, c = play(seed, robot="blue" if seed % 2 else "red", noise=0.1)
        results.append(g.result())
        for k in total:
            total[k] += c[k]
    assert total["multi"] > 0 and total["crowns"] > 0 and total["retakes"] > 0
    assert sum(1 for r in results if r is not None) >= 18        # games reach an end within 200 turns


def test_engine_beats_a_random_mover_most_of_the_time():
    wins = 0
    for seed in range(10):
        g, _ = play(100 + seed)
        end = g.result()
        wins += bool(end) and end[0] == g.robot
    assert wins >= 8


def test_scripted_opening_with_an_exchange():
    """Red 11-15, blue 24-19, red 15x24, blue 28x19: a capture each way, all through the camera view."""
    g = Game(robot="blue")
    script = ["11-15", "24-19", "15x24", "28x19"]
    for text in script:
        move = next(m for m in g.legal() if str(m) == text)
        if g.to_move == g.human:
            assert g.read_human_turn(occ.from_board(apply(g.board, move))).ok
        else:
            g.engine = lambda b, s, m=move: m                   # force the scripted robot move
            g.robot_turn_done(g.plan_robot_turn())
    assert g.board.count("r") == 11 and g.board.count("b") == 11 and g.to_move == "red"
