# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""Planning the robot's turn: what it says and the pick-and-place steps it gives the arm."""
from checkers_game import occupancy as occ
from checkers_game.game import Step, steps_for
from checkers_game.rules import from_fen, legal_moves


def move_in(fen, text):
    board, side = from_fen(fen)
    return board, next(m for m in legal_moves(board, side) if str(m) == text)


def test_double_jump_is_one_carry_and_two_removals():
    board, m = move_in("W:W27,32:B23,14,1", "27x18x9")
    assert steps_for(board, m) == [Step("move", 27, 9, 1, "blue"), Step("remove", 23, None, 1, "red"),
                                   Step("remove", 14, None, 1, "red")]


def test_jump_onto_the_king_row_adds_a_crown_step():
    board, m = move_in("W:W9:B6,7,20", "9x2")
    assert [s.kind for s in steps_for(board, m)] == ["move", "remove", "crown"]
    assert steps_for(board, m)[-1] == Step("crown", 2, None, 1, "blue")


def test_kings_are_stacks_of_two():
    board, m = move_in("W:WK14:BK18,1", "14x23")
    assert steps_for(board, m) == [Step("move", 14, 23, 2, "blue"), Step("remove", 18, None, 2, "red")]


def test_circular_king_jump_moves_nothing_and_removes_four():
    board, m = move_in("W:WK18:B6,7,14,15", "18x9x2x11x18")
    assert [s.kind for s in steps_for(board, m)] == ["remove"] * 4


def test_step_count_is_one_plus_captures_plus_crown():
    board, side = from_fen("B:W18,27,29:B14,1")
    m = legal_moves(board, side)[0]
    assert len(steps_for(board, m)) == 1 + len(m.captures) + int(m.crowns)


def test_plan_changes_nothing_until_done(at):
    g = at("W:W27,32:B23,14,1")
    plan = g.plan_robot_turn()
    assert plan.ok and plan.say == "I'm moving f2 to d4 to b6, capturing e3 and c5."
    assert g.to_move == "blue"
    g.robot_turn_done(plan)
    assert g.to_move == "red" and g.board[8] == "b" and g.board[22] == "."


def test_robot_announces_crowning(at):
    g = at("W:W9:B6,7,20")
    assert g.plan_robot_turn().say == "I'm moving b6 to d8, capturing c7. That piece is now a king."


def test_verify_survey_finds_a_misplaced_piece(at):
    g = at("W:W27,32:B23,14,1")
    plan = g.plan_robot_turn()
    g.robot_turn_done(plan)
    good = g.occupancy()
    assert g.check_board(good).ok
    bad = list(good)
    bad[8], bad[12] = ".", "b"                                  # landed on 13 instead of 9
    r = g.check_board("".join(bad))
    assert r.status == "mismatch" and r.squares == [9, 13]


def test_robot_reports_game_over_instead_of_moving(at):
    g = at("W:W29:B25,22")
    assert g.plan_robot_turn().status == "game_over"
    assert g.plan_robot_turn().say == "Red wins: blue has no legal move."


def test_robot_waits_on_the_human_turn(at):
    assert at("B:W21-32:B1-12").plan_robot_turn().status == "not_robot_turn"


def test_camera_view_never_shows_kings():
    assert occ.from_board("R" + "." * 30 + "B") == "r" + "." * 30 + "b"
