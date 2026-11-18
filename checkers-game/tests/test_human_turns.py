# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""Reading the human's turn from a camera occupancy string (colors only, no kings).

Robot = blue, human = red unless stated. Each test builds what the camera would see.
"""
from checkers_game import occupancy as occ
from checkers_game.rules import apply


def seen(game, move_text=None, edits=None):
    """Camera view after the legal move `move_text` (or of the current board), with optional
    per-square edits {square: 'r' | 'b' | '.' | '?'}."""
    board = game.board
    if move_text:
        board = apply(board, next(m for m in game.legal() if str(m) == move_text))
    view = list(occ.from_board(board))
    for sq, c in (edits or {}).items():
        view[sq - 1] = c
    return "".join(view)


DOUBLE = "B:W18,27,29:B14,1"            # red must play 14x23x32 (crowns)


def test_complete_double_jump_is_accepted_and_crowned(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, "14x23x32"))
    assert r.ok and str(r.move) == "14x23x32"
    assert r.say == ("I read that as c5 to e3 to g1, capturing d4 and f2. "
                     "Your piece reached the king row. I'm crowning it; your turn is over.")
    assert [s.kind for s in r.steps] == ["crown"] and r.steps[0].square == 32
    assert g.board[31] == "R" and g.to_move == "blue"


def test_stopped_part_way_through_a_multi_jump(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, edits={14: ".", 23: "r", 18: "."}))
    assert r.status == "incomplete_jump" and r.say == "That piece must keep jumping from e3."
    assert g.to_move == "red"                                   # nothing was played


def test_jumped_but_captured_pieces_left_on_board(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, edits={14: ".", 32: "r"}))
    assert r.status == "captured_not_removed" and r.squares == [18, 27]
    r = g.read_human_turn(seen(g, edits={14: ".", 32: "r", 18: "."}))
    assert r.status == "captured_not_removed" and r.say == "Please remove the captured piece on f2."


def test_ordinary_move_while_a_capture_is_available(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, edits={1: ".", 5: "r"}))
    assert r.status == "must_capture"
    assert r.say == "You must capture this turn: c5 to e3 to g1, capturing d4 and f2."


def test_human_moved_a_robot_piece(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, edits={29: ".", 25: "b"}))
    assert r.status == "displaced" and r.squares == [25, 29]


def test_camera_not_sure_asks_instead_of_guessing(at):
    g = at(DOUBLE)
    r = g.read_human_turn(seen(g, "14x23x32", edits={7: "?"}))
    assert r.status == "unsure" and r.squares == [7] and g.to_move == "red"


def test_nothing_moved(at):
    g = at(DOUBLE)
    assert g.read_human_turn(seen(g)).status == "no_move"


def test_unmatchable_change(at):
    g = at("B:W21-32:B1-12")
    r = g.read_human_turn(seen(g, edits={9: ".", 17: "r"}))     # a two-square jump forward
    assert r.status == "unclear" and r.squares == [9, 17]


def test_each_branch_of_a_branching_capture_is_told_apart(at):
    fen = "W:W27:B23,22,14,15,1"
    for text in ("27x18x9", "27x18x11"):
        g = at(fen, robot="red")
        r = g.read_human_turn(seen(g, text))
        assert r.ok and str(r.move) == text


def test_human_cannot_move_on_the_robot_turn(at):
    g = at("W:W21-32:B1-12")                                    # blue (robot) to move
    assert g.read_human_turn(seen(g)).status == "not_human_turn"


def test_human_king_moves_backward_and_stays_a_king(at):
    g = at("B:WK30:BK18,1")
    r = g.read_human_turn(seen(g, "18-14"))
    assert r.ok and g.board[13] == "R"
