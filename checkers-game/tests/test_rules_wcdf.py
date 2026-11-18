# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""One test per rule of the WCDF "Rules of Draughts (Checkers)" that a robot game depends on.
https://wcdf.net/rules/rules_of_checkers_english.pdf

Positions are FEN: "B:" = red to move, "W:" = blue to move; the B list is red (squares 1-12 at the
start), the W list is blue (21-32); K marks a king.
"""
from checkers_game.rules import (BLUE, RED, START_BOARD, from_fen, legal_moves, name, row_col,
                                 to_fen)


def moves(fen):
    board, side = from_fen(fen)
    return {str(m): m for m in legal_moves(board, side)}


def test_1_5_square_numbering_and_names():
    assert [name(s) for s in (1, 4, 5, 22, 29, 32)] == ["b8", "h8", "a7", "c3", "a1", "g1"]
    assert from_fen("B:W21-32:B1-12") == (START_BOARD, RED)
    assert to_fen(START_BOARD, RED) == "B:W" + ",".join(map(str, range(21, 33))) + ":B" + ",".join(map(str, range(1, 13)))


def test_1_13_red_moves_first_with_seven_moves():
    m = moves("B:W21-32:B1-12")
    assert sorted(m) == ["10-14", "10-15", "11-15", "11-16", "12-16", "9-13", "9-14"]


def test_1_15_men_move_forward_one_square():
    for fen, forward in (("B:W30:B14", +1), ("W:W14:B1", -1)):
        for m in moves(fen).values():
            assert row_col(m.path[1])[0] - row_col(m.path[0])[0] == forward


def test_1_17_kings_move_one_square_any_diagonal():
    assert sorted(moves("W:WK18:B1")) == ["18-14", "18-15", "18-22", "18-23"]


def test_1_20_capture_is_compulsory():
    assert sorted(moves("W:W18,30:B14,1")) == ["18x9"]           # 30-26 and 30-25 are not allowed


def test_1_20_any_capture_may_be_chosen_not_only_the_longest():
    m = moves("W:W27,31:B23,14,26,1")
    assert sorted(m) == ["27x18x9", "31x22"]                   # a 2-piece and a 1-piece capture


def test_1_20_branching_multi_jump_offers_each_complete_path():
    m = moves("W:W27:B23,22,14,15,1")
    assert sorted(m) == ["27x18x11", "27x18x9"]


def test_1_19_1_20_multi_jump_must_be_finished():
    m = moves("W:W27,32:B23,14,1")
    assert sorted(m) == ["27x18x9"]                             # stopping on 18 is not a move
    assert m["27x18x9"].captures == (23, 14)


def test_men_capture_forward_only():
    assert all(not m.captures for m in moves("W:W14:B18,1").values())


def test_kings_capture_backward():
    m = moves("W:WK14:B18,1")
    assert sorted(m) == ["14x23"]


def test_1_16_1_19_crowning_ends_the_turn_even_during_a_jump():
    m = moves("W:W9:B6,7,20")
    assert sorted(m) == ["9x2"]                                 # a king could go on over 7; a new king may not
    assert m["9x2"].crowns


def test_crowning_on_a_red_multi_jump():
    m = moves("B:W18,27,29:B14,1")
    assert sorted(m) == ["14x23x32"] and m["14x23x32"].crowns


def test_king_may_jump_in_a_circle_back_to_its_start():
    m = moves("W:WK18:B6,7,14,15")
    assert list(m) == ["18x9x2x11x18"]


def test_1_30_no_move_or_no_pieces_loses(at):
    assert at("W:W29:B25,22").result() == (RED, "blue has no legal move")
    assert at("W::B1,2").result() == (RED, "blue has no pieces left")
    assert at("B:W21-32:B1-12").result() is None


SHUFFLE = {"4-8", "8-4", "29-25", "25-29"}          # each king steps out and back


def shuffle(g):
    g._play(next(x for x in g.legal() if str(x) in SHUFFLE))


def test_1_32_1_third_repetition_is_a_draw(at):
    g = at("B:WK29:BK4", robot=BLUE)
    for _ in range(7):
        shuffle(g)
        assert g.result() is None
    shuffle(g)                                                  # start position for the third time
    assert g.result() == ("draw", "same position for the third time")


def test_1_32_2_forty_moves_each_without_capture_or_man_move_is_a_draw(at):
    g = at("B:WK29:BK4,13", robot=BLUE)
    for _ in range(79):
        shuffle(g)
        g.seen.clear()                                          # isolate the 40-move rule from repetition
    assert g.quiet == 79 and g.result() is None
    shuffle(g)
    g.seen.clear()
    assert g.result() == ("draw", "40 moves each with no capture and no man moved")


def test_a_man_move_resets_the_forty_move_count(at):
    g = at("B:WK29:BK4,13", robot=BLUE)
    g._play(next(x for x in g.legal() if x.path[0] == 4))
    assert g.quiet == 1
    g._play(next(iter(g.legal())))
    g._play(next(x for x in g.legal() if x.path[0] == 13))
    assert g.quiet == 0
