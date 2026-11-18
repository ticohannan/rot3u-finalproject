# Copyright (c) 2026 Tico Hannan. BSD-3-Clause (see LICENSE at the repository root).
"""One game, kept from the first move to the last.

The game state is the record of truth. The camera never sets it; each camera reading is checked
against it. That is what lets the robot know whose turn it is, which checkers are kings (a stack
looks like one checker from above), whether a jump was left unfinished, and when a game is drawn.

Rule numbers are from the WCDF "Rules of Draughts (Checkers)":
https://wcdf.net/rules/rules_of_checkers_english.pdf
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Callable

from . import occupancy as occ
from .engine import best_move
from .rules import BLUE, RED, START_BOARD, Move, apply, color_of, legal_moves, name, other, quiet_moves

Engine = Callable[[str, str], Move]


@dataclass(frozen=True)
class Step:
    """One pick-and-place job for the arm.

    kind 'move'   carry the stack on `square` to `to` (2 checkers for a king)
    kind 'remove' take the stack on `square` to its owner's tray
    kind 'crown'  take a `color` checker from its tray and stack it on `square`
    """

    kind: str
    square: int
    to: int | None = None
    checkers: int = 1
    color: str = ""

    def __str__(self) -> str:
        if self.kind == "move":
            return f"move {self.checkers} checker(s) {name(self.square)} -> {name(self.to)}"
        if self.kind == "remove":
            return f"remove {self.checkers} checker(s) from {name(self.square)} to the {self.color} tray"
        return f"crown {name(self.square)} with a {self.color} checker from its tray"


@dataclass
class Reply:
    """Result of reading or planning a turn. `say` is what the robot speaks or prints."""

    status: str
    say: str
    move: Move | None = None
    steps: list[Step] = field(default_factory=list)
    squares: list[int] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == "ok"


def steps_for(board: str, move: Move) -> list[Step]:
    """Turn one legal move into arm jobs. The piece is carried straight from its first square to
    its last; the squares in between are not touched. A circular king jump moves nothing."""
    piece = board[move.path[0] - 1]
    mover = color_of(piece)
    jobs = []
    if move.path[0] != move.path[-1]:
        jobs.append(Step("move", move.path[0], move.path[-1], 2 if piece.isupper() else 1, mover))
    for c in move.captures:
        jobs.append(Step("remove", c, None, 2 if board[c - 1].isupper() else 1, other(mover)))
    if move.crowns:
        jobs.append(Step("crown", move.path[-1], None, 1, mover))
    return jobs


class Game:
    def __init__(self, robot: str = BLUE, board: str = START_BOARD, to_move: str = RED,
                 engine: Engine | None = None, quiet_limit: int = 80):
        self.robot = robot
        self.human = other(robot)
        self.board = board
        self.to_move = to_move
        self.engine = engine or (lambda b, side: best_move(b, side))
        self.quiet_limit = quiet_limit       # 40 moves by each player = 80 turns (WCDF 1.32.2)
        self.quiet = 0
        self.moves: list[Move] = []
        self.seen = Counter([(board, to_move)])

    # ------------------------------------------------------------------ state
    def occupancy(self) -> str:
        return occ.from_board(self.board)

    def legal(self) -> list[Move]:
        return legal_moves(self.board, self.to_move)

    def result(self) -> tuple[str, str] | None:
        """None while play continues, else (winner or 'draw', reason)."""
        if not self.legal():
            reason = "has no pieces left" if self.to_move not in map(color_of, self.board) else "has no legal move"
            return other(self.to_move), f"{self.to_move} {reason}"                 # WCDF 1.30
        if self.seen[(self.board, self.to_move)] >= 3:
            return "draw", "same position for the third time"                       # WCDF 1.32.1
        if self.quiet >= self.quiet_limit:
            return "draw", "40 moves each with no capture and no man moved"          # WCDF 1.32.2
        return None

    def _play(self, move: Move) -> None:
        man_moved = self.board[move.path[0] - 1].islower()
        self.board = apply(self.board, move)
        self.quiet = 0 if (man_moved or move.captures) else self.quiet + 1
        self.to_move = other(self.to_move)
        self.moves.append(move)
        self.seen[(self.board, self.to_move)] += 1

    # ------------------------------------------------------------- robot turn
    def plan_robot_turn(self) -> Reply:
        """Choose the robot's move. Nothing changes until robot_turn_done()."""
        if self.to_move != self.robot:
            return Reply("not_robot_turn", "It's your move.")
        if (end := self.result()) is not None:
            return Reply("game_over", announce_result(end))
        move = self.engine(self.board, self.to_move)
        if move not in self.legal():
            raise ValueError(f"engine returned an illegal move: {move}")
        say = f"I'm moving {move.spoken()}."
        if move.crowns:
            say += " That piece is now a king."
        return Reply("ok", say, move, steps_for(self.board, move))

    def robot_turn_done(self, plan: Reply) -> None:
        """Call after the arm finished the steps of plan_robot_turn()."""
        self._play(plan.move)

    # ------------------------------------------------------------- human turn
    def read_human_turn(self, seen: str) -> Reply:
        """Match a camera reading, taken after the human says "done", with the legal moves.

        On 'ok' the move is played and any crowning step for the arm is returned.
        Any other status changes nothing and says what the human should fix.
        """
        occ.check(seen)
        if self.to_move != self.human:
            return Reply("not_human_turn", "It's my move.")
        before = self.occupancy()
        unsure = [i + 1 for i, c in enumerate(seen) if c == "?"]
        if unsure:
            return Reply("unsure", "I can't see " + _squares(unsure) + " clearly. Please clear the view.",
                         squares=unsure)
        if seen == before:
            return Reply("no_move", "I don't see a move yet.")
        legal = self.legal()
        matches = [m for m in legal if occ.from_board(apply(self.board, m)) == seen]
        if len(matches) == 1:
            m = matches[0]
            jobs = [s for s in steps_for(self.board, m) if s.kind == "crown"]
            say = f"I read that as {m.spoken()}."
            if m.crowns:
                say += " Your piece reached the king row. I'm crowning it; your turn is over."
            self._play(m)
            return Reply("ok", say, m, jobs)
        if len(matches) > 1:                       # cannot happen with complete moves; kept as a guard
            return Reply("unclear", "I can match that to more than one move. Please tell me which.")
        return self._explain_mismatch(before, seen, legal)

    def _explain_mismatch(self, before: str, seen: str, legal: list[Move]) -> Reply:
        mine = "r" if self.to_move == RED else "b"
        theirs = "b" if mine == "r" else "r"
        diff = occ.differences(before, seen)
        if all(before[s - 1] == seen[s - 1] or mine not in (before[s - 1], seen[s - 1]) for s in diff):
            return Reply("displaced", "My pieces on " + _squares(diff) + " were moved. Please put them back.",
                         squares=diff)                                               # WCDF 1.27
        for m in legal:
            if not m.captures:
                continue
            start = m.path[0]
            for k in range(1, len(m.captures)):    # stopped part-way through a multi-jump (1.20)
                partial = list(before)
                partial[start - 1] = "."
                partial[m.path[k] - 1] = mine
                for c in m.captures[:k]:
                    partial[c - 1] = "."
                if "".join(partial) == seen:
                    return Reply("incomplete_jump", f"That piece must keep jumping from {name(m.path[k])}.",
                                 m, squares=[m.path[k]])
            done = list(occ.from_board(apply(self.board, m)))
            left = [c for c in m.captures if seen[c - 1] == theirs]
            for c in left:
                done[c - 1] = theirs
            if left and "".join(done) == seen:
                return Reply("captured_not_removed", "Please remove the captured piece on " + _squares(left) + ".",
                             m, squares=left)
        if any(m.captures for m in legal):
            if any(occ.from_board(apply(self.board, q)) == seen for q in quiet_moves(self.board, self.to_move)):
                options = ", or ".join(m.spoken() for m in legal)
                return Reply("must_capture", f"You must capture this turn: {options}.")       # 1.20
        return Reply("unclear", "I can't match that to a legal move. Please check "
                     + _squares(diff) + " and confirm the start and end squares.", squares=diff)

    # -------------------------------------------------------------- checking
    def check_board(self, seen: str) -> Reply:
        """Compare a camera reading with the game state (after the robot moved, or any time)."""
        occ.check(seen)
        wrong = occ.differences(self.occupancy(), seen)
        unsure = [i + 1 for i, c in enumerate(seen) if c == "?"]
        if wrong:
            return Reply("mismatch", "The board doesn't match my record at " + _squares(wrong) + ".",
                         squares=wrong)
        if unsure:
            return Reply("unsure", "I can't see " + _squares(unsure) + " clearly.", squares=unsure)
        return Reply("ok", "Board checked.")


def _squares(squares: list[int]) -> str:
    return ", ".join(name(s) for s in squares)


def announce_result(end: tuple[str, str]) -> str:
    winner, reason = end
    if winner == "draw":
        return f"The game is a draw: {reason}."
    return f"{winner.capitalize()} wins: {reason}."
