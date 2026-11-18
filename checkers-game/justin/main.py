# Suggested changes to main.py of https://github.com/JutLat/Checkers-Robot-Solver-One-Step-with-FEN-notation-
# Same license terms as that repository.
"""One-step solver: FEN in, best move out (suggested update to the group solver's main.py).

Changes from the original:
  1. Captures are compulsory (WCDF rule 1.20). py-draughts treats them as optional for American
     checkers, so its engine can skip a capture.
  2. Red, the side on squares 1-12, moves first (WCDF 1.13). In py-draughts FEN that side is
     written 'B', so the start position is "B:W21-32:B1-12".
  3. The game-over case is reported instead of raising "No legal moves available".
  4. The whole path is printed, with square names, captured squares and crowning.

    python main.py                       # start position, Red (B) to move
    python main.py "W:W18,27:B14,1"      # any position
"""
import sys

from draughts import AmericanBoard, SimpleEngine
from draughts.boards._core import CORE_AMERICAN

START = "B:W21-32:B1-12"


class CompulsoryAmericanBoard(AmericanBoard):
    """American checkers with compulsory captures (any capture chain may be chosen).

    Relies on a private py-draughts method; tested with py-draughts 1.10.1 only.
    """

    @property
    def legal_moves(self):
        return self._legal_moves_from_core(CORE_AMERICAN, max_capture=False, captures_optional=False)


def square_name(index0):
    return AmericanBoard.SQUARE_NAMES[index0]


def describe(move):
    path = " -> ".join(f"{s + 1} ({square_name(s)})" for s in move.square_list)
    text = path
    if move.captured_list:
        text += "  capturing " + ", ".join(f"{c + 1} ({square_name(c)})" for c in move.captured_list)
    return text


def main(fen):
    board = CompulsoryAmericanBoard.from_fen(fen)
    moves = board.legal_moves
    if not moves:
        loser = "White (W)" if str(board.turn).endswith("WHITE") else "Black (B)"
        print(f"Game over: {loser} has no legal move.")
        return 1
    print("Legal moves:", ", ".join(str(m) for m in moves))
    move = SimpleEngine().get_best_move(board)
    print("Best move:  ", move, "|", describe(move))
    board.push(move)
    if move.is_promotion:
        print("            crowned on", move.square_list[-1] + 1)
    print(board)
    print(board.fen)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) == 2 else START))
