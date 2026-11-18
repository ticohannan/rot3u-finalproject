# Suggested addition for https://github.com/JutLat/Checkers-Robot-Solver-One-Step-with-FEN-notation-
# Same license terms as that repository.
"""Use the group solver (py-draughts SimpleEngine) as the robot's engine in checkers_game.Game.

    from checkers_game import Game
    from draughts_engine import solver_engine
    game = Game(robot="blue", engine=solver_engine(depth=6))

The position goes to py-draughts as FEN; the move comes back as a path of squares 1-32 and is
matched with the shared rules' legal moves, so both rule sets must agree on it.
"""
from checkers_game.rules import legal_moves, to_fen
from draughts import SimpleEngine

from main import CompulsoryAmericanBoard


def solver_engine(depth=6, time_limit=None):
    engine = SimpleEngine(depth_limit=depth, time_limit=time_limit)

    def choose(board, side):
        b = CompulsoryAmericanBoard.from_fen(to_fen(board, side))
        move = engine.get_best_move(b)
        path = tuple(s + 1 for s in move.square_list)
        for m in legal_moves(board, side):
            if m.path == path:
                return m
        raise ValueError(f"solver move {move} is not legal under the shared rules")

    return choose
