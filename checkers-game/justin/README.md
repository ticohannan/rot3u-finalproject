# Suggested changes for the solver (Justin)

For [JutLat/Checkers-Robot-Solver-One-Step-with-FEN-notation-](https://github.com/JutLat/Checkers-Robot-Solver-One-Step-with-FEN-notation-),
reviewed at commit 76bfc92, with py-draughts 1.10.1 (the version its `requirements.txt` pins).

## What to change and why

1. **Make captures compulsory** (WCDF 1.20). py-draughts' `AmericanBoard` lists ordinary moves beside
   captures (same in versions 1.6.4 to 1.10.1). In 58 test positions with a capture available, its
   engine played an ordinary move in 22. `main.py` here adds `CompulsoryAmericanBoard`, which keeps
   only captures when one exists; the engine then follows it too. It calls a private py-draughts
   method, so keep `py-draughts==1.10.1` pinned. `test_justin_solver.py` checks it against the shared
   rules in 2,000 positions.
2. **Red moves first** (WCDF 1.13). In py-draughts FEN the side on squares 1-12 is written `B`, so the
   start position is `B:W21-32:B1-12`. The current default `W:W21-32:B1-12` lets the 21-32 side start.
3. **Report game over.** With no legal move, `SimpleEngine.get_best_move` raises
   "No legal moves available". `main.py` checks first and prints who has no move.
4. **Print the whole path.** A multi-jump is one move: `27x18x9` with `square_list` 27, 18, 9 and
   `captured_list` 23, 14 (py-draughts counts squares from 0; add 1).
5. **King notation in the README.** A king is `K` before the square, inside its color's list:
   `W:WK18,22:BK1`. The form `WK18,W22` is rejected.
6. **requirements.txt.** It is saved as UTF-16 and lists a web server and other packages the solver
   does not use. `py-draughts==1.10.1` alone installs what it needs.

## Watch out

- **Color names.** In py-draughts the side on 1-12 is "Black" and prints as `b`. In the game it is
  red. Use red and blue in shared code and `B`/`W` only inside FEN strings.

## Using the solver as the robot's engine

```python
from checkers_game import Game
from draughts_engine import solver_engine
game = Game(robot="blue", engine=solver_engine(depth=6))
```

Prepared with assistance from AI.
