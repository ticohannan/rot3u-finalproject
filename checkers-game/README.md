# checkers-game

Game logic and game-play tests for the checkers robot, plus suggested changes for the group's solver
and vision code. Rules: [`RULES.md`](RULES.md) (WCDF rule numbers, with the test for each).

| Folder | What | Needs |
| --- | --- | --- |
| `checkers_game/` | Shared game logic: rules, the game record, reading the human's turn from the camera, arm steps for the robot's turn, a small stand-in engine | Python 3.10+ only |
| `justin/` | Solver: `main.py` (one-step solver with compulsory captures) and `draughts_engine.py` (the solver as the robot's engine). See `justin/README.md` | `py-draughts==1.10.1` |
| `omar/` | Vision: `board_reading.py`, additions next to the vision code's `utils.py`. See `omar/README.md` | the vision code and its requirements |
| `tests/` | 57 game-play tests | `pytest` |

**Licenses.** `checkers_game/` and `tests/` are BSD-3-Clause, copyright Tico Hannan, like the rest of this
repository. `justin/` and `omar/` hold suggested changes to Justin's and Omar's code and follow the license terms of
their repositories.

## The one contract between vision and game logic

The camera reports a 32-character **occupancy string**, square 1 first:
`r` red, `b` blue, `.` empty, `?` not sure. Kings are not reported (a stacked king looks like one
checker from above); the game record knows which pieces are kings.

```
square numbers (row 0 = top of the flattened image, AprilTag 1 top-left):
     .  1  .  2  .  3  .  4        b8 d8 f8 h8
     5  .  6  .  7  .  8  .        a7 c7 e7 g7
     9  . 10  . 11  . 12  .   ...
    ...
    29  . 30  . 31  . 32  .        a1 c1 e1 g1
start:  rrrrrrrrrrrr........bbbbbbbbbbbb    red on 1-12 moves first
```

## How a game runs

```python
from checkers_game import Game
game = Game(robot="blue")                      # red (human) moves first

# human turn: after the human says "done"
reply = game.read_human_turn(occupancy)        # occupancy from the camera
print(reply.say)                               # "I read that as ..." or what to fix
# reply.steps: a crowning job for the arm when the human reached the king row

# robot turn
plan = game.plan_robot_turn()
print(plan.say)                                # "I'm moving c3 to e5 to g7, capturing ..."
for step in plan.steps:                        # move, remove (to the owner's tray), crown
    ...                                        # arm does the job
game.robot_turn_done(plan)
print(game.check_board(occupancy).say)         # verify survey after the arm moved

game.result()                                  # None, or (winner or "draw", reason)
```

## Running the tests

```
cd checkers-game
python -m pip install pytest                   # add py-draughts==1.10.1 for the solver tests
python -m pytest tests -q
```

The solver tests are skipped without py-draughts. The photo test runs only when
`CHECKERS_VISION_REPO` points to a checkout of the vision code:

```
set CHECKERS_VISION_REPO=C:\path\to\checkers-bot        (Windows)
export CHECKERS_VISION_REPO=/path/to/checkers-bot       (macOS, Linux)
```

| Test file | What it covers |
| --- | --- |
| `test_rules_wcdf.py` | One test per WCDF rule the game depends on (see `RULES.md`) |
| `test_human_turns.py` | Reading the human's turn: complete multi-jump, stopped part-way, captured piece left on the board, ordinary move when a capture was required, robot piece disturbed, camera not sure, branching captures |
| `test_robot_turns.py` | Robot announcements and arm steps: double jump, crowning, kings as two-checker stacks, circular king jump, verify survey |
| `test_game_flow.py` | 20 whole games (engine against random moves) read through the simulated camera, with 10 % of readings blurred; a scripted opening |
| `test_justin_solver.py` | The solver against the shared rules in 2,000 positions; the stock py-draughts capture gap; `main.py`; the solver as the robot's engine |
| `test_omar_vision.py` | Matrix to occupancy, the fixed FEN, the "not sure" rule, the sample photo |

Prepared with assistance from AI.
