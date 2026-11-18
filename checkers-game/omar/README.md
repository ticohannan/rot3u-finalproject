# Suggested changes for the vision code (Omar)

For [fresh4/checkers-bot](https://github.com/fresh4/checkers-bot), reviewed at commit bd349e5.
`board_reading.py` goes next to that code's `utils.py` and uses its functions unchanged.

## What to change and why

1. **Red is the side that moves first, on squares 1-12** (WCDF 1.13). `board_to_fen()` writes red as
   `W`; py-draughts expects `W` on 21-32 and rejects the saved sample FEN with "man on promotion
   square 1". `board_to_fen_fixed()` writes red as `B`. The square numbering itself already matches
   (row 0 at the top of the flattened image, square 1 at column 1).
2. **Report colors, not a game position.** One photo cannot show whose turn it is or which checkers
   are kings (a stack looks like one checker from above). `board_to_occupancy()` returns 32
   characters (`r`, `b`, `.`, `?`); the game logic keeps the side to move and the kings.
3. **Say when unsure.** `classify_fractions()` returns `?` when both colors show on a square, or when
   the color fraction is near the threshold. The robot then asks instead of guessing.
4. **Check the light squares.** Piece color on a light square means a hand over the board or a
   knocked checker. `read_board()` lists them in `light_hits`.
5. **Work on camera frames.** `read_board(frame)` takes a numpy image (from `grab_frame()` or
   `cv2.imread`) and returns `{"ok": False, "error": ...}` instead of raising when a tag is hidden.

On the repository's sample photo `images/board_with_pieces.jpg`, `read_board()` gives the start
position exactly, with no unsure squares; occupied squares read 0.29 or more, empty ones 0.00.

## Not tested yet

- **Frames from the gripper camera.** Seen from the side at a low angle, a tall checker's top
  appears shifted away from its square: about 14 mm for a 30 mm king stack seen from 250 mm up and
  100 mm to the side.
- **Board size.** The tags sit 4 cm out from each board corner, so an 8 in board needs a 28.3 cm
  square, wider than a letter sheet.

Prepared with assistance from AI.
