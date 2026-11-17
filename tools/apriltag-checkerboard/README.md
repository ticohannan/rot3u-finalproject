# AprilTag checkerboard (printable)

**Version:** 1.0.0
**Updated:** 2026-11-16

`checkers_board.py` makes one printable PDF of an 8 x 8 checkers board with four AprilTags (family
`tag36h11`, IDs 0 to 3) and an orientation arrow above each tag. The assembled board is larger than one
sheet, so the PDF is several pages that you fold at printed lines and tape together.

The same copy of this tool is kept in `ticohannan/rot3u-robots` (`tools/apriltag-checkerboard/`) and
`ticohannan/rot3u-finalproject` (`tools/apriltag-checkerboard/`); the version number says which is newer.

| Version | Date | File | Change |
| --- | --- | --- | --- |
| 1.0.0 | 2026-11-16 | [`v1.0.0/checkers_board.py`](v1.0.0/checkers_board.py) | First version: US Letter or A4, selectable tag size (2.0 to 2.4 cm), optional tag border, orientation arrows, one `SETTINGS` block |

## Install (once)

WinPython 3.12 (or any Python 3.9+):

```
pip install numpy opencv-python reportlab
```

## Use

**In Jupyter:** open a new notebook, paste the whole of `checkers_board.py` into one cell, edit the
`SETTINGS` block at the top of the cell, and run it. The PDF is written next to the notebook and a link
is shown under the cell.

**From a terminal:**

```
python checkers_board.py                                  # US Letter, black and white, 2.4 cm tags with border
python checkers_board.py --paper a4
python checkers_board.py --tag-size 2.0 --no-border
python checkers_board.py --color-1 "#203060" --color-2 "#F0E0B0" --output my_board.pdf
```

The command-line options override `SETTINGS` for that run. `SETTINGS` is the only place the defaults are
set.

| Setting | Option | Default | Meaning |
| --- | --- | --- | --- |
| `paper` | `--paper letter\|a4` | `letter` | Paper size. Portrait or landscape is chosen for the fewest pages |
| `color_1` | `--color-1` | `#000000` | Dark squares (hex) |
| `color_2` | `--color-2` | `#FFFFFF` | Light squares (hex); the top-left square is this color |
| `output` | `--output` | `checkers_board.pdf` | Output file |
| `tag_cm` | `--tag-size` | `2.4` | Side of each AprilTag in cm, 2.0 to 2.4 (outer edge of its black frame) |
| `border` | `--border` / `--no-border` | `True` | Border around each tag. Without it, a tag's corner touches the board corner |
| `border_w_cm`, `border_h_cm` | `--border-w`, `--border-h` | `3.6`, `4.0` | Border width and height in cm (never smaller than the tag) |

## What it draws

- Board: 8 x 8 squares of 2.5 cm (20 cm x 20 cm). The tags use the tag colors black and white whatever
  the square colors are.
- Tags: ID 1 top left, ID 0 top right, ID 3 bottom left, ID 2 bottom right. All four are upright and none
  is on the board. Each tag is centered in its border, and one corner of each border (or of the tag, with
  no border) touches the matching corner of the board.
- Arrows: a black arrow pointing up in a 2 cm x 2 cm footprint, centered above each tag, with 1 cm
  between the arrow and the top edge of the tag. The arrows on the top tags stick out above the top
  border, so the assembled piece is taller than board plus borders: 27.2 x 30.2 cm with the defaults.
- Pages: nothing is printed in the 0.5 inch strip at the page edges. Dim gray dashed fold lines are
  printed 0.5 inch from every edge that joins another page. With two pages along a direction the fold
  moves to the board grid line nearest the middle that does not cut a tag or an arrow. A small gray
  label in a page corner gives its row and column.

## Print and assemble

1. Print the PDF at **100% / Actual size** (not "Fit to page"). A ruler laid on a printed square should
   read 2.5 cm.
2. On every page, fold the strip between each dashed fold line and the page edge to the **back**.
3. Put the folded edges of neighbouring pages together (the labels give row and column) so the board
   continues across the fold, and tape them on the back.

## Checks

Tested on the PC (Linux, Python 3.13, OpenCV 5.0, reportlab): every page of the default Letter and A4
outputs, and of the no-border outputs with 2.0 and 2.2 cm tags, decodes to the expected AprilTag IDs with
an independent detector (`pupil-apriltags`), and no page has ink inside the 0.5 inch edge strips. Not yet
run on WinPython 3.12; no printed copy has been measured or used for camera detection. Whether a tag with
no border is detected reliably by the camera has not been tested.

## Licence

BSD 3-Clause, Copyright (c) 2026 Tico Hannan. See the repository `LICENSE`.

Prepared with assistance from AI.
