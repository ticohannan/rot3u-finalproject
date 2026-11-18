# American checkers rules used by the robot

The official rules are the World Checkers Draughts Federation (WCDF) **Rules of Draughts (Checkers)**:
<https://wcdf.net/rules/rules_of_checkers_english.pdf> (rules index: <https://wcdf.net/rules.htm>).
The American Checker Federation (ACF) and the English Draughts Association are WCDF members
([English draughts, Wikipedia](https://en.wikipedia.org/wiki/English_draughts), citing the WCDF rules and
member pages). The ACF site (<https://www.usacheckers.com/>) has no rules page of its own.

| WCDF rule | Rule | Where in the code | Test |
| --- | --- | --- | --- |
| 1.5 | Dark squares numbered 1-32 | `rules.row_col`, `rules.name` | `test_1_5_square_numbering_and_names` |
| 1.11 | Red starts on squares 1-12, White (here blue) on 21-32 | `rules.START_BOARD` | `test_1_5_square_numbering_and_names` |
| 1.13 | Red moves first | `Game(to_move=RED)`, start FEN `B:W21-32:B1-12` | `test_1_13_red_moves_first_with_seven_moves` |
| 1.15 | A man moves one square diagonally forward | `rules.quiet_moves` | `test_1_15_men_move_forward_one_square` |
| 1.17 | A king moves one square diagonally, forward or backward | `rules.quiet_moves` | `test_1_17_kings_move_one_square_any_diagonal` |
| 1.18 | A man captures only forward: over a diagonally adjacent forward square to the empty square beyond | `rules._directions`, `rules._jumps` | `test_men_capture_forward_only` |
| 1.14, 1.17 | A king moves and captures in either direction, one square | `rules._directions` | `test_kings_capture_backward` |
| 1.20 | Captures are compulsory; any capture may be chosen, not only the one that takes the most | `rules.legal_moves` | `test_1_20_capture_is_compulsory`, `test_1_20_any_capture_may_be_chosen_not_only_the_longest` |
| 1.19, 1.20 | A multiple jump must be finished; a piece is jumped only once | `rules._jumps` | `test_1_19_1_20_multi_jump_must_be_finished` |
| 1.16, 1.19 | A man reaching the far row is crowned (a checker stacked on it), and that ends the turn, even during a jump | `rules._jumps`, `game.steps_for` | `test_1_16_1_19_crowning_ends_the_turn_even_during_a_jump` |
| 1.30 | A player with no legal move (all captured or blocked) loses | `Game.result` | `test_1_30_no_move_or_no_pieces_loses` |
| 1.32.1 | Draw on the third repetition of a position | `Game.result` | `test_1_32_1_third_repetition_is_a_draw` |
| 1.32.2 | Draw after 40 moves by each player with no capture and no man moved | `Game.result` | `test_1_32_2_forty_moves_each_without_capture_or_man_move_is_a_draw` |
| 1.25, 1.25.1 | An illegal move, a missed capture or an unfinished jump is taken back (no "huffing") | `Game.read_human_turn` refuses it and says what to fix | `test_human_turns.py` |
| 1.27 | Accidentally displaced pieces are put back | status `displaced` | `test_human_moved_a_robot_piece` |

Choices made for the robot game, where the WCDF rules are written for a referee:

- **Draws are declared automatically.** WCDF 1.32.1 and 1.32.2 are claims a player makes to the referee.
- **Touch-move (1.23, 1.24) is not enforced.** The camera reads the board only after "done".
- **Colors.** WCDF play uses red and white pieces (1.9); this game uses red and blue. Red moves first.
