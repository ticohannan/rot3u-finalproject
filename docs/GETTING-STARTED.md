# Getting started: camera tracking (Robot Demo 4) on your own ROT3U arm

**Version:** 1.0
**Updated:** 2026-11-16
**Applies to:** `firmware/servo_tester` v3.0, `firmware/servo_bridge` v0.3, `python/rot3u_demo4` v0.3.0 and
`python/demo4_tracking.ipynb`, on a Windows 10/11 laptop

This guide starts from nothing: no software installed, no tools of ours seen before. At the end your arm
keeps a colored block centered in its gripper camera, using the camera as its only sensor.

| Part | What you do | Time |
| --- | --- | --- |
| 1 | Check what you need | 5 min |
| 2 | Install the software and download this repository | 30 min |
| 3 | Wire the servo board and check it | 15 min |
| 4 | Map your arm's joints with `servo_tester` and write your `robot_config.h` | 60–90 min |
| 5 | Mount the camera | 10 min |
| 6 | Upload `servo_bridge` | 5 min |
| 7 | Run the notebook on the simulator (no hardware) | 10 min |
| 8 | Run the notebook on your arm | 45 min |
| 9 | Troubleshooting | |
| 10 | Getting updates, sharing your results | |

**Status.** The tracking notebook ran on Sir Pinch-a-lot on 2026-11-15 and the demo was recorded.
Three pieces are new and have only been tested on a PC simulation so far:

- the mapping commands of `servo_tester` v3.0 (`PT`, `FIT`, `EXPORT` ...);
- `servo_bridge` v0.3, which takes your arm's limits from your `robot_config.h`;
- the input-shaping cell, C.4.

Your arm may be the first real test of them. Go slowly the first time, keep a hand on the servo power
switch, and report anything odd (part 10).

Words used below: **pulse** = the servo command in microseconds (µs), 500 to 2500, about 1500 in the
middle; **channel** = the output on the servo board (0 to 15) a servo is plugged into; **Serial Monitor** =
the Arduino IDE's text window for talking to the board.

---

## 1. What you need

**Hardware**

| Item | Notes |
| --- | --- |
| ROT3U-type 6-servo arm with MG996R servos | base, shoulder, elbow, wrist pitch, wrist roll, gripper |
| Arduino Mega 2560 and its USB cable | other boards are not supported by these sketches |
| PCA9685 16-channel servo board (I²C address 0x40, the default) | |
| A separate servo power supply, 5–6 V, at least 5 A | Never power the servos from the Arduino's 5 V pin: six MG996R can draw several amps |
| A USB camera on the gripper | Sir Pinch-a-lot uses an HBVCAM 3M1917 V22 board camera on a printed bracket (`gripper_camera_bracket` rev 2.1) on the gripper servo. Ask Tico for the bracket print file. Another small USB camera works if you can fix it to the gripper, looking the same way the gripper points |
| A block of one bright color | Sir Pinch-a-lot's is a green painted wooden cube, 24.8 mm per side. Green or blue is easiest; red is hard near skin and wood |
| For mapping (part 4): a phone with a level app, a ruler, tape, a sheet of paper | iPhone: the Measure app has a Level. Android: any "bubble level" app that shows degrees |

**Software** (all free; installed in part 2): Git for Windows, the Arduino IDE 2, WinPython 3.12 (Python,
Jupyter and the packages below), and the color picker (`ticohannan/camera-color-picker`).

---

## 2. Install the software and download this repository

### 2.1 Git

1. Download Git for Windows from <https://git-scm.com/download/win> and run the installer. Keep every
   default choice.
2. Check it later from the WinPython Command Prompt (2.3): `git --version` prints a version.

### 2.2 Arduino IDE

1. Download the Arduino IDE 2 from <https://www.arduino.cc/en/software> and install it.
2. Plug in the Mega. In the IDE: **Tools > Board > Arduino AVR Boards > Arduino Mega or Mega 2560**, and
   **Tools > Port >** the COM port that appeared when you plugged it in (for example `COM3`). Write the port
   down: the notebook needs it.

### 2.3 WinPython (Python and Jupyter)

1. Download WinPython 3.12 (the 64-bit "dot" or full release) from <https://winpython.github.io/> and run
   it: it unpacks into a folder, for example `C:\Users\<you>\Desktop\WPy64-312101`. Nothing is installed system-wide.
2. In that folder, start **WinPython Command Prompt.exe**. Every command in this guide is typed there.
3. Install the packages:
   ```
   pip install pyserial opencv-python numpy ipywidgets matplotlib
   pip install git+https://github.com/ticohannan/camera-color-picker
   ```
   Use `opencv-python`, not `opencv-python-headless` (the tracking view is an OpenCV window). Every
   package comes as a ready-made wheel: no compiler is needed.

### 2.4 Download this repository

1. In the WinPython Command Prompt, go to WinPython's `notebooks` folder (Jupyter opens there) and clone:
   ```
   cd /d C:\Users\<you>\Desktop\WPy64-312101\notebooks
   git clone https://github.com/ticohannan/rot3u-finalproject.git
   ```
   Replace the path with your WinPython folder.
2. Check: `dir rot3u-finalproject` lists `docs`, `firmware`, `python`, `robots` and `README.md`.

What is where:

| Folder | What |
| --- | --- |
| `firmware/servo_tester/` | Sketch for mapping the joints (part 4) |
| `firmware/servo_bridge/` | Sketch the notebook drives (part 6) |
| `robots/` | One `robot_config.h` per arm: `sir_pinch_a_lot_revB/`, `template/`, and yours (part 4) |
| `python/` | The notebook `demo4_tracking.ipynb` and its package `rot3u_demo4` |
| `docs/` | This guide, the full joint-mapping guide, the angle conventions |

---

## 3. Wire the servo board and check it

**Servo power off** for this whole part.

1. PCA9685 to the Mega:

   | PCA9685 pin | Mega pin |
   | --- | --- |
   | GND | GND |
   | VCC (logic) | 5V |
   | SDA | 20 (SDA) |
   | SCL | 21 (SCL) |

2. Servo supply to the PCA9685's screw terminal (V+ and GND), polarity checked twice.
3. Servos to the channels in this order. Every tool here assumes it:

   | Channel | Joint |
   | --- | --- |
   | 0 | base (turns the whole arm) |
   | 1 | shoulder |
   | 2 | elbow |
   | 3 | wrist pitch (tips the gripper up and down) |
   | 4 | wrist roll (turns the gripper about its own axis) |
   | 5 | gripper (opens and closes the jaws) |

   Plug each servo's brown or black wire toward the board's GND row.
4. Plug the Mega into the laptop. Continue with part 4: its first step uploads `servo_tester`, which checks the board.

---

## 4. Map your arm's joints → `robots/<your robot>/robot_config.h`

Every arm is different: the same pulse puts two arms' joints at different angles, the direction can be
reversed, and the safe range differs (brackets collide at different places). `servo_tester` measures this:
for each joint you mark the safe range, put the joint at three or more pulses, measure the angle, and it
fits a line, **pulse = US0 + USPD × angle**. `EXPORT` prints the result in the format of `robot_config.h`.

This part is a short recipe that is enough for camera tracking. `docs/joint-mapping.md` is the full
reference (backlash, gravity side, other measuring methods). Read part 4.1 (the angle conventions) before
measuring anything.

### 4.1 The angle conventions: what each angle means

Angles are in degrees, in the arm's vertical plane, as in the class's DH model (`docs/rot3u-conventions.md`).
"Absolute angle" of a link = its angle **above horizontal, pointing outward** (away from the base):
0 = level pointing outward, +90 = straight up, −90 = straight down, above 90 = leaning back.

| Joint | The angle you enter | Example: the WALL pose |
| --- | --- | --- |
| base θ1 | Direction the arm faces, seen from above, counter-clockwise positive. 0 = straight ahead (your chosen "forward"), +90 = to the robot's left | 0 |
| shoulder θ2 | Absolute angle of the **upper arm** (shoulder axis → elbow axis) | +90 (straight up) |
| elbow θ3 | Absolute angle of the **forearm** (elbow → wrist axis) **minus** the upper arm's. Folding the forearm down is negative | 45 − 90 = −45 |
| wrist θ4 | Absolute angle of the **gripper** (wrist axis → jaw tips) **minus** the forearm's. Tipping the jaws down is negative | 0 − 45 = −45 |
| roll θ5 | Turn of the gripper about its own axis. 0 = the jaws close **sideways** (jaw faces parallel to the arm's vertical plane); +90 = turned clockwise seen from behind the gripper | 0 |

Signs check: raising the upper arm makes θ2 bigger; bending the elbow down makes θ3 more negative;
tipping the jaws down makes θ4 more negative.

**Using a phone level.** Lay the phone's edge along the link, in the arm's vertical plane. The app shows
the tilt from level, without a sign: you decide the sign. Pointing up and outward is +, pointing down and
outward is −. For the upper arm leaning back past vertical, use 180 − reading.

### 4.2 Upload servo_tester and connect

1. Servo power **off**. In the Arduino IDE: **File > Open** →
   `rot3u-finalproject\firmware\servo_tester\servo_tester.ino`. Board and port as in 2.2. **Upload** (→ button).
2. **Tools > Serial Monitor**. Set the two drop-downs at its bottom right to **115200 baud** and **Newline**.
3. It prints `# READY servo_tester v3.0, driver ...`. Type `SCAN` and press Enter: it must list `0x40`
   (the PCA9685). If not, check the SDA/SCL wires.
4. Type `ARM A` (any letter: it names your arm in the data lines).
5. If it says `saved mapping session restored` and this is a new arm: `NEW YES` starts over.
6. Servo power **on**. Keep a hand near the switch.

Moves are slow on purpose. Any key pressed during a move pauses it. `!` releases every channel; the arm
goes limp, so support it first.

### 4.3 Start the joints

A joint jumps to its first pulse at full speed, so **support the arm by hand** for each `START`. Start
every arm joint now, so the ones you are not mapping hold still:

```
SEL 0
START 1500
SEL 1
START 1500
SEL 2
START 1500
SEL 3
START 1500
SEL 4
START 1500
```

If a joint pushes against something at 1500, type `!` (all limp), and start that joint at another pulse
(1300 or 1700). Each channel's soft limits start at 1300–1700 µs; you widen them for each joint in 4.4.

### 4.4 Map one joint (repeat for channels 0 to 4)

Below, `c` is the channel number. Do the base first, then shoulder, elbow, wrist and roll.

1. Define the joint (names up to 8 characters):
   ```
   SEL c
   JT ROT shoulder
   LOAD 0
   PREC 2
   ```
   `JT ROT` is the type (rotary) plus the name: `base`, `shoulder`, `elbow`, `wrist` or `roll`.
   `LOAD 0` leaves out backlash modeling (camera tracking does not need it). `PREC 2` says your readings
   are good to about 2 degrees.
2. Open the window and find the safe range:
   ```
   LIM c 600 2400
   JOG
   ```
   In `JOG`: `+` and `-` move by small steps, `3` switches to 10 µs steps, `q` quits. Watch the joint.
   - Go down in pulse until just **before** anything touches, binds, or a cable pulls tight. `q`, then `LO`.
   - `JOG` again and go up in pulse to the other end, just before contact. `q`, then `HI`.

   `LO` is the **low pulse** end and `HI` the **high pulse** end, whatever the angle does there. Each pulls
   the live limit in by 30 µs.
   - The wrist (ch 3) on a ROT3U: the brackets hit each other at one end; stop before that.
   - The shoulder: stop where the arm would hit the table or the base, and check the cable at both ends.
3. Take **three points** spread over the range: one near `LO`, one near `HI`, one in the middle. At each:
   1. `GO us` (for example `GO 900`) and wait for `# at 900 us`.
   2. Measure the angle as in 4.1.
   3. Type `PT angle` (for example `PT 52`).

   How to measure each joint:
   - **base:** tape a sheet of paper under the gripper with lines from below the base axis at 0, ±45 and
     ±90 degrees; read where the gripper points.
   - **shoulder:** phone along the upper arm.
   - **elbow:** keep the shoulder still. Measure the upper arm once and the forearm at every point;
     enter forearm − upper arm.
   - **wrist:** keep the shoulder and elbow still. Measure the forearm once and the gripper at every
     point; enter gripper − forearm.
   - **roll:** 0 with the jaws closing sideways; ±90 turned a quarter turn. A square against the jaws is
     enough.
4. Fit and check:
   ```
   FIT
   ```
   It prints the line (US0, USPD), the error at each point, and warnings. USPD should be about ±10 to ±11.5
   µs per degree. A minus sign is normal for joints whose servo turns the other way. If the scatter is
   larger than 2 degrees, `PTS` lists the points; re-measure the worst one (`PTDEL n` deletes point n) and
   `FIT` again.

The session is saved in the Mega's memory after every command. If you stop halfway, the next start
restores it.

### 4.5 The gripper (channel 5)

```
SEL 5
JT GRIP gripper
LIM 5 1000 2000
START 1300
```
1. `JOG` until the jaws are open well wider than the block; `q`; `SETP OPEN`.
2. Put the block between the jaws. `JOG` until they hold it firmly **without the servo buzzing** (buzzing
   = straining); `q`; `SETP HOLD`.
3. `JOG` to the widest safe opening; `q`; `LO` or `HI` (whichever end that is in pulse). Then `JOG` to just
   past the hold pulse; `q`; the other one.

### 4.6 Export and write your `robot_config.h`

1. Type `EXPORT`. It prints a block between `// ---- BEGIN SERVO_TESTER EXPORT` and `// ---- END SERVO_TESTER EXPORT ----`.
2. Copy the folder `robots\template` to a new folder named after your arm, for example `robots\don_pinzas`.
   In the Command Prompt:
   ```
   cd /d C:\Users\<you>\Desktop\WPy64-312101\notebooks\rot3u-finalproject
   xcopy robots\template robots\don_pinzas\ /E
   ```
3. Open `robots\don_pinzas\robot_config.h` in a text editor (Notepad works) and:
   1. replace the `const JointCal JOINTS[] = { ... };` lines and the `const GripCal GRIP = ...;` line with the
      exported ones. Keep the `// ch...` comment lines: they record how each joint was measured;
   2. set `#define ROBOT_NAME "..."` to your arm's name;
   3. measure the link lengths with a ruler touching both axis centers, and fill in `ARM_D1` (base plate to
      shoulder axis), `ARM_R2` (shoulder axis to elbow axis), `ARM_R3` (elbow axis to wrist-pitch axis),
      `ARM_TIP` (wrist-pitch axis to the jaw tips, jaws closed). If you have no time, keep Sir Pinch-a-lot's
      numbers: they only change the pictures and the overlay, not the tracking;
   4. set `#define CAL_VALID  true`.
4. Each row must start with its channel: `0` base, `1` shoulder, `2` elbow, `3` wrist, `4` roll, in that
   order; the gripper line starts with `5`.

### 4.7 Check it

In Python (part 7 shows how to start Jupyter), or as the first thing in the notebook (cell 0.2 prints it):

```
import sys; sys.path.insert(0, ".")
from rot3u_demo4 import robot
robot.load_config("../robots/don_pinzas/robot_config.h")
print(robot.describe())
```
It prints one line per joint: the pulse formula and the limits in µs and in degrees. Check the degrees
against the arm: for example, the shoulder's range should include +90 (upper arm vertical), and the WALL
pose (0, 90, −45, −45, 0) must be inside every joint's range. If something is missing or wrong in the
file, `load_config` says what.

### 4.8 Save your robot file in the repository

If Tico has added you as a collaborator on GitHub:
```
cd /d C:\Users\<you>\Desktop\WPy64-312101\notebooks\rot3u-finalproject
git switch -c robot-don-pinzas
git add robots/don_pinzas/robot_config.h
git commit -m "robots: don_pinzas robot_config.h from servo_tester"
git push -u origin robot-don-pinzas
```
Then open the pull request link git prints. Otherwise, send the file to Tico.

---

## 5. Mount the camera

1. Fix the camera to the gripper so it looks the same way the gripper points (along the jaws), above
   the jaws, with the USB cable slack through every joint's full travel (turn the base by hand both ways
   with the servo power off to check).
2. **Which side is up.** With roll 0, Sir Pinch-a-lot's gripper servo, and the camera mounted on it, sit
   **on top** of the gripper. If on your arm they are underneath at roll 0, use `ROLL_DEG = 180` in the
   notebook's settings (cell 0.1), and change the camera mount there to
   `pos_5=(-29.5, 0.0, -55.0), up_5=(-1.0, 0.0, 0.0), right_5=(0.0, -1.0, 0.0)`.
3. A different camera or bracket: the mount numbers in cell 0.1 (`MOUNT`) are where the lens sits in the
   gripper's frame, in mm. They only affect the overlay (the gripper's axis drawn on the picture) and the
   distance estimate, not the tracking. Section C.2 of the notebook checks them.

---

## 6. Upload servo_bridge

1. Close the Serial Monitor if it is open. **File > Open** →
   `rot3u-finalproject\firmware\servo_bridge\servo_bridge.ino`. **Upload**.
2. Check it (optional): open the Serial Monitor (115200, Newline). It prints `# READY servo_bridge v0.3`,
   then `HELLO,servo_bridge,0.3,2`.
3. **Close the Serial Monitor.** Only one program can use the port, and the notebook needs it.

`servo_bridge` only moves servos to pulse targets, smoothly. Its own limits are the servo's whole range
(500–2500 µs); the notebook sends your arm's limits from your `robot_config.h` every time it connects. So
always move the arm through the notebook, not by typing in the Serial Monitor.

---

## 7. Run the notebook on the simulator first

This checks the installation without touching the arm.

1. In the WinPython folder, start **Jupyter Lab.exe** (or **Jupyter Notebook.exe**). A browser tab opens.
2. Open `rot3u-finalproject` → `python` → `demo4_tracking.ipynb`.
3. In the first code cell (0.1 Settings), change `PORT = "COM3"` to `PORT = "SIM"`.
4. **Run > Run All Cells**. It takes about 4 minutes. The picker cells say "simulator: skipped".
5. Expected at the end: section D.3 prints `{'CENTERED': ..., 'TRACK': ...}`, D.5 prints `saved logs/..._session.json`.
   Simulated tracking also opens an OpenCV window unless `SHOW` is `None`; with `"window"`, press **q** in it to stop.

If a cell fails, read the box above it ("If it fails") and part 9, then send the error text.

---

## 8. Run the notebook on your arm

Every section of the notebook starts with a box: what it does, whether to run it (**REQUIRED**,
**ONCE**, **OPTIONAL**, **SKIP IF**), what you should see, and what to do if it fails. Run the sections in
order. Here is the path through it, with what is specific to a new arm.

1. **0.1 Settings.** Set:
   - `PORT` to your Mega's port (2.2);
   - `ROBOT_CONFIG = "../robots/don_pinzas/robot_config.h"` (your folder);
   - `OBJECT` and `ROOM` to name your block and where you are, for example `"blue cube #1"`, `"lab"`;
   - `BLOCK_MM` to your block's side in mm;
   - `DEFAULT_CAMERA`'s `index` to your camera's number if you know it (A.5a lists them).

   Run the cell. Run it again after every change.
2. **0.2** loads your robot file and prints it. Check it as in 4.7.
3. **Part A, colors.** There is no color profile for your block and room yet, so do **A.1 to A.3**:
   - A.1 checks the picker is installed;
   - A.2 opens the color picker, with numbered steps for the camera set-up and for the series of
     "shotgun" captures (6 to 10 captures of the same block at different angles and places in the picture);
   - stop the picker with the second A.2 cell;
   - A.3 shows the captures; uncheck bad ones; the next cell builds and saves your profile in
     `python\profiles\`.

   Then **A.4** opens the camera with the profile. **A.6** must show your block, and almost nothing else,
   white in the mask. A.5 (diagnostics) and A.7 (fine-tuning the color range) are for when something is off.
4. **Part B, the arm.** B.1 connects and sends your limits. **B.2 draws the start pose**: look at the
   pictures, hold the arm close to that pose, put the block about 25–35 cm in front of the camera at
   camera height. B.3 powers the joints up one at a time; type `yes` when asked.
5. **Part C, measurements.** Leave the block still for all of part C. C.1 (noise and image Jacobian) is
   required: it measures how the base and wrist move the block in the picture, so it works whatever your
   calibration's small errors are. C.2–C.5 are recommended; C.4 measures the arm's wobble and turns on the
   smoothing that cancels it.
6. **Part D, tracking.** D.1 builds the tracker, D.3 tracks with base and wrist: move the block, hold it,
   wait for `CENTERED`. Press **q** in the window to stop. D.5 saves a summary of the session in
   `python\logs\`.

---

## 9. Troubleshooting

| Problem | What to do |
| --- | --- |
| `no HELLO` in B.1 | Close the Arduino Serial Monitor. Check `PORT`. Unplug and replug the USB cable. Upload `servo_bridge` again |
| `ValueError ... robot_config.h` in 0.2 | The message says what is missing (a joint row, the GRIP line, an `ARM_*` length, `CAL_VALID`). Fix the file, run 0.1 and 0.2 again |
| `... would need ... us, outside its limits` in B.2 | The start pose is outside one joint's mapped range. Check that joint's mapping (direction and the angles you entered), or try `POSE = "TABLE"` |
| A joint moves the wrong way, or the start pose looks nothing like the pictures | That joint's angles were entered with the wrong sign or the wrong reference. Re-map it (4.4), using the signs check in 4.1 |
| `refused the limits` in B.1 | The Mega still has an older `servo_bridge`: upload `firmware/servo_bridge` (v0.3) |
| Black picture, or the wrong camera | A.5a lists the cameras; A.5b tests every mode; the troubleshooting list in A.5d. Close the color picker (second A.2 cell), Zoom, Teams and the Windows Camera app first |
| `AttributeError` after a `git pull` | Jupyter still has the old code: **Kernel > Restart**, then run 0.1 and 0.2 again |
| Tracking hunts around the center | Run C.3 (slop), then D.1 again: the dead band grows to fit your servos |
| The arm wobbles after each move | Run C.4: it measures the wobble and turns on input shaping |
| A servo buzzes or gets hot holding a pose | It is straining against something, or the gripper's HOLD pulse is too tight. Release the arm (`lk.cmd("OFF *")` in a new cell, support it first) and check the mapping |

---

## 10. Getting updates, sharing your results

- **Updates:** in the Command Prompt,
  ```
  cd /d C:\Users\<you>\Desktop\WPy64-312101\notebooks\rot3u-finalproject
  git pull
  ```
  then in Jupyter **Kernel > Restart**, and run from 0.1. If `git pull` complains about local changes to the
  notebook, you edited it: `git restore python/demo4_tracking.ipynb` discards your edits to the notebook
  (your settings are the only edits; note them first). Your profiles, captures and logs are not touched.
- **Sharing:** your `robot_config.h` (4.8), the `python\logs\<stamp>_session.json` of a good run, and any
  error text, sent to Tico or as a pull request.

Prepared with assistance from AI.
