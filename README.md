# Coordinate-System Conversion for the CV Viewer

DeltaReality gave three 3D scans of a room (`image1.ply`, `image2.ply`, `image3.ply`) and
a file of camera positions (`traj.txt`). Both use DeltaReality's own convention for which
direction counts as "up," "left," "forward," and so on — a convention nobody wrote down.
The Unity viewer expects a specific, different convention. This project figures out the
conversion and exports new files so the viewer shows one solid, right-side-up room.

Finding that conversion **is** the assignment — there was no hint which one to use. This
README documents how it was found, step by step, including a mistake that shipped in the
first version and how it got caught. See **What went wrong, and how it got fixed** below
for that part specifically.

## Setup — do this once

Put this `solution\` folder next to `ComputerVisionAssignment.exe`. The scripts only ever
**read** from a `data_original\` folder and only ever **write** into `StreamingAssets\`
(where the viewer actually looks for its files), so the original data is never at risk —
a bad export can always be thrown away and redone.

A fresh copy of the viewer ships with the original files sitting inside `StreamingAssets\`,
so the first step is to copy them out:

```
mkdir data_original
copy ComputerVisionAssignment_Data\StreamingAssets\Points\image1.ply data_original\
copy ComputerVisionAssignment_Data\StreamingAssets\Points\image2.ply data_original\
copy ComputerVisionAssignment_Data\StreamingAssets\Points\image3.ply data_original\
copy ComputerVisionAssignment_Data\StreamingAssets\traj.txt          data_original\
```

If anything goes wrong later, copy `data_original\*` back over `StreamingAssets\` to
return to the untouched starting point.

## How to run

```
uv venv delta_venv --python 3.12
uv pip install --python delta_venv\Scripts\python.exe numpy open3d opencv-python matplotlib
delta_venv\Scripts\python.exe solution\phase_a1_a2.py
delta_venv\Scripts\python.exe solution\phase_a3.py
delta_venv\Scripts\python.exe solution\ply_writer.py
delta_venv\Scripts\python.exe solution\phase_c.py preview
delta_venv\Scripts\python.exe solution\phase_c.py downsample
delta_venv\Scripts\python.exe solution\phase_d.py 1
delta_venv\Scripts\python.exe solution\phase_d.py 2
delta_venv\Scripts\python.exe solution\phase_e.py
delta_venv\Scripts\python.exe solution\phase_f.py 20
delta_venv\Scripts\python.exe solution\phase_f.py
```

(`uv` just manages the Python environment; `open3d` doesn't have Python 3.13 support yet,
hence pinning 3.12.)

A few of the scripts take an argument:
- `phase_c.py` — `preview` draws a quick picture on your own machine (no Unity needed);
  `downsample` stages a small test file into the viewer. No argument runs both.
- `phase_d.py 1` / `phase_d.py 2` — two different test setups, explained below.
- `phase_f.py 20` — a fast, low-detail export for a quick look; `phase_f.py` with no
  number does the real, full-detail export.

Any script that writes into the viewer's own folder (`phase_c.py downsample`,
`phase_d.py`, `phase_f.py`) overwrites `StreamingAssets\`. Copy `data_original\*` back
any time to reset.

## Why the viewer gets relaunched so often

This project keeps pausing to say "now open the viewer and look." That's deliberate. The
viewer is a closed program — there's no way to see inside it — so the only way to know if
a guess about its coordinate system is right is to actually watch it render something and
check with your own eyes. The alternative is guessing blind and hoping, which wastes far
more time when a guess turns out wrong.

Four checkpoints, each one only a small step past the last so a bad result points
straight at what broke it:

1. **Untouched originals.** Load the viewer with nothing changed, to see the starting
   (broken) state.
2. **A downsampled test file.** Write a much smaller version of the same files through our
   own file-writer, without touching the coordinate math at all. If this loads and looks
   the same, just sparser, the file-writing itself is trustworthy — checked before any
   real math is involved.
3. **A test object, first setup.** A small hand-made 3D object (described below) is sent
   into the viewer with no camera movement applied, to see exactly what the viewer does
   to raw point positions.
4. **A test object, second setup.** The same object, now with a known camera movement
   applied, to see what the viewer does with camera position data specifically.

The final export is checked the same way: run it, relaunch, look.

## The story, phase by phase

### Phase A — what are we actually given? (`phase_a1_a2.py`, `phase_a3.py`)

`traj.txt` has three lines of 16 numbers — one line per camera. Sixteen numbers is
exactly enough to describe a 4×4 matrix, a standard way of encoding "where something is
and which way it's facing" in 3D. Two questions had to be settled before anything else:

- **Do the 16 numbers fill the matrix row by row, or column by column?** Only one of the
  two readings produces something mathematically valid — real rotations plus a real
  position. The other collapses to nonsense. Row by row is correct.
- **Do these matrices place a camera's local view into the room, or the other way
  around?** Testing both directions — actually running the points through the matrices —
  and checking which one makes the three photos click together into one room (rather than
  scatter apart) settled it: they place local points into the shared room.

An early attempt was also made here to guess which direction in the data is "up," using
how the camera turned between shots. **That guess turned out to be wrong** — see
"What went wrong" below.

### Phase B — a file writer we can trust (`ply_writer.py`)

The viewer reads a specific 3D file format and we don't know exactly how forgiving its
reader is, since it's closed-source. The safest approach: copy the exact structure of the
original files byte-for-byte, and change only the numbers inside — never the header or
formatting. This was checked by writing a file out and reading it straight back in,
ourselves, to make sure nothing got corrupted in the round trip.

### Phase C — working fast, without waiting on the viewer (`phase_c.py`)

The full files are almost 3 million points each, and the viewer takes minutes to load
them — far too slow for checking small guesses one at a time. Two shortcuts:

- A quick local picture (no Unity) showing the current best guess at how the room fits
  together, in seconds instead of minutes.
- A much smaller test file (every 100th point) that still goes through the *real* viewer,
  to check the file-writer without waiting on full file sizes.

### Phase D — figuring out what the viewer does to our data (`phase_d.py`)

Since the viewer's code can't be inspected, the only way to learn how it behaves is to
feed it something with a known, obvious shape and see what comes back out. So a simple
test object was built: three colored arms of different lengths pointing along three
different directions, plus a small asymmetric marker at the tip of one arm — a plain
shape can look identical whether it's rotated correctly or accidentally mirrored, but an
asymmetric marker gives a mirror away immediately.

This object was sent into the viewer twice:

- **With no camera movement at all**, to see exactly what the viewer does to raw point
  positions. Result: it flips one axis (up/down) compared to what it's handed.
- **With a small, known camera movement**, to see whether the viewer does anything
  unexpected with camera position data. Result: no, it uses it exactly as given.

Both results are now known facts about the viewer, and the final export has to account
for them. A local picture of the test object (`images/glyph_reference.png`) shows what
"no transform at all" should look like, for comparison against the viewer's own render:

![axis-triad calibration glyph](images/glyph_reference.png)

### Phase E — matching up each photo's private coordinates (`phase_e.py`)

Each of the three point clouds is described from its own camera's point of view — its
own private sense of left/right/up/down/forward/back. Before the camera-position data can
place a cloud into the shared room, that cloud's private directions have to be translated
into the same convention the camera positions use. There are 48 possible ways to do this
translation, and nothing in the assignment hints at which one is right.

So it was measured directly: take a point visible in two different photos, and check —
under each of the 48 possible translations — whether it lands in the same real-world 3D
spot from both cameras' point of view, *and* whether it's the same color there. The right
translation gets both right consistently; a wrong one scatters positions or mismatches
colors.

| Candidate | 3D position match | Color match |
|---|---|---|
| Option A | **best** | poor |
| **Option used** (`−X,−Z,−Y`) | good | **best overall** |
| Option B | good | good |

The candidate with the single best position-match score turned out to be sneaking in
false matches — points that happened to land near each other in 3D but were actually
different, differently-colored surfaces. Checking color as well as position caught this.
Only one candidate scored well on *both* measures at once, and it's the one the viewer
confirms as correct. (The exact numbers behind this table are in the code's own output —
`phase_e.py` prints the full comparison.)

### Phase F — putting it together and standing the room upright (`phase_f.py`)

With everything from Phases A–E known, each of the three clouds is placed into the shared
room. What's left is one final rotation to make sure the room displays genuinely upright
— **and this last rotation is where the bug lived.** The room also comes out mirrored
exactly once by design, not by accident: the original photos use a "right-handed" sense
of direction and Unity uses a "left-handed" one, and converting between the two always
takes exactly one mirror-flip somewhere in the math. That flip is expected and correct;
the bug was entirely about which way "up" pointed.

## What went wrong, and how it got fixed

The first version of this project exported a room that was solid and correctly
assembled — the three photos lined up, the colors matched, the walls were whole — but the
whole thing was lying on its side, like a dollhouse tipped over.

| Before | After |
|---|---|
| ![before](images/before_tipped.png) | ![after](images/after_upright.png) |
| The furniture wall lies on its side. | Doors, shelving, and the chair stand upright. |

Two different attempts had been made to figure out which direction is "up," and **both
were wrong**:

1. One guessed that since the camera swiveled between the three shots, the direction it
   swiveled *around* must be "up" — the way someone filming a room while turning on the
   spot rotates around their own vertical axis. But this rig was handheld, not on a fixed
   stand, so it rolled just as much as it panned. Checked against the real photos, this
   guess pointed almost straight down.
2. The other guess was computed before the axis translation from Phase E was known, so it
   was quietly describing the wrong set of coordinates the whole time. Re-checked in the
   correct, final coordinate system, it turned out to point sideways — roughly at the back
   wall — not up at all.

On top of both, a leftover flip had been baked into the math that didn't belong there: it
was meant to cancel out something the viewer does, but the viewer's behavior was already
being accounted for elsewhere, so this was an unnecessary second flip stacked on top.
Removing just that flip would not have fixed the picture on its own, since both guesses
above were still wrong underneath it.

**The fix:** stop guessing "up" from how the camera moved, and read it off the room
itself instead. The back wall in this scene is a large, flat surface — easy to find
automatically, since most points cluster onto one flat plane — and the camera moved
sideways along that wall rather than toward or away from it. Given those two facts, "up"
has to be the one direction at a right angle to *both* the wall and the direction the
camera moved. There's exactly one such direction, and finding it doesn't depend on any
assumption about how the camera itself was oriented.

To make sure this new answer was actually right, rather than a third guess, it was
checked against something that owes nothing to any of the camera math: in the original
photographs, each row of pixels corresponds to a height in the real room, and rows count
downward going down the image. So if "up" is correct, points near the top of a photo
should end up high in the 3D room, and points near the bottom should end up low.

| Candidate "up" | Photo 1 | Photo 2 | Photo 3 | Result |
|---|---|---|---|---|
| **New, measured from the wall** | matches | matches | matches | correct |
| Old guess #2 (camera swivel) | wrong | wrong | wrong | pointing down |
| Old guess #1 (camera turning) | no relation | no relation | no relation | not vertical at all |

That checked out in all three photos. Finally, it was confirmed the only way that
actually counts: relaunching the real viewer and looking. The room now stands upright.

**The takeaway:** two methods agreeing with each other isn't proof of anything if both
are built on the same unchecked assumption — here, an assumption about which way the
camera itself was facing. The fix was to stop assuming and start measuring, against
evidence (the shape of the room, and the original photos) that doesn't depend on the
thing being questioned.

## Further reading

- [PLY file format](http://paulbourke.net/dataformats/ply/)
- [Open3D documentation](https://www.open3d.org/docs/release/) — point cloud read/write, outlier removal
- [Pinhole camera model](https://en.wikipedia.org/wiki/Pinhole_camera_model) and [camera intrinsics](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Camera-to-world transforms](https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points.html)
- [Rotation matrices](https://en.wikipedia.org/wiki/Rotation_matrix) and [Rodrigues' rotation formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula)
- [Rotations vs. mirror reflections](https://en.wikipedia.org/wiki/Improper_rotation)
- [Unity's coordinate system](https://docs.unity3d.com/Manual/class-Transform.html)
- [`uv`](https://docs.astral.sh/uv/) and [NumPy](https://numpy.org/doc/stable/) documentation
