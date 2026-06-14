# Coordinate-System Conversion for the CV Viewer

Convert the provided `image1-3.ply` + `traj.txt` from their source coordinate system
into the one the Unity viewer expects, so the scene renders as one coherent, upright,
un-mirrored room. The exact source→viewer transform is not specified — finding it is
the assignment. This README grows phase by phase as the search progresses.

## How to run

```
uv venv delta_venv --python 3.12
uv pip install --python delta_venv\Scripts\python.exe numpy open3d opencv-python matplotlib
delta_venv\Scripts\python.exe solution\phase_a1_a2.py
delta_venv\Scripts\python.exe solution\phase_a3.py
delta_venv\Scripts\python.exe solution\ply_writer.py
delta_venv\Scripts\python.exe solution\phase_c.py preview
delta_venv\Scripts\python.exe solution\phase_c.py downsample
```

(open3d ships no Python 3.13 wheels, hence the pinned 3.12 env.)

`phase_c.py` takes a mode: `preview` renders a local PNG, `downsample` stages the
CP1 check into the viewer (see below), and no argument runs both.

## Phase A — what the input files are

Before transforming anything, establish with evidence what was provided.

- **A1 — `traj.txt` is row-major 4×4 homogeneous poses.** The 3 lines × 16 floats only
  fit a 4×4 matrix, and only the row-major reading yields an orthonormal `R` (det +1),
  a `0 0 0 1` bottom row, *and* a nonzero translation column. The column-major reading
  collapses every translation to zero — invalid. So the data is clean rigid transforms;
  the problem is purely a convention mismatch, nothing is corrupted.
- **A2 — the poses are camera-to-world.** Composing `world = M · p` interlocks the three
  clouds into one room (coherence ratio 1.64); the inverse `M⁻¹ · p` scatters them
  (2.45). So each cloud's pose places its camera-local points into the shared world.
- **A3 — the source world's vertical axis is Y, pointing down.** Two independent
  methods agree: (1) the camera's optical axis is local Z (every point has positive
  local Z — a camera can't see behind itself), and step × view gives world-Y as the
  remaining perpendicular axis; (2) the bottom 5% of world-placed points (the floor)
  is thinnest along Y (flatness 4.66) and thick along Z (0.55) — the floor-slab test.
  Conclusion: source world is **Y-down**; the viewer is Y-up; that single axis flip
  is the target of the transformation.

## Phase B — PLY I/O primitive

Before transforming any data, establish a writer that produces files the viewer's
unknown parser will accept without complaint.

- **B — header-faithful ASCII PLY writer.** The viewer's PLY parser is a black box
  (closed Unity build). The safest strategy is to clone the original header byte-for-byte
  — same property names, same types, same line endings, only the `element vertex` count
  rewritten — and emit data rows in the exact same `x y z r g b` format. A round-trip
  self-test (write → re-read with open3d) confirms coordinates are preserved to 1e-5 and
  colors are bit-exact. All later phases write converted clouds through this function,
  so any format problem surfaces here before touching the viewer.

## Phase C — fast iteration and the CP1 checkpoint

The full clouds are ~3M points each, so the viewer loads them in minutes. Every
geometry experiment would be unbearable at that speed, and a wrong guess is hard to
diagnose when each look costs minutes. Phase C fixes both with two tools and the
first rung of a checkpoint ladder.

- **`preview` — the local microscope.** Composes `world = M · p` in numpy and draws
  three fixed viewpoints to a PNG in seconds, no Unity. It renders *our current
  belief* about placement, so it is for fast iteration only, never the final verdict —
  only the real viewer certifies "done". An optional correction `D` can be passed to
  preview a candidate transform before committing it.
- **`downsample` — the CP1 I/O checkpoint.** Writes every 100th point (~30k/cloud)
  through the Phase B writer into the viewer's `StreamingAssets\Points\`, leaving
  `traj.txt` untouched. It transforms *nothing*: CP1 is purely an I/O test. Relaunch
  the viewer and you should see the **same scene as the untouched originals (CP0)**,
  just sparser and orbiting fast. If CP1 looks corrupted or won't load, the bug is in
  the writer — caught here, before any coordinate math is in play.

This is the discipline for the whole solution: each step is gated by a viewer
checkpoint, so a regression is localised to the one thing that changed since the last
green check. CP0 = untouched originals; CP1 = downsampled, same scene; later
checkpoints add the geometry. Reading `data_original\` only and writing
`StreamingAssets\` only, the pristine originals always remain the restore source.

## Searching for the transformation

*To be written after the viewer-calibration probing (Phase D) — that is the actual
search and the heart of this write-up.*
