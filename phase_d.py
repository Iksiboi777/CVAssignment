"""Phase D: stage an axis-triad glyph with known poses to read the viewer's point & pose maps."""
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ply_writer import write_ply

ROOT = Path(__file__).resolve().parent.parent
SA = ROOT / "ComputerVisionAssignment_Data" / "StreamingAssets"
POINTS = SA / "Points"
PROBES = Path(__file__).resolve().parent / "probes"
NAMES = ("image1", "image2", "image3")


def _arm(direction, length, color, n=400, thick=0.03):
    """A thin colored rod of points from origin along `direction`, length `length`."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(d, a); u /= np.linalg.norm(u)
    v = np.cross(d, u)
    t = np.linspace(0, length, n)
    base = t[:, None] * d[None, :]
    offs = [np.zeros(3), thick * u, -thick * u, thick * v, -thick * v]
    pts = np.vstack([base + o for o in offs])
    return pts, np.tile(color, (len(pts), 1))


def make_glyph():
    """Return (xyz (N,3) float, rgb (N,3) 0-255) for the axis-triad calibration glyph."""
    parts = []
    g = np.linspace(-0.15, 0.15, 5)
    bx, by, bz = np.meshgrid(g, g, g)
    blob = np.column_stack([bx.ravel(), by.ravel(), bz.ravel()])
    parts.append((blob, np.tile((255, 255, 255), (len(blob), 1))))
    parts.append(_arm((1, 0, 0), 3.0, (255, 0, 0)))
    parts.append(_arm((0, 1, 0), 2.0, (0, 255, 0)))
    parts.append(_arm((0, 0, 1), 1.0, (0, 0, 255)))
    t = np.linspace(0, 0.8, 150)
    hook = np.column_stack([np.full_like(t, 3.0), t, np.zeros_like(t)])
    parts.append((hook, np.tile((255, 0, 255), (len(hook), 1))))
    xyz = np.vstack([p for p, _ in parts])
    rgb = np.vstack([c for _, c in parts]).astype(int)
    return xyz, rgb


def _Rx(deg):
    th = np.radians(deg); c, s = np.cos(th), np.sin(th)
    M = np.eye(4); M[1, 1] = c; M[1, 2] = -s; M[2, 1] = s; M[2, 2] = c
    return M


def _T(tx, ty, tz):
    M = np.eye(4); M[:3, 3] = (tx, ty, tz)
    return M


def write_traj(path, mats):
    """Write 4x4 matrices row-major (matches the original traj.txt layout, Phase A1)."""
    rows = [" ".join(f"{v:.8g}" for v in M.reshape(-1)) for M in mats]
    Path(path).write_text("\n".join(rows) + "\n")


def reference_render(xyz, rgb, out):
    """Local picture of the glyph in our axes, so you know what 'no transform' looks like."""
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=rgb / 255, s=2, depthshade=False)
    ax.set(xlabel="X", ylabel="Y", zlabel="Z",
           title="Glyph reference (our convention)\nX=red(3) Y=green(2) Z=blue(1), magenta L hook")
    ax.set_xlim(-1, 3.5); ax.set_ylim(-1, 3.5); ax.set_zlim(-1, 3.5)
    ax.view_init(elev=20, azim=-60)
    fig.tight_layout(); fig.savefig(out, dpi=110); plt.close(fig)


def stage_run(run):
    """Write the glyph into all 3 cloud slots and a traj.txt that isolates one viewer map."""
    PROBES.mkdir(parents=True, exist_ok=True)
    POINTS.mkdir(parents=True, exist_ok=True)
    xyz, rgb = make_glyph()
    write_ply(PROBES / "glyph.ply", xyz, rgb)
    reference_render(xyz, rgb, PROBES / "glyph_reference.png")
    for n in NAMES:
        write_ply(POINTS / f"{n}.ply", xyz, rgb)

    if run == 1:
        mats = [np.eye(4), np.eye(4), np.eye(4)]
        desc = "RUN 1 (CP2): identity x3 -> measures V_pt (the point map)."
    else:
        mats = [np.eye(4), _T(1, 2, 3), _Rx(90)]
        desc = "RUN 2 (CP3): [identity, t=(1,2,3), Rx90] -> measures F (the pose map)."
    write_traj(SA / "traj.txt", mats)

    print(f"staged {desc}")
    print(f"  glyph: {len(xyz)} points written to all 3 slots in {POINTS}")
    print(f"  reference image: {PROBES/'glyph_reference.png'}")
    print("  RELAUNCH ComputerVisionAssignment.exe and read off what you see.")
    print("  restore real data anytime by copying data_original\\* back over StreamingAssets.")


if __name__ == "__main__":
    stage_run(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
