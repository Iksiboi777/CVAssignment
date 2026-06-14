import shutil
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ply_writer import write_ply

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data_original"
SA = ROOT / "ComputerVisionAssignment_Data" / "StreamingAssets"
POINTS = SA / "Points"
NAMES = ("image1", "image2", "image3")
DS_STEP = 100
PREVIEW_STEP = 300
PREVIEW_OUT = Path(__file__).resolve().parent / "preview.png"


def place(points, M):
    """Apply a 4x4 homogeneous transform M to (N,3) points."""
    ph = np.column_stack([points, np.ones(len(points))])
    return (M @ ph.T).T[:, :3]


def load_cloud(name, step):
    """Read a cloud's points + colors (0..1 floats), keeping every step-th point."""
    pc = o3d.io.read_point_cloud(str(SRC / f"{name}.ply"))
    xyz = np.asarray(pc.points)[::step]
    rgb = np.asarray(pc.colors)[::step]
    return xyz, rgb


def downsample(step=DS_STEP):
    """Stage CP1: write sparse copies of the originals into StreamingAssets, poses left original."""
    for n in NAMES:
        assert (SRC / f"{n}.ply").exists(), f"missing backup data_original\\{n}.ply"
    assert (SRC / "traj.txt").exists(), "missing backup data_original\\traj.txt"

    POINTS.mkdir(parents=True, exist_ok=True)
    print(f"downsampling every {step}th point -> {POINTS}")
    for n in NAMES:
        xyz, rgb = load_cloud(n, step)
        n_written = write_ply(POINTS / f"{n}.ply", xyz, np.rint(rgb * 255).astype(int))
        print(f"  {n}: kept {n_written} pts")

    shutil.copyfile(SRC / "traj.txt", SA / "traj.txt")
    print("traj.txt set to original.")
    print("\nCP1 ready -> RELAUNCH ComputerVisionAssignment.exe (it reads files only at startup).")
    print("Expect: the same CP0 scene, sparser, orbiting fast. Corrupted/won't load => writer bug.")


def preview(D=None, out=PREVIEW_OUT, step=PREVIEW_STEP):
    """Compose world = D . (M . p) and render 3 viewpoints as a PNG (local microscope, not a verdict)."""
    D = np.eye(4) if D is None else np.asarray(D, dtype=float)
    poses = np.loadtxt(SRC / "traj.txt").reshape(-1, 4, 4)

    worlds, colors, cams = [], [], []
    for i, n in enumerate(NAMES):
        xyz, rgb = load_cloud(n, step)
        w = place(place(xyz, poses[i]), D)
        worlds.append(w)
        colors.append(rgb)
        cams.append(place(poses[i][:3, 3][None, :], D)[0])
    P = np.vstack(worlds)
    C = np.clip(np.vstack(colors), 0, 1)
    cams = np.array(cams)
    print(f"composed {len(P)} pts from 3 clouds (every {step}th); D = "
          f"{'identity' if np.allclose(D, np.eye(4)) else 'custom'}")

    lo, hi = P.min(0), P.max(0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2

    views = [("perspective", 22, -60), ("top-down", 89, -90), ("eye-level", 6, -75)]
    fig = plt.figure(figsize=(18, 6))
    for k, (title, elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(1, 3, k, projection="3d")
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], c=C, s=1, depthshade=False)
        ax.scatter(cams[:, 0], cams[:, 1], cams[:, 2], c="red", s=140,
                   marker="^", edgecolors="k", label="cameras")
        ax.set(title=title, xlabel="X", ylabel="Y", zlabel="Z")
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(elev=elev, azim=azim)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("Phase C microscope -- local placement preview (NOT a final verdict)")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode in ("downsample", "all"):
        downsample()
    if mode in ("preview", "all"):
        preview()


if __name__ == "__main__":
    main()
