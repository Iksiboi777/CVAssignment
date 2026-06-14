"""Phase A1+A2: prove traj.txt's layout, then decide if its poses are c2w or w2c."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

SRC = Path(__file__).resolve().parent.parent / "data_original"
OUT = Path(__file__).resolve().parent
TRAJ = SRC / "traj.txt"
ATOL = 1e-5
STEP = 400
NAMES = ("image1", "image2", "image3")


def check_one_reading(M, label):
    """Run the four structural pose checks on a 4x4 M; return True iff all pass."""
    R = M[:3, :3]
    t = M[:3, 3]
    bottom = M[3, :]

    ortho_err = np.abs(R.T @ R - np.eye(3)).max()
    ortho_ok = ortho_err < ATOL

    det = np.linalg.det(R)
    det_ok = np.isclose(det, 1.0, atol=ATOL)

    bottom_err = np.abs(bottom - np.array([0.0, 0.0, 0.0, 1.0])).max()
    bottom_ok = bottom_err < ATOL

    t_norm = np.linalg.norm(t)
    t_ok = t_norm > ATOL

    all_ok = ortho_ok and det_ok and bottom_ok and t_ok

    print(f"  [{label}]")
    print(f"    orthonormal R^T R == I : {ortho_ok!s:5}  (max|R^T R - I| = {ortho_err:.2e})")
    print(f"    proper rotation det=+1 : {det_ok!s:5}  (det R = {det:+.6f})")
    print(f"    bottom row == 0 0 0 1  : {bottom_ok!s:5}  (got {np.array2string(bottom, precision=3)})")
    print(f"    translation non-zero   : {t_ok!s:5}  (t = {np.array2string(t, precision=3)}, |t| = {t_norm:.3f})")
    print(f"    --> valid pose?          {all_ok}")
    return all_ok


def validate_traj(raw):
    """A1: test row-major vs column-major reading of each line and print the verdict."""
    print(f"loaded {TRAJ.name}: shape {raw.shape}  (expect 3 lines x 16 floats)\n")

    row_major_pass, col_major_pass = [], []
    for i, line in enumerate(raw, start=1):
        print(f"matrix {i}")
        M_row = line.reshape(4, 4)
        M_col = line.reshape(4, 4).T
        row_major_pass.append(check_one_reading(M_row, "row-major"))
        col_major_pass.append(check_one_reading(M_col, "column-major"))
        print()

    print("=" * 64)
    print(f"row-major    : {sum(row_major_pass)}/3 matrices are valid poses")
    print(f"column-major : {sum(col_major_pass)}/3 matrices are valid poses")
    if all(row_major_pass) and not any(col_major_pass):
        print("\nCONCLUSION: traj.txt is ROW-MAJOR 4x4 homogeneous camera poses.")
        print("The data is clean rigid transforms -> the problem is purely a")
        print("coordinate-convention mismatch, nothing is corrupted.")
    else:
        print("\nUnexpected result -- inspect the per-matrix output above.")


def load_clouds():
    """Load the 3 downsampled clouds (every STEPth point) as (points, colors) lists."""
    clouds, colors = [], []
    for name in NAMES:
        pc = o3d.io.read_point_cloud(str(SRC / f"{name}.ply"))
        clouds.append(np.asarray(pc.points)[::STEP])
        colors.append(np.asarray(pc.colors)[::STEP])
    return clouds, colors


def place(points, M):
    """Transform camera-local points (N,3) into the world via homogeneous world = M @ p."""
    ph = np.column_stack([points, np.ones(len(points))])
    return (M @ ph.T).T[:, :3]


def coherence_dump(worlds, label):
    """Print every intermediate of centroid-spread and coherence-ratio; return (spread, ratio)."""
    print(f"\n################  {label}  ################")

    cents = [w.mean(0) for w in worlds]
    print("\n-- centroids (mean of each cloud's points) --")
    for i, c in enumerate(cents, 1):
        print(f"   cloud {i}: ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f})")

    print("\n-- pairwise centroid distances --")
    dists = []
    for a, b in [(0, 1), (0, 2), (1, 2)]:
        diff = cents[a] - cents[b]
        d = np.linalg.norm(diff)
        dists.append(d)
        print(f"   |c{a+1} - c{b+1}| = |({diff[0]:+.4f}, {diff[1]:+.4f}, {diff[2]:+.4f})|"
              f" = sqrt({diff[0]**2:.4f}+{diff[1]**2:.4f}+{diff[2]**2:.4f}) = {d:.4f}")
    spread = max(dists)
    print(f"   => centroid spread = max = {spread:.4f}")

    print("\n-- per-cloud bounding boxes & diagonals --")
    diags = []
    for i, w in enumerate(worlds, 1):
        lo, hi = w.min(0), w.max(0)
        ext = hi - lo
        dg = np.linalg.norm(ext)
        diags.append(dg)
        print(f"   cloud {i}: min=({lo[0]:+.3f},{lo[1]:+.3f},{lo[2]:+.3f}) "
              f"max=({hi[0]:+.3f},{hi[1]:+.3f},{hi[2]:+.3f})")
        print(f"            extent=({ext[0]:.3f},{ext[1]:.3f},{ext[2]:.3f}) "
              f"diag=sqrt({ext[0]**2:.3f}+{ext[1]**2:.3f}+{ext[2]**2:.3f})={dg:.4f}")
    mean_diag = np.mean(diags)
    print(f"   => mean per-cloud diagonal = ({diags[0]:.4f}+{diags[1]:.4f}+{diags[2]:.4f})/3 "
          f"= {mean_diag:.4f}")

    allpts = np.vstack(worlds)
    lo, hi = allpts.min(0), allpts.max(0)
    ext = hi - lo
    comb = np.linalg.norm(ext)
    print("\n-- combined bounding box (all 3 clouds together) --")
    print(f"   min=({lo[0]:+.3f},{lo[1]:+.3f},{lo[2]:+.3f}) "
          f"max=({hi[0]:+.3f},{hi[1]:+.3f},{hi[2]:+.3f})")
    print(f"   extent=({ext[0]:.3f},{ext[1]:.3f},{ext[2]:.3f}) "
          f"combined diag = {comb:.4f}")

    ratio = comb / mean_diag
    print(f"\n-- coherence ratio --")
    print(f"   ratio = combined_diag / mean_per_cloud_diag = {comb:.4f} / {mean_diag:.4f} "
          f"= {ratio:.4f}")
    return spread, ratio


def render(worlds, colors, cams, title, fname):
    """Save a 3-view scatter (perspective / top-down / front) with camera markers."""
    views = [(20, -60, "perspective"), (89, -90, "top-down"), (5, -90, "front")]
    fig = plt.figure(figsize=(18, 6))
    for j, (elev, azim, lab) in enumerate(views):
        ax = fig.add_subplot(1, 3, j + 1, projection="3d")
        for w, c in zip(worlds, colors):
            ax.scatter(w[:, 0], w[:, 1], w[:, 2], c=np.clip(c, 0, 1), s=1, marker=".")
        ax.scatter(cams[:, 0], cams[:, 1], cams[:, 2], c="black", s=90, marker="^")
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(lab)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    fig.suptitle(title)
    fig.savefig(OUT / fname, dpi=90, bbox_inches="tight")
    plt.close(fig)
    print(f"    rendered -> {fname}")


def decide_direction(poses, clouds, colors):
    """A2: place clouds as c2w and w2c, dump metrics + render both, print the verdict."""
    worlds_c2w = [place(clouds[i], poses[i]) for i in range(3)]
    cams_c2w = poses[:, :3, 3]
    s_c2w, r_c2w = coherence_dump(worlds_c2w, "c2w   world = M . p")
    render(worlds_c2w, colors, cams_c2w, "Hypothesis c2w: world = M . p", "a2_c2w.png")

    invs = [np.linalg.inv(poses[i]) for i in range(3)]
    worlds_w2c = [place(clouds[i], invs[i]) for i in range(3)]
    cams_w2c = np.array([inv[:3, 3] for inv in invs])
    s_w2c, r_w2c = coherence_dump(worlds_w2c, "w2c   world = M^-1 . p")
    render(worlds_w2c, colors, cams_w2c, "Hypothesis w2c: world = M^-1 . p", "a2_w2c.png")

    print("\n" + "=" * 64)
    print(f"  c2w : spread {s_c2w:.4f}   ratio {r_c2w:.4f}")
    print(f"  w2c : spread {s_w2c:.4f}   ratio {r_w2c:.4f}")
    winner = "c2w (world = M @ p)" if r_c2w < r_w2c else "w2c (world = M^-1 @ p)"
    print("Lower coherence ratio = more interlocked = correct direction.")
    print(f"CONCLUSION: poses are {winner}.")
    print("Confirm by eye: open a2_c2w.png and a2_w2c.png -- the correct one")
    print("shows ONE coherent room; the wrong one shows scattered/disjoint sheets.")


def main():
    raw = np.loadtxt(TRAJ)
    validate_traj(raw)

    poses = raw.reshape(-1, 4, 4)
    clouds, colors = load_clouds()
    print(f"\nloaded 3 poses + 3 clouds (every {STEP}th point, ~{len(clouds[0])} pts each)")
    decide_direction(poses, clouds, colors)


if __name__ == "__main__":
    main()
