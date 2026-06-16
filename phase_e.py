"""Phase E: score all 48 axis conventions by photo-consistency; highest coincide% is C."""
from itertools import permutations, product
from pathlib import Path
import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data_original"
NAMES = ("image1", "image2", "image3")


def load(n):
    pc = o3d.io.read_point_cloud(str(SRC / f"{n}.ply"))
    return np.asarray(pc.points), np.asarray(pc.colors)


def recover_intrinsics(xyz, W, H):
    """assume row-major one-point-per-pixel; regress u~x/z, v~y/z. returns dict + R^2."""
    n = len(xyz)
    assert n == W * H, f"{n} pts != {W}x{H}={W*H}; ordering assumption broken"
    idx = np.arange(n)
    u = (idx % W).astype(float)
    v = (idx // W).astype(float)
    z = xyz[:, 2]
    good = np.abs(z) > 1e-6
    x, y, z = xyz[good, 0], xyz[good, 1], xyz[good, 2]
    u, v = u[good], v[good]

    def fit(pix, ratio):
        A = np.column_stack([ratio, np.ones_like(ratio)])
        (slope, icpt), *_ = np.linalg.lstsq(A, pix, rcond=None)
        pred = A @ [slope, icpt]
        ss = 1 - np.sum((pix - pred) ** 2) / np.sum((pix - pix.mean()) ** 2)
        return slope, icpt, ss

    fx, cx, r2u = fit(u, x / z)
    fy, cy, r2v = fit(v, y / z)
    return dict(fx=fx, cx=cx, fy=fy, cy=cy), (r2u, r2v)


def proper_and_principled():
    """all 48 signed axis maps (det +-1): the complete space of axis conventions."""
    out = {}
    ax = "XYZ"
    for perm in permutations(range(3)):
        for s in product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for r, c in enumerate(perm):
                M[r, c] = s[r]
            lab = ",".join(("+" if s[r] > 0 else "-") + ax[perm[r]] for r in range(3))
            out[lab] = M
    return out


def main():
    W, H = 2533, 1170
    poses = np.loadtxt(SRC / "traj.txt").reshape(-1, 4, 4)
    pts, cols = zip(*(load(n) for n in NAMES))

    K, (r2u, r2v) = recover_intrinsics(pts[0], W, H)
    print("recovered intrinsics (regressed from the data):")
    print(f"  fx={K['fx']:.1f}  cx={K['cx']:.1f}  (R^2={r2u:.4f})")
    print(f"  fy={K['fy']:.1f}  cy={K['cy']:.1f}  (R^2={r2v:.4f})")
    print(f"  sign(fx)={np.sign(K['fx']):+.0f} sign(fy)={np.sign(K['fy']):+.0f}  "
          f"(reveals points' x/y axis directions)\n")
    if min(r2u, r2v) < 0.95:
        print("  !! low R^2 -> point ordering is not plain row-major; results suspect.\n")

    fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']
    SUB = 25
    cand = proper_and_principled()
    scene = 12.0
    eps = 0.012 * scene

    results = []
    for lab, C in cand.items():
        Ci = C.T
        seen = coinc = 0
        dists, cmatch = [], 0
        for i in range(3):
            p = pts[i][::SUB]; c = cols[i][::SUB]
            wi = (poses[i] @ np.column_stack([(C @ p.T).T, np.ones(len(p))]).T).T[:, :3]
            for j in range(3):
                if j == i:
                    continue
                camP = (np.linalg.inv(poses[j]) @ np.column_stack([wi, np.ones(len(wi))]).T).T[:, :3]
                camB = (Ci @ camP.T).T
                z = camB[:, 2]
                front = z > 1e-6
                u = cx + fx * camB[:, 0] / np.where(front, z, 1)
                v = cy + fy * camB[:, 1] / np.where(front, z, 1)
                inb = front & (u >= 0) & (u < W) & (v >= 0) & (v < H)
                if not np.any(inb):
                    continue
                uu = u[inb].astype(int); vv = v[inb].astype(int)
                pj = pts[j][vv * W + uu]
                wj = (poses[j] @ np.column_stack([(C @ pj.T).T, np.ones(len(pj))]).T).T[:, :3]
                d3 = np.linalg.norm(wi[inb] - wj, axis=1)
                seen += np.sum(inb); coinc += np.sum(d3 < eps)
                dists.append(d3)
                cj = cols[j][vv * W + uu]
                cmatch += np.sum((d3 < eps) & (np.abs(c[inb] - cj).sum(1) < 0.30))
        allc = np.concatenate(dists) if dists else np.array([np.nan])
        results.append((lab, seen, float(np.median(allc)), coinc / max(seen, 1),
                        cmatch / max(coinc, 1)))

    results.sort(key=lambda r: -r[3])
    print(f"{'C':<12}{'#reproj':>9}{'med 3Ddist':>12}{'coincide%':>11}{'+colour%':>10}")
    print("-" * 54)
    for lab, seen, med, fr, cm in results[:12]:
        print(f"{lab:<12}{seen:>9}{med:>12.3f}{fr:>10.1%}{cm:>10.1%}")
    print(f"\neps = {eps:.2f}.  TRUE convention = highest coincide% (shared surface lands in")
    print("the same 3D spot from both cameras), confirmed by +colour%. Identity = no re-aim.")


if __name__ == "__main__":
    main()
