"""Phase F: apply C = -X,-Z,-Y, stand scene upright, export full-res to StreamingAssets."""
import sys
from pathlib import Path
import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ply_writer import write_ply

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data_original"
SA = ROOT / "ComputerVisionAssignment_Data" / "StreamingAssets"
POINTS = SA / "Points"
NAMES = ("image1", "image2", "image3")
C = np.array([[-1., 0, 0], [0, 0, -1], [0, -1, 0]])
V_pt = np.diag([1., -1, 1])


def rot_axis(R):
    w, v = np.linalg.eig(R)
    k = np.argmin(np.abs(w - 1))
    a = np.real(v[:, k]); return a / np.linalg.norm(a)


def minimal_rot(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = a @ b
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1., -1, -1])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def deconfetti(p, rgb):
    """remove isolated specks, keep all wall geometry (gentle std 2.0)."""
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    _, ind = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return p[ind], rgb[ind]


def main(sub=1):
    poses = np.loadtxt(SRC / "traj.txt").reshape(-1, 4, 4)
    Rs = [poses[i][:3, :3] for i in range(3)]
    axes = [rot_axis(Rs[b] @ Rs[a].T) for a, b in [(0, 1), (1, 2), (0, 2)]]
    axes = [a if a[1] >= 0 else -a for a in axes]
    up = np.mean(axes, 0); up /= np.linalg.norm(up)

    D = np.diag([1., -1, -1]) @ minimal_rot(V_pt @ up, np.array([0., 1, 0]))

    S0, RGB = [], []
    for i, n in enumerate(NAMES):
        pc = o3d.io.read_point_cloud(str(SRC / f"{n}.ply"))
        p = np.asarray(pc.points)[::sub]
        rgb = np.rint(np.asarray(pc.colors) * 255).astype(int)[::sub]
        p, rgb = deconfetti(p, rgb)
        w = (poses[i] @ np.column_stack([(C @ p.T).T, np.ones(len(p))]).T).T[:, :3]
        S0.append((V_pt @ w.T).T); RGB.append(rgb)
        print(f"  {n}: {len(p)} pts (full)" if sub == 1 else f"  {n}: {len(p)} pts")
    c0 = np.vstack(S0).mean(0)

    POINTS.mkdir(parents=True, exist_ok=True)
    for i, n in enumerate(NAMES):
        V = (D @ (S0[i] - c0).T).T + c0
        write_ply(POINTS / f"{n}.ply", (V_pt @ V.T).T, RGB[i])
    np.savetxt(SA / "traj.txt", np.tile(np.eye(4).reshape(1, 16), (len(poses), 1)), fmt="%.8g")
    print("traj.txt = identity x3.")
    print("\nRELAUNCH. Expect the SAME coherent scene as the bird's-eye one,")
    print("full-res (solid walls), navigable (spawned outside the cloud). No amputated tails.")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
