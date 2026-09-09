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


def wall_normal(world, Rs):
    """Normal of the scene's dominant plane, by iterating its surface-normal cluster.

    Seeded deterministically from the cameras' own optical axis (under C that is -R[:,1]),
    because the cameras face the wall. The iteration is only locally convergent -- random
    seeds settle on side walls with a tenth of the support -- so the seed is load-bearing.
    """
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(world))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    N = np.asarray(pc.normals)
    n = np.mean([-R[:, 1] for R in Rs], 0); n /= np.linalg.norm(n)
    for _ in range(30):
        sel = N[np.abs(N @ n) > np.cos(np.radians(10))]
        sel = sel * np.sign(sel @ n)[:, None]
        nxt = np.linalg.eigh(sel.T @ sel)[1][:, -1]
        if nxt @ n < 0:
            nxt = -nxt
        settled = np.degrees(np.arccos(np.clip(nxt @ n, -1, 1))) < 1e-4
        n = nxt
        if settled:
            break
    return n


def scene_up(world, Rs, ts):
    """Measure the room's vertical from the scene itself, not from the camera motion.

    The capture faces a wall, and walls are vertical; the rig moved horizontally along it
    (its path measures 89.4 deg off the wall normal). The vertical is therefore the common
    perpendicular of those two directions. The sign comes from the cameras' image-up axis:
    Phase E regressed fy < 0, so the PLY's local +y is image-up, which is -R[:,2] in world.
    """
    up = np.cross(ts[2] - ts[0], wall_normal(world, Rs))
    up /= np.linalg.norm(up)
    cam_up = np.mean([-R[:, 2] for R in Rs], 0)
    return up if up @ cam_up > 0 else -up


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
    ts = [poses[i][:3, 3] for i in range(3)]

    S0, RGB, probe = [], [], []
    for i, n in enumerate(NAMES):
        pc = o3d.io.read_point_cloud(str(SRC / f"{n}.ply"))
        p = np.asarray(pc.points)[::sub]
        rgb = np.rint(np.asarray(pc.colors) * 255).astype(int)[::sub]
        p, rgb = deconfetti(p, rgb)
        w = (poses[i] @ np.column_stack([(C @ p.T).T, np.ones(len(p))]).T).T[:, :3]
        S0.append((V_pt @ w.T).T); RGB.append(rgb); probe.append(w[::max(1, 100 // sub)])
        print(f"  {n}: {len(p)} pts (full)" if sub == 1 else f"  {n}: {len(p)} pts")

    up = scene_up(np.vstack(probe), Rs, ts)
    # the viewer re-applies V_pt on load, so the rotation it actually shows is D @ V_pt;
    # D is exactly what sends the measured up to Unity's +Y under that composition.
    D = minimal_rot(V_pt @ up, np.array([0., 1, 0]))
    print(f"  measured up = {np.array2string(up, precision=4)}; "
          f"displayed up = {np.array2string(D @ V_pt @ up, precision=3)}")

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
