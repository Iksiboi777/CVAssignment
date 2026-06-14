"""Phase A3: find the source world's vertical axis from camera orientation, then test it two independent ways."""

from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data_original"
NAMES = ("image1", "image2", "image3")
AXIS = {0: "X", 1: "Y", 2: "Z"}
STEP_GEOM = 800
STEP_TEST = 200


def angle_between(u, v):
    """Angle in degrees between two vectors."""
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def rotation_angle(Ra, Rb):
    """Degrees of rotation separating Ra and Rb, via the angle of Ra^T Rb."""
    Rrel = Ra.T @ Rb
    return np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1) / 2, -1.0, 1.0)))


def place(points, M):
    """Transform camera-local points (N,3) into the world via homogeneous world = M @ p."""
    ph = np.column_stack([points, np.ones(len(points))])
    return (M @ ph.T).T[:, :3]


def load_local(name, step):
    """Read a cloud's camera-local points, keeping every `step`th point."""
    return np.asarray(o3d.io.read_point_cloud(str(SRC / f"{name}.ply")).points)[::step]


def derive_vertical(poses, Rs, ts):
    """Argue the world vertical from orientation: optical axis is most-consistent, up = step x view."""
    print("== camera local axes, expressed in WORLD coordinates (columns of R) ==")
    for i in range(3):
        print(f"  camera {i+1}  position = ({ts[i][0]:+.3f}, {ts[i][1]:+.3f}, {ts[i][2]:+.3f})")
        for k in range(3):
            v = Rs[i][:, k]
            print(f"     local axis {k} -> world ({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f})")

    print("\n== pairwise rotation between cameras (degrees) ==")
    for a, b in [(0, 1), (0, 2), (1, 2)]:
        print(f"  cam{a+1} vs cam{b+1}: {rotation_angle(Rs[a], Rs[b]):.2f} deg")

    print("\n== consistency of each local axis across the 3 cameras ==")
    spreads, mean_dirs = {}, {}
    for k in range(3):
        vs = [Rs[i][:, k] for i in range(3)]
        spreads[k] = max(angle_between(vs[a], vs[b]) for a, b in [(0, 1), (0, 2), (1, 2)])
        m = np.mean(vs, axis=0)
        mean_dirs[k] = m / np.linalg.norm(m)
        print(f"  local axis {k}: max pairwise angle = {spreads[k]:5.2f} deg "
              f"| mean world dir = ({mean_dirs[k][0]:+.3f}, {mean_dirs[k][1]:+.3f}, {mean_dirs[k][2]:+.3f})")

    optical_local = min(spreads, key=spreads.get)
    view_dir = mean_dirs[optical_local]
    view_axis = int(np.argmax(np.abs(view_dir)))
    print(f"\n  -> most-consistent local axis = {optical_local}: this is the OPTICAL/VIEW axis")
    print(f"     (camera looks along world {'+' if view_dir[view_axis] > 0 else '-'}{AXIS[view_axis]}); "
          f"the other two axes swing => the camera ROLLS about its view axis")

    step = ts[2] - ts[0]
    step_u = step / np.linalg.norm(step)
    up = np.cross(step_u, view_dir)
    up /= np.linalg.norm(up)
    vdom = int(np.argmax(np.abs(up)))
    print(f"\n  step direction (cam1->cam3) = {np.array2string(step_u, precision=3)}  "
          f"(dominant {AXIS[int(np.argmax(np.abs(step_u)))]})")
    print(f"  view direction              = {np.array2string(view_dir, precision=3)}  "
          f"(dominant {AXIS[view_axis]})")
    print(f"  up = step x view            = {np.array2string(up, precision=3)}  "
          f"(dominant {AXIS[vdom]})")
    print(f"  -> VERTICAL WORLD AXIS = {AXIS[vdom]}  "
          f"(perpendicular to both the horizontal step and view axes)")

    print("\n== cross-check (a): OpenCV convention (local +Y = down) ==")
    ymean = np.mean([Rs[i][:, 1] for i in range(3)], axis=0)
    ymean /= np.linalg.norm(ymean)
    ydom = int(np.argmax(np.abs(ymean)))
    print(f"  mean of local +Y in world = {np.array2string(ymean, precision=3)}")
    print(f"  -> if OpenCV, world-DOWN ~ {'+' if ymean[ydom] > 0 else '-'}{AXIS[ydom]}, "
          f"so world-UP ~ {'-' if ymean[ydom] > 0 else '+'}{AXIS[ydom]}")

    print("\n== cross-check (b): geometry along the vertical axis ==")
    worlds = [place(load_local(name, STEP_GEOM), poses[i]) for i, name in enumerate(NAMES)]
    allw = np.vstack(worlds)
    cam_v = np.array([ts[i][vdom] for i in range(3)])
    print(f"  along world {AXIS[vdom]}:  scene points  min={allw[:, vdom].min():+.2f}  "
          f"median={np.median(allw[:, vdom]):+.2f}  max={allw[:, vdom].max():+.2f}")
    print(f"                 camera {AXIS[vdom]} coords = {np.array2string(cam_v, precision=2)}  "
          f"(mean {cam_v.mean():+.2f})")
    frac_below = (allw[:, vdom] < cam_v.mean()).mean()
    print(f"  fraction of scene with {AXIS[vdom]} < camera height = {frac_below*100:.1f}%")


def test_depth_axis(Rs, locals_):
    """Independent test of the optical axis: a camera sees only points in front, so depth is sign-consistent."""
    print("================ TEST 1: which LOCAL axis is the depth (optical) axis ================")
    print("(a camera sees only points in front of it => along depth, all points share one sign)\n")
    for i, p in enumerate(locals_, 1):
        print(f"  cloud {i}  (local coords, {len(p)} pts):")
        sign_consistent = None
        for k in range(3):
            lo, hi = p[:, k].min(), p[:, k].max()
            frac_pos = (p[:, k] > 0).mean()
            same_sign = (lo > 0) or (hi < 0)
            tag = "  <-- all one sign => DEPTH" if same_sign else ""
            if same_sign:
                sign_consistent = k
            print(f"     local {AXIS[k]}: min={lo:+7.3f}  max={hi:+7.3f}  "
                  f"frac>0={frac_pos*100:5.1f}%{tag}")
        if sign_consistent is not None:
            wd = Rs[i - 1][:, sign_consistent]
            wd = wd / np.linalg.norm(wd)
            wdom = int(np.argmax(np.abs(wd)))
            print(f"     => depth = local {AXIS[sign_consistent]};  R[:,{sign_consistent}] in world = "
                  f"({wd[0]:+.3f},{wd[1]:+.3f},{wd[2]:+.3f})  dominant {AXIS[wdom]}")
        print()
    print("  EXPECTED (from the orientation analysis above): depth = local Z, world view ~ -Z. Match confirms C1.\n")


def test_floor_slab(poses, locals_):
    """Independent test of the vertical: the floor is thin along vertical, wide across the other two axes."""
    print("================ TEST 2: which WORLD axis is vertical (floor-slab test) ================")
    print("(the floor is thin along vertical but wide across the other two axes)\n")
    worlds = [place(locals_[i], poses[i]) for i in range(3)]
    allw = np.vstack(worlds)
    print(f"  combined world cloud: {len(allw)} pts\n")
    best = None
    for k in range(3):
        coord = allw[:, k]
        thresh = np.percentile(coord, 5)
        floor = allw[coord <= thresh]
        thickness = floor[:, k].max() - floor[:, k].min()
        others = [j for j in range(3) if j != k]
        lateral = max(floor[:, others].max(0) - floor[:, others].min(0))
        flatness = lateral / thickness if thickness > 0 else np.inf
        print(f"  candidate vertical = {AXIS[k]}:  floor thickness(along {AXIS[k]})={thickness:6.3f}  "
              f"lateral extent={lateral:6.3f}  flatness={flatness:6.2f}")
        if best is None or flatness > best[1]:
            best = (k, flatness)
    print(f"\n  => flattest floor is perpendicular to {AXIS[best[0]]}  =>  VERTICAL = {AXIS[best[0]]}")
    print("  EXPECTED (from the orientation analysis above): vertical = Y. Match confirms C2.")


def main():
    poses = np.loadtxt(SRC / "traj.txt").reshape(-1, 4, 4)
    Rs = [poses[i][:3, :3] for i in range(3)]
    ts = [poses[i][:3, 3] for i in range(3)]

    derive_vertical(poses, Rs, ts)

    print()
    locals_ = [load_local(n, STEP_TEST) for n in NAMES]
    test_depth_axis(Rs, locals_)
    test_floor_slab(poses, locals_)


if __name__ == "__main__":
    main()
