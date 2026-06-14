"""Phase B: write ASCII PLYs that are byte-shaped exactly like the originals, so the viewer's unknown parser accepts them."""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data_original"
DEFAULT_TEMPLATE = SRC / "image1.ply"
SCRATCH = Path(__file__).resolve().parent / "_scratch"


def clone_header(template_path, n_vertices):
    """Return the template's header lines verbatim (only `element vertex` rewritten) plus its line ending."""
    raw = Path(template_path).read_bytes()
    end = raw.find(b"end_header")
    head = raw[: end + len(b"end_header")]
    nl = "\r\n" if b"\r\n" in head else "\n"
    lines = head.decode("ascii").splitlines()
    out = []
    for ln in lines:
        if ln.startswith("element vertex"):
            out.append(f"element vertex {n_vertices}")
        else:
            out.append(ln)
    return out, nl


def write_ply(path, xyz, rgb, template=DEFAULT_TEMPLATE):
    """Write an ASCII PLY structurally identical to the template; xyz (N,3) float, rgb (N,3) in 0..255."""
    xyz = np.asarray(xyz, dtype=np.float64)
    rgb = np.rint(np.asarray(rgb)).astype(np.int64)
    assert xyz.ndim == 2 and xyz.shape[1] == 3, "xyz must be (N,3)"
    assert rgb.shape == xyz.shape, "rgb must match xyz shape"
    assert rgb.min() >= 0 and rgb.max() <= 255, "colors must be in 0..255 (uchar)"

    n = len(xyz)
    header_lines, nl = clone_header(template, n)
    data = np.column_stack([xyz, rgb])
    fmt = ["%.9g", "%.9g", "%.9g", "%d", "%d", "%d"]

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        f.write(nl.join(header_lines) + nl)
        np.savetxt(f, data, fmt=fmt, delimiter=" ", newline=nl)
    return n


def _self_test():
    """Prove the writer's output re-parses to the same numbers and the header is a faithful clone."""
    import open3d as o3d

    src = SRC / "image1.ply"
    pc = o3d.io.read_point_cloud(str(src))
    xyz = np.asarray(pc.points)
    rgb = np.rint(np.asarray(pc.colors) * 255).astype(int)
    print(f"loaded template cloud: {len(xyz)} verts")

    step = 100
    out = SCRATCH / "image1_ds.ply"
    n_written = write_ply(out, xyz[::step], rgb[::step])
    print(f"wrote {out.name}: {n_written} verts (every {step}th)")

    tmpl_lines, _ = clone_header(src, n_written)
    out_head = out.read_bytes().split(b"end_header")[0] + b"end_header"
    out_lines = out_head.decode("ascii").splitlines()
    assert out_lines == tmpl_lines, "header is NOT a faithful clone"
    print("  header check: PASS (identical to template except `element vertex`)")
    for ln in out_lines:
        print(f"    | {ln}")

    back = o3d.io.read_point_cloud(str(out))
    xyz_b = np.asarray(back.points)
    rgb_b = np.rint(np.asarray(back.colors) * 255).astype(int)
    assert len(xyz_b) == n_written, "vertex count changed on round-trip"
    assert np.allclose(xyz[::step], xyz_b, atol=1e-5), "coordinates drifted"
    assert np.array_equal(rgb[::step], rgb_b), "colors changed"
    print(f"  round-trip check: PASS (coords within 1e-5, colors exact, {n_written} verts)")
    print("\nB writer verified. Output a parser reads back identically.")


if __name__ == "__main__":
    _self_test()
