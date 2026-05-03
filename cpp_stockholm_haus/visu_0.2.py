#!/usr/bin/env python3
"""
visualize_planes.py — Visualise RANSAC plane segmentation results.

Three modes:
  1. Load a colored PLY written by save_colored_ply()    — view cloud only.
  2. Load a mesh PLY written by save_plane_mesh()        — view cloud + faces.
  3. Load any raw PCD/PLY and run Open3D RANSAC on it    — segment + display.

Usage:
  # View a pre-colored cloud (no faces)
  python visualize_planes.py result_colored.ply

  # View a mesh PLY produced by save_plane_mesh()
  python visualize_planes.py planes_mesh.ply

  # Run Open3D RANSAC on a raw cloud
  python visualize_planes.py scan.pcd --segment

  # Segment into up to N planes, then write a mesh PLY alongside
  python visualize_planes.py scan.pcd --segment --num-planes 3 --save-mesh out.ply

  # Tune the inlier threshold (metres)
  python visualize_planes.py scan.pcd --segment --threshold 0.05

Dependencies:
  pip install open3d numpy
"""

import argparse
import sys
import struct
import numpy as np
from pathlib import Path

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d not found — install it with:  pip install open3d")


# ── Palette matching pointcloud_io.hpp ───────────────────────────────
PLANE_COLORS = np.array([
    [0.839, 0.153, 0.157],
    [0.122, 0.467, 0.706],
    [0.173, 0.627, 0.173],
    [1.000, 0.498, 0.055],
    [0.580, 0.404, 0.741],
    [0.090, 0.745, 0.812],
    [0.737, 0.741, 0.133],
    [0.894, 0.467, 0.761],
], dtype=np.float64)
OUTLIER_COLOR = np.array([0.47, 0.47, 0.47])


# ─────────────────────────────────────────────────────────────────────
# PLY introspection
# ─────────────────────────────────────────────────────────────────────

def ply_has_faces(path):
    try:
        with open(path, "rb") as f:
            for _ in range(40):
                line = f.readline().decode("ascii", errors="ignore").strip()
                if line == "end_header":
                    break
                if line.startswith("element face"):
                    parts = line.split()
                    return len(parts) == 3 and int(parts[2]) > 0
    except Exception:
        pass
    return False


# ─────────────────────────────────────────────────────────────────────
# Convex hull + plane mesh helpers (pure numpy, mirrors C++ logic)
# ─────────────────────────────────────────────────────────────────────

def convex_hull_2d(pts2d):
    n = len(pts2d)
    if n < 3:
        return list(range(n))
    order = np.lexsort((pts2d[:, 1], pts2d[:, 0]))
    def cross(o, a, b):
        return ((pts2d[a,0]-pts2d[o,0])*(pts2d[b,1]-pts2d[o,1]) -
                (pts2d[a,1]-pts2d[o,1])*(pts2d[b,0]-pts2d[o,0]))
    lower, upper = [], []
    for i in order:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= 0:
            lower.pop()
        lower.append(i)
    for i in reversed(order):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= 0:
            upper.pop()
        upper.append(i)
    return lower[:-1] + upper[:-1]


def grid_filter(pts2d: np.ndarray,
                cell_size: float = 0.10,
                min_pts: int = 2,
                erode_iters: int = 1,
                dilate_iters: int = 1):
    """
    2-D occupancy grid filter with optional morphological opening/closing.

    Pipeline:
      raw counts → threshold → erode (iters) → dilate (iters)

    Returns
    -------
    mask       : bool array (N,) — True = point survives
    pre_morph  : uint8 array (nu*nv,) — binary bitmap after threshold only
    post_morph : uint8 array (nu*nv,) — binary bitmap after morphology
    grid_shape : (nu, nv)
    counts     : int array (nu*nv,) — raw per-cell point counts
    """
    if len(pts2d) == 0:
        return (np.zeros(0, dtype=bool),
                np.zeros(0, dtype=np.uint8), np.zeros(0, dtype=np.uint8),
                (0, 0), np.zeros(0, dtype=int))

    umin, vmin = pts2d.min(axis=0)
    umax, vmax = pts2d.max(axis=0)

    nu = max(1, int(np.ceil((umax - umin) / cell_size)))
    nv = max(1, int(np.ceil((vmax - vmin) / cell_size)))

    if nu * nv > 4_000_000:
        ones = np.ones(len(pts2d), dtype=bool)
        return ones, ones.astype(np.uint8), ones.astype(np.uint8), (nu, nv), np.ones(nu*nv, dtype=int)

    ci = np.clip(((pts2d[:, 0] - umin) / cell_size).astype(int), 0, nu - 1)
    cj = np.clip(((pts2d[:, 1] - vmin) / cell_size).astype(int), 0, nv - 1)
    cell_id = cj * nu + ci

    counts   = np.bincount(cell_id, minlength=nu * nv)
    pre      = (counts >= min_pts).astype(np.uint8)
    bitmap   = pre.reshape(nv, nu).copy()

    def erode(bm, r=1):
        out = np.zeros_like(bm)
        nv_, nu_ = bm.shape
        for dj in range(-r, r+1):
            for di in range(-r, r+1):
                shifted = np.zeros_like(bm)
                sj0, sj1 = max(0,-dj), min(nv_, nv_-dj)
                si0, si1 = max(0,-di), min(nu_, nu_-di)
                dj0, dj1 = max(0, dj), min(nv_, nv_+dj)
                di0, di1 = max(0, di), min(nu_, nu_+di)
                shifted[dj0:dj1, di0:di1] = bm[sj0:sj1, si0:si1]
                if dj == -r and di == -r:
                    out = shifted.copy()
                else:
                    out &= shifted
        return out

    def dilate(bm, r=1):
        out = np.zeros_like(bm)
        nv_, nu_ = bm.shape
        for dj in range(-r, r+1):
            for di in range(-r, r+1):
                sj0, sj1 = max(0,-dj), min(nv_, nv_-dj)
                si0, si1 = max(0,-di), min(nu_, nu_-di)
                dj0, dj1 = max(0, dj), min(nv_, nv_+dj)
                di0, di1 = max(0, di), min(nu_, nu_+di)
                out[dj0:dj1, di0:di1] |= bm[sj0:sj1, si0:si1]
        return out

    for _ in range(erode_iters):
        bitmap = erode(bitmap)
    for _ in range(dilate_iters):
        bitmap = dilate(bitmap)

    post     = bitmap.flatten().astype(np.uint8)
    mask     = post[cell_id].astype(bool)
    return mask, pre, post, (nu, nv), counts


def plane_basis(normal):
    arb = np.array([1.,0.,0.]) if abs(normal[0]) < 0.9 else np.array([0.,1.,0.])
    u = np.cross(normal, arb); u /= np.linalg.norm(u)
    v = np.cross(normal, u);   v /= np.linalg.norm(v)
    return u, v


def build_plane_mesh(plane_model, inlier_pts, color,
                     cell_size=0.10, min_pts=2,
                     erode_iters=1, dilate_iters=1):
    a, b, c, d = plane_model
    normal = np.array([a, b, c]); normal /= np.linalg.norm(normal)
    u, v   = plane_basis(normal)
    pts2d  = np.column_stack([inlier_pts @ u, inlier_pts @ v])

    mask, pre, post, (nu, nv), counts = grid_filter(
        pts2d, cell_size=cell_size, min_pts=min_pts,
        erode_iters=erode_iters, dilate_iters=dilate_iters)

    pts2d_kept = pts2d[mask]
    n_removed  = mask.size - mask.sum()
    if n_removed:
        print(f"    grid filter: removed {n_removed} / {mask.size} inlier pts "
              f"(erode={erode_iters}, dilate={dilate_iters})")

    hidx = convex_hull_2d(pts2d_kept)
    if len(hidx) < 3:
        return None, inlier_pts[mask]

    origin  = -d * normal
    hull_3d = np.array([origin + pts2d_kept[i,0]*u + pts2d_kept[i,1]*v
                        for i in hidx])
    centroid = hull_3d.mean(axis=0)
    centroid -= normal * (normal @ centroid + d)
    verts  = np.vstack([centroid, hull_3d])
    n_ring = len(hull_3d)
    faces  = [[0, 1+i, 1+(i+1)%n_ring] for i in range(n_ring)]
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(faces))
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh, inlier_pts[mask]


# ─────────────────────────────────────────────────────────────────────
# Segment planes with Open3D RANSAC
# ─────────────────────────────────────────────────────────────────────

def segment_planes(pcd, num_planes, threshold, min_frac=0.02):
    pts_np      = np.asarray(pcd.points)
    n_total     = len(pts_np)
    colors      = np.tile(OUTLIER_COLOR, (n_total, 1))
    plane_info  = []
    global_mask = np.ones(n_total, dtype=bool)
    remaining   = pcd

    for pi in range(num_planes):
        if len(remaining.points) < 3:
            break
        plane_model, local_inliers = remaining.segment_plane(
            distance_threshold=threshold, ransac_n=3, num_iterations=1000)
        frac = len(local_inliers) / n_total
        if frac < min_frac:
            print(f"  Plane {pi}: {frac:.1%} inliers — stopping")
            break
        a, b, c, d = plane_model
        print(f"  Plane {pi}: normal=({a:.3f},{b:.3f},{c:.3f}) d={d:.3f}"
              f"  inliers={len(local_inliers)} ({frac:.1%})")
        live      = np.where(global_mask)[0]
        orig_idx  = live[local_inliers]
        col       = PLANE_COLORS[pi % len(PLANE_COLORS)]
        colors[orig_idx] = col
        plane_info.append((plane_model, pts_np[orig_idx]))
        global_mask[orig_idx] = False
        remaining = remaining.select_by_index(local_inliers, invert=True)

    colored = o3d.geometry.PointCloud()
    colored.points = pcd.points
    colored.colors = o3d.utility.Vector3dVector(colors)
    return colored, plane_info


# ─────────────────────────────────────────────────────────────────────
# Save mesh PLY (mirrors C++ save_plane_mesh format exactly)
# ─────────────────────────────────────────────────────────────────────

def save_grid_images_py(prefix: str,
                        pcd,
                        plane_info: list,
                        cell_size=0.25,
                        min_pts=2,
                        erode_iters=1,
                        dilate_iters=1,
                        max_dim=512,
                        write_density=True,
                        write_diff=True,
                        write_morph=True):
    """
    Write per-plane grid diagnostic PNGs matching C++ save_grid_images().

    Three image types per plane:
      _density  — log-scaled heatmap  (black→blue→cyan→yellow→white)
      _diff     — threshold decision  (green=kept, red=removed)
      _morph    — morphology overlay  (cyan=core, yellow=dilated back, red=eroded)
    """
    from PIL import Image   # only needed here

    pts_np = np.asarray(pcd.points, dtype=np.float32)
    n_written = 0

    COLORMAP = np.array([
        [0,    0,   0],
        [0,    0, 200],
        [0,  200, 200],
        [220, 220,  0],
        [255, 255, 255],
    ], dtype=np.float32)
    T_STOPS = np.array([0.0, 0.25, 0.5, 0.75, 1.0])

    def density_color(t_arr):
        """Map t in [0,1] → RGB via 5-stop colormap, returns (N,3) uint8."""
        t = np.clip(t_arr, 0, 1)
        rgb = np.zeros((len(t), 3), dtype=np.float32)
        for i in range(4):
            mask = (t >= T_STOPS[i]) & (t <= T_STOPS[i+1])
            if not mask.any():
                continue
            s = (t[mask] - T_STOPS[i]) / (T_STOPS[i+1] - T_STOPS[i] + 1e-9)
            s = np.clip(s, 0, 1)[:, None]
            rgb[mask] = COLORMAP[i] * (1 - s) + COLORMAP[i+1] * s
        return rgb.astype(np.uint8)

    for pi, (plane_model, inlier_pts) in enumerate(plane_info):
        a, b, c, d = plane_model
        normal = np.array([a, b, c]); normal /= np.linalg.norm(normal)
        u, v_ax = plane_basis(normal)
        pts2d = np.column_stack([inlier_pts @ u, inlier_pts @ v_ax])

        # Scale cell if grid would exceed max_dim
        umin, vmin = pts2d.min(axis=0)
        umax, vmax = pts2d.max(axis=0)
        nu_raw = max(1, int(np.ceil((umax - umin) / cell_size)))
        nv_raw = max(1, int(np.ceil((vmax - vmin) / cell_size)))
        eff_cell = cell_size
        dim = max(nu_raw, nv_raw)
        if dim > max_dim:
            eff_cell *= dim / max_dim

        mask, pre_flat, post_flat, (nu, nv), counts = grid_filter(
            pts2d, cell_size=eff_cell, min_pts=min_pts,
            erode_iters=erode_iters, dilate_iters=dilate_iters)

        if nu == 0 or nv == 0:
            continue

        max_count = counts.max() if counts.max() > 0 else 1
        t_log = np.log(counts.astype(float) + 1) / np.log(max_count + 1)
        bright = (80 + 175 * t_log).astype(np.uint8)  # (nu*nv,)
        pre  = pre_flat.astype(bool)
        post = post_flat.astype(bool)

        def make_image(mode):
            """mode: 'density' | 'diff' | 'morph'"""
            rgb_flat = np.zeros((nv * nu, 3), dtype=np.uint8)
            occ = counts > 0

            if mode == 'density':
                rgb_flat = density_color(t_log)
            elif mode == 'diff':
                rgb_flat[occ &  pre, 1] = bright[occ &  pre]   # green
                rgb_flat[occ & ~pre, 0] = bright[occ & ~pre]   # red
            elif mode == 'morph':
                core     = occ & pre &  post
                restored = occ & ~pre & post
                eroded   = occ & pre  & ~post
                rgb_flat[core,    1] = bright[core]             # cyan (G+B)
                rgb_flat[core,    2] = (bright[core] * 0.9).astype(np.uint8)
                rgb_flat[restored,0] = bright[restored]         # yellow (R+G)
                rgb_flat[restored,1] = bright[restored]
                rgb_flat[eroded,  0] = bright[eroded]           # red

            # Reshape and flip V axis (image y↓ = UV v↑)
            img_arr = rgb_flat.reshape(nv, nu, 3)[::-1]
            return Image.fromarray(img_arr, 'RGB')

        base = f"{prefix}_plane{pi}"
        if write_density:
            make_image('density').save(base + '_density.png')
            n_written += 1
        if write_diff:
            make_image('diff').save(base + '_diff.png')
            n_written += 1
        if write_morph:
            make_image('morph').save(base + '_morph.png')
            n_written += 1

    print(f"Grid images: {n_written} PNG(s) written to {prefix}_plane*")
    return n_written



    pts_np = np.asarray(pcd.points, dtype=np.float32)
    n_pts  = len(pts_np)
    pt_rgb = np.tile(np.array([120,120,120], dtype=np.uint8), (n_pts, 1))

    for pi, (_, inlier_pts) in enumerate(plane_info):
        col_u = (PLANE_COLORS[pi % len(PLANE_COLORS)] * 255).astype(np.uint8)
        # Match inlier pts back to original indices via nearest neighbour
        # (cheap: exact float match on unmodified clouds)
        for p in inlier_pts:
            diffs = np.abs(pts_np - p).sum(axis=1)
            idx   = int(np.argmin(diffs))
            pt_rgb[idx] = col_u

    hull_verts, hull_faces = [], []
    v_off = n_pts

    for pi, (plane_model, inlier_pts) in enumerate(plane_info):
        a, b, c, d = plane_model
        normal = np.array([a,b,c]); normal /= np.linalg.norm(normal)
        u, v_ax = plane_basis(normal)
        pts2d   = np.column_stack([inlier_pts @ u, inlier_pts @ v_ax])
        hidx    = convex_hull_2d(pts2d)
        if len(hidx) < 3:
            continue
        origin  = -d * normal
        hull_3d = [origin + pts2d[i,0]*u + pts2d[i,1]*v_ax for i in hidx]
        centroid = np.mean(hull_3d, axis=0)
        centroid -= normal * (normal @ centroid + d)
        col_u = (PLANE_COLORS[pi % len(PLANE_COLORS)] * 255).astype(np.uint8)

        center_vi  = v_off + len(hull_verts)
        hull_verts.append((*centroid, *col_u))
        ring_start = v_off + len(hull_verts)
        for p3 in hull_3d:
            hull_verts.append((*p3, *col_u))
        n_ring = len(hull_3d)
        for i in range(n_ring):
            hull_faces.append((center_vi,
                               ring_start + i,
                               ring_start + (i+1) % n_ring,
                               *col_u, alpha))

    total_v = n_pts + len(hull_verts)
    total_f = len(hull_faces)

    with open(out_path, "wb") as f:
        hdr = (
            "ply\nformat binary_little_endian 1.0\n"
            "comment plane mesh saved by visualize_planes.py\n"
            f"element vertex {total_v}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            f"element face {total_f}\n"
            "property list uchar uint vertex_indices\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "property uchar alpha\n"
            "end_header\n"
        )
        f.write(hdr.encode())
        for i in range(n_pts):
            f.write(struct.pack("<fffBBB",
                pts_np[i,0], pts_np[i,1], pts_np[i,2],
                int(pt_rgb[i,0]), int(pt_rgb[i,1]), int(pt_rgb[i,2])))
        for hv in hull_verts:
            f.write(struct.pack("<fffBBB",
                float(hv[0]), float(hv[1]), float(hv[2]),
                int(hv[3]),   int(hv[4]),   int(hv[5])))
        for hf in hull_faces:
            f.write(struct.pack("<BIIIBBBB", 3,
                int(hf[0]), int(hf[1]), int(hf[2]),
                int(hf[3]), int(hf[4]), int(hf[5]), int(hf[6])))

    print(f"Saved → {out_path}  ({total_v} verts, {total_f} faces)")


# ─────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────

def visualize(path, segment, num_planes, threshold, save_mesh,
              cell_size, min_pts_per_cell,
              erode_iters, dilate_iters,
              save_grid_prefix):
    print(f"Loading {path} …")
    has_faces = ply_has_faces(path)
    geometries = []

    if has_faces and not segment:
        print("Detected face data — loading as triangle mesh")
        mesh = o3d.io.read_triangle_mesh(path)
        if not mesh.has_vertex_colors():
            mesh.paint_uniform_color(OUTLIER_COLOR.tolist())
        mesh.compute_vertex_normals()
        n_v, n_f = len(mesh.vertices), len(mesh.triangles)
        print(f"  {n_v:,} vertices  {n_f:,} faces")
        # Show cloud points on top for better depth cues
        pcd = o3d.geometry.PointCloud()
        pcd.points = mesh.vertices
        if mesh.has_vertex_colors():
            pcd.colors = mesh.vertex_colors
        geometries += [mesh, pcd]
    else:
        pcd = o3d.io.read_point_cloud(path)
        if len(pcd.points) == 0:
            sys.exit(f"No points loaded from {path}")
        print(f"Loaded {len(pcd.points):,} points")

        if not segment:
            if not pcd.has_colors():
                pcd.paint_uniform_color(OUTLIER_COLOR.tolist())
            geometries.append(pcd)
        else:
            print(f"\nSegmenting ≤{num_planes} plane(s)  threshold={threshold} m …")
            colored_pcd, plane_info = segment_planes(pcd, num_planes, threshold)
            geometries.append(colored_pcd)
            for pi, (pm, inlier_pts) in enumerate(plane_info):
                col = PLANE_COLORS[pi % len(PLANE_COLORS)]
                m, _ = build_plane_mesh(pm, inlier_pts, col * 0.8,
                                        cell_size=cell_size,
                                        min_pts=min_pts_per_cell,
                                        erode_iters=erode_iters,
                                        dilate_iters=dilate_iters)
                if m is not None:
                    geometries.append(m)
            if save_mesh:
                save_plane_mesh_py(save_mesh, pcd, plane_info)
            if save_grid_prefix:
                try:
                    save_grid_images_py(save_grid_prefix, pcd, plane_info,
                                        cell_size=cell_size,
                                        min_pts=min_pts_per_cell,
                                        erode_iters=erode_iters,
                                        dilate_iters=dilate_iters)
                except ImportError:
                    print("Pillow not found — skipping grid images "
                          "(install with: pip install Pillow)")

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    geometries.append(frame)

    print("\nOpening viewer  [q / Esc = quit]\n"
          "  Left drag — rotate  |  Right drag — pan  |  Scroll — zoom  |  R — reset\n")
    o3d.visualization.draw_geometries(geometries,
        window_name="RANSAC plane viewer", width=1280, height=720)


def main():
    ap = argparse.ArgumentParser(
        description="Visualise point cloud + RANSAC plane mesh results.")
    ap.add_argument("file")
    ap.add_argument("--segment",    action="store_true",
                    help="Run Open3D RANSAC (ignores existing colors)")
    ap.add_argument("--num-planes", type=int, default=3, metavar="N")
    ap.add_argument("--threshold",  type=float, default=0.02, metavar="M",
                    help="Inlier distance threshold in metres (default: 0.02)")
    ap.add_argument("--save-mesh",  metavar="OUT.ply", default=None,
                    help="Write mesh PLY after segmentation")
    ap.add_argument("--cell-size",  type=float, default=0.10, metavar="M",
                    help="Grid filter cell size in metres (default: 0.10)")
    ap.add_argument("--min-pts",    type=int,   default=2,    metavar="N",
                    help="Min points per cell to keep (default: 2)")
    ap.add_argument("--erode",      type=int,   default=1,    metavar="N",
                    help="Erosion iterations (default: 1)")
    ap.add_argument("--dilate",     type=int,   default=1,    metavar="N",
                    help="Dilation iterations (default: 1)")
    ap.add_argument("--no-grid",    action="store_true",
                    help="Disable the grid outlier filter")
    ap.add_argument("--save-grid",  metavar="PREFIX", default=None,
                    help="Write grid diagnostic PNGs, e.g. --save-grid debug/plane")
    args = ap.parse_args()
    cell  = 1e9 if args.no_grid else args.cell_size
    visualize(args.file, args.segment, args.num_planes,
              args.threshold, args.save_mesh,
              cell, args.min_pts,
              args.erode, args.dilate,
              args.save_grid)

if __name__ == "__main__":
    main()
