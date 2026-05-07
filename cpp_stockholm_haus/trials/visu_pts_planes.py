import numpy as np
import open3d as o3d

def get_plane_basis(normal):
    n = normal / np.linalg.norm(normal)

    arbitrary = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(n, arbitrary)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)

    return u, v

def point_from_d(normal, d):
    n = np.array(normal, dtype=float)
    return (d / np.dot(n, n)) * n

def create_plane_mesh(normal, d, size=2.0, resolution=10, color=[0.5, 0.8, 0.8]):
    n = np.array(normal, dtype=float)
    p0 = point_from_d(n, d)

    u, v = get_plane_basis(n)

    # grid
    s = np.linspace(-size, size, resolution)
    t = np.linspace(-size, size, resolution)

    vertices = []
    for si in s:
        for ti in t:
            p = p0 + si * u + ti * v
            vertices.append(p)

    vertices = np.array(vertices)

    # triangles
    triangles = []
    for i in range(resolution - 1):
        for j in range(resolution - 1):
            idx = i * resolution + j
            triangles.append([idx, idx + 1, idx + resolution])
            triangles.append([idx + 1, idx + resolution + 1, idx + resolution])

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()

    return mesh


def create_normal_arrow(normal, d, scale=0.5):
    n = np.array(normal, dtype=float)
    n = n / np.linalg.norm(n)
    p0 = point_from_d(n, d)

    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.01,
        cone_radius=0.02,
        cylinder_height=scale,
        cone_height=0.1
    )

    # align arrow with normal
    z = np.array([0, 0, 1])
    v = np.cross(z, n)
    c = np.dot(z, n)

    if np.linalg.norm(v) > 1e-6:
        vx = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
        arrow.rotate(R, center=(0, 0, 0))

    arrow.translate(p0)
    arrow.paint_uniform_color([1, 0, 0])

    return arrow



# Load point cloud
pcd = o3d.io.read_point_cloud("out.ply")

# Optional: downsample (recommended)
pcd = pcd.voxel_down_sample(voxel_size=0.02)

planes = [
    ([0.639, 0.228, 0.734], -0.574, [0, 1, 1]),
    ([0.739, -0.133, -0.660], 0.424, [0, 1, 0]),
    ([0.030, -0.983, 0.183], 0.103, [1, 1, 0]),
]

geometries = [pcd]

for normal, d, color in planes:
    plane_mesh = create_plane_mesh(normal, d, size=2, resolution=10, color=color)
    geometries.append(plane_mesh)
    geometries.append(create_normal_arrow(normal, d))

# Visualize everything
o3d.visualization.draw_geometries(geometries)