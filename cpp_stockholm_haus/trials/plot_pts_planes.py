# -*- coding: utf-8 -*-
"""
Created on Tue May  5 19:53:33 2026

@author: lidar
this can draw points and plane, but is too slow!
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_plane_basis(normal):
    """Given a normal vector, find two orthogonal vectors u, v in the plane."""
    n = normal / np.linalg.norm(normal)  # normalize

    # Pick an arbitrary vector NOT parallel to n
    arbitrary = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])

    u = np.cross(n, arbitrary)
    u = u / np.linalg.norm(u)           # first tangent vector

    v = np.cross(n, u)                  # second tangent vector (already unit length)
    return u, v

def draw_plane(ax, point, normal, size=2, color='cyan', alpha=0.5):
    u, v = get_plane_basis(normal)

    s = np.linspace(-size, size, 20)
    t = np.linspace(-size, size, 20)
    S, T = np.meshgrid(s, t)

    # P(s,t) = point + s*u + t*v
    X = point[0] + S*u[0] + T*v[0]
    Y = point[1] + S*u[1] + T*v[1]
    Z = point[2] + S*u[2] + T*v[2]

    ax.plot_surface(X, Y, Z, alpha=alpha, color=color)

# --- Main ---
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# normal = np.array([1, 1, 1])   # plane normal
# point  = np.array([0, 0, 0])   # point on plane (P₀)

# draw_plane(ax, point, normal, size=2, color='cyan')

# # Draw the normal arrow
# ax.quiver(*point, *normal, length=1.5, color='red', label='Normal')

# ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
# ax.legend()
# plt.show()

def point_from_d(normal, d):
    """
    From ax + by + cz = d, find a point on the plane.
    Strategy: project origin onto the plane → P₀ = d/|n|² * n
    """
    n = np.array(normal, dtype=float)
    return (d / np.dot(n, n)) * n

# Example: plane  2x + 1y + 3z = 6
# normal = np.array([2.0, 1.0, 3.0])
# d      = 6.0

# point = point_from_d(normal, d)
# print(point)  # → [0.857, 0.428, 1.285]

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# draw_plane(ax, point, normal, size=2, color='lightgreen')
# ax.quiver(*point, *normal/np.linalg.norm(normal), length=1, color='red')
# plt.show()

%matplotlib qt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

planes = [
# c++
#    (np.array([0.639,0.228,0.734]), -0.574,   'cyan',   'plane0'),
#    (np.array([0.739,-0.133,-0.660]), 0.424,   'salmon', 'plane1'),
#    (np.array([0.030,-0.983,0.183]), 0.103,   'yellow', 'plane2'),
# python
    (np.array([0.639,0.228,0.734]), 0.574,   'cyan',   'plane0'),
    (np.array([0.739,-0.133,-0.660]), -0.424,   'salmon', 'plane1'),
    (np.array([-0.030,0.983,-0.183]), 0.103,   'yellow', 'plane2'),
#    (np.array([-0.042,-0.000,0.999]), -1.148,   'cyan',   'plane0'),
#    (np.array([1.000,-0.000,-0.0000]), 1.195,   'salmon', 'plane1'),
#    (np.array([0.032,-0.997,-0.069]), -0.090,   'yellow', 'plane2'),
]

for normal, d, color, label in planes:
    p0 = point_from_d(normal, d)
    draw_plane(ax, p0, normal, size=2, color=color, alpha=0.4)
    ax.quiver(*p0, *(normal/np.linalg.norm(normal)),
              length=0.8, color=color, label=label)

ax.legend()
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
#plt.show()

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Load point cloud
pcd = o3d.io.read_point_cloud("out.ply")

# Convert to numpy array
#points = np.asarray(pcd.points)
# Optional: get colors if present
#colors = np.asarray(pcd.colors) if pcd.has_colors() else None


# 🔥 Downsample (tune voxel_size!)
pcd = pcd.voxel_down_sample(voxel_size=0.05)

points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors) if pcd.has_colors() else None

# Plot
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

if colors is not None:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=colors, s=1)
else:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               s=1)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.title("PLY Point Cloud")
plt.show()

o3d.visualization.draw_geometries([pcd])