# -*- coding: utf-8 -*-
"""
Created on Wed May  6 21:30:24 2026

@author: lidar
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_coordinate_plane(A, B, C, D, size=10):

    # Create a grid for X and Y
    x = np.linspace(-size, size, 10)
    y = np.linspace(-size, size, 10)
    X, Y = np.meshgrid(x, y)

    # Calculate Z based on the coordinate form: Ax + By + Cz + D = 0
    # Rearranged: z = (-D - Ax - By) / C
    if C != 0:
        Z = (-D - A*X - B*Y) / C
        ax.plot_surface(X, Y, Z, alpha=0.6, cmap='coolwarm', edgecolor='k')
    else:
        # Handle vertical planes (where C = 0)
        if A != 0:
            # Ax + By + D = 0 -> x = (-D - By) / A
            Z_grid, Y_grid = np.meshgrid(np.linspace(-size, size, 10), y)
            X_grid = (-D - B*Y_grid) / A
#            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.6, color='tab:blue')
            plotly.graph_objects.Surface(X_grid, Y_grid, Z_grid, alpha=0.6, color='tab:blue')
        elif B != 0:
            # By + D = 0 -> y = -D / B
            Z_grid, X_grid = np.meshgrid(np.linspace(-size, size, 10), x)
            Y_grid = np.full_like(X_grid, -D / B)
#            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.6, color='tab:green')
            plotly.graph_objects.Surface(X_grid, Y_grid, Z_grid, alpha=0.6, color='tab:green')

    # Labeling
    ax.set_title(f'Plane: {A}x + {B}y + {C}z + {D} = 0')

    # Set axis limits to keep the plot square
    ax.set_xlim(-size, size)
    ax.set_ylim(-size, size)
    ax.set_zlim(-size, size)

%matplotlib qt

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# --- Example: 2x - 3y + 4z - 10 = 0 ---
#plot_coordinate_plane(A=2, B=-3, C=4, D=-10)
plot_coordinate_plane(A=0.639,B=0.228,C=0.734,D=-0.574)
plot_coordinate_plane(A=0.739,B=-0.133,C=-0.66,D=-0.424)
plot_coordinate_plane(A=0.030,B=0.983,C=-0.182,D=-0.104)

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')


import open3d as o3d
import numpy as np
# Load point cloud
pcd = o3d.io.read_point_cloud("out.ply")
# 🔥 Downsample (tune voxel_size!)
pcd = pcd.voxel_down_sample(voxel_size=0.05)
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors) if pcd.has_colors() else None
if colors is not None:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=colors, s=1)
else:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               s=1)
    
plt.show()