import numpy as np
import matplotlib.pyplot as plt

def draw_plane(normal, d, size=10, resolution=20):
    """
    Plots a plane given by normal vector 'n' and distance 'd' (n·x = d).
    """

    A, B, C = normal

    # Create a grid of points
    x = np.linspace(-size, size, resolution)
    y = np.linspace(-size, size, resolution)
    X, Y = np.meshgrid(x, y)

    # Calculate Z. If C is 0, the plane is vertical (parallel to Z-axis)
    if C != 0:
        Z = (d - A*X - B*Y) / C
        ax.plot_surface(X, Y, Z, alpha=0.5, cmap='viridis', edgecolor='none')
    else:
        # For vertical planes, we plot X or Y as a function of the others
        if A != 0:
            # Ax + By = d  =>  X = (d - By) / A
            Z = np.linspace(-size, size, resolution)
            Y_grid, Z_grid = np.meshgrid(y, Z)
            X_grid = (d - B*Y_grid) / A
            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.5, color='orange')
        else:
            # By = d  =>  Y = d / B
            Z = np.linspace(-size, size, resolution)
            X_grid, Z_grid = np.meshgrid(x, Z)
            Y_grid = (d - 0*X_grid) / B # Constant Y
            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.5, color='orange')
        ax.set_title(f'Plane: {A}x + {B}y + {C}z = {d}')


fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# --- Example ---
# A plane with normal (1, 1, 2) at distance 5 from origin
normal_vec = [0.639,0.228,0.734]
distance = 0.574
draw_plane(normal_vec, distance)

normal_vec = [0.739,-0.133,-0.660]
distance = -0.424
draw_plane(normal_vec, distance)

normal_vec = [-0.030,0.983,-0.183]
distance = 0.103
draw_plane(normal_vec, distance)

# Labeling
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


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
