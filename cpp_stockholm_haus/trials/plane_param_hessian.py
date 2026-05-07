# -*- coding: utf-8 -*-
"""
Created on Wed May  6 07:48:58 2026

@author: lidar
"""

import numpy as np

def parametric_to_hessian(support_vector, dir_vector_1, dir_vector_2):
    """
    Converts a plane from parametric form to Hessian Normal Form.
    
    Parametric: x = a + r*u + s*v
    Hessian: n0 · x = d
    """
    # Convert inputs to numpy arrays for vector math
    a = np.array(support_vector)
    u = np.array(dir_vector_1)
    v = np.array(dir_vector_2)

    # 1. Calculate the normal vector using the cross product
    n = np.cross(u, v)
    
    # Check if vectors are linearly dependent (plane doesn't exist)
    mag = np.linalg.norm(n)
    if mag == 0:
        raise ValueError("Direction vectors are parallel; they do not define a plane.")

    # 2. Create the unit normal vector
    n0 = n / mag

    # 3. Calculate distance from origin (dot product of support and unit normal)
    d = np.dot(n0, a)

    # 4. Standardize (ensure d is positive)
    if d < 0:
        n0 = -n0
        d = -d

    return n0, d

# --- Example Usage ---
# Plane: x = (1, 2, 3) + r*(1, 0, 1) + s*(0, 1, 1)
support = [1.7, 2, 3]
u_dir = [1, 0, 1]
v_dir = [0, 1, 1]

try:
    unit_normal, distance = parametric_to_hessian(support, u_dir, v_dir)
    
    print("--- Hessian Normal Form ---")
    print(f"Unit Normal Vector (n0): {unit_normal}")
    print(f"Distance from Origin (d): {distance:.4f}")
    print(f"\nEquation: ({unit_normal[0]:.3f})x + ({unit_normal[1]:.3f})y + ({unit_normal[2]:.3f})z = {distance:.3f}")

except ValueError as e:
    print(f"Error: {e}")