# -*- coding: utf-8 -*-
"""
Created on Sun Jun  1 17:18:53 2025

@author: lidar
"""

"""
3D RANSAC implementation in python
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import math
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(r"d:\\out.pcd")

    np_points = np.asarray(pcd.points)

    point_cloud = pd.DataFrame({'X': np_points[:,0], 'Y': np_points[:,1], 'Z': np_points[:,2]})
    
    max_iterations=5
    distance_ratio_threshold=1.0

#    inliers_result = set()
    while max_iterations:
        max_iterations -= 1
        # Add 3 random indexes
        random.seed()
        inliers = []
        while len(inliers) < 3:
            random_index = random.randint(0, len(point_cloud.X)-1)
            inliers.append(random_index)
        # print(inliers)
        try:
            # In case of *.xyz data
            x1, y1, z1, _, _, _ = point_cloud.loc[inliers[0]]
            x2, y2, z2, _, _, _ = point_cloud.loc[inliers[1]]
            x3, y3, z3, _, _, _ = point_cloud.loc[inliers[2]]
        except:
            # In case of *.pcd data
            x1, y1, z1 = point_cloud.loc[inliers[0]]
            x2, y2, z2 = point_cloud.loc[inliers[1]]
            x3, y3, z3 = point_cloud.loc[inliers[2]]
        # Plane Equation --> ax + by + cz + d = 0
        # Value of Constants for inlier plane
        a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
        b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
        c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
        d = -(a*x1 + b*y1 + c*z1)
        plane_lenght = max(0.1, math.sqrt(a*a + b*b + c*c))

        for point in point_cloud.iterrows():
            index = point[0]
#                print(index)
            # Skip iteration if point matches the randomly generated inlier point
            if index in inliers:
                continue
            try:
                # In case of *.xyz data
                x, y, z, _, _, _ = point[1]
            except:
                # In case of *.pcd data
                x, y, z = point[1]

            # Calculate the distance of the point to the inlier plane
            distance = math.fabs(a*x + b*y + c*z + d)/plane_lenght
            # Add the point as inlier, if within the threshold distancec ratio
            if distance <= distance_ratio_threshold:
                inliers.append(index)
        # Update the set for retaining the maximum number of inlier points
#        if len(inliers) > len(inliers_result):
#            inliers_result.clear()
#            inliers_result = inliers


    # Segregate inliers and outliers from the point cloud
#     inlier = pd.DataFrame(columns=("X", "Y", "Z"))
#     outlier = pd.DataFrame(columns=("X", "Y", "Z"))
# #######################################
# # pandas 2.0 no more append, but concat
# #######################################
#     for point in point_cloud.iterrows():
#         print(point[0])
#         if point[0] in inliers_result:
#             inlier = pd.concat([inlier,pd.DataFrame([point[1]["X"],point[1]["Y"],point[1]["Z"]])]   , ignore_index=True)
#             continue
#         outlier = pd.concat([outlier,pd.DataFrame([point[1]["X"],point[1]["Y"],point[1]["Z"]])], ignore_index=True)

## %matplotlib qt

    in_ = np_points[inliers]
    outtmp_ = list(range(1,len(np_points)))
    for i in range(0,len(inliers)-1):
        outtmp_[inliers[i]] = 0
    outliers = [i for i in outtmp_ if i != 0]
    out_ = np_points[outliers]


    fig = plt.figure()
#    ax = fig.gca(projection='3d')
    ax = Axes3D(fig)
        
    ax.scatter(in_[:,0] , in_[:,1],  in_[:,2],  c="green")
    ax.scatter(out_[:,0] , out_[:,1],  out_[:,2], c="red")
    plt.show()
        