# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 08:34:08 2025

@author: anton
"""
import numpy as np
import matplotlib.pyplot as plt

inliers = np.genfromtxt(r"C:\GIT\py_RANSAC\cpp\inliers.txt", delimiter=" ")
outliers = np.genfromtxt(r"C:\GIT\py_RANSAC\cpp\outliers.txt", delimiter=" ")

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(inliers[:,0],inliers[:,1],inliers[:,2])
ax.scatter(outliers[:,0],outliers[:,1],outliers[:,2])
plt.show()