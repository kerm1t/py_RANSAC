# 3D Ransac (python, open3d, pandas, numpy)

The result is more random the smaller the ```max_iterations```

![alt text](https://github.com/kerm1t/py_RANSAC/blob/master/img/pyRANSAC_0.5m_thresh.png)

# Result from C++ conversion (Claude.ai)
![image](https://github.com/user-attachments/assets/aacff027-3731-41f2-99bd-e2129a639695)

How to Build + Run C++ (Windows, VS2022, Anaconda)
```
open developer Power Shell for VS2022
cl.exe main.cpp // will create main.obj and main.exe
main.exe // will read pcd from ..\data and write files inliers.txt, outliers.txt
c:\winapp\anaconda3\python.exe vis.py // show above plot
```

### References:
Udacity - Sensor Fusion Nanodegree - Lidar - segmentation  
https://towardsdatascience.com/discover-3d-point-cloud-processing-with-python-6112d9ee38e7  
http://www.open3d.org/docs/release/introduction.html#open3d-a-modern-library-for-3d-data-processing
