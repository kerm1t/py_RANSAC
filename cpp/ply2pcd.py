import open3d as o3d

pcd = o3d.io.read_point_cloud(r"F:\VGGT_out\pcloud_ply\stockholm.ply")
o3d.io.write_point_cloud(r'd:\stockholm.pcd',pcd,write_ascii=True)