import open3d as o3d

pcdpath = r"C:\Users\Stormholt\Documents\Thesis\pointclouds\comp-test-model.ply"
stlpath =  r"C:\Users\Stormholt\Documents\Thesis\models\comp-test-model.stl"

#pcd = o3d.io.read_point_cloud(pcdpath)
stl = o3d.io.read_triangle_mesh(stlpath)
stl.compute_vertex_normals()
pcd = stl.sample_points_uniformly(number_of_points=10000)


o3d.io.write_point_cloud(pcdpath, pcd)

