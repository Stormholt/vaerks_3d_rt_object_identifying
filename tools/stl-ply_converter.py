import open3d as o3d

pcdpath = "./pointclouds/sbt-marvo-test.pcd"
stlpath = "./referenceModels/component_1.stl"

pcd = o3d.io.read_point_cloud(pcdpath)
pcd_down = pcd.voxel_down_sample(0.01)
cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=10, # neighbours in consideration, Higher = aggressive
                                                    std_ratio=0.01) # Threshold based on deviation of avg. dist. lower = aggressive
o3d.io.write_point_cloud("./converted/sbt-marvo-test.ply", pcd_down)

#stl = o3d.io.read_triangle_mesh(stlpath)

#o3d.io.write_triangle_mesh("./converted/sbt-marvo-test.ply", pcd)
