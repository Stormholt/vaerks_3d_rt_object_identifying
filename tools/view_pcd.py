
import pyrealsense2 as rs 
import open3d as o3d #pip install open3d
import copy


source = o3d.io.read_point_cloud("pointclouds/pointCloud.pcd")
source_tmp = copy.deepcopy(source)
source_tmp.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([source_tmp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])