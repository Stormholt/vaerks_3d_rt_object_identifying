from matplotlib.pyplot import table
import open3d as o3d
import numpy as np

from d435 import VOXEL_SIZE

TABLE_FILTER_THRESHOLD = 3
BBOX_FILTER_THRESHOLD = 6
BG_THRESHOLD =  0.4 #40 cm 
VOXEL_SIZE = 0.01 # 1cm # smaller = longer cpu time and worse registration.

class Pcloud():
    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.tvector = [self.x,self.y,self.z]
        
        self.pcd = o3d.geometry.PointCloud()
    
    def updateTvector(self):
        self.tvector = [self.x,self.y,self.z]

    def filterBackground(self):
        points = np.asarray(self.pcd.points)
        self.pcd = self.pcd.select_by_index(np.where(points[:, 2] < BG_THRESHOLD)[0])
    
    def processPcloud(self, model, table ):
        self.filterBackground()
    
        #radius_normal = voxel_size * 2
        #self.pcd.estimate_normals( o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        #self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.])) 
    
        self.pcd.scale(1000,np.asarray(self.pcd.points)[0])
        self.pcd.rotate([[-1,0,0],[0,-1,0],[0,0,-1]]) # 180° counterclockwise about the origin, Mirroring about the origin

        self.pcd.translate(self.tvector,False)
        self.pcd = self.filterPcloud(self.pcd, table)
        self.pcd = self.filterModelbbox(self.pcd, model)
        return
    
    def downsamplePcloud(self, number_of_points): # NOT TESTED
        num_pcd_points= len(np.asarray(self.pcd.points))
        downsample_factor = num_pcd_points /  number_of_points
        if downsample_factor > 1:
            self.pcd = self.pcd.uniform_down_sample(downsample_factor) # Parameter: "Every k points"
        return

    def filterPcloud(self, pointcloud): # Removes 
        
        dist = np.asarray(self.pcd.compute_point_cloud_distance(pointcloud))
        index = np.where(dist > TABLE_FILTER_THRESHOLD)[0]
        filtered_pcd = self.pcd.select_by_index(index)
        self.pcd = filtered_pcd
        return filtered_pcd

    def filterModelbbox(self, scene, model):
        model_bbox =  model.get_axis_aligned_bounding_box()
        box_points = np.asarray(model_bbox.get_box_points())
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,0] <= box_points[1][0] + BBOX_FILTER_THRESHOLD)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,0] >= box_points[0][0] -  BBOX_FILTER_THRESHOLD)[0])
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,1] <= box_points[2][1] +  BBOX_FILTER_THRESHOLD)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,1] >= box_points[1][1] -  BBOX_FILTER_THRESHOLD)[0])
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,2] <= box_points[3][2] + BBOX_FILTER_THRESHOLD)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,2] >= box_points[2][2] - BBOX_FILTER_THRESHOLD)[0])
        self.pcd = scene
        return scene 

   

    def visualizePcloud(self):
        if not self.pcd.is_empty():
            self.pcd.paint_uniform_color([1, 0.706, 0])
            o3d.visualization.draw_geometries([self.pcd])  
            # Press 'n' to see normals, press '+' and '`' to change voxel size.
            # 'Rigth mouse' rotates, 'mouse wheel' drags

        