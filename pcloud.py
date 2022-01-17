import open3d as o3d
import numpy as np

TABLE_FILTER_THRESHOLD = 3
BBOX_FILTER_THRESHOLD = 10
BG_THRESHOLD =  0.4 #40 cm 

class Pcloud():
    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.tvector = [self.x,self.y,self.z]
        
        self.pcd = o3d.geometry.PointCloud()
    
    #Updates the coordinates of the translation vector
    def updateTvector(self):
        self.tvector = [self.x,self.y,self.z]

    # Filters the background from the pointcloud based on a threshold
    def filterBackground(self):
        points = np.asarray(self.pcd.points)
        self.pcd = self.pcd.select_by_index(np.where(points[:, 2] < BG_THRESHOLD)[0])
    
    # Process pipeline for the D435 pointclouds
    def processPcloud(self, model, table ):
        if not (self.pcd.is_empty() or model.is_empty() or table.is_empty()):
            self.filterBackground()
            self.pcd.scale(1000,np.asarray(self.pcd.points)[0])
            self.pcd.rotate([[-1,0,0],[0,-1,0],[0,0,-1]]) # 180° counterclockwise about the origin, Mirroring about the origin
            self.pcd.translate(self.tvector,False)
            self.filterPcloud(table, TABLE_FILTER_THRESHOLD)
            self.filterModelbbox( model)
        else:
            print("Something here is empty")
    
    #Downsampling of the pointcloud
    def downsamplePcloud(self, number_of_points): 
        if not self.pcd.is_empty():
            num_pcd_points= len(np.asarray(self.pcd.points))
            downsample_factor = num_pcd_points /  number_of_points
            if downsample_factor > 1:
                self.pcd = self.pcd.uniform_down_sample(int(downsample_factor)) # Parameter: "Every k points"
        else:
            print("pointcloud empty")

    # Removes the points that are within the space of another pointcloud based on a threshold.
    def filterPcloud(self, pointcloud, threshold): # Removes 
        if not self.pcd.is_empty():
            dist = np.asarray(self.pcd.compute_point_cloud_distance(pointcloud))
            index = np.where(dist > threshold)[0]
            self.pcd = self.pcd.select_by_index(index)
        else:
            print("pointcloud empty")

    #Removes the points outside the a models bounding box + a threshold
    def filterModelbbox(self, model):
        if not self.pcd.is_empty():
            model_bbox =  model.get_axis_aligned_bounding_box()
            box_points = np.asarray(model_bbox.get_box_points())
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,0] <= box_points[1][0] + BBOX_FILTER_THRESHOLD)[0]) 
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,0] >= box_points[0][0] -  BBOX_FILTER_THRESHOLD)[0])
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,1] <= box_points[2][1] +  BBOX_FILTER_THRESHOLD)[0]) 
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,1] >= box_points[1][1] -  BBOX_FILTER_THRESHOLD)[0])
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,2] <= box_points[3][2] + BBOX_FILTER_THRESHOLD)[0]) 
            scene_points = np.asarray(self.pcd.points)
            self.pcd = self.pcd.select_by_index(np.where(scene_points[:,2] >= box_points[2][2] - BBOX_FILTER_THRESHOLD)[0])
        else:
            print("pointcloud empty")

    def visualizePcloud(self):
        if not self.pcd.is_empty():
            self.pcd.paint_uniform_color([1, 0.706, 0])
            o3d.visualization.draw_geometries([self.pcd])  
            # Press 'n' to see normals, press '+' and '`' to change voxel size.
            # 'Rigth mouse' rotates, 'mouse wheel' drags
        else:
            print("pointcloud empty")

        