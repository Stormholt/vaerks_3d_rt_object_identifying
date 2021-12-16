from matplotlib.pyplot import table
import open3d as o3d
import numpy as np

table_filter_threshold = 3
bbox_filter_threshold = 6
bg_threshold =  0.4 #40 cm 
voxel_size = 0.01 # 1cm # smaller = longer cpu time and worse registration.
class Pcloud():
    def __init__(self ):

        self.x = 0
        self.y = 0
        self.z = 0

        self.tvector = [self.x,self.y,self.z]
        
        self.pcd = o3d.geometry.PointCloud()
    
    def updateTvector(self):
        self.tvector = [self.x,self.y,self.z]

    def filter_by_background(self):
        points = np.asarray(self.pcd.points)
        self.pcd = self.pcd.select_by_index(np.where(points[:, 2] < bg_threshold)[0])
    
    def process_point_cloud(self, model, table ):
        self.filter_by_background()
    
        radius_normal = voxel_size * 2
        self.pcd.estimate_normals( o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.])) 
    
        self.pcd.scale(1000,np.asarray(self.pcd.points)[0])
        self.pcd.rotate([[-1,0,0],
                        [0,-1,0],
                        [0,0,-1]]) # 180° counterclockwise about the origin, Mirroring about the origin
        #self.pcd.translate([(14.7*2),(-14.2*4),-29.5])notmiwire_scene_0
        #self.pcd.translate([147*1.18,142*0.7,1.5],False) notmiwire_scene_0
        #self.pcd.translate([157*1.22,142*0.7,0],False)#notmiwire_scene_0
        self.pcd.translate(self.tvector,False)
        #self.pcd = self.filter_by_pointcloud(self.pcd, table)
        #self.pcd = self.filter_by_model_bbox(self.pcd, model)
    
    def downsample_pointcloud(self, number_of_points): # NOT TESTED
        num_pcd_points= len(np.asarray(self.pcd.points))
        downsamplefactor = num_pcd_points /  number_of_points
        if downsamplefactor > 2:
            self.pcd = self.pcd.uniform_down_sample(downsamplefactor) # Parameter: "Every k points"
        return

    def filter_by_pointcloud(self, scene, pointcloud): # Removes 
        
        dist = np.asarray(scene.compute_point_cloud_distance(pointcloud))
        index = np.where(dist > table_filter_threshold)[0]
        filtered_pcd = scene.select_by_index(index)
        return filtered_pcd

    def filter_by_model_bbox(self, scene, model):
        model_bbox =  model.get_axis_aligned_bounding_box()
        box_points = np.asarray(model_bbox.get_box_points())
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,0] <= box_points[1][0] + bbox_filter_threshold)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,0] >= box_points[0][0] -  bbox_filter_threshold)[0])
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,1] <= box_points[2][1] +  bbox_filter_threshold)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,1] >= box_points[1][1] -  bbox_filter_threshold)[0])
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,2] <= box_points[3][2] + bbox_filter_threshold)[0]) 
        scene_points = np.asarray(scene.points)
        scene = scene.select_by_index(np.where(scene_points[:,2] >= box_points[2][2] - bbox_filter_threshold)[0])

        return scene 

   

    def visualizeMothership(self,source):
        if not source.is_empty():
            source.paint_uniform_color([1, 0.706, 0])
            o3d.visualization.draw_geometries([source])  
            # Press 'n' to see normals, press '+' and '`' to change voxel size.
            # 'Rigth mouse' rotates, 'mouse wheel' drags

        