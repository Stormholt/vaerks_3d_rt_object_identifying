
import pyrealsense2 as rs   #pip install pyrealsense2   API: https://intelrealsense.github.io/librealsense/python_docs/index.html
import open3d as o3d        #pip install open3d         API: http://www.open3d.org/docs/release/introduction.html                         
import numpy as np          #pip install numpy
import copy                 #pip install opencv-python  API :https://docs.opencv.org/master/
from pcloud import * 
import d435
from altiZ import *
import enum 
import pandas as pd
from matplotlib import pyplot as plt

VOXEL_SIZE = 0.01 # 1cm # smaller = longer cpu time and worse registration.
DEPTH_THRESHOLD = 0.4 #40 cm 
MAX_PCLOUD_SIZE = 10000

class App():
        
    def __init__(self, rootdir,camera_enable):
        self.root = rootdir
        self.pcd_path = self.root +"/pointclouds/"
        self.model_path = self.root + "/3dmodels/"
        self.img_path = self.root + "/images/"
        self.camera_enable = camera_enable
        self.camera = d435.D435(self.camera_enable)
        self.plys_generated = 0 # Counter for .ply files generated
        self.pcds_generated = 0 # Counter for .pcd files generated
        self.mesh_generated = 0 # Counter for 3d models files generated
        self.imgs_generated = 0 # Counter for .png files generated
        self.model = o3d.geometry.PointCloud()
        self.table = o3d.geometry.PointCloud()
        self.scene = Pcloud()

    class Filetype(enum.Enum):
        PCD = 0
        PLY = 1
        STL = 2 
        IMG = 3

    def generatePointcloud(self, filetype, name):
        tmp = Pcloud()
        tmp.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.camera.depth_frame_open3d,self.camera.intrinsic) # Creating pointcloud from depth 
        tmp.x = self.camera.x
        tmp.y =  self.camera.y
        tmp.z = self.camera.z 
        tmp.updateTvector()
        tmp.processPcloud( self.model, self.table)
        self.scene.pcd += tmp.pcd
        if (filetype == self.Filetype.PCD):
            filename = self.pcd_path + name +"_"+ str(self.pcds_generated) + ".pcd"
            self.pcds_generated += 1
        elif (filetype == self.Filetype.PLY):
            filename = self.pcd_path + name +"_"+ str(self.plys_generated) + ".ply"
            self.plys_generated+=1
        else:
            print("Unsupported filetype for saving pointclouds")
            return

        o3d.io.write_point_cloud(filename,tmp.pcd)
        

    def captureScenePointcloud(self):
        tmp = Pcloud()
        tmp.x = self.camera.x
        tmp.y =  self.camera.y
        tmp.z = self.camera.z 
        tmp.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.camera.depth_frame_open3d,self.camera.intrinsic) # Creating pointcloud from depth 
        tmp.updateTvector()
        tmp.processPcloud( self.model, self.table)
        self.scene.pcd += tmp.pcd

    def saveScenePointcloud(self, filetype, name):
        if (filetype == self.Filetype.PCD):
            filename = self.pcd_path + name +"_"+ str(self.pcds_generated) + ".pcd"
        elif (filetype == self.Filetype.PLY):
            filename = self.pcd_path + name +"_"+ str(self.plys_generated) + ".ply"
        else:
            print("Unsupported filetype for saving pointclouds")
            return
        o3d.io.write_point_cloud(filename,self.scene.pcd)

    def gen3Dmodel(self,kpoints,stdRatio, depth, iterations,filetype):
        if not self.scene.pcd.is_empty():
            stl_pcd = self.scene.pcd
            #stl_pcd = stl_pcd.uniform_down_sample(every_k_points=kpoints)
            #stl_pcd, ind = stl_pcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=stdRatio)
            bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.13,-0.13,0),(0.13,0.13,0.01))
            bottom = stl_pcd.crop(bbox1)
            try:
                hull, _ = bottom.compute_convex_hull()  
                bottom = hull.sample_points_uniformly(number_of_points=10000)
                bottom.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,max_nn=30))
                bottom.orient_normals_towards_camera_location(camera_location=np.array([0., 0., -10.]))
                bottom.paint_uniform_color([0, 0, 0])
                _, pt_map = bottom.hidden_point_removal([0,0,-1], 1)
                bottom = self.bottom.select_by_index(pt_map)
                stl_pcd = stl_pcd + self.bottom
            except:
                print("No bottom could be made") 
                pass
            finally:
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(stl_pcd, depth=depth)
                mesh = mesh.filter_smooth_simple(number_of_iterations=iterations)
                mesh.scale(1000, center=(0,0,0))
                mesh.compute_vertex_normals()
            if filetype == self.Filetype.STL:
                filename = self.model_path  +"model" + str(self.mesh_generated) + ".stl"
            elif filetype == self.Filetype.PLY:
                filename = self.model_path  +"model" + str(self.mesh_generated) + ".ply"
            else:
                print("invalid file format")
                return
            o3d.io.write_triangle_mesh(filename, mesh)
            

    def loadPointcloud2Scene(self, filename, tvector):
        tmp = Pcloud() 
        tmp.pcd = o3d.io.read_point_cloud(filename)
        tmp.tvector = tvector
        tmp.process_pcloud(self.model,self.table)
        self.scene.pcd =+ tmp.pcd
        

    def save3Dmodel (self, name, pcloud, filetype):
        if not pcloud.pcd.is_empty():
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson( pcloud.pcd, depth=9)
        else:
            print("No pointcloud data to generate 3d model from")
            return
        if (filetype == self.Filetype.STL):
            filename = self.model_path + name + ".stl"
        elif (filetype == self.Filetype.PLY):
            filename = self.model_path + name + ".ply"
        else:
            print("Unsupported filetype for saving 3dmodels")
            return
        o3d.io.write_triangle_mesh(filename, mesh)

    def compareScene2Model(self):
        if not (self.scene.pcd.is_empty() or self.model.is_empty()):
            if len(self.scene.pcd.points) > MAX_PCLOUD_SIZE:
                self.scene.downsamplePcloud(MAX_PCLOUD_SIZE)
            distance = self.scene.pcd.compute_point_cloud_distance(self.model)
            haussdorf = max(distance)
            print("Haussdorf distance = " + str(haussdorf) + "\n")
           
            df = pd.DataFrame({"distances": distance}) # transform to a dataframe
            # Some graphs
            ax1 = df.boxplot(return_type="axes") # BOXPLOT
            ax2 = df.plot(kind="hist", alpha=0.5, bins = 1000) # HISTOGRAM
            ax3 = df.plot(kind="line") # SERIE
            plt.show()
        else:
            print("Empty pointcloud given to compare")

    def compareScene2ModelBbox(self):
        model_bbox =  self.model.get_axis_aligned_bounding_box()
        box_points = np.asarray(model_bbox.get_box_points())
        scene_points = np.asarray(self.scene.pcd.points)
        boundary_1 = self.scene.pcd.select_by_index(np.where(scene_points[:,0] >= box_points[1][0])[0]) 
        print(str(boundary_1)+ "outside "+ str( box_points[1]))
        boundary_2 = self.scene.pcd.select_by_index(np.where(scene_points[:,0] <= box_points[0][0])[0])
        print(str(boundary_2)+ "outside "+ str( box_points[0]))
        boundary_3 = self.scene.pcd.select_by_index(np.where(scene_points[:,1] >= box_points[2][1])[0]) 
        print(str(boundary_3)+ "outside "+ str( box_points[2]))
        boundary_4 = self.scene.pcd.select_by_index(np.where(scene_points[:,1] <= box_points[1][1])[0])
        print(str(boundary_4)+ "outside "+ str( box_points[1]))
        boundary_5 = self.scene.pcd.select_by_index(np.where(scene_points[:,2] >= box_points[3][2])[0]) 
        print(str(boundary_5)+ "outside "+ str( box_points[3]))
        #boundary_6 = self.scene.pcd.select_by_index(np.where(scene_points[:,2] >= box_points[2][2])[0])
        #print(str(boundary_6)+ " outside "+ str( box_points[2]))
        tmp_scene = self.scene
        if boundary_1.has_points():
            tmp_scene.filterPcloud(boundary_1)
        if boundary_2.has_points() != 0 :
            tmp_scene.filterPcloud(boundary_2)
        if boundary_3.has_points() != 0 :
            tmp_scene.filterPcloud(boundary_3)
        if boundary_4.has_points() != 0 :
            tmp_scene.filterPcloud(boundary_4)
        if boundary_5.has_points() != 0 :
            tmp_scene.filterPcloud(boundary_5)
        model_color = copy.deepcopy(self.model)
        model_color.paint_uniform_color([1., 1., 0.])
        boundary_1.paint_uniform_color([1., 0., 1.])
        boundary_2.paint_uniform_color([1., 0., 1.])
        boundary_3.paint_uniform_color([1., 0., 1.])
        boundary_4.paint_uniform_color([1., 0., 1.])
        boundary_5.paint_uniform_color([1., 0., 1.])
        tmp_scene.pcd.paint_uniform_color([1., 0., 0.])
        o3d.visualization.draw_geometries([tmp_scene.pcd + model_color + boundary_1 + boundary_2 + boundary_3 + boundary_4+ boundary_5])
       
        
        
        
    