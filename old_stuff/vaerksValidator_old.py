
import pyrealsense2 as rs   #pip install pyrealsense2   API: https://intelrealsense.github.io/librealsense/python_docs/index.html
import open3d as o3d        #pip install open3d         API: http://www.open3d.org/docs/release/introduction.html                         
import numpy as np          #pip install numpy
import cv2                  #pip install opencv-python  API :https://docs.opencv.org/master/
from pcloud import * 
import d435
from altiZ import *
import enum 
import pandas as pd
from matplotlib import pyplot as plt

voxel_size = 0.01 # 1cm # smaller = longer cpu time and worse registration.
depth_threshold = 0.4 #40 cm 



class App():
        
    def __init__(self, rootdir,camera_enable):
        self.root = rootdir
        self.pcd_path = self.root +"/pointclouds/"
        self.model_path = self.root + "/3dmodels/"
        self.img_path = self.root + "/images/"
        self.camera_enable = camera_enable
        #self.laser = AltiZ()
        self.camera = d435.D435(self.camera_enable)
        self.pcdmodel = o3d.geometry.PointCloud() # Final pointcloud
        self.pcloud_array = []
        self.plys_generated = 0
        self.pcds_generated = 0 # Counter for .pcd files generated
        self.stls_generated = 0 # Counter for .stl files generated
        self.imgs_generated = 0 # Counter for .png files generated
        self.model = o3d.geometry.PointCloud()
        self.table = o3d.geometry.PointCloud()
        self.scene = Pcloud()
        #self.scene.pcd = o3d.io.read_point_cloud(self.pcd_path + "notmiwire_scene_2.ply")

    class Filetype(enum.Enum):
        PCD = 0
        PLY = 1
        STL = 2 
        IMG = 3

    def savetoPCD(self, path, name, pcloud):
        filename = path + name + ".pcd"
        if not pcloud.pcd_down.is_empty(): # If filtered poitncloud exists
            o3d.io.write_point_cloud(filename,pcloud.pcd_down)
        elif not pcloud.pcd.is_empty():
            o3d.io.write_point_cloud(filename,pcloud.pcd)
        else:
            print("Cant save pointcloud: Its Empty")


    def generatePointCloud(self, filetype, name):
        wtf = Pcloud()
        wtf.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.camera.depth_frame_open3d,self.camera.intrinsic) # Creating pointcloud from depth 
        wtf.x = self.camera.x
        wtf.y =  self.camera.y
        wtf.z = self.camera.z 
        wtf.updateTvector()
        wtf.process_point_cloud( self.model, self.table)
        self.scene.pcd += wtf.pcd
        if (filetype == self.Filetype.PCD):
            filename = self.pcd_path + name +"_"+ str(self.pcds_generated) + ".pcd"
            self.pcds_generated += 1
        elif (filetype == self.Filetype.PLY):
            filename = self.pcd_path + name +"_"+ str(self.plys_generated) + ".ply"
            self.plys_generated+=1
        else:
            print("Unsupported filetype for saving pointclouds")
            return

        o3d.io.write_point_cloud(filename,wtf.pcd)
        

    def captureScenePointcloud(self):
        wtf = Pcloud()
        wtf.x = self.camera.x
        wtf.y =  self.camera.y
        wtf.z = self.camera.z 
        wtf.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.camera.depth_frame_open3d,self.camera.intrinsic) # Creating pointcloud from depth 
        wtf.updateTvector()
        wtf.process_point_cloud( self.model, self.table)
        self.scene.pcd += wtf.pcd

    def saveScenePointcloud(self, filetype, name):
        if (filetype == self.Filetype.PCD):
            filename = self.pcd_path + name +"_"+ str(self.pcds_generated) + ".pcd"
        elif (filetype == self.Filetype.PLY):
            filename = self.pcd_path + name +"_"+ str(self.plys_generated) + ".ply"
        else:
            print("Unsupported filetype for saving pointclouds")
            return
        o3d.io.write_point_cloud(filename,self.scene.pcd)

    def genRegMothership(self):# Generate translated and registrated mothership 
        if self.pcloud_array :
            self.pcdmodel = self.pcloud_array[0].pcd_down
            self.pcloud_array.pop(0)
            for pcloud in self.pcloud_array:
                pcloud.pcd_down, self.pcdmodel, source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, pcloud.pcd_down, self.pcdmodel)
                result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh,voxel_size)
                pcloud.pcd_down = pcloud.pcd_down.transform(result_ransac.transformation)
               # result_icp = refine_registration(pcloud.pcd, self.pcdmodel, source_fpfh, target_fpfh, voxel_size, result_ransac)
               # pcloud.pcd = pcloud.pcd.transform(result_icp.transformation)
                self.pcdmodel = self.pcdmodel + pcloud.pcd_down
            
            self.savePointCloud(self.pcd_path, "pointCloud", self.pcdmodel)

    def genTransMothership(self): #Generate translated mothership
        if self.pcloud_array:
            for pcloud in self.pcloud_array:
                self.pcdmodel = self.pcdmodel + pcloud.pcd_down
            self.savePointCloud(self.pcd_path, "pointCloud", self.pcdmodel)

    def gen3Dmodel(self,kpoints,stdRatio, depth, iterations):
        if not self.pcdmodel.is_empty():
            stl_pcd = self.pcdmodel
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
            
            filename = self.model_path  +"model" + str(self.stls_generated) + ".stl"
            o3d.io.write_triangle_mesh(filename, mesh)
            filename = self.model_path  +"model" + str(self.stls_generated) + ".ply"
            self.stls_generated = self.stls_generated + 1

    def loadPointClouds(self, numberOfPointclouds):
        wtf = Pcloud() 
        tvector = [0,0,0]
        wtf.pcd = o3d.io.read_point_cloud("./pointclouds/%i.pcd" % 0)
        for i in range(1,numberOfPointclouds):
            pcd = o3d.io.read_point_cloud("./pointclouds/%i.pcd" % i)
            pcd.translate(tvector)
            wtf.pcd += pcd
            tvector[0]+=0.004
        wtf.process_point_cloud(voxel_size,depth_threshold)
        wtf.visualizeMothership(wtf.pcd)
        print(wtf.pcd)
        wtf.save3Dmodel(self.model_path,"mother", wtf)

    def save3Dmodel (self, path, name, pcloud, filetype):
        if not pcloud.pcd_down.is_empty():
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson( pcloud.pcd_down, depth=9)
        elif not pcloud.pcd.is_empty():
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson( pcloud.pcd, depth=9)
        else:
            print("No pointcloud data to generate 3d model from")
            return
        if (filetype == self.Filetype.STL):
            filename = path + name + ".stl"
        elif (filetype == self.Filetype.PLY):
            filename = path + name + ".ply"
        else:
            print("Unsupported filetype for saving 3dmodels")
            return
        o3d.io.write_triangle_mesh(filename, mesh)

    def compareScene2Model(self):
        if not (self.scene.pcd.is_empty() or self.model.is_empty()):
            distance = self.scene.pcd.compute_point_cloud_distance(self.model)
            haussdorf = max(distance)
            print("Haussdorf distance = " + str(haussdorf) + "\n")
            df = pd.DataFrame({"distances": distance}) # transform to a dataframe
            # Some graphs
            ax1 = df.boxplot(return_type="axes") # BOXPLOT
            ax2 = df.plot(kind="hist", alpha=0.5, bins = 1000) # HISTOGRAM
            #ax3 = df.plot(kind="line") # SERIE
            plt.show()
        else:
            print("Empty pointcloud given to compare")
