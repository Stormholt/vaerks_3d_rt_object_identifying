#Author: Ajs R. Stormholt

import pyrealsense2 as rs   #pip install pyrealsense2   API: https://intelrealsense.github.io/librealsense/python_docs/index.html
import open3d as o3d        #pip install open3d         API: http://www.open3d.org/docs/release/introduction.html                         
import numpy as np          #pip install numpy
import cv2                  #pip install opencv-python  API :https://docs.opencv.org/master/
import copy

voxel_size = 0.01 # 1cm 
# smaller = longer cpu time and worse registration.

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=10, # neighbours in consideration, Higher = aggressive
                                                    std_ratio=0.01) # Threshold based on deviation of avg. dist. lower = aggressive
    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source, target):
    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], 
                             [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], 
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
   # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)

    src_normal = source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    tar_normal = target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

class Pcloud():
    def __init__(self, x, y ,z ):

        self.x = x
        self.y = y
        self.z = z

        self.tvector = [x,y,z]

        self.pcd = o3d.geometry.PointCloud()


# Camera class 
class D435():
    def __init__(self):

        self.depth_im_width = 848   #Image dimensions
        self.depth_im_height = 480
        self.depth_framerate = 10 
        self.rgb_im_width = 640
        self.rgb_im_height = 480
        self.rgb_framerate = 30 

        self.pipeline = rs.pipeline()   #The pipeline simplifies the user interaction with the device and computer vision processing modules
        self.config = rs.config()       # The config allows pipeline users to request filters for the pipeline streams and device selection and configuration.
        self.pc = rs.pointcloud() # Init pointcloud 
        self.pcdmodel = o3d.geometry.PointCloud() # Final pointcloud
        self.pcloud_array = []
        self.colorizer = rs.colorizer() # Colorizer filter generates color images based on input depth frame
        # Get device product line for setting a supporting resolution
        #self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        #self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        #self.device = self.pipeline_profile.get_device()
        #self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, self.depth_im_width, self.depth_im_height, rs.format.z16, self.depth_framerate) # Setup depth stream
        self.config.enable_stream(rs.stream.color, self.rgb_im_width, self.rgb_im_height, rs.format.bgr8, self.rgb_framerate)
        
        self.begin() 
        
        self.profile = self.pipeline.get_active_profile() #The pipeline profile includes a device and a selection of active streams, with specific profiles
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth)) # Stores details about the profile of a stream.
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height # Width and height of the image in pixels
        self.fx, self.fy = self.depth_intrinsics.fx,self.depth_intrinsics.fy # Focal length of the image plane as a multiple of pixel width and height 
        self.px, self.py = self.depth_intrinsics.ppx,self.depth_intrinsics.ppy # Horizontal and vertical coordinate of the principal point of the image, as a pixel offset from the left edge and the top edge, Basicly center of the image
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w,self.h,self.fx,self.fy,self.px,self.py) # Create Open3d intrinsic object
        
        self.plys_generated = 0 # Counter for .ply files generated
        self.pcds_generated = 0 # Counter for .pcd files generated
        self.stls_generated = 0 # Counter for .stl files generated
        self.imgs_generated = 0 # Counter for .png files generated
        
        #BIG MATH basicly camera extrinsics
        self.x = 0.0  
        self.y = 0.0  
        self.z = 0.0 
        self.dist = 1.0  # distance from last position
        self.angle = 0.0 # degrees of change. 
        self.omega = 0.0 # x rotation
        self.alpha = 0.0 # y rotation
        self.theta = 0.0 # z rotation
        self.d2r = np.pi/180 # multiplication translates degrees to radians
        self.rmatrix =  [[0.0,0.0,0.0],
                        [0.0,0.0,0.0],
                        [0.0,0.0,0.0]] #3x3 Rotational matrix
        self.tvector = [self.x,self.y,self.z] # Translation vector 
    
    # Start streaming
    def begin(self):
        self.pipeline.start(self.config) # Start the pipeline streaming according to the configuration
        self.align = rs.align(rs.stream.color) # Align depth image to other 

    #Stop streaming
    def release(self):
        self.pipeline.stop()
        self.pipe = None
        self.config = None
    
    def generatePLY(self):
        self.mapped_frame = self.depth_frame
        self.points = self.pc.calculate(self.depth_frame)
        self.pc.map_to(self.mapped_frame)
        #Colorize to give ply file texture 
        colorized = self.colorizer.process(self.frames)
        # Create save_to_ply object
        
        filename = "./3dmodels/"+str(self.plys_generated) +".ply"
        ply = rs.save_to_ply(filename)

            # Set options to the desired values
        # In this example we'll generate a textual PLY with normals (mesh is already created by default)
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)
        # Apply the processing block to the frameset which contains the depth frame and the texture
        ply.process(colorized)
        self.plys_generated = self.plys_generated +1
        #points.export_to_ply('./out.ply', mapped_frame)

    
    def generatePCD(self):
        
        wtf = Pcloud(self.x,self.y,self.z)
        wtf.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.depth_frame_open3d,self.intrinsic) # Creating pointcloud from depth image
        wtf.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01,max_nn=30)) # Calculates normals of every point. 
        wtf.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.])) 

        self.y = self.y +0.05
        #self.calcPosition(self.dist,self.angle)
        #self.calcRmatrix()
        #wtf.rotate(self.rmatrix, (0, 0, 0))
        
        #self.calcTvector()
        wtf.pcd.translate(wtf.tvector)

        wtf.pcd, self.ind = wtf.pcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=0.5) 
        #self.pcdmodel = self.pcdmodel + wtf
        self.pcloud_array.append(wtf)
        filename = "./pointclouds/"+str(self.pcds_generated) +".pcd"
        o3d.io.write_point_cloud(filename,wtf.pcd)
       # filename = "./pointclouds/pointCloud.pcd"
       # o3d.io.write_point_cloud(filename,self.pcdmodel)
        self.pcds_generated = self.pcds_generated +1

    def generateMothership(self):
        if self.pcloud_array:
            self.pcdmodel = self.pcloud_array[0].pcd
            self.pcloud_array.pop(0)
            for pcloud in self.pcloud_array:
                pcloud.pcd, self.pcdmodel, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, pcloud.pcd, self.pcdmodel)
                result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh,voxel_size)
                pcloud.pcd = pcloud.pcd.transform(result_ransac.transformation)
                result_icp = refine_registration(pcloud.pcd, self.pcdmodel, source_fpfh, target_fpfh, voxel_size, result_ransac)
                pcloud.pcd = pcloud.pcd.transform(result_icp.transformation)
                self.pcdmodel = self.pcdmodel + pcloud.pcd
            
            filename = "./pointclouds/pointCloud.pcd"
            o3d.io.write_point_cloud(filename,self.pcdmodel)
        
    def genSimpleMothership(self):
        if self.pcloud_array:
            for pcloud in self.pcloud_array:
                self.pcdmodel = self.pcdmodel + pcloud.pcd
            filename = "./pointclouds/pointCloud.pcd"
            o3d.io.write_point_cloud(filename,self.pcdmodel)


    def generateSTL(self,kpoints,stdRatio, depth, iterations):
        if not self.pcdmodel.is_empty():
            stl_pcd = self.pcdmodel
            stl_pcd = stl_pcd.uniform_down_sample(every_k_points=kpoints)
            stl_pcd, ind = stl_pcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=stdRatio)
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
            
            filename = "3dmodels/model"+str(self.stls_generated)+".stl"
            o3d.io.write_triangle_mesh(filename, mesh)
            self.stls_generated = self.stls_generated + 1

   
    def calcTvector(self):
        self.tvector = [self.x,self.y,self.z]
    
    def stream(self):
        self.frames = self.pipeline.wait_for_frames()# Wait for a coherent pair of frames: depth and color
        self.frames = self.align.process(self.frames) # Aligns frames
        self.depth_frame = self.frames.get_depth_frame() #Extract frames
        self.color_frame = self.frames.get_color_frame()

         # Convert images to numpy arrays
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        self.depth_frame_open3d = o3d.geometry.Image(self.depth_image) # Create opend3d image obejct from depth image
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_TURBO) # 

        self.depth_colormap_dim = self.depth_colormap.shape
        self.color_colormap_dim = self.color_image.shape

        if self.depth_colormap_dim != self.color_colormap_dim:
            resized_color_image = cv2.resize(self.color_image, dsize=(self.depth_colormap_dim[1], self.depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            self.images = np.hstack((resized_color_image, self.depth_colormap))
        else:
            self.images = np.hstack((self.color_image, self.depth_colormap))
        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.images)
    
    def generateImg(self):
         
        filename = "images/"+ str(self.imgs_generated)+".png"
        cv2.imwrite(filename, self.images)
        self.imgs_generated= self.imgs_generated + 1
        

    def snapshot(self):
        
        self.generateImg()
        self.generatePCD()

    def visualizeMothership(self,source):
        source_tmp = copy.deepcopy(source)
        source_tmp.paint_uniform_color([1, 0.706, 0])
        o3d.visualization.draw_geometries([source_tmp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])   
        #Press 'n' to see normals, press '+' and '`' to change voxel size.
        # 'Rigth mouse' rotates, 'mouse wheel' drags

        
camera = D435()   
try:
    while True:
        camera.stream()
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == 32:
            camera.snapshot()
           # camera.generatePLY()
finally:
    camera.generateMothership()
    camera.generateSTL(kpoints=10,stdRatio=0.5,depth=8, iterations=8)
    camera.visualizeMothership(camera.pcdmodel)
    camera.release()




#pcd0 = pcl.read("0.pcd","PointXYZRGBA")

#preprocess_point_cloud(pcd0, voxel_size)


#source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset( voxel_size)

#result_ransac = execute_global_registration(source_down, target_down,
                                        #    source_fpfh, target_fpfh,
                                       #     voxel_size)

