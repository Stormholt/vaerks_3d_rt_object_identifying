import pyrealsense2 as rs 
import open3d as o3d #pip install open3d
import numpy as np #pip install numpy
import cv2
import pclpy 
from pclpy import pcl   

# Camera class 
class D435():
    def __init__(self):

        self.im_width = 640
        self.im_height = 480
        self.framerate = 30 

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, self.framerate)
        self.config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, self.framerate)
        
        self.colorizer = rs.colorizer()
        self.begin()
        self.profile = self.pipeline.get_active_profile()
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.plys_generated = 0
        self.pcds_generated = 0
        self.pc = rs.pointcloud()
    
    # Start streaming
    def begin(self):
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

    #Stop streaming
    def release(self):
        self.pipeline.stop()
        self.pipe = None
        self.config = None
    
    def generatePLY(self, frames, points, mapped_frame):
        #Colorize to give ply file texture 
        colorized = self.colorizer.process(frames)
        # Create save_to_ply object
        
        filename = str(self.plys_generated) +".ply"
        ply = rs.save_to_ply(filename)

            # Set options to the desired values
        # In this example we'll generate a textual PLY with normals (mesh is already created by default)
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)
        # Apply the processing block to the frameset which contains the depth frame and the texture
        ply.process(colorized)
        self.plys_generated = self.plys_generated +1
        #points.export_to_ply('./out.ply', mapped_frame)
    
    def generatePCD(self,verts):
        filename = "./"+str(self.pcds_generated) +".pcd"
        pc2 = pcl.PointCloud.PointXYZ(verts)
        writer = pcl.io.PCDWriter()
        writer.writeBinary(filename,pc2)
        self.pcds_generated = self.pcds_generated +1

camera = D435()
#camera.begin()    
try:

    #camera = D435()
    #camera.begin()

    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = camera.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        camera.depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        camera.w, camera.h = camera.depth_intrinsics.width, camera.depth_intrinsics.height
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_TURBO)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        mapped_frame = depth_frame
        points = camera.pc.calculate(depth_frame)
        camera.pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == 32:
            camera.generatePLY(frames, points, mapped_frame)
            camera.generatePCD(verts)

finally:
    # Stop streaming
    camera.release()
    

