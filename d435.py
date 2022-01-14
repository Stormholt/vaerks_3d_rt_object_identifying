#Author: Ajs R. Stormholt

import pyrealsense2 as rs   #pip install pyrealsense2   API: https://intelrealsense.github.io/librealsense/python_docs/index.html
import open3d as o3d        #pip install open3d         API: http://www.open3d.org/docs/release/introduction.html                         
import numpy as np          #pip install numpy
import cv2                  #pip install opencv-python  API :https://docs.opencv.org/master/

class D435():
    def __init__(self, camera_enable):
        
        self.camera_enable = camera_enable #Is the camera connected or not.
        
        self.depth_im_width = 848   #Image dimensions and framerate
        self.depth_im_height = 480
        self.depth_framerate = 30 
        self.rgb_im_width = 640
        self.rgb_im_height = 480
        self.rgb_framerate = 30 

        self.pipeline = rs.pipeline()   #The pipeline simplifies the user interaction with the device and computer vision processing modules
        self.config = rs.config()       # The config allows pipeline users to request filters for the pipeline streams and device selection and configuration.
        self.device = rs.device()
        self.colorizer = rs.colorizer() # Colorizer filter generates color images based on input depth frame

        self.initCam() # Initialisationonlypossibleif the camera is connected
        
        self.plys_generated = 0 # Counter for number of files generated
        self.imgs_generated = 0
        
        #camera position
        self.depth_startpoint_offset = 4.2 # Offset to the actualt depth startpoint, following the d435 datasheet.
        self.x = 0.0  
        self.y = 0.0  
        self.z = 0.0 
    
    def initCam(self):
        if self.camera_enable == True:
            
            self.config.enable_stream(rs.stream.depth, self.depth_im_width, self.depth_im_height, rs.format.z16, self.depth_framerate) # Setup depth stream
            self.config.enable_stream(rs.stream.color, self.rgb_im_width, self.rgb_im_height, rs.format.bgr8, self.rgb_framerate)
            
            self.begin() 

            self.profile = self.pipeline.get_active_profile() #The pipeline profile includes a device and a selection of active streams, with specific profiles
            depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth)) # Stores details about the profile of a stream.
            depth_intrinsics = depth_profile.get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height # Width and height of the image in pixels
            fx, fy = depth_intrinsics.fx,depth_intrinsics.fy # Focal length of the image plane as a multiple of pixel width and height 
            px, py = depth_intrinsics.ppx,depth_intrinsics.ppy # Horizontal and vertical coordinate of the principal point of the image, as a pixel offset from the left edge and the top edge, Basicly center of the image
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(w,h,fx,fy,px,py) # Create Open3d intrinsic object
    
    # Start streaming
    def begin(self):
        self.profile = self.pipeline.start(self.config) # Start the pipeline streaming according to the configuration
        self.align = rs.align(rs.stream.color) # Align depth image to other 

    #Stop streaming
    def release(self):
        self.pipeline.stop()
        self.pipe = None
        self.config = None
    
    
    def stream(self, depth_enable, rgb_enable):
        self.frames = self.pipeline.wait_for_frames()# Wait for a coherent pair of frames: depth and color # if fails here it couldn't receive frames
        self.frames = self.align.process(self.frames) # Aligns frames

        if depth_enable == True : 
            self.depth_frame = self.frames.get_depth_frame() #Extract frames
            self.depth_image = np.asanyarray(self.depth_frame.get_data())# Convert images to numpy arrays
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_TURBO) 
            self.depth_colormap_dim = self.depth_colormap.shape
            
        if rgb_enable == True:
            self.color_frame = self.frames.get_color_frame()
            self.color_image = np.asanyarray(self.color_frame.get_data())
            self.color_colormap_dim = self.color_image.shape
        
        if depth_enable == True and rgb_enable == True:
            if self.depth_colormap_dim != self.color_colormap_dim:
                resized_color_image = cv2.resize(self.color_image, dsize=(self.depth_colormap_dim[1], self.depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                self.images = np.hstack((resized_color_image, self.depth_colormap))
            else:
                self.images = np.hstack((self.color_image, self.depth_colormap))
        elif depth_enable == True and rgb_enable == False:
            self.images = self.depth_colormap
        elif depth_enable == False and rgb_enable == True:
            self.images = self.color_image
        else :
            print("Must enable either depth, RGB or both to use stream method")
            return
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.images)

    def generateImg(self, dir):
        if self.images == None:
            print("stream function must be called first")
            return
        else:
            filename = dir + str(self.imgs_generated)+".png"
            cv2.imwrite(filename, self.images)
            self.imgs_generated= self.imgs_generated + 1


    def updatePosition(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

   
