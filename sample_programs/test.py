import cv2
import pyrealsense2
from realsense_depth import *

point = (400,300)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

cv2.namedWindow("color frame")
cv2.setMouseCallback("color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()



# Show distance for a specific point
    
    cv2.circle(color_frame, point, 4, (0, 0, 255))
    distance = depth_frame[point[1], point[0]]
   # cv2.putText(color_frame, "{}mm at X{} Y {}".format(distance, point[0],point[1]), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
    cv2.putText(color_frame,  "{},{}".format( point[0],point[1]), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
    print(distance)

    cv2.imshow("depth frame",depth_frame)
    cv2.imshow("color frame",color_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
    
cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

