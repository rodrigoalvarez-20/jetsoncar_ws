#!/usr/bin/env python
from __future__ import division
import rospy
import sys
import cv2
import pyrealsense2 as rs
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.depth, 640, 240, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 10)

config.enable_stream(rs.stream.color, 640, 480, rs.format.y16, 10)


#from __future__ import print_function

def image_publisher():
    color_pub = rospy.Publisher('color_image', Image, queue_size = 10)
    #depth_pub = rospy.Publisher('depth_image', Image, queue_size = 10)
    rospy.init_node('realsense_node', anonymous=True)
    # Start streaming
    pipeline.start(config)
    #rate = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():
        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
	color_image = (color_image / 256).astype('uint8')
        #gray=cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, None, 0.1, 0), cv2.COLORMAP_JET)
        #print np.shape(depth_colormap)
        
        #Publishing color image 

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(n)
        try:
	    color_pub.publish(bridge.cv2_to_imgmsg(color_image, 'mono8'))
            #color_pub.publish(bridge.cv2_to_imgmsg(color_image, 'bgr8'))
            #depth_pub.publish(bridge.cv2_to_imgmsg(depth_colormap, 'bgr8'))
        except CvBridgeError as e:
            print e
    
if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pipeline.stop()
        cv2.destroyAllWindows()
