#!/usr/bin/env python
from __future__ import division
import rospy
import math
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

#this script stops the robot when there is an obstcle in front in the range (0,40) cm (-20, 20) degrees

#-----------------------------------------------------------------------------------------------------------#
# Global variables
distance = 0
inf = float("inf")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 240, rs.format.z16, 30)


def start():
    global pub
    global twist
    twist = Twist()
    #Publisher for Arduino subscriber
    pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    #pub = rospy.Publisher('distance_realsense', String, queue_size=10)
    rospy.init_node('realsense_stop', anonymous = True)
    image_center = (320, 120)
    size_window = 3
    pipeline.start(config)
    twist.angular.z = 0
 
    while not rospy.is_shutdown():
        dp = 0
        count = 0
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if not depth_frame:
            continue
        
	for x in range(image_center[0]-size_window, image_center[0]+size_window):
            for y in range(image_center[1]-size_window, image_center[1]):
	        dp = dp + depth_frame.get_distance(x,y)	    
        distance = 0.01*(dp / 2*(size_window)**2)
        if distance < 0.3 and distance != 0:
            twist.linear.x = 0
        else:
            twist.linear.x = 1.2            
        pub.publish(twist)    
   
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pipeline.stop()
        

