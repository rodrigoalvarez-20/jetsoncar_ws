#!/usr/bin/env python
from __future__ import division
import math
import rospy
import pyrealsense2 as rs
from estimate_motion_variables2 import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np


# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class obstacleDetection(object):
    def __init__(self):
        # camera Parameters
	self.inf = float("inf")
	self.distance = self.inf
	self.search_min_angle = -10
	self.search_max_angle = 10
	self.angle_min = -3.1400001049
    	self.angle_max = 3.1400001049
    	self.step_angle = 0.0010000000475
	self.idx1 = 0
	self.idx2 = 0
	self.clustering = DBSCAN(eps=0.1, min_samples=3)
	
	
        # Publisher
        self.pub = rospy.Publisher('obstacles', Float32MultiArray, queue_size=10)
        # Subscriber

	rospy.Subscriber('scan', LaserScan, self.sweep_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def get_groups(self, labels, points):
	groups = []
	for i in range(max(labels)):
	    group = []
	    for j, label in enumerate(labels):
		if i == label:
		    group.append(points[j])
	    groups.append(group)
	return groups
		

    def sweep_callback(self, msg):
        
    	dp = []
	hello_float = Float32MultiArray()
	points = []
    	#hello_float.data = []
	groups = []
	
    	
    	for i, i_range in enumerate(msg.ranges):
	    if i_range < 1.0:
	       	rad = self.index_to_radians(i)
	    	dp = dp + [[i_range * np.cos(rad), i_range * np.sin(rad)]]
		points.append([i_range, rad * 180 / np.pi])
	if dp:
	    self.clustering.fit(np.array(dp))
	    #self.pub.publish(np.max(self.clustering.labels_) + 1)
	    if np.max(self.clustering.labels_) >= 0:
	        groups = self.get_groups(self.clustering.labels_, points)
	    	#print self.clustering.labels_
	    	min_groups = []
	        for group in groups:
	            group = np.array(group)
	            min_idx = np.argmin(group[:, 0])
	            min_groups.append(group[min_idx][0])
	            min_groups.append(group[min_idx][1])
		    #print "points:", dp
		    #print "groups:", groups
		    #print "min of groups:", min_groups

	        hello_float.data = min_groups
	        self.pub.publish(hello_float)


    def index_to_radians(self, index):
	return ((2 * np.pi) / 6279) * index - np.pi
    
    def obstacle_node(self):
	t = 0
 

        while not rospy.is_shutdown():
	    if t < 10000:
	    	t += 1
	    else:
		t = 0
	    #print self.clustering.labels_

	    #rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node("obstacle_node", anonymous=True)
        obstacle_node = obstacleDetection()
        obstacle_node.obstacle_node()
    except rospy.ROSInterruptException:
        pass
