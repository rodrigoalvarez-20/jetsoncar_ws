#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import pyrealsense2 as rs
import cv2
#from estimate_motion_variables2 import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError

class CNNControlNode(object):
    def __init__(self):
        # camera Parameters
	self.model_number = 1
        self.episode = 0
        self.frame_counter = -1
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 30)
        self.color_image = np.zeros([180, 320, 3])
        self.img_shape = np.shape(self.color_image)
        self.roi = (90, 170, 0, self.img_shape[1])


        # Start streaming
        self.pipeline.start(self.config)
        # feature extraction parameters
        self.vel_state = False
        self.twist = Twist()
        self.bridge = CvBridge()

        # Node cycle rate (in Hz)'
        self.rate = rospy.Rate(27)

        # Publisher
        self.pub1 = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
	self.pub2 = rospy.Publisher('bin_img', Image, queue_size = 10)
	self.pub3 = rospy.Publisher('model', Int16, queue_size=10)
        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
	rospy.Subscriber('cnn_instructions', Twist, self.get_actions)
	rospy.Subscriber('obstacles', Float32MultiArray, self.obstacles_callback)
        #rospy.Subscriber2("cnn_instructions", Image, self.image_callback)
    def obstacles_callback(self, data):
	if self.vel_state:
	    distances = [dist for i, dist in enumerate(data.data) if i % 2 == 0]
	    angles = [angle for i, angle in enumerate(data.data) if i % 2 == 1]

	    to_left = [x for x in angles if - 15 < x < 15]
	    to_right = [i for i, x in enumerate(angles) if x < -90]

	    if not self.on_left_lane and to_left: 
	        self.on_left_lane = True
	        self.model_number = 2
	    elif self.on_left_lane and to_right:
	        for ang in to_right:
		    if distances[ang] <= 0.6:
		        self.on_left_lane = False
		        self.model_number = 1
		        break
    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions

	if not self.vel_state:

        	self.twist.angular.z = 45 * data.axes[0]
 
        # Throttle Comands, while button 7 is pressed -> capture experience

        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.vel_state = False
	    self.model_number = 1
	    self.on_left_lane = False
 
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
	    #self.search_min_angle = -15
	    #self.search_max_angle = 15
            self.vel_state = True
	    #self.on_left_lane = False

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
	    self.model_number = 1
	    self.on_left_lane = False

        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
	    self.model_number = 1
	    self.on_left_lane = False

        else:
            self.vel_state = False
	    self.model_number = 1
	    self.on_left_lane = False


    def get_bin_image(self, img):
	
	crop_img = img[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
    	gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    	ret,bin_image = cv2.threshold(gray, 150, 255,cv2.THRESH_BINARY)
        bin_image = bin_image // 255
        return bin_image

    def get_actions(self, data):
	self.twist.angular = data.angular




    def control_node(self):
     
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
	counter = 0
        while not rospy.is_shutdown():

            if self.vel_state:

                #self.frame_counter += 1
		self.twist.linear.x = 1.25

                if counter < 55:
                    self.twist.linear.x = 1.8
                    counter += 1
                elif counter < 60:
                    self.twist.linear.x = 0
                    counter += 1
                else:
                    counter = 0
		
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                self.color_image = np.asanyarray(color_frame.get_data())
    		bin_img = self.get_bin_image(self.color_image)

		try:
         	    self.pub2.publish(self.bridge.cv2_to_imgmsg(bin_img, "mono8"))
  	     	except CvBridgeError as e:
         	    print e

   	    self.pub1.publish(self.twist)
	    self.pub3.publish(self.model_number)
            self.rate.sleep()

        self.pipeline.stop()
	"""with open("experiences/testing.txt", "w") as exp_doc:
            for i in self.experience:
                for j in i:
                    if isinstance(j, float):
                        exp_doc.write("%.8f" % j)
                    else:
                        exp_doc.write(str(j))
                    exp_doc.write('\t')
                exp_doc.write('\n')
        exp_doc.close()"""


if __name__ == '__main__':
    try:
        rospy.init_node("control_node", anonymous=True)
        control_node = CNNControlNode()
        control_node.control_node()
    except rospy.ROSInterruptException:
        pass
