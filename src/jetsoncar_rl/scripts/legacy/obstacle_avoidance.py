#!/usr/bin/env python
from __future__ import division
import math
import rospy
import pyrealsense2 as rs
from estimate_motion_variables2 import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class ControlNode(object):
    def __init__(self):
        # camera Parameters
	self.r = - 20
        self.episode = 0
        self.frame_counter = -1
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 30)
        self.color_image = np.zeros([180, 320, 3])
        self.img_shape = np.shape(self.color_image)
        self.roi = (90, 170, 0, self.img_shape[1])
	self.none_counter = 0
	self.on_left_lane = False

        # Start streaming
        self.pipeline.start(self.config)
        # feature extraction parameters
        self.eta = None
        self.delta_x = None
        self.experience = []
        self.vel_state = False
        self.twist = Twist()
        self.last_delta_x = None
        self.last_eta = None
        self.first_frame = True
        self.dashed_calculation = True

        # self.bridge = CvBridge()

        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(27)

        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
	#self.pub2 = rospy.Publisher('state_variables', Float64, queue_size = 10)
        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
	rospy.Subscriber('obstacles', Float32MultiArray, self.obstacles_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def obstacles_callback(self, data):
	if self.vel_state:
	    distances = [dist for i, dist in enumerate(data.data) if i % 2 == 0]
	    angles = [angle for i, angle in enumerate(data.data) if i % 2 == 1]

	    to_left = [x for x in angles if - 15 < x < 15]
	    to_right = [i for i, x in enumerate(angles) if x < -100]

	    if not self.on_left_lane and to_left: 
	        self.on_left_lane = True
	        self.r = 20
	    elif self.on_left_lane and to_right:
	        for ang in to_right:
		    if distances[ang] <= 0.35:
		        self.on_left_lane = False
		        self.r = - 20
		        break
	

    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions

	if not self.vel_state:

        	self.twist.angular.z = 45 * data.axes[0]
 
        # Throttle Comands, while button 7 is pressed -> capture experience

        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.vel_state = False
            self.first_frame = True
	    self.r = -20
	    self.on_left_lane = False
 
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
	    #self.search_min_angle = -15
	    #self.search_max_angle = 15
            self.vel_state = True
	    #self.on_left_lane = False

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
	    self.first_frame = True
	    self.r = -20
	    self.on_left_lane = False

        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
	    self.first_frame = True
	    self.r = -20
	    self.on_left_lane = False

        else:
            self.vel_state = False
	    self.first_frame = True
	    self.r = -20
	    self.on_left_lane = False

    def get_state_values(self, img, last_eta, last_delta_x):
	
	crop_img = img[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]

        delta_x = None
        eta = None

        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interesting points of the road
        points, bin_img = get_lane_points(crop_img, thresh=150, stride=3)
        points = list(filter(lambda x: x, points))
        #cv2.imwrite("images/ep" + str(self.episode) + "_frame" + str(self.frame_counter) + ".png", bin_img.astype('uint8'))
        groups = clustering_points(points)

        if self.first_frame:

            left_group, dashed_group, right_group = classify_groups(groups)
            if dashed_group:
                dashed_eq = polynomial_fitting(dashed_group)
		if dashed_eq:
                    eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, self.roi)
		

            if eta and delta_x:
                self.first_frame = False
                self.dashed_calculation = True
                return 180 * eta / np.pi, delta_x
            else:
                return None, None
        else:
            eta, delta_x = get_nearest_state_variables(groups, last_eta, last_delta_x,
                                                       self.dashed_calculation, self.roi)


            if eta and delta_x:

                return eta, delta_x
            else:
                return None, None



    def P_controller(self, eta, delta_x):
	#kp1 = -0.16
    	#kp2 = -0.22
        kp1 = -0.115 * 7
        kp2 = -0.17 * 7

        if delta_x > -10 or delta_x <= -30:
            alpha = 1
            beta = 1
        else:
            alpha = 1
            beta = 1

        e1 = eta + 1.26
        e2 = delta_x - self.r
	
	if abs(e2) <= 5:
            e2 = 0
    	
	if abs(e1) <= 4:
            e1 = 0

        st = alpha * kp1 * e1 + beta * kp2 * e2
        if st < - 50:
            st = - 50

        elif st > 50:
            st = 50

        return st

         

    def control_node(self):
        
        counter = 0
	counter_left = 0

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        while not rospy.is_shutdown():

            if self.vel_state:

                if counter < 50:
                    self.twist.linear.x = 1.18
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
                self.eta, self.delta_x = self.get_state_values(self.color_image, self.last_eta, self.last_delta_x)
                #self.twist.angular.z = self.P
                # self.experience = self.experience + [(self.delta_x, self.eta, self.twist.angular.z)]
                #self.experience.append((self.delta_x, self.eta, self.twist.angular.z, self.frame_counter))

                if self.eta and self.delta_x:
			
                    self.last_eta = self.eta
                    self.last_delta_x = self.delta_x
		    self.dahsed_calculation = True
		    self.none_counter = 0
		    self.experience.append((self.delta_x, self.eta, self.twist.angular.z))
		    #st, e1, e2 = self.PID_controller(self.eta, self.delta_x, e1, e2, st)
		    st = self.P_controller(self.eta, self.delta_x)
		    #self.pub2.publish(self.delta_x)
		    #if counter % 2 == 0:
		    # 	self.pub2.publish(self.delta_x)
	
		    #else:
		    #	self.pub2.publish(self.eta)
			
		    self.twist.angular.z = st
		else:
		    self.none_counter += 1
		    if self.none_counter >= 5:
			self.dashed_calculation = False
			self.experience.append((self.delta_x, self.eta, self.twist.angular.z))
		
                # cv2.imwrite('images/frame_'+ str(self.frame_counter) + '_' + str(self.episode) + '.jpg', self.gray_image)
                    
                    #st, e1, e2 = self.PID_controller(self.eta, self.delta_x, e1, e2, st)
                    #self.twist.angular.z = st[0]
            self.pub.publish(self.twist)
	    #self.pub2.publish(self.distance)
            self.rate.sleep()

        #self.pipeline.stop()
	#with open("experiences/testing.txt", "w") as exp_doc:
        #    for i in self.experience:
        #        for j in i:
        #            if isinstance(j, float):
        #                exp_doc.write("%.8f" % j)
        #            else:
        #                exp_doc.write(str(j))
        #            exp_doc.write('\t')
        #        exp_doc.write('\n')
        #exp_doc.close()


if __name__ == '__main__':
    try:
        rospy.init_node("control_node", anonymous=True)
        control_node = ControlNode()
        control_node.control_node()
    except rospy.ROSInterruptException:
        pass
