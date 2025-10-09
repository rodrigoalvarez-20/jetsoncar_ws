#!/usr/bin/env python
from __future__ import division
import rospy
import pyrealsense2 as rs
from estimate_motion_variables2 import * 
from keras.models import Sequential
from keras.layers import Dense, Activation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#This node is for the lane follower control with supervised learning approach 
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class MLPControlNode(object):
    def __init__(self):
        # camera Parameters
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
	self.nb_actions = 11
        self.actions = np.linspace(-45, 45, self.nb_actions)
        self.action = 0
	self.model = Sequential()

        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(27)

        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions
	if not self.vel_state:

        	self.twist.angular.z = 45 * data.axes[0]

        # Throttle Comands, while button 7 is pressed -> capture experience

        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.vel_state = False
            self.first_frame = True
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.vel_state = True
	    #self.first

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
	    self.first_frame = True
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
	    self.first_frame = True
        else:
            self.vel_state = False
	    self.first_frame = True

    def get_state_values(self, img, last_eta, last_delta_x):
	
	crop_img = img[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]

        delta_x = None
        eta = None

        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interesting points of the road
        points = get_lane_points(crop_img, thresh=150, stride=3)
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
    def create_model(self):
	self.model.add(Dense(300, input_dim=2))
	self.model.add(Activation('relu'))
	self.model.add(Dense(200))
	self.model.add(Activation('relu'))
	self.model.add(Dense(self.nb_actions))
	self.model.add(Activation('softmax'))
	self.model.load_weights(str(self.nb_actions)+'_actions_mlp_weights.h5')
	self.model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
	self.model.predict(np.array([[-20, 0]]))

    def mlp_control(self):
        counter = 0
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
	self.create_model()
        while not rospy.is_shutdown():

            if self.vel_state:

                #self.frame_counter += 1
		self.twist.linear.x = 1.25

                if counter < 40:
                    #self.twist.linear.x = 1.18
                    counter += 1
                elif counter < 40:
                    #self.twist.linear.x = 0
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

		    #self.pub2.publish(self.delta_x)
		    #if counter % 2 == 0:
		    # 	self.pub2.publish(self.delta_x)

		    #else:
		    #	self.pub2.publish(self.eta)
			
		    prediction = self.model.predict(np.array([[self.eta, self.delta_x]]))
		    self.twist.angular.z = self.actions[np.argmax(prediction[0])]
		else:
		    self.none_counter += 1
		    if self.none_counter >= 5:
			self.dashed_calculation = False
			self.experience.append((self.delta_x, self.eta, self.twist.angular.z))
		
  
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()

if __name__ == '__main__':
    try:
        rospy.init_node("mlp_control", anonymous=True)
        mlp_control = MLPControlNode()
        mlp_control.mlp_control()
    except rospy.ROSInterruptException:
        pass
