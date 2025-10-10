#!/usr/bin/env python
from __future__ import division

import pickle
import socket
import struct  # ## new code
import sys

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from estimate_motion_variables2 import (classify_groups, clustering_points,
                                        get_heading_angle_lateral_deviation,
                                        get_lane_points,
                                        get_nearest_state_variables,
                                        polynomial_fitting)
from geometry_msgs.msg import Twist
from keras.layers import Activation, Dense
from keras.models import Sequential
from sensor_msgs.msg import Joy

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',8089))

class MLPControlNode(object):
    def __init__(self):
        # camera Parameters
        self.episode = 45
        self.frame_counter = -1
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 60)
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
        self.nb_actions = 5
        self.actions = np.linspace(-45, 45, self.nb_actions)
        self.action = 10
        self.model = Sequential()
        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(40)
        # Publisher
        self.pub = rospy.Publisher("jetsoncar_instructions", Twist, queue_size=10)
        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):
        # print(data.buttons)
        # print(self.vel_state)
        # print(data.axes)
        # Steering angle , transform joystick commands to actions
        if not self.vel_state:
            self.twist.angular.z = 45 * data.axes[0]
        # Throttle Comands, while button 7 is pressed -> capture experience
        # L2 - 6 | R2 - 7
        if data.buttons[6] == 1 and data.buttons[7] == 0:
            print("6 - On | 7 - Off")
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10  # / 8
            self.vel_state = False
            self.first_frame = True
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            print("6 - Off | 7 - On")
            self.vel_state = True
            self.first_frame = True
        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            print("6 - On | 7 - On")
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
            self.first_frame = True
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            print("6 - Off | 7 - Off")
            self.twist.linear.x = 0  # (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
            self.first_frame = True
        else:
            print("WTF!!!")
            self.vel_state = False
            self.first_frame = False
            self.twist.linear.x = 0  # 5 * (data.axes[4] + 1) - 10

    def get_state_values(self, img, last_eta, last_delta_x):
        crop_img = img[self.roi[0] : self.roi[1], self.roi[2] : self.roi[3]]
        delta_x = None
        eta = None
        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interesting points of the road
        points, bin_img = get_lane_points(crop_img, thresh=190, stride=3)
        points = list(filter(lambda x: x, points))
        # cv2.imwrite("images/ep" + str(self.episode) + "_frame" + str(self.frame_counter) + ".png", bin_img.astype('uint8'))
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
            eta, delta_x = get_nearest_state_variables(
                groups, last_eta, last_delta_x, self.dashed_calculation, self.roi
            )
            if eta and delta_x:
                return eta, delta_x
            else:
                return None, None

    def create_model(self):
        self.model.add(Dense(300, input_dim=2))
        self.model.add(Activation("relu"))
        self.model.add(Dense(200))
        self.model.add(Activation("relu"))
        self.model.add(Dense(1))
        self.model.add(Activation("linear"))
        self.model.load_weights("regression_mlp_weights.h5")
        # self.model.load_weights('mlp_regressor_weights_left.h5')
        self.model.compile(loss="mse", optimizer="adam", metrics=["accuracy"])
        self.model.predict(np.array([[-20, 0]]))

    def mlp_control(self):
        counter = 0
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        self.create_model()
        while not rospy.is_shutdown():
            if self.vel_state:
                self.frame_counter += 1
                # self.twist.linear.x = 2
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
                data = pickle.dumps(np.asanyarray(color_frame.get_data())) #pickle.dumps(color_frame.) ### new code
                clientsocket.sendall(struct.pack("H", len(data))+data) ### new code
                # self.eta, self.delta_x = self.get_state_values(
                #    self.color_image, self.last_eta, self.last_delta_x
                # )
                self.eta = None
                self.delta_x = None
                # self.twist.angular.z = self.P
                # self.experience = self.experience + [(self.delta_x, self.eta, self.twist.angular.z)]
                # self.experience.append((self.delta_x, self.eta, self.twist.angular.z, self.frame_counter))

                if self.eta and self.delta_x:
                    self.last_eta = self.eta
                    self.last_delta_x = self.delta_x

                self.dahsed_calculation = True
                self.none_counter = 0
                self.experience.append((self.delta_x, self.eta, self.twist.angular.z))

            # self.pub2.publish(self.delta_x)
            # if counter % 2 == 0:
            # 	self.pub2.publish(self.delta_x)
            # else:
            # 	self.pub2.publish(self.eta)
            # st = self.model.predict(np.array([[self.eta, self.delta_x]]))[0]
            # self.twist.angular.z = st
            else:
                self.none_counter += 1
                if self.none_counter >= 5:
                    self.dashed_calculation = False
                    self.experience.append(
                        (self.delta_x, self.eta, self.twist.angular.z)
                    )

            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()

        # with open("experiences/testing_mlp.txt", "w") as exp_doc:
        #    for i in self.experience:
        #        for j in i:
        #            if isinstance(j, float):
        #                exp_doc.write("%.8f" % j)
        #            else:
        #                exp_doc.write(str(j))
        #            exp_doc.write('\t')
        #        exp_doc.write('\n')
        # exp_doc.close()


if __name__ == "__main__":
    rospy.loginfo("Control iniciado")
    try:
        rospy.init_node("mlp_control", anonymous=True)
        mlp_control = MLPControlNode()
        mlp_control.mlp_control()
    except rospy.ROSInterruptException:
        pass
