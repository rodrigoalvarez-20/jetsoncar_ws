#!/usr/bin/env python
from __future__ import division
import rospy
import pyrealsense2 as rs
from lane_detection import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense, Activation
from keras.optimizers import Adam
from keras.models import load_model
from collections import deque



class QnetworkControlNode(object):
    def __init__(self):
        # camera Parameters
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 60)
        self.color_image = np.zeros([180, 320])

        # Start streaming
        self.pipeline.start(self.config)
        # feature control parameters
        self.eta = None
        self.delta_x = None
        self.actions = [-5, 0, 5]
        self.action = 1
        self.control_state = False
        self.twist = Twist()
        self.last_angle = None
        self.last_distance = None
        self.first_measure = True
        self.angle_variance = 16
        self.distance_variance = 15

        # Q network, variables
        #self.memory_size = 5000
        
        self.sum_reward = 0
        self.time_step_counter = 0
        self.episode_counter = 0
        self.training_info = []
        self.model = load_model('mlp_controller.h5')
        #self.memory = deque(maxlen = self.memory_size)
        self.batch_size = 20
        # self.bridge = CvBridge()

        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(10)

        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions
        # self.twist.angular.z = 10 * data.axes[0]
        if data.axes[0] == -1:
            self.action = self.actions[0]

        elif data.axes[0] == 1:
            self.action = self.actions[2]
        else:
            self.action = self.actions[1]

        self.twist.angular.z = self.action
        # Throttle Commands, while button 7 is pressed -> Control
        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.control_state = False

        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.control_state = True
            self.episode_counter = self.episode_counter + 1

        elif data.buttons[0] == 1:
            self.training_info = self.training_info + [[self.episode_counter, self.time_step_counter, self.sum_reward]]
            self.first_measure = True
            self.sum_reward = 0
            self.time_step_counter = 0

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.control_state = False
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.control_state = False
        else:
            self.control_state = False

    def get_state_values(self, img):
        delta_x = None
        eta = None

        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interest points of the road
        points = get_lane_points(img, thresh=200)
        points = list(filter(lambda x: x, points))
        # Classifying the points in 3 line groups
        g1, g2, g3 = clustering_points(points)

        if self.first_measure:
            left_line, central_line, right_line = get_lines_labels(g1, g2, g3)

            eta, delta_x = get_heading_angle_lateral_deviation(central_line)

            if eta != None and delta_x != None:
                if abs(delta_x) > 33:
                    return None, None
                eta = eta * (180 / np.pi)

                # self.first_measure = False
                self.last_distance = delta_x
                self.last_angle = eta
                return eta, delta_x
            else:
                return None, None
        else:

            eta1, delta_x1 = get_heading_angle_lateral_deviation(g1)
            eta2, delta_x2 = get_heading_angle_lateral_deviation(g2)

            if delta_x1 and delta_x2:
                eta1 = eta1 * (180 / np.pi)
                g1_values = np.array([delta_x1 + 43, delta_x1, delta_x1 - 43])
                g1_values_dif = abs(g1_values - self.last_distance)
                min_g1 = g1_values[int(np.argmin(g1_values_dif))]

                eta2 = eta2 * (180 / np.pi)
                g2_values = np.array([delta_x2 + 43, delta_x2, delta_x2 - 43])
                g2_values_dif = abs(g2_values - self.last_distance)
                min_g2 = g2_values[int(np.argmin(g2_values_dif))]

                if np.min(g1_values_dif) <= self.distance_variance and eta1 <= self.angle_variance:
                    delta_x = min_g1
                    eta = eta1
                    self.last_distance = delta_x
                    self.last_angle = eta

                elif np.min(g2_values_dif) <= self.distance_variance and eta2 <= self.angle_variance:
                    delta_x = min_g2
                    eta = eta2
                    self.last_distance = delta_x
                    self.last_angle = eta
                else:
                    delta_x = None
                    eta = None
            elif delta_x1:
                eta1 = eta1 * (180 / np.pi)
                g1_values = np.array([delta_x1 + 43, delta_x1, delta_x1 - 43])
                g1_values_dif = abs(g1_values - self.last_distance)
                min_g1 = g1_values[int(np.argmin(g1_values_dif))]

                if np.min(g1_values_dif) <= self.distance_variance and eta1 <= self.angle_variance:
                    delta_x = min_g1
                    eta = eta1
                    self.last_distance = delta_x
                    self.last_angle = eta
            elif delta_x2:
                eta2 = eta2 * (180 / np.pi)
                g2_values = np.array([delta_x2 + 43, delta_x2, delta_x2 - 43])
                g2_values_dif = abs(g2_values - self.last_distance)
                min_g2 = g2_values[int(np.argmin(g2_values_dif))]

                if np.min(g2_values_dif) <= self.distance_variance and eta2 <= self.angle_variance:
                    delta_x = min_g2
                    eta = eta2
                    self.last_distance = delta_x
                    self.last_angle = eta
            else:
                delta_x = None
                eta = None

            return eta, delta_x

        # Asign a reward given at a state

    def get_reward(self, state):

        if (state[0] <= -15) and (state[0] >= -30) and (state[1] >= -15) and (state[1] <= 15):
            return 1
        elif (state[0] <= -15) and (state[0] >= -30):
            return 0
        elif (state[0] > -15) and (state[0] <= 0):
            return -1
        elif (state[0] > 0) and (state[0] <= 30):
            return -2
        else:
            return -3


    def q_network_control(self):
        state = np.random.rand(4)
        self.model.predict(np.array([state]))
        last_state = []
        R = 0
        counter = 0
        batch = []

        while not rospy.is_shutdown():
            # this is for reduce the speed of the car
            if self.control_state:
                if counter < 10:
                    self.twist.linear.x = 1.18
                    counter += 1
                elif counter < 12:
                    self.twist.linear.x = 0
                    counter += 1
                else:
                    counter = 0
                # Wait for a coherent pair of frames: color
                frames = self.pipeline.wait_for_frames()

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Feed forward 

                self.color_image = np.asanyarray(color_frame.get_data())
                self.eta, self.delta_x = self.get_state_values(self.color_image)

                if self.delta_x and self.eta:  # If state variables have a value:
                    self.time_step_counter = self.time_step_counter + 1
                    if self.first_measure:
                        current_state = [self.delta_x, self.eta, self.delta_x, self.eta]
	
                        current_action = self.model.predict(np.array([current_state]))
                        self.twist.angular.z = current_action
                        self.first_measure = False
                        last_state = current_state
                    else:
                        current_state = [self.delta_x, self.eta, last_state[0], last_state[1]]
                        next_action = self.model.predict(np.array([current_state]))

                        R = self.get_reward(current_state[0:2])
                        self.sum_reward += R
                        self.twist.angular.z = next_action
                    
                        last_state = current_state
                        current_action = next_action

                              
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()


        # Saving training info
        with open("testing_info_network.txt", "a") as training_info_file:
            training_info_file.write("\n")
            for i in self.training_info:
                training_info_file.write(str(i[0]) + "\t" + str(i[1]) + "\t" + str(i[2]) + "\n")
        training_info_file.close()


if __name__ == '__main__':
    try:
        rospy.init_node("q_network_control", anonymous=True)
        q_network_control = QnetworkControlNode()
        q_network_control.q_network_control()
    except rospy.ROSInterruptException:
        pass

# yalja@ipn.mx
