#!/usr/bin/env python
from __future__ import division
import rospy
import pyrealsense2 as rs
#from lane_detection import *
from estimate_motion_variables import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class CollectExpNode(object):
    def __init__(self):
        # camera Parameters
        self.frame_counter = -1
        self.episode = 3
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 60)
        self.color_image = np.zeros([180, 320, 3])
        self.img_shape = np.shape(self.color_image)
        self.roi = (90, 170, 0, self.img_shape[1])

        # Start streaming
        self.pipeline.start(self.config)
        # feature extraction parameters
        self.eta = None
        self.delta_x = None
        self.experience = []
        self.vel_state = False
        self.twist = Twist()
        self.last_left_point = []
        self.last_right_point = []
        self.first_measure = True
        # self.bridge = CvBridge()

        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(15)

        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)
	

    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions

        self.twist.angular.z = 5 * data.axes[0]

        # Throttle Comands, while button 7 is pressed -> capture experience

        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.vel_state = False
            self.first_measure = True
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.vel_state = True

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
        else:
            self.vel_state = False

    def get_state_values(self, img):
	max_dist = 25
        delta_x = None
        eta = None

        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interesting points of the road
        points, bin_img = get_lane_points(img, thresh=190, roi= self.roi)
        cv2.imwrite("images/ep" + str(self.episode) +"_frame" + str(self.frame_counter)+ ".jpg", bin_img)
        groups = get_groups(points, max_dist)

        if self.first_measure:

            left_group, dashed_group, right_group, p_left, p_right = classify_groups(self.first_measure, groups,
                                                                                     self.last_left_point,
                                                                                     self.last_right_point)
            if dashed_group:
                dashed_eq = polynomial_fitting(dashed_group)
                eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, self.roi)

            if eta and delta_x:
                self.first_measure = False
        else:
            left_group, dashed_group, right_group, p_left, p_right = classify_groups(self.first_measure, groups,
                                                                                     self.last_left_point,
                                                                                     self.last_right_point)
            if dashed_group:
                dashed_eq = polynomial_fitting(dashed_group)
                eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, self.roi)
            if eta and delta_x:
                eta = eta * 180 / np.pi

        return eta, delta_x, p_left, p_right

    def get_experience(self):

	counter = 0
	
	frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        while not rospy.is_shutdown():

            if self.vel_state:

                self.frame_counter += 1

                
                if counter < 8:
                    self.twist.linear.x = 1.18
                    counter += 1
                elif counter < 10:
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
                self.eta, self.delta_x, self.last_left_point, self.last_right_point = self.get_state_values(
                    self.color_image)
                # self.experience = self.experience + [(self.delta_x, self.eta, self.twist.angular.z)]
                self.experience.append((self.delta_x, self.eta, self.twist.angular.z, self.frame_counter))
                #cv2.imwrite('images/frame_'+ str(self.frame_counter) + '_' + str(self.episode) + '.jpg', self.gray_image)
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()

        with open("experiences/exp_ep" + str(self.episode) + ".txt", "w") as exp_doc:
            for i in self.experience:
                for j in i:
                    if isinstance(j, float):
                        exp_doc.write("%.8f" % j)
                    else:
                        exp_doc.write(str(j))
                    exp_doc.write('\t')
                exp_doc.write('\n')
        exp_doc.close()


if __name__ == '__main__':
    try:
        rospy.init_node("collect_experience", anonymous=True)
        collect_exp = CollectExpNode()
        collect_exp.get_experience()
    except rospy.ROSInterruptException:
        pass
