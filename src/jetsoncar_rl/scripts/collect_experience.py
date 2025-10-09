#!/usr/bin/env python
import rospy
from lane_detection import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CollectExpNode(object):
    def __init__(self):
        # Parameters
       
        self.eta = None
        self.delta_x = None
        self.experience = []
        self.actions = [-10, -5, 0, 5, 10]
        self.action = 2
        self.vel_state = False
        self.twist = Twist()
        self.bridge = CvBridge()

    
        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(10)
    
        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    
        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):

        #Steering angle , transform joystick commands to actions
        if data.axes[0] <= -0.7:
            self.action = self.actions[0]
        elif data.axes[0] > -0.7 and data.axes[0] <= -0.3:
            self.action = self.actions[1]
        elif data.axes[0] > -0.3  and data.axes[0] <= 0.2:
            self.action = self.actions[2]
        elif data.axes[0] > 0.2 and data.axes[0] <= 0.6:
            self.action = self.actions[3]
        else:
            self.action = self.actions[4]

        self.twist.angular.z = self.action
        #Throttle Comands, while button 7 is pressed -> capture experience
        if data.buttons[6] == 1 and data.buttons[7] == 0:
    	    self.twist.linear.x = (5 * (data.axes[4] + 1) - 10)/8
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.vel_state = True  
               
        elif data.buttons[6] == 1 and data.buttons[7] == 1:
	    self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
	    self.twist.linear.x = (-5 * (data.axes[4]+ 1) + 10)/8

    def image_callback(self, data):
    
        central_group = []
    
        try:
            # Converting sensor_msgs.Image data type to cv2 image
            img = self.bridge.imgmsg_to_cv2(data, "brg8")
            # Getting interest points of the road
            points = get_lane_points(img, thresh = 200)
            points = list(filter(lambda x : x, points))
            #Classifying the points in 3 line groups
            g1, g2, g3 = clustering_points(points)
            # if at least two lines where detected, compute heading angle and lateral deviation
            if g1 and g2:
                central_group = get_central_line(g1, g2, g3)
	        if central_group:
	            self.eta, self.delta_x = get_heading_angle_lateral_deviation(central_group)
                    self.eta = self.eta * (180/np.pi)
                #If not posible to determine the central line
       	        else:
                    self.eta = None
                    self.delta_x = None
            
        except CvBridgeError as e:
            print e

    #Asign a reward given at a state
    def get_reward(self, delta_x, eta):
        if delta_x >= -2 and delta_x <= 2 and eta >= -4 and eta <= 4:
            return 1

        elif delta_x >= -2 and delta_x <= 2 and ((eta >= -8 and eta < -4) or (eta > 4 and eta <= 8)):
            return -1
    
        elif eta >= -15 and eta <= 15 and ((delta_x > 2 and delta_x <= 7) or (delta_x >= -7 and delta_x < -2)):
            return -2
        elif eta == None and delta_x == None:
            return None
        else:
            return -3
      
    	
    def get_experience(self):
   
        while not rospy.is_shutdown():
      
            if self.vel_state:

                self.twist.linear.x = 1.18
                self.experience = self.experience + [(self.delta_x, self.eta, self.actions.index(self.action), self.get_reward(self.delta_x, self.eta))]
            
           
            self.pub.publish(self.twist)
            self.rate.sleep()

        with open("experience.txt", "w") as exp_doc:
            for i in self.experience:
                for j in i:
                    exp_doc.write(str(j))
                    exp_doc.write('\t')
                exp_doc.write('\n')
        exp_doc.close()
   

if __name__ == '__main__':
    try:
        rospy.init_node("collect_experience", anonymous = True)
        collect_exp = CollectExpNode()
        collect_exp.get_experience()
    except rospy.ROSInterruptException: 
        pass
