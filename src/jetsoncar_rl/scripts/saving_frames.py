#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import pyrealsense2 as rs
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class CollectFramesNode(object):
    def __init__(self):
        # camera Parameters
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 60)
        self.color_image = np.zeros([180, 320])

        # Start streaming
        self.pipeline.start(self.config)
	self.action = 1
	self.actions = [-5, 0, 5]
   
        self.vel_state = False
	self.frame_counter = -1
        self.twist = Twist()
        #self.bridge = CvBridge()

    
        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(10)
    
        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    
        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        #rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):

        #Steering angle , transform joystick commands to actions
	self.twist.angular.z = 5 * data.axes[0]
        """if data.axes[0] == -1:
            self.action = self.actions[0]

        elif data.axes[0] == 1:
            self.action = self.actions[2]
        else:
            self.action = self.actions[1]"""

        #self.twist.angular.z = self.action
        #Throttle Comands, while button 7 is pressed -> capture experience
        if data.buttons[6] == 1 and data.buttons[7] == 0:
    	    self.twist.linear.x = (5 * (data.axes[4] + 1) - 10)/8
            self.vel_state = False
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.vel_state = True
	     
               
        elif data.buttons[6] == 1 and data.buttons[7] == 1:
	    self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
	    self.twist.linear.x = (-5 * (data.axes[4]+ 1) + 10)/8
            self.vel_state = False
        else:
            self.vel_state = False

     
      
    	
    def get_frames(self):

        
        
   
        while not rospy.is_shutdown():
            
            
      
            if self.vel_state:
		self.frame_counter = self.frame_counter + 1
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
       
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                self.color_image = np.asanyarray(color_frame.get_data())
                self.twist.linear.x = 1.18
                
          	cv2.imwrite('images2/frame'+str(self.frame_counter)+'.jpg', self.color_image) 
            
           
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()

   

if __name__ == '__main__':
    try:
        rospy.init_node("collect_frames", anonymous = True)
        collect_frames = CollectFramesNode()
        collect_frames.get_frames()
    except rospy.ROSInterruptException: 
        pass
