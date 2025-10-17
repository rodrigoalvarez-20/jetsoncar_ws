#!/usr/bin/env python
import rospy
import sys
import cv2
#import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

#pipeline = rs.pipeline()
#config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 240, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 10)

vel_state = False

def callback(data):
    global vel_state 
    vel_state = False
    twist.angular.z = 10*data.axes[0]
    if data.buttons[6] == 1 and data.buttons[7] == 0:
    	twist.linear.x = (5 * (data.axes[4] + 1) - 10)/8
    elif data.buttons[6] == 0 and data.buttons[7] == 1:
        vel_state = True
               
    elif data.buttons[6] == 1 and data.buttons[7] == 1:
	twist.linear.x = 5 * (data.axes[4] + 1) - 10
    elif data.buttons[6] == 0 and data.buttons[7] == 0:
	twist.linear.x = (-5 * (data.axes[4]+ 1) + 10)/8
	
    


def start():
    global pub
    global twist
    twist = Twist()  
    #Publisher for Arduino subscriber
    pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('joy_capture', anonymous = True)
    rate = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
	if vel_state:
            if counter < 10:
                twist.linear.x = 1.18
                counter = counter + 1
            else:
                counter = 0
                twist.linear.x = 0
             
            
        pub.publish(twist)
        rate.sleep()    

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

