#!/usr/bin/env python
from __future__ import division
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#this script stops the robot when there is an obstcle in front in the range (0,40) cm (-20, 20) degrees

#-----------------------------------------------------------------------------------------------------------#
#Global variables
distance = 0
inf = float("inf")


#Convert angles of the lidar to index ranges
def deg_range_to_index(check_min, check_max, angle_min, angle_max , step):
    min_index = 6279 * (math.radians(check_min) - angle_min) / (angle_max - angle_min) 
    max_index = 6279 * (math.radians(check_max) - angle_min) / (angle_max - angle_min)
         
    return int(min_index), int(max_index) 

def sweep_callback(msg, args):
    global distance
    idx1 = args[0]
    idx2 = args[1]
    dp = 0
    count = 0
    #idx1, idx2 = deg_range_to_index(-20, 20, msg.angle_min, msg.angle_max, msg.angle_increment)
    for i in range(idx1, idx2):
        if msg.ranges[i] != inf:
            count = count + 1
            dp = dp + msg.ranges[i]
    if count != 0 :
        distance = dp/count
    else:
        distance = inf     
    
    #measures = np.array(filter(lambda x: x != inf, msg.ranges[idx1, idx2]))
    #if len(measures) > 0:
    #    distance = measures.sum()/len(measures)
    #else:
    #    distance = inf
    



def start():
    global pub
    global twist
    twist = Twist()
    #Features of lidar sensor
    angle_min = -3.1400001049
    angle_max = 3.1400001049
    step_angle = 0.0010000000475
    idx1, idx2 = deg_range_to_index(-20, 20, angle_min, angle_max , step_angle)
    #Publisher for Arduino subscriber
    pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    #Suscrito a joystick para manipular la tortuga
    rospy.Subscriber('scan', LaserScan, sweep_callback, (idx1, idx2))
    rospy.init_node('lidar_stop', anonymous = True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if distance <= 0.4:
            twist.angular.z = 0
            twist.linear.x = 0
        else:
            twist.angular.z = 0
            twist.linear.x = 1.20
        
        pub.publish(twist)
        rate.sleep()    

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

