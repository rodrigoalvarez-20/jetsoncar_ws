#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt32
from std_msgs.msg import UInt16
from sweeppy import *


def sweep_publisher():
    angle = 0
    distance = 0
    pub1 = rospy.Publisher('sweep_angle', UInt32, queue_size=10)
    pub2 = rospy.Publisher('sweep_distance', UInt16, queue_size=10)
    rospy.init_node('sweep_publisher', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    n=0 
    with Sweep('/dev/ttyUSB1') as sweep:
        sweep.start_scanning()  
        for scan in sweep.get_scans():
            if rospy.is_shutdown() == True:
                sweep.stop_scanning()
                break

            size = len(scan[0])
            for i in range(size):
                angle = scan[0][i].angle 
	        distance = scan[0][i].distance
                pub1.publish(angle)
                pub2. publish(distance)
    
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(n)
        #pub1.publish(n)
        #rate.sleep()
	#n=n+1
if __name__ == '__main__':
    try:
        sweep_publisher()
    except rospy.ROSInterruptException:
        pass
  
        
