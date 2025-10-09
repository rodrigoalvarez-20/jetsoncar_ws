#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt32MultiArray
from sweeppy import *


def sweep_publisher():

    sweep_data = UInt32MultiArray()
    pub = rospy.Publisher('sweep_data', UInt32MultiArray, queue_size=10)
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
                sweep_data.data = [angle, distance]
		pub.publish(sweep_data)	
    
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
  
        
