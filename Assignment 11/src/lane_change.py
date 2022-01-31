#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt
import rospy
import autominy_msgs.msg 
import tf.transformations
from autominy_msgs.msg._Speed import Speed
from std_msgs.msg import String

def listener(num=0):
    rospy.init_node('listener', anonymous=True)

    pub_lane = rospy.Publisher('/lane_number', Speed, queue_size=10)
    
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown():
        pub_lane.publish(None,num)
        #print(num)
    	
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener(float(sys.argv[1]))
     except rospy.ROSInterruptException:
         pass
