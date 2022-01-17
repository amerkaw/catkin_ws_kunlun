#!/usr/bin/env python

import numpy as np
import rospy
import autominy_msgs.msg 
from std_msgs.msg import String
from autominy_msgs.msg._Speed import Speed
from nav_msgs.msg._Odometry import Odometry
from autominy_msgs.msg._SpeedCommand import SpeedCommand
    
values = []
speed = 0.3
    
def get_odometry(data):
    global values, speed
    values.append([data.header, data.pose.pose.position, 0.0])
    
    i = len(values)-1
    if i > 0:
        values[i][2] = np.sqrt((values[i][1].x-values[i-1][1].x)**2+(values[i][1].y-values[i-1][1].y)**2+(values[i][1].z-values[i-1][1].z)**2) / (values[i][0].stamp.secs - values[i-1][0].stamp.secs + (values[i][0].stamp.nsecs - values[i-1][0].stamp.nsecs) / (10**9))
    print('time = ', values[i][0].stamp.secs - values[0][0].stamp.secs + (values[i][0].stamp.nsecs - values[0][0].stamp.nsecs) / (10**9), '   speed = ', values[i][2])
    
    if values[i][2] >= speed:
        """for i in range(1, len(values)):
            print('time = ', values[i][0].stamp.nsecs-values[0][0].stamp.nsecs / 10**9, '   speed = ', values[i][2])
        return"""
    
    """for i in range(1, len(values)):
        values[i][3] = np.sqrt((values[i][1].x-values[i-1][1].x)**2+(values[i][1].y-values[i-1][1].y)**2+(values[i][1].z-values[i-1][1].z)**2) / (values[i][0].stamp.secs - values[0][0].stamp.secs)"""
    
def get_speed(data):
    return

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    #rospy.Subscriber("/sensors/speed", Speed, get_speed)
    
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        global speed
        pub_speed.publish(None,speed)
    	
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
         
