#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import autominy_msgs.msg 
import tf.transformations
from std_msgs.msg import String
from autominy_msgs.msg._Speed import Speed
from nav_msgs.msg._Odometry import Odometry
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
    
steering = 0.0
dest = np.pi
last_error = 0.0
last_pos = None
past = None
Buffer = 0.0
d_t = 0.1

in_time = 0.0
i = 3

time_arr = []
in_arr = []
soll_arr = []
ist_arr = []
start = None
    
def get_odometry(data):
    rot = data.pose.pose.orientation
    #print(data.pose.pose.orientation,'\n')
    rotation = np.array([rot.x,rot.y,rot.z,rot.w], dtype=np.float_)
    #rotation = tf.transformations.quaternion_matrix(rotation)[0:-1, 0:-1]
    print(rotation,'\n')
    
    z  = np.arctan2(2.0*(rotation[0]*rotation[1] + rotation[2]*rotation[3]), rotation[3]**2 + rotation[0]**2 - rotation[1]**2 - rotation[2]**2);
    
    
    global steering, past, Buffer, last_error, d_t, dest, i, time_arr, in_arr, soll_arr, ist_arr, start

    time = data.header.stamp
    error = 0.0
    
    if start == None:
        start = time.secs + time.nsecs / (10**9)
    
    if past != None:
        
        if abs(dest - z) <= abs(-dest - z):
            error = dest - z
        else:
            error = -dest - z
    
        K_p = 100
        K_i = 0
        K_d = 0
        
        Min = -1.0
        Max = 1.0
    
        Buffer += error * d_t
    
        P = K_p * error
        I = K_i * Buffer
        D = K_d * (error - last_error) / d_t
        
        steering = P + I + D
            
                
        print(' soll steering = ', dest, '\n  ist steering = ', z, '\ninput steering = ', steering, '\n')
        
    else:
        error = 0.0
        
    last_error = error
    past = time
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown():
        global steering
        pub_speed.publish(None,0.1)
        pub_steering.publish(None,steering)
    	
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
