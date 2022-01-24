#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import autominy_msgs.msg 
from std_msgs.msg import String
from autominy_msgs.msg._Speed import Speed
from nav_msgs.msg._Odometry import Odometry
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
    
speed = 0.0
dest = [0.0, 0.2, 0.5, 0.8, 0.2]
last_error = 0.0
last_pos = None
past = None
Buffer = 0.0
d_t = 0.1

in_time = 0.0
i = 0

time_arr = []
in_arr = []
soll_arr = []
ist_arr = []
start = None
    
def pid(val, act, K_p, K_i, K_d, Min, Max, d_t):
    global Buffer, last_error
    
    try:
        dest = val.value
    except:
        dest = val

    error = dest - act
    
    Buffer += error * d_t
    
    P = K_p * error
    I = K_i * Buffer
    D = K_d * (error - last_error) / d_t
        
    speed = P + I + D
            
    if speed > Max:
        speed = Max
    elif speed < Min:
        speed = Min
        
    last_error = error
        
    return speed
    
def get_odometry(data):
	
    global speed, last_pos, past, d_t, dest, in_time, i, time_arr, in_arr, soll_arr, ist_arr, start

    time = data.header.stamp
    pos = data.pose.pose.position
    error = 0.0
    
    if start == None:
        start = time.secs + time.nsecs / (10**9)
    
    if in_time <= 3.0:
        if last_pos != None and past != None:
            in_time += (time.secs - past.secs + (time.nsecs - past.nsecs) / (10**9))
    
            vel = [pos.x-last_pos.x, pos.y-last_pos.y, pos.z-last_pos.z]
            act = (np.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) / (time.secs - past.secs + (time.nsecs - past.nsecs) / (10**9)))
            
            speed = pid(dest[i], act, 9.139, 18.209, 1.186, 0.0, 1.5, d_t)
    
                
            time_arr.append(time.secs + time.nsecs / (10**9) - start)
            in_arr.append(speed)
            soll_arr.append(dest[i])
            ist_arr.append(np.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) / (time.secs - past.secs + (time.nsecs - past.nsecs) / (10**9)))
                
            print(' soll speed = ', dest[i], '\n  ist speed = ', np.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) / (time.secs - past.secs + (time.nsecs - past.nsecs) / (10**9)), '\ninput speed = ', speed, '\n')
        
    else:
        in_time = 0.0
        if i < len(dest)-1:
            i += 1
        else:
            i = 0
            plt.plot(time_arr, in_arr, 'r', time_arr, soll_arr, 'b', time_arr, ist_arr, 'g')
            plt.show()
     
    last_pos = pos
    past = time

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown():
        global speed
        pub_speed.publish(None,speed)
        pub_steering.publish(None,0.2)
    	
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
         
