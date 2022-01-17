#!/usr/bin/env python

import numpy as np
import rospy
import autominy_msgs.msg 
import tf.transformations
import sensor_msgs.point_cloud2
from std_msgs.msg import String
from autominy_msgs.msg._SteeringFeedback import SteeringFeedback
from nav_msgs.msg._Odometry import Odometry
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
from sensor_msgs.msg._PointCloud2 import PointCloud2

dist = 0
steering = 1
last = None
line1 = None
line2 = None

helper = 50
header = None
points = []
switch = True

ster = 0

def get_steering(data):
    global ster
    ster = data.value
    return
    
def get_odometry(data):
    #print(data)
    pos = data.pose.pose.position
    rot = data.pose.pose.orientation
    #print(pos)
    
    global dist, steering, last, line1, line2, switch, helper, ster
    
    if steering >= -1:
        if dist < 2:
            if last != None:
                n = np.sqrt(abs(last.x - pos.x)**2 + abs(last.y - pos.y)**2 + abs(last.z - pos.z)**2)
                dist += n
            last = pos
        else:
            steering -= 1
            dist = 0
            
    position = np.array([pos.x,pos.y,pos.z], dtype=np.float_)
    rotation = np.array([rot.x,rot.y,rot.z,rot.w], dtype=np.float_)
    rotation = tf.transformations.quaternion_matrix(rotation)[0:-1, 0:-1]
    transform = np.array([[0,1,0]], dtype=np.float_)
    for t in range(len(transform)):
        transform[t] = np.array(np.dot(rotation,transform[t]), dtype=np.float_)
        transform[t] = np.array(np.add(transform[t],position), dtype=np.float_)
        #print(transform[t])
    
    if helper == 50:
        if switch:
            line1 = [position, transform[0]-position]
        else:
            line2 = [position, transform[0]-position]
        switch = not switch
        helper = 0
    
    if line1 != None and line2 != None:
        t1 = (line2[0][0]-line1[0][0])/line1[1][0]
        t2 = line2[1][0]/line1[1][0]
    
        s1 = line1[0][1] + line1[1][1] * t1
        s2 = line1[1][1] * t2
    
        s = (line2[0][1]-s1) / (s2-line2[1][1])
        t = t2 * s + t1
    
        sp = (line2[0] + s * line2[1]) #+ (-0.55,-3.5,0)
        sp2 = line1[0] + t * line1[1] 
    
        R = np.sqrt((pos.x-sp[0])**2+(pos.y-sp[1])**2+(pos.z-sp[2])**2)
        sig = np.arctan2(0.27,R)
    
        print('Steering = ', ster, '\nR   = ', R, '\nSig = ', sig, '\nPoint = ', sp, '\n')
    
        global header, points
        header = data.header
        points = [line1[0],line1[0]+line1[1],line2[0],line2[0]+line2[1],sp,sp2]
        
    helper += 1


def listener():
    rospy.init_node('listener', anonymous=True)

    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    pub_help = rospy.Publisher('/circle_center', PointCloud2, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    for i in range(30):
        pub_steering.publish(None,1)
        pub_speed.publish(None,0.3)
        rate.sleep()
    for i in range(70):
        pub_steering.publish(None,0)
        pub_speed.publish(None,0.3)
        rate.sleep()
        
    rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, get_steering)
    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    
    while not rospy.is_shutdown():
        global steering
        pub_steering.publish(None,steering)
        pub_speed.publish(None,0.3)
        
        global points, header
        if len(points) > 0:
            cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(header, points)
            pub_help.publish(cloud)
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
         
