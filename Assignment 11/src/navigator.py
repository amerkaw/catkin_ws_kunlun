#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import autominy_msgs.msg 
import tf.transformations
from std_msgs.msg import String
from autominy_msgs.msg._Speed import Speed
from nav_msgs.msg._Odometry import Odometry
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
#from map import Map

from importlib.machinery import SourceFileLoader
map = SourceFileLoader("map","/home/fabio/autominy/catkin_ws/src/assignment11/src/map.py").load_module()

myMap = map.Map()
speed = 0.3
steering = 0.0
lane_index = 0
map_info = None

last_error = 0.0
Buffer = 0.0

def get_map(data):
    map.MapVisualization()
    global map_info
    map_info = data

def get_lane(data):
    global lane_index
    lane_index = int(data.value)

def get_odometry(data):
    global map, speed, steering, lane_index, last_error, Buffer
    pos = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
    rot = rot = data.pose.pose.orientation
    #lane_index = 0
    lookahead_distance = 0.7
    
    rotation = np.array([rot.x,rot.y,rot.z,rot.w], dtype=np.float_)
    rotation = tf.transformations.quaternion_matrix(rotation)[0:-1, 0:-1]
    front_point = np.array([lookahead_distance,0,0], dtype=np.float_)
    front_point = np.array(np.dot(rotation,front_point), dtype=np.float_)[0:-1]
    front_point = np.array(np.add(front_point,pos), dtype=np.float_)
    
    p, _ = myMap.lanes[lane_index].lookahead_point(pos, lookahead_distance)
    p = p[0]
    dist = np.sqrt((p[0]-front_point[0])**2 + (p[1]-front_point[1])**2)
    
    side_point = np.array([0,lookahead_distance,0], dtype=np.float_)
    side_point = np.array(np.dot(rotation,side_point), dtype=np.float_)[0:-1]
    side_point = np.array(np.add(side_point,pos), dtype=np.float_)
    
    dist_s = np.sqrt((p[0]-side_point[0])**2 + (p[1]-side_point[1])**2)
    
    if dist_s < np.pi/3:
        error = np.arctan2(lookahead_distance, dist)
    else:
        error = -np.arctan2(lookahead_distance, dist)
    
    K_p = 4
    K_i = 0.1
    K_d = 2
    
    Buffer += error * 0.1
    
    P = K_p * error
    I = K_i * Buffer
    D = K_d * (error - last_error) / 0.1
    
    Min = -1.0
    Max = 1.0
        
    steering = P + I + D
    
    if steering > Max:
        steering = Max
    elif steering < Min:
        steering = Min
        
    last_error = error
    
    print(front_point, '\n', p, '\n', dist, '\n', error, '\n')

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/simulation/odom_ground_truth", Odometry, get_odometry)
    rospy.Subscriber("/lane_number", Speed, get_lane)
    rospy.Subscriber('/sensors/map', OccupancyGrid, get_map)
    
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_map = rospy.Publisher('/sensors/map', OccupancyGrid, queue_size=10)
    
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown():
        global speed, steering, myMap, map_info
        pub_speed.publish(None,speed)
        pub_steering.publish(None,steering)
        #if map_info != None:
            #pub_map.publish(map_info.header,map_info.info,myMap)
        #map.MapVisualization()
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
         
