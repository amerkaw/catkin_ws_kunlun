#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import autominy_msgs.msg 
import sensor_msgs.point_cloud2
import tf.transformations
from std_msgs.msg import String
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand
from sensor_msgs.msg._LaserScan import LaserScan
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.msg._JointState import JointState
from geometry_msgs.msg._TransformStamped import TransformStamped
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from nav_msgs.msg._MapMetaData import MapMetaData
from std_msgs.msg._Header import Header

#cloud = None
points = []
header1 = None
header2 = None

def callback(data):
    global header1, header2
    header1 = Header(seq=data.header.seq , stamp=data.header.stamp , frame_id="laser") #data.header
    #header1.frame_id = "laser"
    header2 = Header(seq=data.header.seq , stamp=data.header.stamp , frame_id="lab") #data.header
    #header2.frame_id = "lab"
    
    global points
    points = []
    for i in range(len(data.ranges)):
        if data.ranges[i] != math.inf:
            angle = data.angle_min + (i * data.angle_increment)
            x = data.ranges[i] * math.cos(angle)
            y = data.ranges[i] * math.sin(angle)
            points.append([x,y,0])
    
def get_header1(data):
    global header1
    header1 = data.header
    
def get_header2(data):
    global header2
    header2 = data.header
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/sensors/rplidar/scan", LaserScan, callback)
    #rospy.Subscriber("/sensors/pointcloud/points_xyz", PointCloud2, get_header1)
    #rospy.Subscriber("/lab/joint_states", JointState, get_header2)
    
    pub1 = rospy.Publisher('/A1', PointCloud2, queue_size=10)
    pub2 = rospy.Publisher('/A2', PointCloud2, queue_size=10)
    pub3 = rospy.Publisher('/A3', OccupancyGrid, queue_size=10)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    global points
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if len(points) != 0 and header1 != None:
            
            cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(header1, points)
            pub1.publish(cloud)



            transform = tfBuffer.lookup_transform('lab', 'laser', rospy.Time(0), rospy.Duration(1.0))
            cords = np.array(list(sensor_msgs.point_cloud2.read_points(cloud)), dtype=np.float_)
            offset = [transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]
            rot = [transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]
            rot = tf.transformations.quaternion_matrix(rot)[0:-1, 0:-1]
            
            for i in range(len(cords)):
                cords[i] = np.array(np.dot(rot,cords[i]), dtype=np.float_)
                cords[i] = np.array(np.add(cords[i],offset), dtype=np.float_)
            cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(header2, cords)
            
            pub2.publish(cloud)
            
            
            
            cords = np.array(cords, dtype=np.int_)
            
            grid = OccupancyGrid()
            grid = OccupancyGrid(header=header2, info=MapMetaData(resolution=0.01, width=900, height=800), data=cords.reshape(-1).tolist())
            pub3.publish(grid)
            
            print(grid)

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
