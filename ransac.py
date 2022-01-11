#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
import autominy_msgs.msg 
#from autominy_msgs.msg import Speed
#from autominy_msgs.msg._Speed import Speed
from sensor_msgs.msg._CameraInfo import CameraInfo
from sensor_msgs.msg._Image import Image
from std_msgs.msg import String

var = []

def callback(data):
    global var

    #print(data.D)
    #print(data.K)
    var = data
    
def display(data):
    image = (CvBridge().imgmsg_to_cv2(data, "bgr8"))
    #image = cv2.resize(image, (int(image.shape[1]/2), int(image.shape[0]/2)))
    binary = cv2.threshold(image, 220, 255, cv2.THRESH_BINARY)[1]
    binary = cv2.rectangle(binary, (320,270), (380,320), (0,0,0), -1)
    binary = cv2.rectangle(binary, (0,0), (image.shape[1],145), (0,0,0), -1)
    binary = cv2.drawContours(binary, [np.array( [(400,145),(image.shape[1],image.shape[0]-25),(image.shape[1],145)] )], 0, (0,0,0), -1)
    #binary = cv2.drawContours(binary, [np.array( [(0,145),(0,250),(150,145)] )], 0, (0,0,0), -1)
    #image = binary.copy()
    
    #cv2.imshow('Can you see it?', binary);
    #cv2.waitKey(3)
    
    points = []
    lanes = []
    
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if binary[i,j,0] == 255:
                points.append((j,i))
                
    iterations = 20
    
    for r in range(iterations):
        i, j = 0, 0
        while points[i][0] == points[j][0]:
            i, j = np.random.randint(0,len(points)), np.random.randint(0,len(points))
        m = (points[j][1]-points[i][1]) / (points[j][0]-points[i][0]) + 0.00001
        b = (points[j][1]-m*points[j][0])
        lanes.append([m,b,2])
        for k in range(len(points)):
            if points[k][0] != points[i][0] and points[k][0] != points[j][0]:
                n = points[k][1] + 1/m * points[k][0]
                x = (n-b) / (m + 1/m)
                y = -1/m * x + n
                if abs(np.sqrt((points[k][0]-x)**2+(points[k][1]-y)**2)) <= 1.0:
                    lanes[-1][2] = lanes[-1][2]+1

    """for i in range(iterations):
        i, j = np.random.randint(0,len(points)), np.random.randint(0,len(points))
        while i != j:
            lanes.append([points[i],points[j],2])
            for k in range(len(points)):
                if k!=i and k!=j and ( abs( (points[j][0]-points[i][0])*(points[i][1]-points[k][1])-(points[i][0]-points[k][0])*(points[j][1]-points[i][1]) ) / np.sqrt( (points[j][0]-points[i][0])**2+(points[j][1]-points[i][1])**2 ) ) <= 1.0:
                    lanes[-1][2] = lanes[-1][2]+1
            break"""
    
    """for i in range(len(points)):
        for j in range(i+1,len(points)):
            lanes.append([points[i],points[j],2])
            for k in range(len(points)):
                if k!=i and k!=j and ( abs( (points[j][0]-points[i][0])*(points[i][1]-points[k][1])-(points[i][0]-points[k][0])*(points[j][1]-points[i][1]) ) / np.sqrt( (points[j][0]-points[i][0])**2+(points[j][1]-points[i][1])**2 ) ) <= 1.0:
                    lanes[-1][2] = lanes[-1][2]+1"""
                    
    st, nd, rd = (None,None,0), (None,None,0), (None,None,0)
    for lane in lanes:
        if lane[2] > st[2]:
            st = lane
        elif lane[2] > nd[2]:
            nd = lane
        elif lane[2] > rd[2]:
            rd = lane
            
    print(st, '\n', nd, '\n', rd, '\n')
    cv2.line(image, (0,int(st[1])), (image.shape[1], int(st[0]*image.shape[1] + st[1])), (0,0,255), 2)
    cv2.line(image, (0,int(nd[1])), (image.shape[1], int(nd[0]*image.shape[1] + nd[1])), (0,0,255), 2)
    cv2.line(image, (0,int(rd[1])), (image.shape[1], int(rd[0]*image.shape[1] + rd[1])), (0,0,255), 2)
    
    """cv2.line(image, st[0], st[1], (0,0,255), 2)
    cv2.line(image, nd[0], nd[1], (0,0,255), 2)
    cv2.line(image, rd[0], rd[1], (0,0,255), 2)"""

    #point1 = (n[0], n[1])
    #point2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
    #cv2.line(image, points[i], points[j], (255,255,255), 2)
    
    #image = cv2.resize(image, (int(image.shape[1]*2), int(image.shape[0]*2)))
    cv2.imshow('Can you see it?', image);
    cv2.waitKey(3)


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, callback)
    rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, display)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
    
    
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     try:
         listener()
     except rospy.ROSInterruptException:
         pass
