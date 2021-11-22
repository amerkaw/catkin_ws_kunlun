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
    """print('k1 = ',data.D[0])
    print('k2 = ',data.D[1])
    print('t1 = ',data.D[2])
    print('t2 = ',data.D[3])
    print('k3 = ',data.D[4])
    print('')
    print('f1 = ',data.K[0])
    print('f2 = ',data.K[4])
    print('c1 = ',data.K[2])
    print('c2 = ',data.K[5])
    print('')"""
    #print(data.D)
    #print(data.K)
    var = data
    
def display(data):
    global var
    image = (CvBridge().imgmsg_to_cv2(data, "bgr8")) #108:247,190:520
    binary = cv2.threshold(image, 180, 255, cv2.THRESH_BINARY)[1]
    binary = cv2.rectangle(binary, (0,0), (image.shape[1],108), (0,0,0), -1)
    binary = cv2.rectangle(binary, (0,247), (image.shape[1],image.shape[0]), (0,0,0), -1)
    binary = cv2.rectangle(binary, (0,0), (190,image.shape[0]), (0,0,0), -1)
    binary = cv2.rectangle(binary, (520,0), (image.shape[1],image.shape[0]), (0,0,0), -1)
    
    """binary = cv2.rectangle(binary, (190,108), (300,130), (255,255,255), 1)
    binary = cv2.rectangle(binary, (190,130), (300,200), (255,255,255), 1)
    binary = cv2.rectangle(binary, (190,200), (300,247), (255,255,255), 1)
    binary = cv2.rectangle(binary, (300,108), (520,130), (255,255,255), 1)
    binary = cv2.rectangle(binary, (300,130), (520,200), (255,255,255), 1)
    binary = cv2.rectangle(binary, (300,200), (520,247), (255,255,255), 1)"""
    #binary = binary[99:249,180:540]
    
    centre = np.array(get_centre(binary), dtype="double")
    
    if var != [] and centre.size > 0:
        success, rotation_vector, translation_vector = extrinsic(np.array(var.D, dtype="double").reshape((5,1)), np.array(var.K, dtype="double").reshape((3,3)), centre) #np.array([[var.K[0],var.K[1],var.K[2]],[var.K[3],var.K[4],var.K[5]],[var.K[6],var.K[7],var.K[8]]], dtype="double")
        print('rotation vector:\n', rotation_vector,'\n\ntranslation vector:\n',translation_vector,'\n')
        rot, _ = cv2.Rodrigues(rotation_vector)
        print('rotation matrix:\n', rot, '\n')
 
        nose_end_point2D, jacobian = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, np.array(var.K, dtype="double").reshape((3,3)), np.array(var.D, dtype="double").reshape((5,1)))
        n = [0,0,0]
        for p in centre:
            cv2.circle(image, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
            n[0] += 1
            n[1] += p[0]
            n[2] += p[1]
        n = (int(round(n[1]/n[0])),int(round(n[2]/n[0])))
        point1 = (n[0], n[1])
        point2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
        cv2.line(image, point1, point2, (255,255,255), 2)
    
    cv2.imshow('Can you see it?', image);
    cv2.waitKey(3)
    
    
def get_centre(image):
    #cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]]) → retval, rvec, tvec
    
    centres = []
    x = [190,300,520]
    y = [108,130,200,247]
    for g in range(3):
        for h in range(2):
            n = [0,0,0]
            for i in range(y[g],y[g+1]):
    	        for j in range(x[h],x[h+1]):
    	            #print(image[i,j])
    	            if image[i,j,0] == 255:
    	                n[0] += 1
    	                n[1] += j
    	                n[2] += i
            if n[0] != 0:
                centres.append((round(n[1]/n[0]),round(n[2]/n[0])))
            else:
                return []
    #print(centres)    
    return(centres)
    
    
def extrinsic(arr1, arr2, arr3):
    #arr4 = np.array([(50,0,20),(50,0,-20),(80,0,20),(80,0,-20),(110,0,20),(110,0,-20)], dtype="double")
    #arr4 = np.array([(110,20,0),(110,-20,0),(80,20,0),(80,-20,0),(50,20,0),(50,-20,0)], dtype="double")
    arr4 = np.array([(-20,110,0),(20,110,0),(-20,80,0),(20,80,0),(-20,50,0),(20,50,0)], dtype="double")
    #print(arr1,'\n\n',arr2,'\n\n',arr3,'\n\n',arr4,'\n')
    return(cv2.solvePnP(arr4, arr3, arr2, arr1))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, callback)
    rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, display)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
