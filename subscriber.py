#!/usr/bin/env python3
import rospy
import autominy_msgs.msg 
from autominy_msgs.msg._Speed import Speed
from std_msgs.msg import String

def callback(data):
    print(data)
    
def listener():

  
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensors/speed", Speed, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()