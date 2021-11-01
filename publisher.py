#!/usr/bin/env python3
# license removed for brevity

import rospy
import autominy_msgs.msg
from autominy_msgs.msg._SpeedCommand import SpeedCommand
from autominy_msgs.msg._NormalizedSteeringCommand import NormalizedSteeringCommand

def talker():
    pub1 = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub1.publish(None,1.0)
        pub2.publish(None,0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass