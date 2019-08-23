#!/usr/bin/env python

# Important Library

from std_msgs.msg import Int32MultiArray
import rospy

class ultrasonic_arduino(object):

    def __init__(self):
        self.data = None

    def callback(self,data):
        self.data = data.data

    def listener(self):
        rospy.init_node('ultrasonic_publisher_node')
        rospy.Subscriber('depth_ultrasonic', Int32MultiArray, self.callback)


if __name__=="__main__":
    ultrasonicSub_node = ultrasonic_arduino()
    ultrasonicSub_node.listener()

    rate = rospy.Rate(50)

    while(not rospy.is_shutdown()):
        ultrasonic_data = ultrasonicSub_node.data
        rate.sleep()