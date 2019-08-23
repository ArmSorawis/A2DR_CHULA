#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float32MultiArray

class wheelvel_subscriber(object):

    def __init__(self):
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0

    def callback(self, data):
        self.left_wheel_velocity = data.data[0]
        self.right_wheel_velocity = data.data[1]

    def listener(self):
        rospy.Subscriber('wheel_vel', Float32MultiArray, self.callback)


if __name__ == '__main__':
	rospy.init_node('wheelvel_publisher_node', anonymous=True)

    wheelvel_publisher = rospy.Publisher('wheel_vel', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(20)
    
    wheelvel_node = wheelvel_subscriber()
    wheelvel_node.listener()

	while(not rospy.is_shutdown()):

        lw_vel = wheelvel_node.left_wheel_velocity
        rw_vel = wheelvel_node.right_wheel_velocity
		array = [lw_vel, rw_vel]

		wheel_velocity = Float32MultiArray(data=array)
		wheelvel_publisher.publish(wheel_velocity)
		rate.sleep()