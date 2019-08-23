#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class parallel_alignment():
	def __init__(self):
		rospy.init_node('planeAlignment_ultrasonic_node', disable_signals=True)

		self.rotate_round = 0
		self.omeOld = 0

		self.cmdvel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.listener()

	def listener(self):
		rospy.Subscriber('ultrasonic_depth', Int32MultiArray, self.callback)
		rospy.spin()

	def callback(self, ultrasonic_data):
		self.ultrasonic_data = ultrasonic_data.data

		ultrasonicData_leftIndex = self.ultrasonic_data[0]
		ultrasonicData_rightIndex = self.ultrasonic_data[1]
		d = ultrasonicData_leftIndex - ultrasonicData_rightIndex
		rospy.loginfo(d)

		self.alignment_mode(d)

	def alignment_mode(self, diff):

		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0

		if diff < -2:
			self.twist_robot.angular.z = 0.13
			if self.twist_robot.angular.z != self.omeOld :
				self.rotate_round = self.rotate_round + 1
			self.cmdvel_publisher.publish(self.twist_robot)
			self.omeOld = self.twist_robot.angular.z

		if diff > 2:
			self.twist_robot.angular.z = -0.13
			if self.twist_robot.angular.z != self.omeOld :
				self.rotate_round = self.rotate_round + 1
			self.cmdvel_publisher.publish(self.twist_robot)
			self.omeOld = self.twist_robot.angular.z

		if diff <= 1 and diff >= -1:
			print("stop")
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')

		if self.rotate_round >= 3:
			print("stop")
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')
	
if __name__ == '__main__':
	process = parallel_alignment()