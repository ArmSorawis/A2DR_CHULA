#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class depth_alignment():
	def __init__(self):
		rospy.init_node('depthAlignment_ultrasonic_node', disable_signals=True)
		self.cmdvel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		self.k_p = 0.1
		self.target_distance = rospy.get_param("~distance_target", 65.0)

		self.listener()

	def listener(self):
		rospy.Subscriber('ultrasonic_depth', Float32, self.callback)
		rospy.spin()

	def callback(self, depth_data):
		self.depth_data = depth_data.data
		rospy.loginfo(self.depth_data)

		self.alignment_mode()

	def alignment_mode(self):

		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

		if self.depth_data > self.target_distance:
			velocity_x = (self.k_p * (self.depth_data - self.target_distance) * 0.25) / (self.target_distance * self.k_p)
			if velocity_x < 0.05:
				velocity_x = 0.05
			if velocity_x > 0.25:
				velocity_x = 0.25
			print(velocity_x)
			self.twist_robot.linear.x = velocity_x
			self.cmdvel_publisher.publish(self.twist_robot)
			self.omeOld = self.twist_robot.angular.z

		elif self.depth_data <= self.target_distance:
			print("stop")
			self.twist_robot.linear.x = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')

if __name__ == '__main__':
	process = depth_alignment()