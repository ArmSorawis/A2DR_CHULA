#!/usr/bin/env python

import rospy
import roslaunch

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String

import math
import numpy as np

class obstacle_detection():
	def __init__(self):
		rospy.init_node('obstacleDetection_lidar_node', disable_signals=True)
		self.cmdvel_publisher = rospy.Publisher('break_vel', Twist, queue_size=10)
		self.emer_publisher = rospy.Publisher('silent', String, queue_size=10)

		self.scan_range = 100 
		self.scan_fov = 40

		self.first_time = True

		self.listener()
	
	def find_interestAngle(self):
		anotherAngle = math.atan(self.scan_range/(self.scan_fov/2))
		anotherAngle = self.rad2deg(anotherAngle)
		self.interestAngle = 180 - (90 + anotherAngle)

	def listener(self):
		rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1000)
		rospy.spin()

	def callback(self, data):
		scan_time = data.scan_time
		scan_time_increment = data.time_increment
		count = scan_time / scan_time_increment

		angle_min = self.rad2deg(data.angle_min)
		angle_max = self.rad2deg(data.angle_max)
		angle_increment = self.rad2deg(data.angle_increment)
		ranges = data.ranges

		i = 0 
		obstacle_range = []
		obstacle_check = False
		 
		self.find_interestAngle()
		while i < count:
			degree = angle_min + (angle_increment * i)
			if degree >= -self.interestAngle and degree <= self.interestAngle:
				if ranges[i] != float("inf") and ranges[i] > 0.55:
					#rospy.loginfo(": [%f, %f]", degree, ranges[i])
					obstacle_check = False
				elif ranges[i] != float("inf") and ranges[i] < 0.55:
					#rospy.loginfo(": [%f, %f]", degree, ranges[i])
					obstacle_check = True
					break
			i = i + 1
		self.check_mode(obstacle_check)

	def check_mode(self, obstacle):
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		if obstacle == True:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			# self.cmdvel_publisher.publish(True)
			if self.first_time == True:
				self.emer_publisher.publish("True")
				self.first_time = False

		elif obstacle == False:
			# print("Continue moving")
			self.first_time = True
			# self.cmdvel_publisher.publish(False)
			# self.twist_robot.linear.x = 0.15
			# self.cmdvel_publisher.publish(self.twist_robot)
			pass

	def rad2deg(self, radians):
		pi = math.pi
		degrees = (180 * radians) / pi
		return degrees

if __name__ == '__main__':
	process = obstacle_detection()