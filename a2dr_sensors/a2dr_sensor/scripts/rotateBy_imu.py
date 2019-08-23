#!/usr/bin/env python

# Important Library
import sys
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Twist
import rospy

import math

class send_commandvelocity():

	def __init__(self):

		self.target = rospy.get_param("~rotate_target", -179.0)

		# OFFSET
		if self.target > 0:
			self.target = self.target - 2
		elif self.target < 0:
			self.target = self.target + 2
  
		self.k_p = 0.1
		self.imu_yaw = 0
		
		self.setIMU = True
		self.robot_theta = 0
		self.first_theta = 0

		# Setting data of Linear Velocity of twist and Angular Velocity of twist
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

		self.cmdvel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	
	def get_imuData(self, imu_data):

		self.imu_data = imu_data

		accuracy = self.imu_data[0]
		imuData_leftIndex = self.imu_data[1]
		imuData_rightIndex = self.imu_data[2]
		
		# print([accuracy, imuData_leftIndex, imuData_rightIndex])

		imu_theta = -imuData_leftIndex - imuData_rightIndex + 180 

		if self.setIMU == True:
			self.first_theta = imu_theta
			self.setIMU = False

		if imuData_leftIndex != 0:
			diff = imu_theta - self.first_theta
			self.robot_theta += diff
			self.first_theta = imu_theta
		
		elif imuData_rightIndex != 0:
			diff = self.first_theta - imu_theta 
			self.robot_theta += diff
			self.first_theta = imu_theta
	
	def filter_theta(self, robot_theta):
		if robot_theta <= -180:
			self.robot_theta = robot_theta + 360
		
		if robot_theta > 180:
			self.robot_theta = robot_theta - 360

	def send_commandvel(self, yaw):
		if self.target > 0 and self.target <= 179:
			self.state = "rotate_right"
			self.imu_yaw = yaw
			self.selecting_state()

		elif self.target < 0 and self.target >= -179:
			self.state = "rotate_left"
			self.imu_yaw = yaw
			self.selecting_state()

		elif self.target == "turn_around":
			self.state = "rotate_arround"
			self.imu_yaw = yaw
			self.selecting_state()

	def selecting_state(self):
		self.imu_yaw_rad = self.imu_yaw*(math.pi/180)

		target_rad = self.target*math.pi/180

		print(target_rad, self.imu_yaw_rad)

		if self.state == "rotate_right":
			if self.imu_yaw_rad < target_rad:
				# velocity = (self.k_p * (target_rad - self.imu_yaw_rad) * 0.25) / (target_rad * self.k_p)
				# if velocity < 0.15:
					# velocity = 0.15
				self.twist_robot.angular.z = 0.15
				self.cmdvel_publisher.publish(self.twist_robot)

			elif self.imu_yaw_rad >= target_rad:
				self.twist_robot.angular.z = 0
				self.cmdvel_publisher.publish(self.twist_robot)
				rospy.signal_shutdown('Quit')
				# sys.exit(1)
		
		if self.state == "rotate_left":
			target_rad = -target_rad
			self.imu_yaw_rad = -self.imu_yaw_rad
			if self.imu_yaw_rad < target_rad:
				# velocity = (self.k_p * (target_rad - self.imu_yaw_rad) * 0.25) / (target_rad * self.k_p)
				# if velocity < 0.15:
					# velocity = 0.15
				self.twist_robot.angular.z = -0.15 
				self.cmdvel_publisher.publish(self.twist_robot)

			elif self.imu_yaw_rad >= target_rad:
				self.twist_robot.angular.z = 0
				self.cmdvel_publisher.publish(self.twist_robot)
				rospy.signal_shutdown('Quit')
				# sys.exit(1)
		
		if  self.state == "rotate_around":

			self.twist_robot.angular.z = 0.15 
			self.cmdvel_publisher.publish(self.twist_robot)
		   
		   
class imu_subscriber(object):

	def __init__(self):
		self.data = None

	def callback(self,data):
		self.data = data.data
		# print(self.data)

	def listener(self):
		rospy.init_node('rotateBy_imu_node', disable_signals=True)
		rospy.Subscriber('angle_rotate', Int32MultiArray, self.callback)


if __name__=="__main__":
	imuSub_node = imu_subscriber()
	imuSub_node.listener()

	sendCmdvel_node = send_commandvelocity()

	rate = rospy.Rate(20)

	while(not rospy.is_shutdown()):
		imu_data = imuSub_node.data
		if imu_data != None:
			# sendCmdvel_node.send_commandvel(imu_data)
			sendCmdvel_node.get_imuData(imu_data)
			sendCmdvel_node.filter_theta(sendCmdvel_node.robot_theta)
			print(sendCmdvel_node.robot_theta)
			sendCmdvel_node.send_commandvel(sendCmdvel_node.robot_theta)
		rate.sleep()