#!/usr/bin/env python

# Important Library

from std_msgs.msg import Int32MultiArray
import rospy

class imuTheta_odomTheta():
	def __init__(self):
		self.robot_theta = 0
		self.setIMU = True
		self.filter_theta = 0 

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


class imu_arduino(object):

	def __init__(self):
		self.imu_data = None

	def callback(self,data):
		self.imu_data = data.data

	def listener(self):
		rospy.init_node('imu_publisher_node', anonymous=True)
		rospy.Subscriber('imu2odometry', Int32MultiArray, self.callback)

if __name__=="__main__":
	imuSub_node = imu_arduino()
	imuSub_node.listener()

	process = imuTheta_odomTheta()
	rate = rospy.Rate(50)

	while(not rospy.is_shutdown()):
		imu_data = imuSub_node.imu_data
		if imu_data != None:
			process.get_imuData(imu_data)
			print(process.robot_theta)
		# process.filter_theta(process.robot_theta)
		# process.compute_odometry_theta(process.robot_theta)
		rate.sleep()