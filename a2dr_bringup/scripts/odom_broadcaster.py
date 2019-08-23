#!/usr/bin/env python

import rospy

import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String, Float32MultiArray

import math

class odometry_broadcaster(object):

	def __init__(self):

		self.baseFrame = rospy.get_param("~base_id", "base_footprint")
		self.odomFrame = rospy.get_param("~odom_id", "odom") 
 
		self.wheelSep = float(rospy.get_param("~wheel_separation", "0.342"))

		self.VL = 0 
		self.VR = 0

		self.x = 0.0
		self.y = 0.0 
		self.theta = 0.0

		self.v_rx = 0.0
		self.v_ry = 0.0
		self.vth = 0.0

		self.linear_velocity_x = 0 
		self.linear_velocity_y = 0
		self.angular_velocity_z = 0

		self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

		self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=10)

		self.odom_broadcaster = tf.TransformBroadcaster()

		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

	def robot_velocity_subscriber(self, VL, VR):
		self.VL = VL * 0.04
		self.VR = VR * 0.04
		# self.VL = VL
		# self.VR = VR
		VL_node = rospy.Publisher('left_vel', String, queue_size=10)
		VR_node = rospy.Publisher('right_vel', String, queue_size=10)
		VL_node.publish(str(VL))
		VR_node.publish(str(VR))

	# def robot_theta_subscriber(self, theta)
		# self.theta = thetapublishpublish

	def compute_odom(self):
		self.current_time = rospy.Time.now()

		dt = (self.current_time - self.last_time).to_sec()

		robot_linear_velocity = (self.VL + self.VR)/2
		robot_angular_velocity = (self.VR - self.VL)/self.wheelSep

		self.v_rx = robot_linear_velocity
		self.vth = robot_angular_velocity

		v_wx = self.v_rx * math.cos(self.theta) - self.v_ry * math.sin(self.theta)
		v_wy = self.v_rx * math.sin(self.theta) + self.v_ry * math.cos(self.theta)

		self.x += v_wx * dt
		self.y += v_wy * dt
		
		self.theta += self.vth * dt

		self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
		self.odom_broadcaster.sendTransform((self.x, self.y, 0.), self.odom_quat, self.current_time, self.baseFrame, self.odomFrame)

	def publish_odom(self):
		odom = Odometry()
		odom.header.stamp = self.current_time

		odom.header.frame_id = self.odomFrame
		odom.child_frame_id = self.baseFrame

		odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))
		odom.twist.twist = Twist(Vector3(self.v_rx, self.v_ry, 0), Vector3(0, 0, self.vth))

		self.odom_publisher.publish(odom)
		#print(odom)
		self.last_time = self.current_time


class wheelvel_subscriber(object):

	def __init__(self):
		self.left_wheel_velocity = 0.0
		self.right_wheel_velocity = 0.0

	def callback(self, data):
		self.left_wheel_velocity = data.linear.x #data.data[0]
		self.right_wheel_velocity = data.linear.y #data.data[1]
		#print(self.left_wheel_velocity)

	def listener(self):
		rospy.Subscriber('wheel_vel', Twist, self.callback)


# class imu_subsciber(object):

# 	def __init__(self):
# 		self.imu_data = None
	
# 	def callback(self, data):
# 		self.imu_data = data.data

# 	def listener(self):
# 		rospy.Subscriber("imu2odometry", Float32MultiArray,self.callback)


# class imuTheta_odomTheta():
	
# 	def __init__(self):
# 		self.setIMU = True
# 		self.robot_theta = 0
# 		self.filter_theta = 0 

# 	def get_imuData(self, imu_data):

# 		self.imu_data = imu_data

# 		accuracy = self.imu_data[0]
# 		imuData_leftIndex = self.imu_data[1]
# 		imuData_rightIndex = self.imu_data[2]

# 		imu_theta = -imuData_leftIndex - imuData_rightIndex + 180 

# 		if self.setIMU == True:
# 			self.first_theta = imu_theta
# 			self.setIMU = False

# 		if imuData_leftIndex != 0:
# 			diff = imu_theta - self.first_theta
# 			self.robot_theta += diff
# 			self.first_theta = imu_theta
		
# 		elif imuData_rightIndex != 0:
# 			diff = self.first_theta - imu_theta 
# 			self.robot_theta += diff
# 			self.first_theta = imu_theta


if __name__ == "__main__":
	rospy.init_node("odom_broadcaster_node", anonymous=True)

	wheelvel_node = wheelvel_subscriber()
	wheelvel_node.listener()

	# imu_node = imu_subsciber()
	# imu_node.listener()

	# imuTheta_odomTheta_node = imuTheta_odomTheta()
	odo = odometry_broadcaster()
	
	rate = rospy.Rate(15)

	while(not rospy.is_shutdown()):
		rate.sleep()

		lw_vel = wheelvel_node.left_wheel_velocity
		rw_vel = wheelvel_node.right_wheel_velocity
		#print(lw_vel, rw_vel)
		# imu_theta = imu_node.imu_data

		# if imu_theta != None:
			# imuTheta_odomTheta_node.get_imuData(imu_theta)
			# robot_theta = imuTheta_odomTheta_node.robot_theta
			# odo.robot_theta_subscriber(robot_theta)
		
		odo.robot_velocity_subscriber(lw_vel, rw_vel)

		odo.compute_odom()
		odo.publish_odom()