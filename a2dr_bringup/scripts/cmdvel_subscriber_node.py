#!/usr/bin/env python

#Important Library
import rospy
from geometry_msgs.msg import Twist
import serial

class serialInit(object):
	def __init__(self,port_,baudrate_):
		self.serial = serial.Serial(port=port_,baudrate=baudrate_,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE)
		self.buffy = [0, 0, 0, 0, 0, 0]
		self.ID = 0x01
		self.dir_angular = 0 #0 := (+) , 1 := (-)
		self.angular_vel = 0
		self.dir_linear = 0 #0 := (+) , 1 := (-)
		self.linear_vel = 0
		self.package = [0,0,0,0,0,0,0,0]
		self.lenght = 0x04
		
	def resetPackage(self):
		self.dir_angular = 0 
		self.angular_vel = 0
		self.dir_linear = 0 
		self.linear_vel = 0
		self.package = []

	def send(self, linear_vel, angular_vel):
		
		#reset parameter
		self.resetPackage()
		# print(all_velocity,all_omega)
		# set parameter to send velocity of robot
		vel_msg = Twist()
		#Since we are moving just in x-axis
		vel_msg.linear.x = linear_vel
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = angular_vel
			
		# send package
		# rospy.loginfo(self.package)
		self.serial.write(vel_msg)

	def close(self):
		self.serial.close()


class teleop_subscriber(object):

	def __init__(self):
		# self.listener()
		self.linear_velocity_x = 0
		self.angular_velocity_z = 0

	
	def callback(self,data):
		self.linear_velocity_x = data.linear.x
		self.angular_velocity_z = data.angular.z

	# Function! Build Subscriber Node name Rasppi_node which subscribe Message type Twist on the Topic name cmd_vel 
	def listener(self):
		rospy.init_node('cmdvel_subscriber_node')
		# rospy.Subscriber('cmd_vel', Twist, self.callback)
		rospy.Subscriber('/twist_mux/cmd_vel', Twist, self.callback)
		# rospy.Subscriber('nav_vel', Twist, self.callback)
		# rospy.Subscriber('key_vel', Twist, self.callback)

# Operate when run listener.py and run class teleop_subcriber
if __name__ == '__main__':
	# initial serial
	serial1 = serialInit('/dev/ttyACM0',115200)

	teleSub_node = teleop_subscriber()
	teleSub_node.listener()

	rate = rospy.Rate(10)
	veloOld = 0
	omeOld = 0
	while(not rospy.is_shutdown()):
		robotVelocity = teleSub_node.linear_velocity_x
		robotOmega = teleSub_node.angular_velocity_z
		
		print("Robot Velocity: {}, Old velocity: {}".format(robotVelocity, veloOld))
		print("Robot Omega: {}, Old omega: {}".format(robotOmega, omeOld))
		if robotVelocity != veloOld or robotOmega != omeOld:
			# print(robotVelocity, robotOmega)
			serial1.send(robotVelocity , robotOmega)
			print("send")
			# print(robotVelocity,veloOld, robotOmega, omeOld)	
		
		# print(serial1.package)
		veloOld = robotVelocity
		omeOld = robotOmega

		rate.sleep()

	serial1.close()