#!/usr/bin/env python

#Important Library
import rospy
from geometry_msgs.msg import Twist
import serial

class serialInit(object):
	def __init__(self,port_,baudrate_):
		self.serial = serial.Serial(port=port_,baudrate=baudrate_,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE)
		self.buffy = [0,0,0,0,0,0,0]
		self.Stct_Write = 0x03
		self.ID = 0x01
		self.right_omega = 0
		self.left_omega = 0
		self.forword_velocity = 0
		self.backword_velocity = 0
		self.package = [0,0,0,0,0,0,0,0,0]
		self.lenght = 0x05
		
	def resetPackage(self):
		self.right_omega = 0
		self.left_omega = 0
		self.forword_velocity = 0
		self.backword_velocity = 0
		self.package = []

	def send(self,all_velocity, all_omega):
		
		#reset parameter
		self.resetPackage()
		# print(all_velocity,all_omega)
		# set parameter to send velocity of robot
		if all_velocity >= 0.0 :
			self.forword_velocity = all_velocity
		elif all_velocity <= 0.0 :
			self.backword_velocity = abs(all_velocity)

		# setparameter to send omega of robot 
		if all_omega >= 0.0 :
			self.right_omega = all_omega
		elif all_omega < 0.0 :
			self.left_omega = abs(all_omega)

		# print(self.forword_velocity, self.backword_velocity, self.left_omega,self.right_omega)
		self.forword_velocity = int(self.forword_velocity * 100)
		self.backword_velocity = int(self.backword_velocity * 100)
		self.left_omega = int(self.left_omega * 100)
		self.right_omega = int(self.right_omega * 100)

		# if self.forword_velocity == 0 and self.backword_velocity == 0 and self.left_omega == 0 and self.right_omega = 0:
			# self.package = [0xFF, 0xFF, self.ID, self.lenght, self.Stct_Write, self.forword_velocity, self.backword_velocity,self.left_omega,self.right_omega]

		self.package = [0xFF, 0xFF, self.ID, self.lenght, self.Stct_Write, self.forword_velocity, self.backword_velocity,self.left_omega,self.right_omega]
						
		# send package
		rospy.loginfo(self.package)
		self.serial.write(self.package)

		
	def getPackage(self,len_package):
		self.buffy = []
		for i in range(0,len_package):
			self.buffy.append( ord(self.serial.read()) )
		#print(self.buffy)

	def recieve(self):
		if self.serial.is_open  and self.serial.inWaiting():
			if ord(self.serial.read()) == 0xff:
				if ord(self.serial.read()) == 0xff:
					if ord(self.serial.read()) == self.ID:
						self.getPackage(ord(self.serial.read()))

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
		rospy.Subscriber('cmd_vel', Twist, self.callback)

# Operate when run listener.py and run class teleop_subcriber
if __name__ == '__main__':
	# initial serial
	serial1 = serialInit('/dev/ttyBASE',115200)

	teleSub_node = teleop_subscriber()
	teleSub_node.listener()

	rate = rospy.Rate(5)
	veloOld = 0
	omeOld = 0
	while(not rospy.is_shutdown()):
		robotVelocity = teleSub_node.linear_velocity_x
		robotOmega = -teleSub_node.angular_velocity_z

		#send serial
		
		if robotVelocity != veloOld or robotOmega != omeOld :
			# print(robotVelocity, robotOmega)
			serial1.send(robotVelocity , robotOmega)
			# print(robotVelocity,veloOld, robotOmega, omeOld)	
			
		print(serial1.package)
		veloOld = robotVelocity
		omeOld = robotOmega

		rate.sleep()

	serial1.close()