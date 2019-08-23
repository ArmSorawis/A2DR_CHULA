#!/usr/bin/env python

#Important Library
import rospy
from geometry_msgs.msg import Twist
from KalmanFilter import KalmanFilter
from std_msgs.msg import Float32MultiArray
import serial

class serialInit(object):
	def __init__(self,port_,baudrate_):
		self.serial = serial.Serial(port=port_,baudrate=baudrate_,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE)
		self.buffy = [0, 0, 0, 0, 0, 0]
		self.ID = 0x01
		self.write_inst = 0x01
		self.read_inst = 0x00
		self.dir_angular = 0 #0 := (+) , 1 := (-)
		self.angular_vel = 0
		self.dir_linear = 0 #0 := (+) , 1 := (-)
		self.linear_vel = 0
		self.write_package = [0,0,0,0,0,0,0,0]
		self.write_lenght = 0x04
		self.read_lenght = 0x06
		self.read_buffer = [0,0,0,0,0,0]

	def resetPackage(self):
		self.dir_angular = 0 
		self.angular_vel = 0
		self.dir_linear = 0 
		self.linear_vel = 0
		self.write_package = []

	def send_vel(self, linear_vel, angular_vel):
		
		#reset parameter
		self.resetPackage()
		# print(all_velocity,all_omega)
		# set parameter to send velocity of robot
		if linear_vel >= 0.0 :
			self.dir_linear = 0
		else :
			self.dir_linear = 1

		# setparameter to send omega of robot 
		if angular_vel >= 0.0 :
			self.dir_angular = 0
		else :
			self.dir_angular = 1

		# print(self.forword_velocity, self.backword_velocity, self.left_omega,self.right_omega)
		self.linear_vel = int(abs(linear_vel) * 100.0)
		self.angular_vel = int(abs(angular_vel) * 100.0)

		# print(self.dir_linear, self.dir_angular, self.linear_vel, self.angular_vel)
		# if self.forword_velocity == 0 and self.backword_velocity == 0 and self.left_omega == 0 and self.right_omega = 0:
			# self.package = [0xFF, 0xFF, self.ID, self.lenght, self.Stct_Write, self.forword_velocity, self.backword_velocity,self.left_omega,self.right_omega]

		self.write_package = [0xFF, 0xFF, self.ID, self.write_inst, self.write_lenght, self.dir_linear, self.linear_vel, self.dir_angular, self.angular_vel]
						
		# send package
		# rospy.loginfo(self.write_package)
		self.serial.write(self.write_package)

	def close(self):
		self.serial.close()

	def getPackage(self,len_package):
		self.read_buffer = []
		for i in range(0,len_package):
			self.read_buffer.append(ord(self.serial.read()))

	def read_vel(self):
		self.write_package = [0xFF, 0xFF, self.ID, self.read_inst]
		self.serial.write(self.write_package)
	
	def receive(self):
		if self.serial.is_open  and self.serial.inWaiting():
			if ord(self.serial.read()) == 0xff:
				if ord(self.serial.read()) == 0xff:
					if ord(self.serial.read()) == self.ID:
						if ord(self.serial.read()) == 0x01:   #read_vel
							self.getPackage(ord(self.serial.read()))


class FilterVel(object):
	def __init__(self):
		self.filterL = KalmanFilter(1, 30)
		self.filterR = KalmanFilter(1, 30)

		self.filter_vw_r = 0.0
		self.filter_vw_l = 0.0
	
	def updateVW_L(self, v_l):
		self.filterL.input_latest_noisy_measurement(v_l)
		self.filter_vw_l = self.filterL.get_latest_estimated_measurement()

	def updateVW_R(self, v_r):
		self.filterR.input_latest_noisy_measurement(v_r)
		self.filter_vw_r = self.filterR.get_latest_estimated_measurement()


class a2dr_bringing(object):

	def __init__(self):
		# self.listener()
		self.linear_velocity_x = 0
		self.angular_velocity_z = 0

	
	def callback(self,data):
		self.linear_velocity_x = data.linear.x
		self.angular_velocity_z = data.angular.z

	# Function! Build Subscriber Node name Rasppi_node which subscribe Message type Twist on the Topic name cmd_vel 
	def listener(self):
		rospy.init_node('a2dr_bringing_node')
		rospy.Subscriber('cmd_vel', Twist, self.callback)
		self.pub_wheelvel = rospy.Publisher('wheel_vel', Float32MultiArray, queue_size=10)

# Operate when run listener.py and run class teleop_subcriber
if __name__ == '__main__':
	# initial serial
	serial1 = serialInit('/dev/ttyACM0',115200)
	a2dr_node = a2dr_bringing()
	a2dr_node.listener()

	filter = FilterVel()

	rate = rospy.Rate(20)

	veloOld = 0
	omeOld = 0

	while(not rospy.is_shutdown()):
		robotVelocity = a2dr_node.linear_velocity_x
		robotOmega = a2dr_node.angular_velocity_z
		
		# print("Robot Velocity: {}, Old velocity: {}".format(robotVelocity, veloOld))
		# print("Robot Omega: {}, Old omega: {}".format(robotOmega, omeOld))
		if robotVelocity != veloOld or robotOmega != omeOld:
			# print(robotVelocity, robotOmega)
			serial1.send_vel(robotVelocity , robotOmega)
			# print("send")
			# print(robotVelocity,veloOld, robotOmega, omeOld)	
		
		serial1.read_vel()
		serial1.receive()

		vw_left = ((-1) ** (serial1.read_buffer[0])) * (serial1.read_buffer[1] | (serial1.read_buffer[2] << 8))
		vw_right = ((-1) ** (serial1.read_buffer[3])) * (serial1.read_buffer[4] | (serial1.read_buffer[5] << 8))
		

		filter.updateVW_L(vw_left)
		filter.updateVW_R(vw_right)


		if robotVelocity == 0 and robotOmega == 0:
			filter.filter_vw_l = 0
			filter.filter_vw_r = 0

		array = [filter.filter_vw_l, filter.filter_vw_r]
		# print(serial1.read_buffer)
		print(array)
		wheel_velocity = Float32MultiArray(data=array)
		a2dr_node.pub_wheelvel.publish(wheel_velocity)

		veloOld = robotVelocity
		omeOld = robotOmega

		rate.sleep()

	serial1.close()