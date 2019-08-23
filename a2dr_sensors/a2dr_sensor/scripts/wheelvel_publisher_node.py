#!/usr/bin/env python
import serial
import rospy 
from KalmanFilter import KalmanFilter
from std_msgs.msg import Float32MultiArray

class serialInit(object):
	def __init__(self,port_,baudrate_):
		self.serial = serial.Serial(port=port_,baudrate=baudrate_,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE)
		self.buffy = [0,0,0,0,0,0]
		self.ID = 0x01
		self.lenght = 0x06
		
	def getPackage(self,len_package):
		self.buffy = []
		for i in range(0,len_package):
			self.buffy.append(ord(self.serial.read()))
		# print(self.buffy)

	def recieve(self):
		if self.serial.is_open  and self.serial.inWaiting():
			if ord(self.serial.read()) == 0xff:
				if ord(self.serial.read()) == 0xff:
					if ord(self.serial.read()) == self.ID:
						if ord(self.serial.read()) == 0x01:   #read_vel
							self.getPackage(ord(self.serial.read()))

	def close(self):
		self.serial.close()


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

if __name__ == '__main__':
	rospy.init_node('wheelvel_publisher_node')

	filter = FilterVel()
	ser = serialInit('/dev/ttyACM0',115200)
	rate = rospy.Rate(20)
	first_time = True
	while(not rospy.is_shutdown()):
		ser.recieve()
		wheelvel_publisher = rospy.Publisher('wheel_vel', Float32MultiArray, queue_size=10)
		# print(ser.buffy)
		vw_left = ((-1) ** (ser.buffy[0])) * (ser.buffy[1] | (ser.buffy[2] << 8))
		vw_right = ((-1) ** (ser.buffy[3])) * (ser.buffy[4] | (ser.buffy[5] << 8))
		
		print(vw_left, vw_right)
		if first_time == True:
			if abs(vw_left) == 65535 or abs(vw_right) == 65535:
				vw_left = 0.0
				vw_right = 0.0
				first_time = False 
		filter.updateVW_L(vw_left)
		filter.updateVW_R(vw_right)

		# filter.filter_vw_l = filter.filter_vw_l * 0.075/100
		# filter.filter_vw_r = filter.filter_vw_r * 0.075/100
		array = [filter.filter_vw_l, filter.filter_vw_r]
		print(array)
		wheel_velocity = Float32MultiArray(data=array)
		wheelvel_publisher.publish(wheel_velocity)
		rate.sleep()