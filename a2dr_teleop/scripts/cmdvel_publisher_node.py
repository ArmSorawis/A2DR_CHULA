#!/usr/bin/env python

#Important Library

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

#Introduction
msg = """Reading input from the keyboard and Publishing cmd_vel to Subsciber!
---------------------------------------------------
s : stop
w/x : increase/decrease linear speed by 10%
a/d : increase/decrease angular speed by 10%
CTRL-C to quit
"""
import time

class teleop_publisher():

	def __init__(self):
		print(msg)
		# Build Publisher Node name PC_node which publish the Topic name cmd_vel to sent Message Type Twist
		# Twist.linear ---> float64 x float64 y float64 z 
		# Twist.angular---> float64 x float64 y float64 z 
		rospy.init_node('cmdvel_publisher_node', anonymous=False)
		# self.pub_robot_velocity = rospy.Publisher('cmd_vel', Twist, queue_size = 1)  
		self.pub_robot_velocity = rospy.Publisher('key_vel', Twist, queue_size = 1)  
		# Init Linear Velocity and Angular velocity equal to 0
		self.linear_velocity = rospy.get_param("~linear_velocity", 0)
		self.angular_velocity = rospy.get_param("~angular_velocity", 0)

		# Show currently Velocity
		print(self.vels(self.linear_velocity,self.angular_velocity))
		
		# Dictionary! ----> {keys : function}
		self.options = {'w' : self.forward,
						'a' : self.rotateLeft,
						's' : self.stop,
						'd' : self.rotateRight,
						'x' : self.backward}

		# Loop in try for 10 round per second 
		rate = rospy.Rate(10) # 10hz
		self.step = 0.05 #m/s
		self.limit_linear = 0.30
		self.limit_angular = 0.75

		# Setting data of Linear Velocity of twist and Angular Velocity of twist 
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

		# Main loop
		self.main_loop(rate)
		

	# Fucntion! for show the currently Linear Velocity and currently Angular Velocity ########################
	def vels(self, linear_velocity, angular_velocity):
		return "currently:\tlinear velocity = %s\tangular velocity = %s" % (linear_velocity, angular_velocity)

	# Function! for Read key from Pushing keyboard ###############################################################
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

	def input_keyboard(self,key):
		# If user pushed w a d x s 
		if key in self.options.keys():
			print("\n")
			if key == 'w':
				self.options['w'](self.twist_robot.linear.x)

			elif key == 'x':
				self.options['x'](self.twist_robot.linear.x)

			elif key == 'a':
				self.options['a'](self.twist_robot.angular.z)

			elif key == 'd':
				self.options['d'](self.twist_robot.angular.z)

			elif key == 's':
				self.options['s'](self.twist_robot.linear.x,self.twist_robot.angular.z)

			print(self.vels(float(self.twist_robot.linear.x), float(self.twist_robot.angular.z)))

		# If user not pushed w a d x s but pused other button  
		if key not in self.options.keys() and key != '\x03':
			print('\n')
			print(self.vels(float(self.twist_robot.linear.x), float(self.twist_robot.angular.z)))
			print("Please push only 'w','a','s','d','x' button")

		# If user pushed ctrl+c break loop 
		else:
			if (key == '\x03'):
				sys.exit(1)

	def publish_velocity(self):
		# Output ---> twist.linear and twist.angular
		# Show Output 
		rospy.loginfo(self.twist_robot)
		# Publish Output
		self.pub_robot_velocity.publish(self.twist_robot)

	def main_loop(self, rate):
		# Main Loop until ros is shutdown 
		while not rospy.is_shutdown():
			# Get key which have been pushed on keyboard 
			key = self.getKey()
			self.input_keyboard(key)
			# Same as delay
			rate.sleep()

	# Function! for Increase and Decrease ---> Linear Velocity and Angular Velocity ##############################
	# Limit Max Velocity to 2.5 m/s
	def forward(self, linear_velocity):
		if linear_velocity < self.limit_linear:
			if linear_velocity == -0.05:
				linear_velocity = 0.0 
				linear_velocity = round(linear_velocity, 15)
			else:
				linear_velocity = linear_velocity + self.step
				linear_velocity = round(linear_velocity, 15)
		print("Increase linear velocity to {}.".format(linear_velocity))
		self.twist_robot.linear.x = linear_velocity
		self.publish_velocity()

	def backward(self, linear_velocity):
		if linear_velocity > -self.limit_linear:
			if linear_velocity == 0.05:
				linear_velocity = 0.0
				linear_velocity = round(linear_velocity, 15)
			else:     
				linear_velocity = linear_velocity - self.step
				linear_velocity = round(linear_velocity, 15)
		print("Decrease linear velocity to {}.".format(linear_velocity))
		self.twist_robot.linear.x = linear_velocity
		self.publish_velocity()
		
	def rotateRight(self, angular_velocity):
		if angular_velocity < self.limit_angular:      
			if angular_velocity == -0.05:     
				angular_velocity = 0.0 
				angular_velocity = round(angular_velocity, 15)
			else:
				angular_velocity = angular_velocity + self.step
				angular_velocity = round(angular_velocity, 15)
		print("Increase angular_velocity to {}.".format(angular_velocity))
		self.twist_robot.angular.z = angular_velocity
		self.publish_velocity()
		
	def rotateLeft(self, angular_velocity):
		if angular_velocity > -self.limit_angular:
			if angular_velocity == 0.05:
				angular_velocity = 0.0
				angular_velocity = round(angular_velocity, 15)
			else:     
				angular_velocity = angular_velocity - self.step
				angular_velocity = round(angular_velocity, 15)
		print("Decrease angular_velocity to {}.".format(angular_velocity))
		self.twist_robot.angular.z = angular_velocity
		self.publish_velocity()
		
	def stop(self, linear_velocity, angular_velocity):
		# while True:
		# 	if linear_velocity != 0 or angular_velocity != 0:
		# 		if linear_velocity > 0:
		# 			linear_velocity -= 0.01
		# 			linear_velocity = round(linear_velocity, 15)
		# 		if angular_velocity > 0:
		# 			angular_velocity -= 0.01
		# 			angular_velocity = round(angular_velocity, 15)
		# 		if linear_velocity < 0:
		# 			linear_velocity += 0.01
		# 			linear_velocity = round(linear_velocity, 15)
		# 		if angular_velocity < 0:
		# 			angular_velocity += 0.01
		# 			angular_velocity = round(angular_velocity, 15)
		# 	self.twist_robot.linear.x = linear_velocity
		# 	self.twist_robot.angular.z = angular_velocity
		# 	self.publish_velocity()
		# 	if linear_velocity == 0 and angular_velocity == 0:
		# 		break
		# 	time.sleep(0.150)
		self.twist_robot.linear.x = 0
		self.twist_robot.angular.z = 0
		self.publish_velocity()
		print("The robot stop!")

# Operate when run talker.py
if __name__=="__main__":

	settings = termios.tcgetattr(sys.stdin)

	try:
		teleop_publisher()
	except Exception as e:
		print(e)
	finally:
		pub_vel_robot = rospy.Publisher('cmd_vel', Twist, queue_size = 10)     

		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

		pub_vel_robot.publish(twist)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)