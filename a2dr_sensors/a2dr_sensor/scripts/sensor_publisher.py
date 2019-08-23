#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float32MultiArray, Float32


class ultrasonic_subsciber(object):

    def __init__(self):
        self.ultrasonic_data = None
    
    def callback(self, data):
        self.ultrasonic_data = data.data

    def listener(self):
        rospy.Subscriber("ultrasonic_low2high", Float32MultiArray,self.callback)


class imu_subsciber(object):

    def __init__(self):
        self.imu_data = None
    
    def callback(self, data):
        self.imu_data = data.data

    def listener(self):
        rospy.Subscriber("imu_low2high", Float32MultiArray,self.callback)


if __name__ == '__main__':

	rospy.init_node('sensor_publisher_node', anonymous=True)

    imu2odometry_publisher = rospy.Publisher('imu2odometry', Float32MultiArray, queue_size=10)
	angle_rotate_publisher = rospy.Publisher('angle_rotate', Float32MultiArray, queue_size=10)
	ultrasonice_publisher = rospy.Publisher('ultrasonic_depth', Float32MultiArray, queue_size=10)

	imu_limit = 360
    imu_node = imu_subsciber()
    imu_node.listener()

	ultrasonic_limit = 100
    ultrasonic_node = ultrasonic_subsciber()
    ultrasonic_node.listener()

    rate = rospy.Rate(20)

	while(not rospy.is_shutdown()):
        
        imu_accuracy = imu_node.imu_data[0]
        imu_theta = imu_node.imu_data[1]

		imu_array = [imu_accuracy, imu_theta]
		imu_data = Float32MultiArray(data=imu_array)

        ultrasonic1 =  ultrasonic_node.ultrasonic_data[0]
        ultrasonic2 = ultrasonic_node.ultrasonic_data[1]

		ultrasonic_array = [ultrasonic1, ultrasonic2]
        ultrasonic_data = Float32MultiArray(data=ultrasonic_array)
    
		if imu_accuracy == 3 and imu_theta < imu_limit:
			imu2odometry_publisher.publish(imu_data) # Publish imu data to update odometry

		if imu_accuracy == 3 and imu_theta < imu_limit:
			angle_rotate_publisher.publish(imu_data) # Publish imu data to rotate the robot

		if ultrasonic1 < ultrasonic_limit and ultrasonic2 < ultrasonic_limit:
			ultrasonice_publisher.publish(ultrasonic_data) # Publish ultrasonic data to rotate the robot (mirror)

		rate.sleep()