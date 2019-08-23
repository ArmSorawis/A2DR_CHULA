#!/usr/bin/env python

import rospy
import time
from KalmanFilter import KalmanFilter

from std_msgs.msg import Float32, Float32MultiArray

class WheelVelFilter:
    def __init__(self):
        rospy.init_node('wheelVel_filter_node', anonymous=True)

        self.odomFreq = float(rospy.get_param("~odom_freq", "20"))

        self.sub_vw_r = rospy.Subscriber("v_wheel_right", Float32, self.updateVW_R)
        self.sub_vw_l = rospy.Subscriber("v_wheel_left", Float32, self.updateVW_L)
        # self.sub_imu = rospy.Subscriber("/imu", Float32, self.updateIMU)

        # self.pub_raw_vel = rospy.Publisher("/raw_vel", Float32MultiArray, queue_size=1)
        self.pub_filter_vel = rospy.Publisher("filter_vel", Float32MultiArray, queue_size=1)
        # self.pub_filter_yaw = rospy.Publisher("/filter_yaw", Float32, queue_size=1)

        self.timer_v_wheel = rospy.Timer(rospy.Duration(1.0/self.odomFreq), self.timerVWheelCB)

        self.filterR = KalmanFilter(1, 30)
        self.filterL = KalmanFilter(1, 30)
        # self.filterYaw = KalmanFilter(0.01, 2)

        self.filter_yaw = 0.0
        self.yaw = 0.0
        self.vw_r = 0.0
        self.vw_l = 0.0
        self.filter_vw_r = 0.0
        self.filter_vw_l = 0.0


    # def updateIMU(self, data):
        # self.yaw = data.data
        # self.filterYaw.input_latest_noisy_measurement(self.yaw)
        # self.filter_yaw = self.filterYaw.get_latest_estimated_measurement()

    def updateVW_L(self, data):
        
        self.vw_l = data.data
        self.filterL.input_latest_noisy_measurement(self.vw_l)
        self.filter_vw_l = self.filterL.get_latest_estimated_measurement()

    def updateVW_R(self, data):

        self.vw_r = data.data
        self.filterR.input_latest_noisy_measurement(self.vw_r)
        self.filter_vw_r = self.filterR.get_latest_estimated_measurement() 
        

    def timerVWheelCB(self, event):
        # self.filter_vw_l = self.filter_vw_l * 0.075 / 100
        # self.filter_vw_r = self.filter_vw_r * 0.075 / 100
        # vl = float(str(round(self.filter_vw_l, 5))) 
        # vr = float(str(round(self.filter_vw_r, 5)))
        # print(vl, vr)
        
        raw_vel_array = Float32MultiArray(data=[self.vw_l, self.vw_r])
        filter_vel_array = Float32MultiArray(data=[self.filter_vw_l, self.filter_vw_r])

        # self.pub_raw_vel.publish(raw_vel_array)
        self.pub_filter_vel.publish(filter_vel_array)
        # self.pub_filter_yaw.publish(self.filter_yaw)
	
if __name__ == "__main__":

	imu = WheelVelFilter()
	rospy.spin()