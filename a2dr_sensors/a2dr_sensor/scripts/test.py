#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math

import sys

class robotRotate():
    def __init__(self):
        self.target = -90
        self.k_p = 0.1

        self.twist_robot =Twist()
        self.twist_robot.linear.x = 0
        self.twist_robot.linear.y = 0
        self.twist_robot.linear.z = 0
        self.twist_robot.angular.x = 0
        self.twist_robot.angular.y = 0
        self.twist_robot.angular.z = 0

        self.cmdvel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
    def send_commandvel(self, start_yaw, robot_yaw):
        if self.target > 0 and self.target <= 179:
            self.selecting_state("rotate_right", start_yaw, robot_yaw)
        elif self.target < 0 and self.target >= -179:
            self.selecting_state("rotate_left", start_yaw, robot_yaw)

    def selecting_state(self, state, start_yaw, robot_yaw):
        
        target_rad = self.target * math.pi /180
    
        if state == "rotate_right":
            if abs(robot_yaw - start_yaw) < target_rad:
                # velocity = (self.k_p * (target_rad - yaw) * 0.50) / (target_rad * self.k_p)
                # if velocity < 0.45:
                velocity = 0.45
                self.twist_robot.angular.z = velocity    
                self.cmdvel_publisher.publish(self.twist_robot)

            elif abs(robot_yaw - start_yaw) >= target_rad:
                self.twist_robot.angular.z = 0
                self.cmdvel_publisher.publish(self.twist_robot)
                rospy.signal_shutdown('Quit')
                sys.exit(1)
        
        elif state == "rotate_left":
            if robot_yaw - start_yaw >= target_rad:
                # velocity = (self.k_p * (target_rad - yaw) * 0.55) / (target_rad * self.k_p)
                # if velocity < 0.45:
                velocity = 0.45
                self.twist_robot.angular.z = -velocity    
                self.cmdvel_publisher.publish(self.twist_robot)

            elif robot_yaw - start_yaw < target_rad:
                self.twist_robot.angular.z = 0
                self.cmdvel_publisher.publish(self.twist_robot)
                rospy.signal_shutdown('Quit')
                sys.exit(1)

        print("target={} current:{} start{}",target_rad, robot_yaw, start_yaw)

class odom_subscriber(object):

    def __init__(self):
        # self.listener()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def get_rotation(self, msg):
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def listener(self):
        rospy.Subscriber ('odom', Odometry, self.get_rotation)
    

# Operate when run send_cmdvel.py
if __name__=="__main__":
    rospy.init_node('rotateBy_odom_node')

    odomSub_node = odom_subscriber()
    odomSub_node.listener()

    sendCmdvel_node = robotRotate()
    rate = rospy.Rate(20)

    first_yaw = True
    start_yaw = 0 

    while(not rospy.is_shutdown()):
        robot_yaw = odomSub_node.yaw
        if first_yaw == True:
            start_yaw = robot_yaw
            first_yaw = False
        sendCmdvel_node.send_commandvel(start_yaw, robot_yaw)
        rate.sleep()