#! /usr/bin/env python

import rospy
import numpy as np
from math import sin, cos
import tf
from nav_msgs.msg import Odometry
from a2dr_msgs.msg import robotState
from std_msgs.msg import Int8, Float32


class BaseControl:
    max_count = 20
    imu_tol = 0.01

    def __init__(self):
        self.odomFreq = float(rospy.get_param("~odom_freq", "20"))
        self.baseId = rospy.get_param("~base_id", "base_footprint")
        self.odomId = rospy.get_param("~odom_id", "odom") 

        self.wheelSep = float(rospy.get_param("~wheel_separation", "0.342"))
        self.wheelRad = float(rospy.get_param("~wheel_radius", "0.075"))

        self.pub_odom = rospy.Publisher("/Odom", Odometry, queue_size=1)
        self.pub_yaw = rospy.Publisher("/Yaw", Float32, queue_size=1)
        self.sub_vw_r = rospy.Subscriber("/v_wheel_right", Float32, self.vw_rCB, queue_size=1)
        self.sub_vw_l = rospy.Subscriber("/v_wheel_left", Float32, self.vw_lCB, queue_size=1)

        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odomFreq), self.timerOdomCB)

        self.transform = tf.TransformBroadcaster()

        self.v_right = 0
        self.v_left = 0
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0

        self.previous_time = rospy.Time.now()

    def vw_rCB(self, resp):
        self.v_right = resp.data

    def vw_lCB(self, resp):
        self.v_left = resp.data

    def timerOdomCB(self, event):
        self.lin_x = round((self.v_right + self.v_left)/2.0, 4)
        self.ang_z = round((self.v_right - self.v_left)*self.wheelRad/self.wheelSep, 4)

        current_time = rospy.Time.now()
        dt = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        self.lin_x = round((self.v_right + self.v_left)*self.wheelRad/2.0, 4)
        self.ang_z = round((self.v_right - self.v_left)*self.wheelRad/self.wheelSep, 4)

        self.pose_th += self.ang_z*dt
        # self.pose_th = self.mapQuadrant((self.pose_th + self.ang_z*dt)%(2*np.pi))

        self.pose_x += (self.lin_x*cos(self.pose_th)*dt)
        self.pose_y += (self.lin_x*sin(self.pose_th)*dt)

        pose_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.pose_th)

        msg = Odometry()
        msg.header.stamp = current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id = self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = self.lin_x
        msg.twist.twist.angular.z = self.ang_z

        for i in range(36):
            msg.pose.covariance[i] = 0
        msg.pose.covariance[0] = 0.01 #Error X
        msg.pose.covariance[7] = 0.01 #Error Y
        msg.pose.covariance[14] = 99999
        msg.pose.covariance[21] = 99999
        msg.pose.covariance[28] = 99999
        msg.pose.covariance[35] = 0.01 #Error Yaw

        msg.twist.covariance = msg.pose.covariance

        # angle = tf.transformations.euler_from_quaternion([pose_quat[0],pose_quat[1],pose_quat[2],pose_quat[3]])
        self.pub_odom.publish(msg)
        # self.pub_yaw.publish(angle[2])

        self.transform.sendTransform((self.pose_x, self.pose_y, 0), (
            pose_quat[0], pose_quat[1], pose_quat[2], pose_quat[3]), current_time, self.baseId, self.odomId)

    def mapQuadrant(self, theta):
        if 0 <= theta and theta <= np.pi:
            return theta
        elif theta > np.pi and theta < 2*np.pi:
            return (theta - np.pi) - np.pi
        elif -np.pi <= theta and theta <= 0:
            return theta
        elif -2*np.pi < theta and theta < -np.pi:
            return np.pi - (-theta - np.pi)

if __name__ == "__main__":
    try:
        rospy.init_node("a2dr_base_control")
        robot = BaseControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass      
        
    

