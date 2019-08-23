#!/usr/bin/env python

import rospy
import roslaunch
import os
rospy.init_node('a2dr_test', anonymous=False)

nodes = os.popen("rosnode list").read().splitlines()
interest_node = '/obstacleDetection_lidar_node'
if interest_node in nodes:
	os.system("rosnode kill {}".format(interest_node))