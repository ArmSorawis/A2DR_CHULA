#!/usr/bin/env python

from tf.transformations import euler_from_quaternion

orientation_x = 0.0
orientation_y = 0.0
orientation_z = 0.99
orientation_w = 0.10

orientation_list = [orientation_x, orientation_y, orientation_z, orientation_w]
(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

print("roll = {}, pitch = {}, yaw = {}".format(roll, pitch, yaw))