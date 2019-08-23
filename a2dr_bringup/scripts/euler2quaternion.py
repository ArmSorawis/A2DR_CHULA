#!/usr/bin/env python

from tf.transformations import quaternion_from_euler

roll = 0.0 
pitch = 0.0
yaw = 2.94

(orientation_x, orientation_y, orientation_z, orientation_w) = quaternion_from_euler(roll, pitch, yaw)

print("orientation_x = {}, orientation_y = {}, orientation_z = {}, orientation_w = {}"\
.format(orientation_x, orientation_y, orientation_z, orientation_w))