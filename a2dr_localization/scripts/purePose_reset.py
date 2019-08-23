#!/usr/bin/env python

import rospy
import tf
import roslaunch

if __name__ == '__main__':
    rospy.init_node('purePose_reset_node', disable_signals=True)

    listener = tf.TransformListener()
    first_time = True
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        x_pose = trans[0]
        y_pose = trans[1]

        
        if first_time == True:
            old_x_pose = x_pose
            old_y_pose = y_pose
            first_time = False
            continue

        diff_y = abs(old_y_pose-y_pose)
        diff_x = abs(old_x_pose-x_pose)

        rospy.loginfo("{},{}".format(diff_x, diff_y))

        if diff_x > 2 or diff_y > 2:
            rotate360_node = roslaunch.core.Node(package='a2dr_sensor', 
													 node_type='rotate360By_imu.py',
													 name='rotate360By_imu_node')

            rotate360_node.args = "_rotate360_target:=%d" %359

            rotate360_launch = roslaunch.scriptapi.ROSLaunch()
            rotate360_launch.start()

            rotate360_process = rotate360_launch.launch(rotate360_node)

            while rotate360_process.is_alive():
                if rotate360_process.is_alive() == False:
                    break

            rotate360_process.stop()
            first_time = True
        
        old_x_pose = x_pose
        old_y_pose = y_pose

        rate.sleep()

rospy.signal_shutdown("Exit")