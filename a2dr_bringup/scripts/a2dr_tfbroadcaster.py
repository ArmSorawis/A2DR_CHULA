#!/usr/bin/env python 

import rospy

import tf

class defind_frame():
	def __init__(self):
		rospy.init_node("tf_broadcaster_node")
		self.br = tf.TransformBroadcaster()
		self.listener = tf.TransformListener()
		self.broadcastFrame()

	def broadcastFrame(self):
		rate = rospy.Rate(15)
		while (not rospy.is_shutdown()):
			try:
				# time = self.listener.getLatestCommonTime("/odom", "/base_link")
				self.br.sendTransform((0.443, 0.0, 0.08), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 
		        "base_laser", 
		        "base_footprint"
			    )
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			rate.sleep()

if __name__ == '__main__':
	try:
		defind_frame()
	except Exception as e:
		print(e)