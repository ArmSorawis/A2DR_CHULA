#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String

class play_silent:
	def __init__(self):
		rospy.init_node('a2dr_silentplay_node')
		self.sound_path = '/home/kitti/catkin_ws/src/A2DR/a2dr_voice/voice/excuse_me.wav'
		self.sound_cycleTime = 4        
		self.listener()

	def listener(self):
		rospy.Subscriber('silent', String, self.play)
		rospy.spin()
	
	def play(self, data):
		print("play")
		self.soundhandle = SoundClient()
		self.sound_volume = rospy.get_param("~sound_volume", 1.0)
		self.sound_to_play = rospy.get_param("~sound_to_play", self.sound_path)
		self.sound_cycle_time = rospy.get_param("~sound_cycle_time", self.sound_cycleTime)
		rospy.sleep(1)
		self.soundhandle.playWave(self.sound_to_play, self.sound_volume)
		rospy.sleep(self.sound_cycle_time)

if __name__ == '__main__':
	process = play_silent()