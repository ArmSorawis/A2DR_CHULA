#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class play_sound:
	def __init__(self):
		rospy.init_node('a2dr_soundplay_node')
		self.soundhandle = SoundClient()
		self.sound_volume = rospy.get_param("~sound_volume", 1.0)
		self.sound_to_play = rospy.get_param("~sound_to_play", "/home/kitti/catkin_ws/src/A2DR/a2dr_voice/voice/base_station.wav")
		self.sound_cycle_time = rospy.get_param("~sound_cycle_time", 5.0)
		self.play()
	
	def play(self):
		rospy.sleep(1)
		self.soundhandle.playWave(self.sound_to_play, self.sound_volume)
		rospy.sleep(self.sound_cycle_time)

if __name__ == '__main__':
	process = play_sound()