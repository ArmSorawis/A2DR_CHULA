#!/usr/bin/env python

import rospy
import roslaunch

from read_goalFile_floor5 import readGoal_floor5
from read_actionFile_floor5 import selection_state_floor5

sound_path = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/machine.wav'
sound_cycleTime = 4

sound_node = roslaunch.core.Node(package='a2dr_voice', 
										 node_type='a2dr_soundplay.py', 
										 name='a2dr_soundplay_node')
sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(sound_path, sound_cycleTime)
sound_launch = roslaunch.scriptapi.ROSLaunch()
sound_launch.start()
sound_process = sound_launch.launch(sound_node)
while sound_process.is_alive():
	if sound_process.is_alive() == False:
		break