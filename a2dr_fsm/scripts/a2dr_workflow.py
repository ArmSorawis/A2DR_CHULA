#!/usr/bin/env python

import rospy
import smach
import smach_ros
import roslaunch

import os

from std_msgs.msg import Int32MultiArray

from read_goalFile_floor5 import readGoal_floor5
from read_actionFile_floor5 import selection_state_floor5

goal = readGoal_floor5()

def clear_costmaps():
	rospy.loginfo('Executing state clear costmaps')

	clear_costmaps_node = roslaunch.core.Node(package='a2dr_msgs', 
											  node_type='clear_cost_map.py', 
											  name='clear_cost_map_node')

	clear_costmaps_launch = roslaunch.scriptapi.ROSLaunch()
	
	clear_costmaps_launch.start()

	clear_costmaps_process = clear_costmaps_launch.launch(clear_costmaps_node)
	
	while clear_costmaps_process.is_alive():
		if clear_costmaps_process.is_alive() == False:
			break

	clear_costmaps_process.stop()


def open_obstacleDetection_node():
	global obstacle_process
	obstacle_node = roslaunch.core.Node(package='a2dr_sensor', 
										node_type='obstacleDetection_lidar.py', 
										name='obstacleDetection_lidar_node')
	obstacle_launch = roslaunch.scriptapi.ROSLaunch()
	obstacle_launch.start()
	obstacle_process = obstacle_launch.launch(obstacle_node)


def close_obstacleDetection_node():
	nodes = os.popen("rosnode list").read().splitlines()
	interest_node = '/obstacleDetection_lidar_node'
	if interest_node in nodes:
		os.system("rosnode kill {}".format(interest_node))

class systemAvailability(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	
	def execute(self, userdata):
		rospy.loginfo('Executing state system availability')
	
		print(userdata.goalList_input)

		selection_state_floor5("start", None)

		sound_node = roslaunch.core.Node(package='a2dr_voice', 
										 node_type='a2dr_soundplay.py', 
										 name='a2dr_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(selection_state_floor5.soundPath, 
																	   selection_state_floor5.sound_cycleTime)

		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break
		
		userdata.goalList_output = userdata.goalList_input

		rospy.sleep(1)

		clear_costmaps()

		return 'success'


class move2machine(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		open_obstacleDetection_node()

		rospy.loginfo('Executing state move to machine')

		rospy.loginfo('Robot have been moving to drug machine')

		print(userdata.goalList_input)

		nav_node = roslaunch.core.Node(package='a2dr_navigation', 
									   node_type='send_goal.py', 
									   name='movebase_client_py')
		machine_goal = goal[0] 
		nav_node.args = """ _position_x:={}
							_position_y:={}
							_position_z:={}
							_orientation_x:={} 
							_orientation_y:={} 
							_orientation_z:={}  
							_orientation_w:={} """\
							.format(machine_goal[0],
									machine_goal[1],
									machine_goal[2],
									machine_goal[3],
									machine_goal[4],
									machine_goal[5],
									machine_goal[6])

		nav_launch = roslaunch.scriptapi.ROSLaunch()
		nav_launch.start()
		nav_process = nav_launch.launch(nav_node)
		while nav_process.is_alive():
			if nav_process.is_alive == False:
				break
		nav_process.stop()

		userdata.goalList_output = userdata.goalList_input

		rospy.sleep(1)

		clear_costmaps()

		# kill obstacle detection node
		close_obstacleDetection_node()
		
		return 'success'


class align2machine(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state align to machine')

		print(userdata.goalList_input)

		rospy.loginfo('Robot alingment at machine station')

		# planeAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 											node_type='planeAlignment_ultrasonic.py', 
		# 											name='planeAlignment_ultrasonic_node')

		# planeAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# planeAlignment_launch.start()
		# planeAlignment_process = planeAlignment_launch.launch(planeAlignment_node)
		# while planeAlignment_process.is_alive():
		# 	if planeAlignment_process.is_alive() == False:
		# 		break
		# planeAlignment_process.stop()

		rospy.sleep(1)

		rospy.loginfo('Depth aligning at machine staton')

		# depthAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 								node_type='depthAlignment_ultrasonic.py', 
		# 								name='depthAlignment_ultrasonic_node',
		# 								output='screen')
		# depthAlignment_node.args = "_distance_target:=%f" %(65.0)

		# depthAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# depthAlignment_launch.start()
		# depthAlignment_process = depthAlignment_launch.launch(depthAlignment_node)
		# while depthAlignment_process.is_alive():
		# 	if depthAlignment_process.is_alive() == False:
		# 		break
		# depthAlignment_process.stop()

		# rospy.sleep(1)

		selection_state_floor5("machine", userdata.goalList_input[0])

		rospy.loginfo('Robot recieve drug from machine')
		
		sound_node = roslaunch.core.Node(package='a2dr_voice', 
										 node_type='a2dr_soundplay.py', 
										 name='a2dr_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(selection_state_floor5.soundPath, 
																	   selection_state_floor5.sound_cycleTime)

		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break

		rospy.sleep(5)

		rospy.loginfo('Robot rotate heading to room {}'.format(userdata.goalList_input[0]))

		# nextgoalAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 												 node_type='rotateBy_imu.py', 
		# 												 name='rotateBy_imu_node')
		# nextgoalAlignment_node.args = "_rotate_target:=%d" %selection_state_floor5.rotate2nextStation

		# nextgoalAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# nextgoalAlignment_launch.start()
		# nextgoalAlignment_process = nextgoalAlignment_launch.launch(nextgoalAlignment_node)
		# while nextgoalAlignment_process.is_alive():
		# 	if nextgoalAlignment_process.is_alive() == False:
		# 		break
		# nextgoalAlignment_process.stop()

		userdata.goalList_output = userdata.goalList_input
		rospy.sleep(2)

		clear_costmaps()
		
		return 'success'


class wait4confirm(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							outcomes=['confirmed'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state confirm')
		user_input = raw_input("Do you want to go to the next State? => ")
		if user_input ==  "Y" or user_input == "y":
			return 'confirmed'
		elif user_input ==  "N" or user_input == "n":
			rospy.loginfo("Please type only 'Y' or 'y'")


class move2goal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])
		

	def execute(self, userdata):
		open_obstacleDetection_node()

		rospy.loginfo('Executing state move to goal')

		print(userdata.goalList_input)

		rospy.loginfo('Robot have been moving to room {}'.format(userdata.goalList_input[0]))

		nav_node = roslaunch.core.Node(package='a2dr_navigation', 
									   node_type='send_goal.py', 
									   name='movebase_client_py')

		room_goal = goal[userdata.goalList_input[0]]

		nav_node.args = """ _position_x:={}
							_position_y:={}
							_position_z:={}
							_orientation_x:={} 
							_orientation_y:={} 
							_orientation_z:={}  
							_orientation_w:={} """\
							.format(room_goal[0],
									room_goal[1],
									room_goal[2],
									room_goal[3],
									room_goal[4],
									room_goal[5],
									room_goal[6])

		nav_launch = roslaunch.scriptapi.ROSLaunch()
		nav_launch.start()
		nav_process = nav_launch.launch(nav_node)
		while nav_process.is_alive():
			if nav_process.is_alive == False:
				break
		nav_process.stop()
		
		# del userdata.goalList_input[0]
		userdata.goalList_output = userdata.goalList_input

		rospy.sleep(2)

		# clear_costmaps()

		# kill obstacle detection node
		close_obstacleDetection_node()

		return 'success'


class planeAlignment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state plane alignment')

		print(userdata.goalList_input)

		rospy.loginfo('Plane aligning at room {}'.format(userdata.goalList_input[0]))

		# planeAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 											node_type='planeAlignment_ultrasonic.py', 
		# 											name='planeAlignment_ultrasonic_node')

		# planeAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# planeAlignment_launch.start()
		# planeAlignment_process = planeAlignment_launch.launch(planeAlignment_node)
		# while planeAlignment_process.is_alive():
		# 	if planeAlignment_process.is_alive() == False:
		# 		break
		# planeAlignment_process.stop()

		userdata.goalList_output = userdata.goalList_input
		
		rospy.sleep(1)
		
		return 'success'


class depthAlignment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state depth alignment')

		print(userdata.goalList_input)

		rospy.loginfo('Depth aligning at room {}'.format(userdata.goalList_input[0]))

		# depthAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 								node_type='depthAlignment_ultrasonic.py', 
		# 								name='depthAlignment_ultrasonic_node',
		# 								output='screen')
		# depthAlignment_node.args = "_distance_target:=%f" %(65.0)

		# depthAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# depthAlignment_launch.start()
		# depthAlignment_process = depthAlignment_launch.launch(depthAlignment_node)
		# while depthAlignment_process.is_alive():
		# 	if depthAlignment_process.is_alive() == False:
		# 		break
		# depthAlignment_process.stop()

		userdata.goalList_output = userdata.goalList_input

		rospy.sleep(1)
	
		return 'success'


class drugProcess(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state drug process')
		
		print(userdata.goalList_input)

		rospy.loginfo('Robot deliveried drug to room {}'.format(userdata.goalList_input[0]))

		if len(userdata.goalList_input) > 1:
			selection_state_floor5(userdata.goalList_input[0], userdata.goalList_input[1])
		elif len(userdata.goalList_input) == 1:
			selection_state_floor5(userdata.goalList_input[0], 'base')

		sound_node = roslaunch.core.Node(package='a2dr_voice', 
										 node_type='a2dr_soundplay.py', 
										 name='a2dr_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(selection_state_floor5.soundPath, 
																	   selection_state_floor5.sound_cycleTime)

		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break

		userdata.goalList_output = userdata.goalList_input

		return 'success'


class nextgoalAlignment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['action_success', 'all_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state nextgoal alignment')
		
		if len(userdata.goalList_input) > 1:
			print(userdata.goalList_input)
			rospy.loginfo('Robot rotate heading to room {}'.format(userdata.goalList_input[1]))
		
			selection_state_floor5(userdata.goalList_input[0], userdata.goalList_input[1])

			# nextgoalAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
			# 											 node_type='rotateBy_imu.py', 
			# 											 name='rotateBy_imu_node')
			# nextgoalAlignment_node.args = "_rotate_target:=%d" %selection_state_floor5.rotate2nextStation

			# nextgoalAlignment_launch = roslaunch.scriptapi.ROSLaunch()
			# nextgoalAlignment_launch.start()
			# nextgoalAlignment_process = nextgoalAlignment_launch.launch(nextgoalAlignment_node)
			# while nextgoalAlignment_process.is_alive():
			# 	if nextgoalAlignment_process.is_alive() == False:
			# 		break
			# nextgoalAlignment_process.stop()

			del userdata.goalList_input[0]
			userdata.goalList_output = userdata.goalList_input

			rospy.sleep(2)
			clear_costmaps()

			return 'action_success'	

		elif len(userdata.goalList_input) == 1:
			print(userdata.goalList_input)
			rospy.loginfo('Robot rotate heading to base station')

			selection_state_floor5(userdata.goalList_input[0], 'base')

			# nextgoalAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
			# 											 node_type='rotateBy_imu.py', 
			# 											 name='rotateBy_imu_node')
			# nextgoalAlignment_node.args = "_rotate_target:=%d" %selection_state_floor5.rotate2nextStation

			# nextgoalAlignment_launch = roslaunch.scriptapi.ROSLaunch()
			# nextgoalAlignment_launch.start()
			# nextgoalAlignment_process = nextgoalAlignment_launch.launch(nextgoalAlignment_node)
			# while nextgoalAlignment_process.is_alive():
			# 	if nextgoalAlignment_process.is_alive() == False:
			# 		break
			# nextgoalAlignment_process.stop()

			userdata.goalList_output = userdata.goalList_input

			rospy.sleep(2)
			# clear_costmaps()

			return 'all_success'
		

class move2base(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'])

	def execute(self, userdata):
		open_obstacleDetection_node()

		rospy.loginfo('Executing state move to base')

		rospy.loginfo('Robot have been moving to back to base station')

		nav_node = roslaunch.core.Node(package='a2dr_navigation', 
									   node_type='send_goal.py', 
									   name='movebase_client_py')
		base_goal = goal[-1] 
		nav_node.args = """ _position_x:={}
							_position_y:={}
							_position_z:={}
							_orientation_x:={} 
							_orientation_y:={} 
							_orientation_z:={}  
							_orientation_w:={} """\
							.format(base_goal[0],
									base_goal[1],
									base_goal[2],
									base_goal[3],
									base_goal[4],
									base_goal[5],
									base_goal[6])

		nav_launch = roslaunch.scriptapi.ROSLaunch()
		nav_launch.start()
		nav_process = nav_launch.launch(nav_node)
		while nav_process.is_alive():
			if nav_process.is_alive == False:
				break
		nav_process.stop()

		rospy.sleep(1)
		
		clear_costmaps()

		# kill obstacle detection node
		close_obstacleDetection_node()

		return 'success'


class align2base(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'])

	def execute(self, userdata):

		rospy.loginfo('Executing state align to base')
		rospy.loginfo('Robot alingment at base station')

		selection_state_floor5('base', None)

		# baseAlignment_node = roslaunch.core.Node(package='a2dr_sensor', 
		# 										 node_type='rotateBy_imu.py',
		# 										 name='rotateBy_imu_node')
		# baseAlignment_node.args = "_rotate_target:=%d" %selection_state_floor5.rotate2nextStation

		# baseAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		# baseAlignment_launch.start()
		# baseAlignment_process = baseAlignment_launch.launch(baseAlignment_node)
		# while baseAlignment_process.is_alive():
		# 	if baseAlignment_process.is_alive() == False:
		# 		break
		# baseAlignment_process.stop()

		rospy.sleep(1)

		sound_node = roslaunch.core.Node(package='a2dr_voice', 
										 node_type='a2dr_soundplay.py', 
										 name='a2dr_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(selection_state_floor5.soundPath, 
																	   selection_state_floor5.sound_cycleTime)

		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break

		clear_costmaps()
		
		return 'success'


class wait4drug(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['success'])

	def execute(self, userdata):
		rospy.loginfo('Executing state wait for drug')
		rospy.sleep(1)
		return 'success'


def main():
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	# sm_nav.userdata.goal_list = list(.data)
	accept_goal = []
	goal_1 = rospy.get_param("~goal_1", 1)
	goal_2 = rospy.get_param("~goal_2", 2)
	goal_3 = rospy.get_param("~goal_3", 3)
	goal_4 = rospy.get_param("~goal_4", 4)
	goal_5 = rospy.get_param("~goal_5", 5)
	all_goal = [goal_1, goal_2, goal_3, goal_4, goal_5]
	for index in range(len(all_goal)):
		if all_goal[index] != 0:
			accept_goal.append(all_goal[index])
		else:
			pass
	print(accept_goal)
	sm_nav.userdata.goal_list = accept_goal
	with sm_nav:
		smach.StateMachine.add('SYSTEM_AVAILABILITY', systemAvailability(),
								transitions={'success':'MOVE2MACHINE'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE2MACHINE', move2machine(),
								transitions={'success':'ALIGN2MACHINE'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})
		
		smach.StateMachine.add('ALIGN2MACHINE', align2machine(),
								transitions={'success':'WAIT4CONFIRM'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})
		
		smach.StateMachine.add('WAIT4CONFIRM', wait4confirm(),
								transitions={'confirmed':'MOVE2GOAL'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE2GOAL', move2goal(),
								transitions={'success':'SM_ACTION'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE2BASE', move2base(),
								transitions={'success':'ALIGN2BASE'})

		smach.StateMachine.add('ALIGN2BASE', align2base(),
								transitions={'success':'WAIT4DRUG'})

		smach.StateMachine.add('WAIT4DRUG', wait4drug(),
								transitions={'success':'shutdown'})

		sm_act = smach.StateMachine(outcomes=['action_finished', 'all_finished'])
		# sm_act.userdata.goal_list = list(goal.data)
		sm_act.userdata.goal_list = accept_goal
		with sm_act:
			smach.StateMachine.add('PLANE_ALIGNMENT', planeAlignment(), 
									transitions={'success':'DEPTH_ALIGNMENT'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})

			smach.StateMachine.add('DEPTH_ALIGNMENT', depthAlignment(), 
									transitions={'success':'DRUG_PROCESS'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})

			smach.StateMachine.add('DRUG_PROCESS', drugProcess(), 
									transitions={'success':'WAIT4CONFIRM'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})
			
			smach.StateMachine.add('WAIT4CONFIRM', wait4confirm(),
								transitions={'confirmed':'NEXTGOAL_ALIGNMENT'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

			smach.StateMachine.add('NEXTGOAL_ALIGNMENT', nextgoalAlignment(),
									transitions={'action_success':'action_finished', 'all_success':'all_finished'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'}) 

		smach.StateMachine.add('SM_ACTION', sm_act,
							   transitions={'action_finished':'MOVE2GOAL', 'all_finished': 'MOVE2BASE'})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV/SM_ACT')
	sis.start()
	# Execute SMACH plan
	outcome = sm_nav.execute()
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


class goal_subscriber(object):
	def __init__(self):
		self.data = None
		self.listener()

	def callback(self,data):
		self.data = data.data

	def listener(self):
		rospy.Subscriber("goal_sequence", Int32MultiArray, main, self.data, queue_size=10)
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('a2dr_workflow_node', anonymous=False)
	# process = goal_subscriber()
	main()