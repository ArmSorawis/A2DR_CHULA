#!/usr/bin/env python

from PyQt5 import QtWidgets, uic, QtCore, QtGui

import rospy
import roslaunch

from functools import partial

import sys

from functools import partial

from std_msgs.msg import Int32MultiArray

class MainWindow(QtWidgets.QMainWindow):
	def __init__(self):
		super(MainWindow,self).__init__()
		uic.loadUi("/home/kitti/catkin_ws/src/A2DR/a2dr_gui/ui/a2dr.ui",self)
		self.setWindowTitle('A2DR')

		# Show a2dr picture
		pixmap_a2dr = QtGui.QPixmap('/home/kitti/catkin_ws/src/A2DR/a2dr_gui/picture/a2dr.png')
		self.label_a2dr_pic.setPixmap(pixmap_a2dr)

		self.enterButton.clicked.connect(self.buttonEnter_clicked)
		
		self.room1_checkBox.stateChanged.connect(self.checkBox1)
		self.room2_checkBox.stateChanged.connect(self.checkBox2)
		self.room3_checkBox.stateChanged.connect(self.checkBox3)
		self.room4_checkBox.stateChanged.connect(self.checkBox4)
		self.room5_checkBox.stateChanged.connect(self.checkBox5)
		self.allroom_checkBox.stateChanged.connect(self.checkBoxAll)

		self.room_list = []
		rospy.init_node('a2dr_gui_node', anonymous=True)
		self.goal_publisher = rospy.Publisher('goal_sequence', Int32MultiArray, queue_size=10)

	def buttonEnter_clicked(self): 
		if len(self.room_list) > 0:
			adding_round = 5 - len(self.room_list)
			for index in range(adding_round):
				self.room_list.append(0)
				# print("Send the drug to room {}".format(self.room_list[index]))

			print(self.room_list)

			fsm_node = roslaunch.core.Node(	package='a2dr_fsm', 
											node_type='a2dr_workflow.py', 
											name='a2dr_workflow_node',
											output='screen')
			
			fsm_node.args = "_goal_1:=%d _goal_2:=%d _goal_3:=%d _goal_4:=%d _goal_5:=%d" %(self.room_list[0], 
																	   						self.room_list[1],
																							self.room_list[2],
																							self.room_list[3],
																							self.room_list[4])

			fsm_launch = roslaunch.scriptapi.ROSLaunch()
			fsm_launch.start()
			fsm_process = fsm_launch.launch(fsm_node)
			while fsm_process.is_alive():
				if fsm_process.is_alive() == False:
					break

			# goal_data = Int32MultiArray(data=self.room_list)
			# self.goal_publisher.publish(goal_data)

			for data in self.room_list:
				if 0 in self.room_list:
					self.room_list.remove(0)

		else: 
			print("Please select the room before click 'Enter' button")

	def checkBox1(self, state):
		if state == QtCore.Qt.Checked:
			self.room_list.append(1)
			self.label_room1.setText("{} || Room 1".format(self.room_list.index(1) + 1))
		else:
			self.label_room1.setText("")
			self.room_list.remove(1)

			if self.room2_checkBox.isChecked():
				self.label_room2.setText("{} || Room 2".format(self.room_list.index(2) + 1))
			if self.room3_checkBox.isChecked():
				self.label_room3.setText("{} || Room 3".format(self.room_list.index(3) + 1))
			if self.room4_checkBox.isChecked():
				self.label_room4.setText("{} || Room 4".format(self.room_list.index(4) + 1))
			if self.room5_checkBox.isChecked():
				self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))

	def checkBox2(self, state):
		if state == QtCore.Qt.Checked:
			self.room_list.append(2)
			self.label_room2.setText("{} || Room 2".format(self.room_list.index(2) + 1))
		else:
			self.label_room2.setText("")
			self.room_list.remove(2)

			if self.room1_checkBox.isChecked():
				self.label_room1.setText("{} || Room 1".format(self.room_list.index(1) + 1))
			if self.room3_checkBox.isChecked():
				self.label_room3.setText("{} || Room 3".format(self.room_list.index(3) + 1))
			if self.room4_checkBox.isChecked():
				self.label_room4.setText("{} || Room 4".format(self.room_list.index(4) + 1))
			if self.room5_checkBox.isChecked():
				self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))
	
	def checkBox3(self, state):
		if state == QtCore.Qt.Checked:
			self.room_list.append(3)
			self.label_room3.setText("{} || Room 3".format(self.room_list.index(3) + 1))
		else:
			self.label_room3.setText("")
			self.room_list.remove(3)

			if self.room1_checkBox.isChecked():
				self.label_room1.setText("{} || Room 1".format(self.room_list.index(1) + 1))
			if self.room2_checkBox.isChecked():
				self.label_room2.setText("{} || Room 2".format(self.room_list.index(2) + 1))
			if self.room4_checkBox.isChecked():
				self.label_room4.setText("{} || Room 4".format(self.room_list.index(4) + 1))
			if self.room5_checkBox.isChecked():
				self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))

	def checkBox4(self, state):
		if state == QtCore.Qt.Checked:
			self.room_list.append(4)
			self.label_room4.setText("{} || Room 4".format(self.room_list.index(4) + 1))
		else:
			self.label_room4.setText("")
			self.room_list.remove(4)

			if self.room1_checkBox.isChecked():
				self.label_room1.setText("{} || Room 1".format(self.room_list.index(1) + 1))
			if self.room2_checkBox.isChecked():
				self.label_room2.setText("{} || Room 2".format(self.room_list.index(2) + 1))
			if self.room3_checkBox.isChecked():
				self.label_room3.setText("{} || Room 3".format(self.room_list.index(3) + 1))
			if self.room5_checkBox.isChecked():
				self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))

	def checkBox5(self, state):
		if state == QtCore.Qt.Checked:
			self.room_list.append(5)
			self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))
		else:
			self.label_room5.setText("")
			self.room_list.remove(5)

			if self.room1_checkBox.isChecked():
				self.label_room1.setText("{} || Room 1".format(self.room_list.index(1) + 1))
			if self.room2_checkBox.isChecked():
				self.label_room2.setText("{} || Room 2".format(self.room_list.index(2) + 1))
			if self.room4_checkBox.isChecked():
				self.label_room4.setText("{} || Room 4".format(self.room_list.index(4) + 1))
			if self.room5_checkBox.isChecked():
				self.label_room5.setText("{} || Room 5".format(self.room_list.index(5) + 1))

	def checkBoxAll(self, state):
		if state == QtCore.Qt.Checked:
			self.room1_checkBox.setChecked(True)
			self.room2_checkBox.setChecked(True)
			self.room3_checkBox.setChecked(True)
			self.room4_checkBox.setChecked(True)
			self.room5_checkBox.setChecked(True)

			self.label_room1.setText("1 || Room 1")
			self.label_room2.setText("2 || Room 2")
			self.label_room3.setText("3 || Room 3")
			self.label_room4.setText("4 || Room 4")
			self.label_room5.setText("5 || Room 5")

			self.room_list = [1,2,3,4,5]

		else:
			self.room1_checkBox.setChecked(False)
			self.room2_checkBox.setChecked(False)
			self.room3_checkBox.setChecked(False)
			self.room4_checkBox.setChecked(False)
			self.room5_checkBox.setChecked(False)

			self.label_room1.setText("")
			self.label_room2.setText("")
			self.label_room3.setText("")
			self.label_room4.setText("")
			self.label_room5.setText("")

			self.room_list = []
			 
if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	Window = MainWindow()
	Window.show()
	sys.exit(app.exec_())