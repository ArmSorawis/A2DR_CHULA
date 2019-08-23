#! /usr/bin/env python

## Library
import rospy
from geometry_msgs.msg import Twist

msg = "-- Subscriber ready to receive data! --"
class cmdvel_sub(object):

    def __init__(self):
        self.linear_velocity_x = 0
        self.angular_velocity_z = 0
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1) 
        self.listener() 
    
    def callback(self, data):
        self.linear_velocity_x = data.linear.x
        self.angular_velocity_z = data.angular.z
        rospy.loginfo("Linear command: {}".format(self.linear_velocity_x))
        rospy.loginfo("Angular command: {}".format(self.angular_velocity_z))
        self.velocity_publisher.publish(data)
        
    def listener(self):
        rospy.Subscriber('keyboard_teleop', Twist, self.callback)
        rospy.spin()

if __name__ == "__main__":
    print(msg)
    rospy.init_node('cmdvel_subscriber_node', anonymous=True)
    process = cmdvel_sub()