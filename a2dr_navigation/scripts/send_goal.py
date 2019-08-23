#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import tf

class move2goal:
    def __init__(self):
        # rospy.Subscriber('cmd_gui', String, self.movebase_client, goal)
        # rospy.spin()

        rospy.init_node('movebase_client_py', disable_signals=True)

        pose_x = rospy.get_param("~position_x", 0.0)
        pose_y = rospy.get_param("~position_y", 0.0)
        pose_z = rospy.get_param("~position_z", 0.0)
        orien_x = rospy.get_param("~orientation_x", 0.0)
        orien_y = rospy.get_param("~orientation_y", 0.0)
        orien_z = rospy.get_param("~orientation_z", 0.0)
        orien_w = rospy.get_param("~orientation_w", 0.0)

        self.goal = [pose_x, pose_y, pose_z, orien_x, orien_y, orien_z, orien_w]
        self.movebase_client()

    def movebase_client(self):
        
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.goal[0]
        goal.target_pose.pose.position.y = self.goal[1]
        goal.target_pose.pose.position.z = self.goal[2]
        
        goal.target_pose.pose.orientation.x = self.goal[3]
        goal.target_pose.pose.orientation.y = self.goal[4]
        goal.target_pose.pose.orientation.z = self.goal[5]
        goal.target_pose.pose.orientation.w = self.goal[6]
        
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        rospy.loginfo(wait)
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        result = move2goal()
        if result:
            rospy.loginfo("Goal execution done!")
            rospy.signal_shutdown("Navigate Finished!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        rospy.signal_shutdown("Navigate Finished!")