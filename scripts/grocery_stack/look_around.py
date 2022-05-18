#! /usr/bin/env python

import rospy
import sys
import actionlib
from std_msgs.msg import Bool
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryGoal


class LookAround():
    
    def __init__(self):
        self.head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()
        self.look_flag_pub = rospy.Publisher('/look_flag', Bool, queue_size = 1)

    def look_around(self):
        
        rospy.loginfo("Scanning environment to create octomap")
        self.look_to_point(0, 0, 0)
        rospy.sleep(3)
        self.look_to_point(0.75, 0.75, 0)
        self.look_to_point(1.5, 0.75, 0)
        self.look_to_point(1.5, 0, 0)
        self.look_to_point(1.5, -0.75, 0)
        self.look_to_point(0.75, -0.75, 0)
        self.look_to_point(0, 0, 0)
        self.look_to_point(1.5, 0, 0)
        rospy.loginfo("Scan complete")
        flag = Bool()
        flag.data = False
        self.look_flag_pub.publish(flag)

    def look_to_point(self, x, y, z):

	rospy.wait_for_message('/head_controller/state', JointTrajectoryControllerState)

	head_goal = PointHeadGoal()
	head_goal.target.header.frame_id = 'base_link'
	head_goal.target.point.x = x
	head_goal.target.point.y = y
	head_goal.target.point.z = z

	head_goal.pointing_frame = 'xtion_depth_frame'
	head_goal.pointing_axis.x = 1
	head_goal.pointing_axis.y = 0
	head_goal.pointing_axis.z = 0

	head_goal.min_duration = rospy.Duration(3)
	self.head_client.send_goal_and_wait(head_goal, rospy.Duration(15))


if __name__ == '__main__':
    rospy.init_node('look_around', anonymous=True)
    la = LookAround()
    la.look_around()
    rospy.spin()    
