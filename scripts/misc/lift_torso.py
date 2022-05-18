#! /usr/bin/env python

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Header


def joint_controller():
        
        pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size = 10)
        move = JointTrajectory()
        move.header = Header()
        move.header.stamp = rospy.Time.now()
        move.joint_names.append('torso_lift_joint')
        point=JointTrajectoryPoint()
        point.positions = [0.5, 0, 0]
        point.time_from_start = rospy.Duration(2)
        move.points.append(point)
        for i in range(50):
            pub.publish(move)
            rospy.sleep(0.1)


if __name__ == '__main__':
	rospy.init_node('lower_torso', anonymous=True)
        joint_controller()
