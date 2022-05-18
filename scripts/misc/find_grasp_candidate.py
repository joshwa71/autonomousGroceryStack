#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from tf.transformations import euler_from_quaternion
import math
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState
from final_year_pkg.msg import PickGraspMsgArray, PickGraspMsg
import sys

class FindBestGrasp():

    def __init__(self):
        rospy.init_node('find_best_grasp')
        self.grasps = []
        self.used_grasp = []
        self.chosen_grasp_array = PickGraspMsgArray()
        self.found_grasp_flag = False
        self.look_down_flag = False
        sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.callback)
        self.pub = rospy.Publisher('/chosen_grasp', PickGraspMsgArray, queue_size=10)
        self.head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.rate = rospy.Rate(1)

    def look_down(self):
        print('looking down')

	rospy.wait_for_message('/head_controller/state', JointTrajectoryControllerState)

	head_goal = PointHeadGoal()
	head_goal.target.header.frame_id = 'base_link'
	head_goal.target.point.x = 4
	head_goal.target.point.y = 0
	head_goal.target.point.z = 3

	head_goal.pointing_frame = 'xtion_depth_frame'
	head_goal.pointing_axis.x = 1
	head_goal.pointing_axis.y = 0
	head_goal.pointing_axis.z = 0

	head_goal.min_duration = rospy.Duration(1)
	self.head_client.send_goal_and_wait(head_goal, rospy.Duration(5))


    def callback(self,msg):

	if(len(msg.grasps) > 1):
            self.grasps = msg.grasps

            self.chosen_grasp_array = PickGraspMsgArray()

            for grasp in self.grasps:
             
                r21 = grasp.approach.y
                r11 = grasp.approach.x
                r31 = grasp.approach.z
                r32 = grasp.binormal.z
                r33 = grasp.axis.z

                yaw = math.atan(r21/r11)
                pitch = math.atan(-r31/((r32**2 + r33**2)**(1/2)))
                roll = math.atan(r32/r33)
   
                if(roll > 0):

                    approach_x = grasp.top.x - 0.45*grasp.approach.x + 0.02*grasp.binormal.x
                    approach_y = grasp.top.y - 0.45*grasp.approach.y + 0.02*grasp.binormal.y
                    approach_z = grasp.top.z - 0.45*grasp.approach.z + 0.02*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.15*grasp.approach.x
                    pose_y = approach_y + 0.15*grasp.approach.y
                    pose_z = approach_z + 0.15*grasp.approach.z

                if(roll < 0):
                    approach_x = grasp.top.x - 0.45*grasp.approach.x - 0.02*grasp.binormal.x
                    approach_y = grasp.top.y - 0.45*grasp.approach.y - 0.02*grasp.binormal.y
                    approach_z = grasp.top.z - 0.45*grasp.approach.z - 0.02*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.15*grasp.approach.x
                    pose_y = approach_y + 0.15*grasp.approach.y
                    pose_z = approach_z + 0.15*grasp.approach.z
     

                chosen_grasp = PickGraspMsg()
                chosen_grasp.data.append(roll)
                chosen_grasp.data.append(pitch)
                chosen_grasp.data.append(yaw)
                chosen_grasp.data.append(approach_x)
                chosen_grasp.data.append(approach_y)
                chosen_grasp.data.append(approach_z)
                chosen_grasp.data.append(pose_x)
                chosen_grasp.data.append(pose_y)
                chosen_grasp.data.append(pose_z)
                self.chosen_grasp_array.grasps.append(chosen_grasp)

    def publish(self):
	if(len(self.chosen_grasp_array.grasps) > 0):
            self.pub.publish(self.chosen_grasp_array)
            print(self.chosen_grasp_array)
            rospy.sleep(2)

def main(args):
    find = FindBestGrasp()
    #find.look_down()
    while not rospy.is_shutdown():
        find.publish()
    #try:
	#rospy.spin()
    #except KeyboardInterrupt:
	#print("shutting down")

        
if __name__ == '__main__':
    main(sys.argv)

