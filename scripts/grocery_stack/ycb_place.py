#! /usr/bin/env python

import rospy
import sys
import tf
import moveit_commander
import actionlib
import time
import math
import geometry_msgs.msg

from std_msgs.msg import Bool, Header, String, Int32
from geometry_msgs.msg import PoseStamped, WrenchStamped, Twist
from moveit_msgs.msg import PlanningScene, Constraints, OrientationConstraint, PositionConstraint
from final_year_pkg.msg import colour_objects, rgb_boxes, RgbPoses, RgbPosesArray
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest
from shape_msgs.msg import SolidPrimitive


class Place():

    def __init__(self):

        #moveit_commander.roscpp_shutdown()
        #ospy.sleep(2)

        moveit_commander.roscpp_initialize(sys.argv)


        rospy.set_param('use_sim_time', 'true')

	self.group_name = "arm_torso"
	self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_planning_time(15)
	self.group.clear_path_constraints()
        self.group.goal_tolerance = 0.5

	self.robot = moveit_commander.RobotCommander()
	self.scene = moveit_commander.PlanningSceneInterface()

        self.pose_sub = rospy.Subscriber('ycb_poses', RgbPosesArray, self.callback, queue_size = 1)
        self.held_object = 3

        self.held_object_flag = False

        self.pose_data = RgbPosesArray()
        
        self.callback_flag = False

        self.pose_goal = geometry_msgs.msg.Pose()

        self.place_flag = False

        self.group.set_end_effector_link('arm_tool_link')
        self.group.clear_path_constraints()

        self.held_object_sub = rospy.Subscriber('held_object_int',  Int32, self.callback_2, queue_size = 1)

  	self.client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
	self.client.wait_for_server()

        self.abort_count = 0

	self.client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
	self.client.wait_for_server()

	self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
	self.clear_octomap_srv.wait_for_service()



    def callback(self, data):

        pose_data = data

        for obj in data.objects:

            if(obj.pose[0] == self.held_object and self.place_flag == False):

                pose = PoseStamped()
	        pose.header.frame_id = "base_footprint"
	        pose.pose.position.x = obj.pose[1] - 0.35
	        pose.pose.position.y = obj.pose[2]
	        pose.pose.position.z = obj.pose[3] + 0.09
	        quaternion = tf.transformations.quaternion_from_euler(1.571, 0, 0)
       	        pose.pose.orientation.x = quaternion[0]
       	        pose.pose.orientation.y = quaternion[1]
       	        pose.pose.orientation.z = quaternion[2]
       	        pose.pose.orientation.w = quaternion[3]
	        self.pose_goal = geometry_msgs.msg.Pose()
	        self.pose_goal.orientation = pose.pose.orientation
	        self.pose_goal.position = pose.pose.position
                self.callback_flag = True      
                self.place() 

    def callback_2(self, msg):

        if(self.held_object_flag == False):
            print('Assigning held object')
            self.held_object = msg.data
            self.held_object_flag = True
	
    def place(self):
        
        print('Trying to place object')

        while(self.callback_flag == True and not rospy.is_shutdown() and self.place_flag == False):
            self.group.clear_path_constraints()

            box = SolidPrimitive()
            box.type = 1
            box.dimensions = [1, 1, 0.3]

            move_constraints = Constraints()
            move_constraints.name = 'move'

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'base_link'
            position_constraint.link_name = 'arm_tool_link'
            position_constraint.target_point_offset.x = self.pose_goal.position.x - 0.523543260689
            position_constraint.target_point_offset.y = self.pose_goal.position.y  + 0.6
            position_constraint.target_point_offset.z = self.pose_goal.position.z - 1.2
            position_constraint.constraint_region.primitives.append(box)
            position_constraint.weight = 1

            move_constraints.position_constraints.append(position_constraint)

            self.group.set_path_constraints(move_constraints)

            move_flag = False
            self.group.set_pose_target(self.pose_goal)
            while(move_flag == False):

                
                move = self.group.go(wait=True)
                self.group.stop()
                if(move):
                    self.place_flag = True
                    self.open_gripper()

    def lower(self):

        print('lower')

        self.group.clear_path_constraints()

        box = SolidPrimitive()
        box.type = 1
        box.dimensions = [0.1, 0.1, 0.4]

        move_constraints = Constraints()
        move_constraints.name = 'move'

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'arm_tool_link'
        position_constraint.target_point_offset.x = 0
        position_constraint.target_point_offset.y = 0
        position_constraint.target_point_offset.z = 0.15
        position_constraint.constraint_region.primitives.append(box)
        position_constraint.weight = 1

        move_constraints.position_constraints.append(position_constraint)
        self.group.set_path_constraints(move_constraints)


        final_goal = geometry_msgs.msg.Pose()
        final_goal.orientation = self.pose_goal.orientation
        final_goal.position.x = self.pose_goal.position.x
        final_goal.position.y = self.pose_goal.position.y
        final_goal.position.z = self.pose_goal.position.z - 0.25

        move_flag = False

        self.group.set_pose_target(final_goal)
        move = self.group.go(wait=True)
        self.group.stop()
        self.place_flag = True
        self.open_gripper()

    
    def open_gripper(self):
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Open gripper")
	open_ = PlayMotionGoal()
	open_.motion_name = "open"
	open_.skip_planning = False
        self.client.send_goal(open_)
	self.client.wait_for_result()
        self.lift()


    def lift(self):

        self.group.clear_path_constraints()

        pose = geometry_msgs.msg.Pose() 
	pose.orientation = self.pose_goal.orientation 
	pose.position = self.pose_goal.position 
        pose.position.z = 1

        box = SolidPrimitive()
        box.type = 1
        box.dimensions = [0.2, 0.2, 1]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'arm_tool_link'
        position_constraint.target_point_offset.x = 0
        position_constraint.target_point_offset.y = 0
        position_constraint.target_point_offset.z = 1
        position_constraint.constraint_region.primitives.append(box)
        position_constraint.weight = 1


        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = 'arm_tool_link'
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1
        
        move_constraints = Constraints()
        move_constraints.name = 'move'

        move_constraints.position_constraints.append(position_constraint)
        move_constraints.orientation_constraints.append(orientation_constraint)
        self.group.set_path_constraints(move_constraints)
        
        move_flag = False

        while(move_flag == False):

            self.group.set_planner_id("PRMstarkConfigDefault")
	    grasp_pose_goal = pose
	    self.group.set_pose_target(grasp_pose_goal)
	    move = self.group.go(wait=True)
	    self.group.stop()
            if(move):
                
                self.move_arm_right()
                move_flag = True


    def move_arm_right(self):

        self.group.clear_path_constraints()

        lift_pose_goal = geometry_msgs.msg.Pose()

        lift_pose_goal.orientation.w = 1
        lift_pose_goal.position.x = 0.523543260689
        lift_pose_goal.position.y = -0.55
        lift_pose_goal.position.z = 1.1127472811

        move_flag = False

        while(move_flag == False):

            self.group.set_planner_id("PRMstarkConfigDefault")
	    grasp_pose_goal = lift_pose_goal
	    self.group.set_pose_target(grasp_pose_goal)
	    move = self.group.go(wait=True)
	    self.group.stop()
            if(move):
                self.turn()
                move_flag = True


    def turn(self):
        vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        velocity = Twist()

        for i in range(30):
            velocity.angular.z = -1.0472
            vel_pub.publish(velocity)
            rospy.sleep(0.1)
        moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
	rospy.init_node('place', anonymous=True)
        p = Place()
        rospy.spin()


