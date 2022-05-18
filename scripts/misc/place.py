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
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_msgs.msg import PlanningScene, Constraints, OrientationConstraint
from final_year_pkg.msg import colour_objects, rgb_boxes, RgbPoses, RgbPosesArray
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest


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

        self.pose_sub = rospy.Subscriber('rgb_poses', RgbPosesArray, self.callback, queue_size = 1)
        self.held_object = 0

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
	        pose.pose.position.x = obj.pose[1] - 0.25
	        pose.pose.position.y = obj.pose[2]
	        pose.pose.position.z = obj.pose[3] + 0.5
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

        while(self.callback_flag == True and not rospy.is_shutdown()):

            #self.scene.remove_world_object('cylinder')  
            box_pose = PoseStamped()
            box_pose.header.frame_id = 'arm_tool_link'
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = 0.2
            #box_pose.pose.position.z = 0.09
            box_name = 'cylinder'
            self.scene.add_box(box_name, box_pose, size=(0.06, 0.25, 0.06))
            
            grasping_group = 'arm_torso'
            touch_links = ['arm_7_link','arm_tool_link', 'gripper_link', 'gripper_grasping_frame', 'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_tool_link']
            self.scene.attach_box('arm_tool_link', 'cylinder', touch_links=touch_links)        
            self.group.attach_object('cylinder', link_name= 'arm_tool_link', touch_links = touch_links)  
            rospy.sleep(2)

            if(self.pose_goal.position.y < 0):
                self.pre_place()


	    place_constraints = Constraints()
            place_constraints.name = 'place'

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'base_footprint'
            orientation_constraint.link_name = 'arm_tool_link'
            orientation_constraint.orientation = self.pose_goal.orientation
            orientation_constraint.absolute_x_axis_tolerance = 5
            orientation_constraint.absolute_y_axis_tolerance = 5
            orientation_constraint.absolute_z_axis_tolerance = 5
            orientation_constraint.weight = 1

	    place_constraints.orientation_constraints.append(orientation_constraint)

            self.group.set_path_constraints(place_constraints)

            #print(self.pose_goal)
            #print(self.scene.get_attached_objects())
            #print(self.robot.get_link_names())
            place_pose_goal = self.pose_goal
 
            self.group.set_planner_id("SBLkConfigDefault")
            self.group.set_pose_target(place_pose_goal)
            self.group.clear_path_constraints()
            move = self.group.go(wait=True)
            self.group.stop()
            
            if(move):
                self.callback_flag = False
                print('Tiago moved to place pose')
                self.place_flag = True
                self.open_gripper()
                moveit_commander.roscpp_shutdown()
            else:
                self.scene.remove_world_object('cylinder')
                self.scene.remove_attached_object('cylinder')
                if(self.abort_count%8 == 4):
                    print('moving arm left')
                    self.pre_place('left')
                    self.group.set_planner_id("ESTkConfigDefault")
                if(self.abort_count%8 == 0):
                    print('moving arm right')
                    self.pre_place('right')
                    self.group.set_planner_id("ESTkConfigDefault")
                if(self.abort_count%8 == 1):
                    print('changing planner')
                    self.group.set_planner_id("SBLkConfigDefault")
                if(self.abort_count%8 == 2):
                    print('changing planner')             
                    self.group.set_planner_id("PRMstarkConfigDefault")
                if(self.abort_count%8 == 3):
                    print('changing planner')
                    self.group.set_planner_id("LBKPIECEkConfigDefault")
                if(self.abort_count%8 == 5):
                    print('changing planner')
                    self.group.set_planner_id("SBLkConfigDefault")
                if(self.abort_count%8 == 6):
                    print('changing planner')
                    self.group.set_planner_id("PRMstarkConfigDefault")
                if(self.abort_count%8 == 7):
                    print('changing planner')
                    self.group.set_planner_id("LBKPIECEkConfigDefault")
                    self.abort_count +=1

    def refresh_octomap(self):
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Refreshing octomap")
	unfold_ = PlayMotionGoal()
	unfold_.motion_name = "head_look_around"
	unfold_.skip_planning = False
	self.client.send_goal(unfold_)
	self.client.wait_for_result()

    def reset_scene(self):
        print('resetting_scene')
        #self.scene.clear()
        print(self.abort_count)
	self.clear_octomap_srv.call(EmptyRequest())
	self.clear_octomap_srv.wait_for_service()   
        self.scene.remove_world_object('cylinder')   
        self.refresh_octomap()

    def pre_place(self, direction):
        lift_pose_goal = geometry_msgs.msg.Pose()

        if(direction == 'left'):
            lift_pose_goal.orientation.w = 1
            lift_pose_goal.position.x = 0.523543260689
            lift_pose_goal.position.y = -0.55
            lift_pose_goal.position.z = 1.1127472811
        if(direction == 'right'):
            lift_pose_goal.orientation.w = 1
            lift_pose_goal.position.x = 0.523543260689
            lift_pose_goal.position.y = 0.55
            lift_pose_goal.position.z = 1.1127472811 

        move_constraints = Constraints()
        move_constraints.name = 'move'

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_footprint'
        orientation_constraint.link_name = 'arm_tool_link'
        orientation_constraint.orientation.w = 1
        orientation_constraint.absolute_x_axis_tolerance = 1
        orientation_constraint.absolute_y_axis_tolerance = 1
        orientation_constraint.absolute_z_axis_tolerance = 1
        orientation_constraint.weight = 1

        move_constraints.orientation_constraints.append(orientation_constraint)

        self.group.set_path_constraints(move_constraints)
    
        self.group.set_pose_target(lift_pose_goal)
        move = self.group.go(wait=True)
        self.group.stop()
        
    def open_gripper(self):
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Open gripper")
	open_ = PlayMotionGoal()
	open_.motion_name = "open"
	open_.skip_planning = False
        self.client.send_goal(open_)
	self.client.wait_for_result(rospy.Duration(5))

    def lift(self):

        self.group.clear_path_constraints()

        pose = geometry_msgs.Pose() 
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
        orientation_constraint.orientation = poseorientation
        orientation_constraint.absolute_x_axis_tolerance = 1
        orientation_constraint.absolute_y_axis_tolerance = 1
        orientation_constraint.absolute_z_axis_tolerance = 1
        orientation_constraint.weight = 1
        
        move_constraints = Constraints()
        move_constraints.name = 'move'

        move_constraints.position_constraints.append(position_constraint)
        move_constraints.orientation_constraints.append(orientation_constraint)
        self.group.set_path_constraints(move_constraints)
        
        move_flag = False

        while(move_flag == False):

            group.set_planner_id("PRMstarkConfigDefault")
	    grasp_pose_goal = pose
	    group.set_pose_target(grasp_pose_goal)
	    move = group.go(wait=True)
	    group.stop()
            if(move):
                move_flag = True

if __name__ == '__main__':
	rospy.init_node('place', anonymous=True)
        p = Place()
        rospy.spin()


