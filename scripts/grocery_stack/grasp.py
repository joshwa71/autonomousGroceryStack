#! /usr/bin/env python

import rospy
import sys
import tf
import moveit_commander
import actionlib
import time
import math
import copy

import geometry_msgs.msg
from gpd.msg import GraspConfigList
from std_msgs.msg import Bool, Header
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_srvs.srv import Empty, EmptyRequest
from moveit_msgs.msg import PlanningScene, Constraints, OrientationConstraint, PositionConstraint
from final_year_pkg.msg import PickGraspMsgArray, PickGraspMsg
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive

chosen_grasps = PickGraspMsgArray()
chosen_grasp = PickGraspMsg()
group = []
scene = []
clear_octomap_srv = []
client = []
head = []
head_client = []
approach_flag = False
array_flag = True
planning_flag = True
ycb_flag_pub = []
sub = []
grasp_pose_goal = geometry_msgs.msg.Pose()
move_arm_flag = False
abort_count = 0

def init():
        
        rospy.set_param('use_sim_time', 'true')        
	rospy.loginfo("Connecting to clear octomap service...")
        global clear_octomap_srv
	clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
	clear_octomap_srv.wait_for_service()
	rospy.loginfo("Connected!")

	global head_client
	head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)

	global client
	client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
	client.wait_for_server()


        moveit_commander.roscpp_initialize(sys.argv)

	group_name = "arm_torso"
        global group
	group = moveit_commander.MoveGroupCommander(group_name)
        group.set_planner_id("SBLkConfigDefault")

        
	robot = moveit_commander.RobotCommander()

        global scene
	scene = moveit_commander.PlanningSceneInterface()

	group.set_planning_time(15)

	group.clear_path_constraints()

        global grasp_flag_pub
        grasp_flag_pub = rospy.Publisher('/grasp_flag', Bool, queue_size = 1)

        global ycb_flag_pub
        ycb_flag_pub = rospy.Publisher('/ycb_flag', Bool, queue_size = 10)        
        global sub
        sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback, queue_size = 1)

        global planning_flag
  


def approach():
        print('Trying to approach object')
        ycb_flag_pub.publish(False)
        local_chosen_grasp = []
        global approach_flag
        global move_arm_flag
        global abort_count
        global chosen_grasps
        group.clear_path_constraints()
        #group.set_planner_id("SBLkConfigDefault")

        while(approach_flag == False and not rospy.is_shutdown()):
            grasps = chosen_grasps.grasps



	    for grasp in grasps:

                #if(abort_count%8 == 4):
                #    print('moving arm left')
                #    move_arm_left()
                #    group.set_planner_id("ESTkConfigDefault")
                #if(abort_count%8 == 0):
                #    print('moving arm right')
                #    move_arm_right()
                #    group.set_planner_id("ESTkConfigDefault")
                #if(abort_count%8 == 1):
                #    print('changing planner')
                #    group.set_planner_id("SBLkConfigDefault")
                #if(abort_count%8 == 2):
                #    print('changing planner')             
                #    group.set_planner_id("PRMstarkConfigDefault")
                #if(abort_count%8 == 3):
                #    print('changing planner')
                #    group.set_planner_id("LBKPIECEkConfigDefault")
                #if(abort_count%8 == 5):
                #    print('changing planner')
                #    group.set_planner_id("SBLkConfigDefault")
                #if(abort_count%8 == 6):
                #    print('changing planner')
                #    group.set_planner_id("PRMstarkConfigDefault")
                #if(abort_count%8 == 7):
                #    print('changing planner')
                #    group.set_planner_id("LBKPIECEkConfigDefault")
                ##print(grasp)
                grasp_flag_pub.publish(False)
	        approach_pose_goal = get_approach_pose(grasp)
	        group.set_pose_target(approach_pose_goal)
	        move = group.go(wait=True)
	        group.stop()
                local_chosen_grasp = grasp
                if(move):
                    print(grasp)
                    approach_flag = True
                    break
                if not(move):
                    planning_flag = True
                    abort_count += 1
                    chosen_grasps = PickGraspMsgArray() 

        global chosen_grasp
        chosen_grasp = local_chosen_grasp


def grasp():
    if(approach_flag == True):

        global grasp_pose_goal 
	grasp_pose_goal = get_grasp_pose()
        start_pose = geometry_msgs.msg.Pose()
        start_pose = get_approach_pose(chosen_grasp)

	grasp_constraints = Constraints()
        grasp_constraints.name = 'grasp'

        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = 'arm_tool_link'
        orientation_constraint.orientation = grasp_pose_goal.orientation
        orientation_constraint.absolute_x_axis_tolerance = 1
        orientation_constraint.absolute_y_axis_tolerance = 1
        orientation_constraint.absolute_z_axis_tolerance = 1
        orientation_constraint.weight = 1
        
        box = SolidPrimitive()
        box.type = 1
        box.dimensions = [0.5, 0.5, 0.1]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'arm_tool_link'
        position_constraint.target_point_offset.x = grasp_pose_goal.position.x - start_pose.position.x
        position_constraint.target_point_offset.y = grasp_pose_goal.position.y - start_pose.position.y
        position_constraint.target_point_offset.z = grasp_pose_goal.position.z - start_pose.position.z
        position_constraint.constraint_region.primitives.append(box)
        position_constraint.weight = 1

        grasp_constraints.position_constraints.append(position_constraint)
	grasp_constraints.orientation_constraints.append(orientation_constraint)

        group.set_path_constraints(grasp_constraints)
	rospy.loginfo("Clearing octomap")
	clear_octomap_srv.call(EmptyRequest())
	clear_octomap_srv.wait_for_service()    
	group.set_planner_id("PRMstarkConfigDefault")

        start_pose = geometry_msgs.msg.Pose()
        start_pose = get_approach_pose(chosen_grasp)

	mid_pose_goal = geometry_msgs.msg.Pose()	
	mid_pose_goal.orientation = grasp_pose_goal.orientation
	mid_pose_goal.position.x = start_pose.position.x + (grasp_pose_goal.position.x - start_pose.position.x)/2
	mid_pose_goal.position.y = start_pose.position.y + (grasp_pose_goal.position.y - start_pose.position.y)/2
	mid_pose_goal.position.z = start_pose.position.z + (grasp_pose_goal.position.z - start_pose.position.z)/2
        
	group.set_pose_target(mid_pose_goal)
	mid_move = group.go(wait=True)  

	group.stop() 

	group.set_pose_target(grasp_pose_goal)
	move = group.go(wait=True)  

	group.stop()    
	

def callback(data):

    chosen_grasp_array = PickGraspMsgArray()
    global array_flag
    if(len(data.grasps) > 1 and not approach_flag and planning_flag):
        grasps = data.grasps

        chosen_grasp_array = PickGraspMsgArray()
        roll_var = [1.571, 0, -1.571]
        for grasp in grasps:
             for i in roll_var:
                r21 = grasp.approach.y
                r11 = grasp.approach.x
                r31 = grasp.approach.z
                r32 = grasp.binormal.z
                r12 = grasp.binormal.x
                r22 = grasp.binormal.y
                r33 = grasp.axis.z
                r23 = grasp.axis.y
                r13 = grasp.axis.x
                roll_absolute = math.atan(r32/r33)
                yaw = math.atan(r21/r11)
                pitch = math.atan(-r31/(r32**2+r33**2)**(1/2))
                roll = math.atan(r32/r33) + i

                if(grasp.binormal.y < 0 and grasp.binormal.x < 0):
                    approach_x = grasp.top.x - 0.5*grasp.approach.x + 0.01*grasp.binormal.x
                    approach_y = grasp.top.y - 0.5*grasp.approach.y + 0.01*grasp.binormal.y
                    approach_z = grasp.top.z - 0.5*grasp.approach.z + 0.02*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.23*grasp.approach.x
                    pose_y = approach_y + 0.23*grasp.approach.y
                    pose_z = approach_z + 0.23*grasp.approach.z
                    

                if(grasp.binormal.y < 0 and grasp.binormal.x > 0):
                    approach_x = grasp.top.x - 0.5*grasp.approach.x + 0.02*grasp.binormal.x
                    approach_y = grasp.top.y - 0.5*grasp.approach.y + 0.02*grasp.binormal.y 
                    approach_z = grasp.top.z - 0.5*grasp.approach.z + 0.02*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.23*grasp.approach.x
                    pose_y = approach_y + 0.23*grasp.approach.y
                    pose_z = approach_z + 0.23*grasp.approach.z

                if(grasp.binormal.y > 0 and grasp.binormal.x < 0):
                    approach_x = grasp.top.x - 0.5*grasp.approach.x + 0.02*grasp.binormal.x
                    approach_y = grasp.top.y - 0.5*grasp.approach.y + 0.02*grasp.binormal.y 
                    approach_z = grasp.top.z - 0.5*grasp.approach.z + 0.02*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.23*grasp.approach.x
                    pose_y = approach_y + 0.23*grasp.approach.y
                    pose_z = approach_z + 0.23*grasp.approach.z

                if(grasp.binormal.y > 0 and grasp.binormal.x > 0):
                    approach_x = grasp.top.x - 0.5*grasp.approach.x + 0.03*grasp.binormal.x
                    approach_y = grasp.top.y - 0.5*grasp.approach.y + 0.03*grasp.binormal.y 
                    approach_z = grasp.top.z - 0.5*grasp.approach.z + 0.03*grasp.binormal.z #compensate for error on binormal axis

                    pose_x = approach_x + 0.23*grasp.approach.x
                    pose_y = approach_y + 0.23*grasp.approach.y
                    pose_z = approach_z + 0.23*grasp.approach.z

                if(yaw < 1.3 and yaw > -1.3):
                    if(approach_z > 0.59):#(-1.7 < roll < -1.5 or 1.7 > roll > 1.5):
                        if(pitch > 0):
                    	    chosen_grasp = PickGraspMsg()

                            #if(roll<0):
                    	    #    chosen_grasp.data.append(-1.571)
                            #else:
                            #    chosen_grasp.data.append(1.571)

                            chosen_grasp.data.append(1.571)
                    	    chosen_grasp.data.append(pitch)

                 	    chosen_grasp.data.append(yaw)
                  	    chosen_grasp.data.append(approach_x)
                  	    chosen_grasp.data.append(approach_y)
                	    chosen_grasp.data.append(approach_z)
                 	    chosen_grasp.data.append(pose_x)
                	    chosen_grasp.data.append(pose_y)
                	    chosen_grasp.data.append(pose_z)
                            chosen_grasp.data.append(roll_absolute)
                            chosen_grasp.data.append(grasp.binormal.x)
                            chosen_grasp.data.append(grasp.binormal.y)
                            chosen_grasp.data.append(grasp.binormal.z)
                 	    chosen_grasp_array.grasps.append(chosen_grasp)
             global chosen_grasps
	     chosen_grasps = chosen_grasp_array
             rospy.sleep(3)


def get_approach_pose(grasp):
	pose = PoseStamped()
	pose.header.frame_id = "base_footprint"
	pose.pose.position.x = grasp.data[3]
	pose.pose.position.y = grasp.data[4]
	pose.pose.position.z = grasp.data[5]
	quaternion = tf.transformations.quaternion_from_euler(grasp.data[0], grasp.data[1], grasp.data[2])
       	pose.pose.orientation.x = quaternion[0]
       	pose.pose.orientation.y = quaternion[1]
       	pose.pose.orientation.z = quaternion[2]
       	pose.pose.orientation.w = quaternion[3]
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation = pose.pose.orientation
	pose_goal.position = pose.pose.position
	return pose_goal

def get_grasp_pose():
	pose = PoseStamped()
	pose.header.frame_id = "base_footprint"
	pose.pose.position.x = chosen_grasp.data[6]
	pose.pose.position.y = chosen_grasp.data[7]
	pose.pose.position.z = chosen_grasp.data[8]
	quaternion = tf.transformations.quaternion_from_euler(chosen_grasp.data[0], chosen_grasp.data[1], chosen_grasp.data[2])
       	pose.pose.orientation.x = quaternion[0]
       	pose.pose.orientation.y = quaternion[1]
       	pose.pose.orientation.z = quaternion[2]
       	pose.pose.orientation.w = quaternion[3]
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation = pose.pose.orientation
	pose_goal.position = pose.pose.position
	return pose_goal

def close_gripper():
    if(approach_flag == True):
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Close gripper")
	close = PlayMotionGoal()
	close.motion_name = "close"
	close.skip_planning = False


	client.send_goal(close)
	client.wait_for_result(rospy.Duration(5))

def lift_item():
    if(approach_flag == True):

        group.clear_path_constraints()    
 
	pose = PoseStamped()
	pose.header.frame_id = "base_footprint"
	pose.pose.position.x = chosen_grasp.data[6]
	pose.pose.position.y = chosen_grasp.data[7]
	pose.pose.position.z = 1.15

	quaternion = tf.transformations.quaternion_from_euler(chosen_grasp.data[0], chosen_grasp.data[1], chosen_grasp.data[2])
       	pose.pose.orientation.x = quaternion[0]
       	pose.pose.orientation.y = quaternion[1]
       	pose.pose.orientation.z = quaternion[2]
       	pose.pose.orientation.w = quaternion[3]

	end_pose = geometry_msgs.msg.Pose()
	end_pose.orientation = pose.pose.orientation
	end_pose.position = pose.pose.position

        move_constraints = Constraints()
        move_constraints.name = 'move'

        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = 'arm_tool_link'
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1

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

        move_constraints.position_constraints.append(position_constraint)

        move_constraints.orientation_constraints.append(orientation_constraint)

        group.set_path_constraints(move_constraints)
        
        move_flag = False

        while(move_flag == False):

            group.set_planner_id("PRMstarkConfigDefault")
	    grasp_pose_goal = end_pose
	    group.set_pose_target(grasp_pose_goal)
	    move = group.go(wait=True)
	    group.stop()
            if(move):
                move_flag = True
            move_item_right()


def move_item_left():

    #group.clear_path_constraints()

    quaternion = tf.transformations.quaternion_from_euler(chosen_grasp.data[0], chosen_grasp.data[1], chosen_grasp.data[2])

    lift_pose_goal = geometry_msgs.msg.Pose()

    lift_pose_goal.orientation.x = quaternion[0]
    lift_pose_goal.orientation.y = quaternion[1]
    lift_pose_goal.orientation.z = quaternion[2]
    lift_pose_goal.orientation.w = quaternion[3]

    lift_pose_goal.position.x = 0.523543260689
    lift_pose_goal.position.y = 0.6
    lift_pose_goal.position.z = 1.15
    group.set_planner_id("PRMstarkConfigDefault")

    move_constraints = Constraints()
    move_constraints.name = 'move'

    orientation_constraint = OrientationConstraint()
    orientation_constraint.link_name = 'arm_tool_link'
    orientation_constraint.orientation = grasp_pose_goal.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1

    move_constraints.orientation_constraints.append(orientation_constraint)

    box = SolidPrimitive()
    box.type = 1
    box.dimensions = [1, 1, 0.2]

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = 'base_link'
    position_constraint.link_name = 'arm_tool_link'
    position_constraint.target_point_offset.x = lift_pose_goal.position.x - chosen_grasp.data[6]
    position_constraint.target_point_offset.y = lift_pose_goal.position.y - chosen_grasp.data[7]
    position_constraint.target_point_offset.z = lift_pose_goal.position.z - 1
    position_constraint.constraint_region.primitives.append(box)
    position_constraint.weight = 1

    move_constraints.position_constraints.append(position_constraint)

    group.set_path_constraints(move_constraints)
    
    group.set_pose_target(lift_pose_goal)
    move = group.go(wait=True)
    group.stop()
    move_flag = False

    while(move_flag == False):
        move = group.go(wait=True)
        group.stop()
        if(move):
            move_flag = True


def move_item_right():

    #group.clear_path_constraints()

    quaternion = tf.transformations.quaternion_from_euler(chosen_grasp.data[0], chosen_grasp.data[1], chosen_grasp.data[2])
    lift_pose_goal = geometry_msgs.msg.Pose()

    lift_pose_goal.orientation.x = quaternion[0]
    lift_pose_goal.orientation.y = quaternion[1]
    lift_pose_goal.orientation.z = quaternion[2]
    lift_pose_goal.orientation.w = quaternion[3]

    lift_pose_goal.position.x = 0.523543260689
    lift_pose_goal.position.y = -0.6
    lift_pose_goal.position.z = 1.15
    group.set_planner_id("PRMstarkConfigDefault")


    move_constraints = Constraints()
    move_constraints.name = 'move'

    orientation_constraint = OrientationConstraint()
    orientation_constraint.link_name = 'arm_tool_link'
    orientation_constraint.orientation = grasp_pose_goal.orientation
    orientation_constraint.absolute_x_axis_tolerance = 1
    orientation_constraint.absolute_y_axis_tolerance = 1
    orientation_constraint.absolute_z_axis_tolerance = 1
    orientation_constraint.weight = 1

    move_constraints.orientation_constraints.append(orientation_constraint)

    box = SolidPrimitive()
    box.type = 1
    box.dimensions = [1, 1, 0.3]

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = 'base_link'
    position_constraint.link_name = 'arm_tool_link'
    position_constraint.target_point_offset.x = lift_pose_goal.position.x - chosen_grasp.data[6]
    position_constraint.target_point_offset.y = lift_pose_goal.position.y - chosen_grasp.data[7]
    position_constraint.target_point_offset.z = lift_pose_goal.position.z - 1
    position_constraint.constraint_region.primitives.append(box)
    position_constraint.weight = 0.5

    move_constraints.position_constraints.append(position_constraint)

    group.set_path_constraints(move_constraints)
    group.set_pose_target(lift_pose_goal)

    move_flag = False

    while(move_flag == False):
        move = group.go(wait=True)
        group.stop()
        if(move):
            move_flag = True
        else:
            move_item_left()

def move_arm_left():
    lift_pose_goal = geometry_msgs.msg.Pose()
    #lift_pose_goal.header.frame_id = "base_footprint"
    lift_pose_goal.position.x = 0.523543260689
    lift_pose_goal.position.y = 0.55
    lift_pose_goal.position.z = 1.1127472811
    group.set_planner_id("SBLkConfigDefault")
    
    group.set_pose_target(lift_pose_goal)
    move = group.go(wait=True)
    group.stop()

def move_arm_right():

    quaternion = tf.transformations.quaternion_from_euler(1.571, 0, 0)
    lift_pose_goal = geometry_msgs.msg.Pose()
    #lift_pose_goal.header.frame_id = "base_footprint"
    lift_pose_goal.orientation.x = quaternion.x
    lift_pose_goal.orientation.y = quaternion.y
    lift_pose_goal.orientation.z = quaternion.z
    lift_pose_goal.orientation.w = quaternion.w
    lift_pose_goal.position.x = 0.523543260689
    lift_pose_goal.position.y = -0.55
    lift_pose_goal.position.z = 1.1127472811
    group.set_planner_id("SBLkConfigDefault")
    
    group.set_pose_target(lift_pose_goal)
    move = group.go(wait=True)
    group.stop()

def open_gripper():
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Open gripper")
	open_ = PlayMotionGoal()
	open_.motion_name = "open"
	open_.skip_planning = False


	client.send_goal(open_)
	client.wait_for_result(rospy.Duration(5))

def unfold_arm():
	rospy.wait_for_message("joint_states", JointState)
	rospy.loginfo("Unfolding Arm")
	unfold_ = PlayMotionGoal()
	unfold_.motion_name = "unfold_arm"
	unfold_.skip_planning = False


	client.send_goal(unfold_)
	client.wait_for_result(rospy.Duration(5))


        

def look_up():
    if(approach_flag == True):
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
	head_client.send_goal_and_wait(head_goal, rospy.Duration(15))

def look_down():

	rospy.wait_for_message('/head_controller/state', JointTrajectoryControllerState)

	head_goal = PointHeadGoal()
	head_goal.target.header.frame_id = 'base_link'
	head_goal.target.point.x = 1
	head_goal.target.point.y = 0
	head_goal.target.point.z = 0.5

	head_goal.pointing_frame = 'xtion_depth_frame'
	head_goal.pointing_axis.x = 1
	head_goal.pointing_axis.y = 0
	head_goal.pointing_axis.z = 0

	head_goal.min_duration = rospy.Duration(1)
	head_client.send_goal_and_wait(head_goal, rospy.Duration(15))
        ycb_flag_pub.publish(True)

def look_up_half():

	rospy.wait_for_message('/head_controller/state', JointTrajectoryControllerState)

	head_goal = PointHeadGoal()
	head_goal.target.header.frame_id = 'base_link'
	head_goal.target.point.x = 1.5
	head_goal.target.point.y = 0
	head_goal.target.point.z = 0.8

	head_goal.pointing_frame = 'xtion_depth_frame'
	head_goal.pointing_axis.x = 1
	head_goal.pointing_axis.y = 0
	head_goal.pointing_axis.z = 0

	head_goal.min_duration = rospy.Duration(1)
	head_client.send_goal_and_wait(head_goal, rospy.Duration(15))
        ycb_flag_pub.publish(True)


def add_cylinder():
    if(approach_flag == True):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'arm_tool_link'
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        #box_pose.pose.position.z = 0.09
        box_name = 'cylinder'
        scene.add_box(box_name, box_pose, size=(0.06, 0.25, 0.06))
        grasping_group = 'arm_torso'
        touch_links = ['arm_7_link','arm_tool_link', 'gripper_link', 'gripper_grasping_frame', 'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_tool_link']
        scene.attach_box('arm_tool_link', 'cylinder', touch_links=touch_links)        
        group.attach_object('cylinder', link_name= 'arm_tool_link', touch_links = touch_links)
        moveit_commander.roscpp_shutdown()   

def kill_movegroup():
    if(approach_flag == True):
        moveit_commander.roscpp_shutdown()
        

if __name__ == '__main__':
	rospy.init_node('grasp', anonymous=True)
	init()
        #joint_controller()
        #unfold_arm()
        open_gripper()
        approach()
        look_up()
        grasp()
        close_gripper()
        lift_item()
        move_arm_right()
        #move_item_left()
        look_down()
        kill_movegroup()
        rospy.spin()


