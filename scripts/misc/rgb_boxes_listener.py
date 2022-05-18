#!/usr/bin/env python
import rospy
import ros_numpy
import tf
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
from final_year_pkg.msg import colour_objects, rgb_boxes, RgbPoses, RgbPosesArray
import cv2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, Point
import sensor_msgs.point_cloud2 as pc2

pcl_array = []
pcl = PointCloud2()

def callback(msg, data):

    poses_pub = rospy.Publisher('rgb_poses', RgbPosesArray, queue_size = 1)
 
    global pcl
    pcl = data
    global pcl_array
    pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    used_pcl = pcl
    used_pcl.header.stamp = rospy.Time(0)

    t = tf.TransformListener()

    poses_array = RgbPosesArray()

    if(msg.red.data):

        red_x = msg.red_box[0]
        red_y = msg.red_box[1]
        red_w = msg.red_box[2]
        red_h = msg.red_box[3]

        full_pcl_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pixel = full_pcl_array[red_y + int(round(0.5*red_h)), red_x + int(round(0.5*red_w))]

        red_xyz = [pixel[0], pixel[1], pixel[2]]

        t.waitForTransform(used_pcl.header.frame_id, 'base_footprint', used_pcl.header.stamp, rospy.Duration(1))

        r_xyz = Point()
        r_xyz.x = red_xyz[0]
        r_xyz.y = red_xyz[1]
        r_xyz.z = red_xyz[2]
        r_xyz_stamped = PointStamped(used_pcl.header, r_xyz)
        r_xyz_transform = t.transformPoint('base_footprint', r_xyz_stamped)

        print('Red pose = ') 
        print(r_xyz_transform.point)

        red_pose = RgbPoses()
        red_pose.pose = [1, r_xyz_transform.point.x, r_xyz_transform.point.y, r_xyz_transform.point.z]

        poses_array.objects.append(red_pose)

    if(msg.green.data):

        green_x = msg.green_box[0]
        green_y = msg.green_box[1]
        green_w = msg.green_box[2]
        green_h = msg.green_box[3]

        full_pcl_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pixel = full_pcl_array[green_y + int(round(0.5*green_h)), green_x + int(round(0.5*green_w))]

        green_xyz = [pixel[0], pixel[1], pixel[2]]

        t.waitForTransform(used_pcl.header.frame_id, 'base_footprint', used_pcl.header.stamp, rospy.Duration(1))

        g_xyz = Point()
        g_xyz.x = green_xyz[0]
        g_xyz.y = green_xyz[1]
        g_xyz.z = green_xyz[2]
        g_xyz_stamped = PointStamped(used_pcl.header, g_xyz)
        g_xyz_transform = t.transformPoint('base_footprint', g_xyz_stamped)

        print('Green pose = ')
        print(g_xyz_transform.point)

        green_pose = RgbPoses()
        green_pose.pose = [2, g_xyz_transform.point.x, g_xyz_transform.point.y, g_xyz_transform.point.z]

        poses_array.objects.append(green_pose)

    if(msg.blue.data):

        blue_x = msg.blue_box[0]
        blue_y = msg.blue_box[1]
        blue_w = msg.blue_box[2]
        blue_h = msg.blue_box[3]

        full_pcl_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pixel = full_pcl_array[blue_y + int(round(0.5*blue_h)), blue_x + int(round(0.5*blue_w))]

        blue_xyz = [pixel[0], pixel[1], pixel[2]]

        t.waitForTransform(used_pcl.header.frame_id, 'base_footprint', used_pcl.header.stamp, rospy.Duration(1))

        b_xyz = Point()
        b_xyz.x = blue_xyz[0]
        b_xyz.y = blue_xyz[1]
        b_xyz.z = blue_xyz[2]
        b_xyz_stamped = PointStamped(used_pcl.header, b_xyz)
        b_xyz_transform = t.transformPoint('base_footprint', b_xyz_stamped)

        print('Blue pose = ')
        print(b_xyz_transform.point)

        blue_pose = RgbPoses()
        blue_pose.pose = [3, b_xyz_transform.point.x, b_xyz_transform.point.y, b_xyz_transform.point.z]

        poses_array.objects.append(blue_pose)

    poses_pub.publish(poses_array)
      
    rospy.sleep(1)


     
def listener():

    rospy.init_node('rgb_boxes_listener', anonymous=True)

    pcl_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2, queue_size = 1)
    rgb_boxes_sub = message_filters.Subscriber('/rgb_boxes', rgb_boxes, queue_size = 1)
    ts = message_filters.ApproximateTimeSynchronizer([rgb_boxes_sub, pcl_sub], 1, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
