#!/usr/bin/env python
import rospy
import ros_numpy
import tf
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
from final_year_pkg.msg import colour_objects, rgb_boxes, RgbPoses, RgbPosesArray
from final_year_pkg.msg import YcbBoxes, YcbBoxesArray
import cv2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, Point
import sensor_msgs.point_cloud2 as pc2

pcl_array = []
pcl = PointCloud2()

def callback(msg, data):

    poses_pub = rospy.Publisher('ycb_poses', RgbPosesArray, queue_size = 1)
 
    global pcl
    pcl = data
    global pcl_array
    pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    used_pcl = pcl
    used_pcl.header.stamp = rospy.Time(0)

    t = tf.TransformListener()

    poses_array = RgbPosesArray()

    for item in msg.list:

        object_label = item.data[6]
        object_x = item.data[0]*640
        object_y = item.data[1]*480
        object_w = (item.data[2] - item.data[0])*640
        object_h = (item.data[3] - item.data[1])*480

        full_pcl_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pixel = full_pcl_array[int(round(object_y)) + int(round(0.5*object_h)), int(round(object_x)) + int(round(0.5*object_w))]

        object_xyz = [pixel[0], pixel[1], pixel[2]]

        t.waitForTransform(used_pcl.header.frame_id, 'base_footprint', used_pcl.header.stamp, rospy.Duration(1))

        o_xyz = Point()
        o_xyz.x = object_xyz[0]
        o_xyz.y = object_xyz[1]
        o_xyz.z = object_xyz[2]
        o_xyz_stamped = PointStamped(used_pcl.header, o_xyz)
        o_xyz_transform = t.transformPoint('base_footprint', o_xyz_stamped)
        print('------------------')
        print('object_label =')
        print(object_label)
        print(' pose = ') 
        print(o_xyz_transform.point)

        object_pose = RgbPoses()
        object_pose.pose = [object_label, o_xyz_transform.point.x, o_xyz_transform.point.y, o_xyz_transform.point.z]

        poses_array.objects.append(object_pose)

    poses_pub.publish(poses_array)
      
    rospy.sleep(1)


     
def listener():

    rospy.init_node('rgb_boxes_listener', anonymous=True)

    pcl_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2, queue_size = 1)
    ycb_boxes_sub = message_filters.Subscriber('/boxes', YcbBoxesArray, queue_size = 1)
    ts = message_filters.ApproximateTimeSynchronizer([ycb_boxes_sub, pcl_sub], 1, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
