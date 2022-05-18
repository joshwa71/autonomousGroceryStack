#!/usr/bin/env python
import rospy
import numpy as np
from final_year_pkg.msg import YcbBoxes, YcbBoxesArray, GpdInput, ItemIndex
from sensor_msgs.msg import PointCloud2, Image
from jeff_segment_objects.srv import SegmentObjects
from jeff_segment_objects.msg import Cluster, ClusterBB, ClusterBBList
from std_msgs.msg import Int32, UInt8

ycb_boxes = YcbBoxesArray()
grasp_pcl = ItemIndex()
gpd_in = GpdInput()
pcl_array = []


def callback(data):

    global ycb_boxes 
    ycb_boxes = data

def callback2(data):

    prev_cluster_area = 0
    prev_pcl = Cluster()
    prev_label = 0

    for cluster in data.clusters:
        
        for list in ycb_boxes.list:

            if not(list.data[0] > cluster.bb[2] or cluster.bb[0] > list.data[2] or list.data[1] > cluster.bb[3] or cluster.bb[1] > list.data[3]):

                x1 = max(cluster.bb[0], list.data[0])
                y1 = max(cluster.bb[1], list.data[1])
                x2 = min(cluster.bb[2], list.data[2])
                y2 = min(cluster.bb[3], list.data[3])

                IArea = (x2 - x1 + 1) * (y2 - y1 + 1)

                cluster_area = (cluster.bb[2] - cluster.bb[0] + 1) * (cluster.bb[3] - cluster.bb[1] + 1)
                ycb_area = (list.data[2] - list.data[0] + 1) * (list.data[3] - list.data[1] +1)
                iou = IArea / float(cluster_area + ycb_area - IArea)
 
                if(iou > 0.7):

                    if (cluster_area > prev_cluster_area): # need object that holds point cloud and pointcloud for gpd

                        prev_cluster_area = cluster_area
                        prev_pcl = cluster.pcl
                        prev_label = list.data[4]

    global grasp_pcl
    grasp_pcl.item = prev_label
    grasp_pcl.pcl = prev_pcl
    #print(grasp_pcl.pcl)
    

    pcl_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, callback_3)


def callback_3(data):
    global gpd_in
    gpd_in.item = grasp_pcl.item
    gpd_in.pcl.header = data.header
    gpd_in.pcl.height = data.height
    gpd_in.pcl.width = data.width
    gpd_in.pcl.fields = data.fields
    gpd_in.pcl.is_bigendian = data.is_bigendian
    gpd_in.pcl.point_step = data.point_step
    gpd_in.pcl.row_step = data.row_step
    gpd_in.pcl.is_dense = data.is_dense

    pcl2_array = np.frombuffer(data.data, dtype=np.uint8)
  
    global pcl_array
    for i in grasp_pcl.pcl.indices:
        pcl_array.append(pcl2_array[i])
    gpd_in.pcl.data = pcl_array


def yolo_listener():

    yolo_sub = rospy.Subscriber('/formatted_ycb_boxes', YcbBoxesArray, callback)

def cluster_listener():

    cluster_sub = rospy.Subscriber('object_bb', ClusterBBList, callback2)

def pcl_publisher():

    pcl_pub = rospy.Publisher('/gpd_grasp_pcl', PointCloud2, queue_size = 1)
    label_pub = rospy.Publisher('/gpd_grasp_label', Int32, queue_size = 1)

    if (len(gpd_in.pcl.data) > 0):

        while not rospy.is_shutdown():
            print('publishing')
            print(gpd_in)
            pcl_pub.publish(gpd_in.pcl)
            label_pub.publish(gpd_in.item)
            rospy.sleep(1)
        

if __name__ == '__main__':
    rospy.init_node('box_compare', anonymous=True)
    yolo_listener()
    cluster_listener()    
    while not rospy.is_shutdown():
        pcl_publisher()
    rospy.spin()
