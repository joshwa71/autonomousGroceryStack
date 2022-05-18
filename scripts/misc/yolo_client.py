#!/usr/bin/env python
import numpy as np
import rospy
import sys

from rospy import ServiceException, ROSInterruptException, ROSException
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest, YoloDetectionResponse
from sensor_msgs.msg import Image

class DetectionClient():
    def __init__(self):
	image_sub = rospy.Subscriber('/xtion/rgb/image_rect_color', Image, self.callback)
	rospy.wait_for_service('/yolo_detection')
	self.detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
	self.pub = rospy.Publisher('/detections', YoloDetectionResponse, queue_size=10)
	print('init')

    def callback(self, data):
	print('callback')
	result = self.detect_objects(data, 'coco', 0.7, 0.3)	
	#print(result)
	print(result.detected_objects[0])
	self.pub.publish(result.detected_objects)	
	

if __name__ == '__main__':
    rospy.init_node('yolo_client')
    detector = DetectionClient()
    rospy.spin()
