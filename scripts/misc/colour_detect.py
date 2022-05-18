#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from final_year_pkg.msg import colour_objects, rgb_boxes

class colourIdentifier():
    def __init__(self):
        self.sensitivity = 15
        sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.objects_pub = rospy.Publisher('/colour_objects', colour_objects, queue_size = 1)
        self.boxes_pub = rospy.Publisher('/rgb_boxes', rgb_boxes, queue_size = 1)
        self.objects = colour_objects()

    def callback(self, data):

	try:
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	    print(e)

	hsv_blue_lower = np.array([120-self.sensitivity, 100, 100])
	hsv_blue_upper = np.array([120+self.sensitivity, 255, 255])
        hsv_green_lower = np.array([60-self.sensitivity, 100, 100])
	hsv_green_upper = np.array([60+self.sensitivity, 255, 255])
	hsv_red_lower = np.array([0-self.sensitivity, 100, 100])
	hsv_red_upper = np.array([0+self.sensitivity, 255, 255])

        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        mask_blue = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        mask_rg = cv2.bitwise_or(mask_red, mask_green)
        mask_rgb = cv2.bitwise_or(mask_rg, mask_blue)
	
	display_image = cv2.bitwise_and(hsv_image,hsv_image, mask=mask_rgb)

        red_contours = cv2.findContours(mask_red,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_contours = cv2.findContours(mask_green,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours = cv2.findContours(mask_blue,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.objects = colour_objects()
        result_img = cv_image
        boxes_msg = rgb_boxes()
 
        print('New Iteration')

        if len(red_contours) == 0 and len(green_contours) == 0 and len(blue_contours) == 0:
            print('No objects found')


        if len(red_contours) > 0:
            x, y, w, h = cv2.boundingRect(red_contours[0])
            result_img = cv2.rectangle(result_img,(x,y),(x+w,y+h),(0, 0, 255),2)
            if(w != 0):
                boxes_msg.red.data = True
                boxes_msg.red_box = [x, y, w, h]
                global flag_red
                flag_red = True
                print('Red Object Found')
                word=String()
                word.data= 'Red'
                self.objects.objects.append(word)
  
            
        if len(green_contours) > 0:
            x, y, w, h = cv2.boundingRect(green_contours[0])
            result_img = cv2.rectangle(result_img,(x,y),(x+w,y+h),(0, 255, 0),2)
            if(w != 0):
                boxes_msg.green.data = True
                boxes_msg.green_box = [x, y, w, h]
                global flag_green
                flag_green = True
                print('Green Object Found')
                word=String()
                word.data= 'Green'
                self.objects.objects.append(word)


        if len(blue_contours) > 0:
            x, y, w, h = cv2.boundingRect(blue_contours[0])
            result_img = cv2.rectangle(result_img,(x,y),(x+w,y+h),(255, 0, 0),2)
            if(w != 0):
                boxes_msg.blue.data = True
                boxes_msg.blue_box = [x, y, w, h]
                global flag_blue
                flag_blue = True
                print('Blue Object Found')
                word=String()
                word.data= 'Blue'
                self.objects.objects.append(word)
        self.objects_pub.publish(self.objects)
        self.boxes_pub.publish(boxes_msg)

        cv2.imshow('Colour detection', result_img)
        cv2.waitKey(1) 


def main(args):
    rospy.init_node('colour_identifier', anonymous=True)
    cI = colourIdentifier()
    try:
	rospy.spin()
    except KeyboardInterrupt:
	print("shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)



