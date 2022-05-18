#!/usr/bin/env python
import rospy
from final_year_pkg.msg import YcbBoxes, YcbBoxesArray
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
import cv2
from sensor_msgs.msg import Image

look_flag = True
prev_objects = []
pub_flag = False

def callback_2(data):
    global look_flag
    look_flag = data.data

def callback(data):
    list_difference = []
    item_pub = rospy.Publisher('held_object', Int32, queue_size = 1)
    global prev_objects
    global pub_flag
    objects = []

    if(pub_flag == False and look_flag == False):
        for obj in data.list:
            objects.append(obj.data[6])

        print('prev_objects')
        print(prev_objects)
        print('objects')
        print(objects)

        if(objects != prev_objects and len(prev_objects) > len(objects)):
            

            for item in prev_objects:
                if item not in objects:
                    list_difference.append(item)
            print('Tiago picked up')
            print(list_difference)

        if(len(list_difference) > 0):
            item_pub.publish(int(round(list_difference[0])))
            pub_flag = True

    prev_objects = objects
    rospy.sleep(2)
            
     
def listener():
    rospy.init_node('ycb_listener', anonymous=True)
    boxes_sub = rospy.Subscriber('/boxes', YcbBoxesArray, callback)
    look_flag_sub = rospy.Subscriber('/look_flag', Bool, callback_2)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
