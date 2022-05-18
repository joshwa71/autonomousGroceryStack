#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String
from final_year_pkg.msg import colour_objects
import cv2
from sensor_msgs.msg import Image

grasp_flag = True
prev_objects = []
detect_flag = True

def callback_2(data):
    global grasp_flag
    grasp_flag = data.data
    if (data.data == False):
        print('Tiago grabbing object, disabling detection')
    if (data.data == True):
        print('Grasping completed, rescanning table')

def callback(data):

    object_pub = rospy.Publisher('held_object', String, queue_size = 1)
    global detect_flag  
    if(grasp_flag and detect_flag == True):
        global prev_objects

        #print(data)
        objects = data.objects
        print('new')
        print('prev_objects')
        print(prev_objects)
        print('objects')
        print(objects)
        if(objects != prev_objects and len(prev_objects) > len(objects) ):
            list_difference = []
            for item in prev_objects:
                if item not in objects:
                    list_difference.append(item)
            print('Tiago picked up')
            print(list_difference)
            detect_flag = False
            held_object = String()
            #held_object.data = list_difference[0].data
            object_pub.publish(list_difference[0].data)
            
        prev_objects = objects
        rospy.sleep(2)
            
     
def listener():
    rospy.init_node('colour_listener', anonymous=True)
    boxes_sub = rospy.Subscriber('/colour_objects', colour_objects, callback)
    print('creating sub')
    flag_sub = rospy.Subscriber('/ycb_flag', Bool, callback_2, queue_size = 1)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
