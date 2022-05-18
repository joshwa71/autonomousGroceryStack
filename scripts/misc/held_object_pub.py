#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist

held_object = 0

def callback(data):

    global held_object
    print(data)

    if(data.data == 'Red'):
        held_object = 1
    if(data.data == 'Green'):
        held_object = 2
    if(data.data == 'Blue'):
        held_object = 3

    vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
    velocity = Twist()

    for i in range(32):
        velocity.angular.z = -1.0472
        vel_pub.publish(velocity)
        rospy.sleep(0.1)
        

def listener():
    rospy.init_node('held_listener', anonymous=True)
    object_sub = rospy.Subscriber('held_object', String, callback)
    object_pub = rospy.Publisher('held_object_int', Int32, queue_size = 1)
    while not rospy.is_shutdown():
        object_pub.publish(held_object)
        rospy.sleep(1)
        

if __name__ == '__main__':
    listener()



