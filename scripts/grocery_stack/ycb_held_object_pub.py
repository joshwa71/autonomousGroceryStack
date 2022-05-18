#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String, Int32, Float32
from geometry_msgs.msg import Twist

held_object = 0

def callback(data):

    global held_object
    held_object = data.data

    vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
    velocity = Twist()

    for i in range(60):
        velocity.angular.z = -1.0472/2
        vel_pub.publish(velocity)
        rospy.sleep(0.1)
        

def listener():
    rospy.init_node('held_listener', anonymous=True)
    object_sub = rospy.Subscriber('held_object', Int32, callback)
    object_pub = rospy.Publisher('held_object_int', Int32, queue_size = 1)
    while not rospy.is_shutdown():
        object_pub.publish(held_object)
        rospy.sleep(1)
        

if __name__ == '__main__':
    listener()



