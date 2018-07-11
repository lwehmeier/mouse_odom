#!/usr/bin/python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import struct
import evdev
global scaling
scaling=1.0/200.0*25.4/1000 #900 dpi, 25.4mm/inch, 10Hz rate, distance in m
global mov_y
global mov_x
global abs_x
global abs_y
global pub
global odom_pub
mov_x=0
mov_y=0
abs_x=0
abs_y=0
pub=None

def my_callback(event):
    global mov_x
    global mov_y
    global abs_x
    global abs_y
    abs_x = abs_x + mov_x*scaling
    abs_y = abs_y + mov_y*scaling
    pub.publish(Vector3(x=mov_x*scaling, y=mov_y*scaling, z=0))
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "mouse_odom"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    odom.pose.pose = Pose(Point(abs_x, abs_y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(mov_x, mov_y, 0), Vector3(0, 0, 0))
    odom_pub.publish(odom)
    mov_x = 0
    mov_y = 0

device = evdev.InputDevice( "/dev/input/event1" );
rospy.init_node('mouse_odom')
print(device)
pub = rospy.Publisher('/mouse/rel', Vector3, queue_size=3)
odom_pub = rospy.Publisher("/mouse/odom", Odometry, queue_size=3)
r = rospy.Rate(10) # 10hz
rospy.Timer(rospy.Duration(0.1), my_callback)
for event in device.read_loop():
#    print(event)
    if event.type == evdev.ecodes.EV_REL:
        if event.code == 0:
            mov_x += event.value
        if event.code == 1:
            mov_y+=event.value
