#!/usr/bin/python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
import struct
import evdev
import tf2_ros
import tf2_geometry_msgs
global scaling
scaling=1.10/400.0*25.4/1000 #900 dpi, 25.4mm/inch, 10Hz rate, distance in m
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
BASE_FRAME="base_footprint"
global tfBuffer

def my_callback(event):
    global mov_x
    global mov_y
    global abs_x
    global abs_y
    vec = Vector3Stamped()
    vec.header.stamp = rospy.Time.now()
    vec.vector = Vector3(x=mov_x*scaling, y=mov_y*scaling, z=0)
    pub.publish(vec)
    #odom = Odometry()
    #odom.header.stamp = rospy.Time.now()
    #try:
    #    transf = tfBuffer.lookup_transform(BASE_FRAME, 'mouse_front', rospy.Time())
    #    vec = tf2_geometry_msgs.do_transform_vector3(vec, transf)
    #    abs_x += vec.vector.x
    #    abs_y += vec.vector.y
    #except Exception as ex:
    #    print(ex)
    #    abs_x = abs_x + mov_x*scaling
    #    abs_y = abs_y + mov_y*scaling
    #odom.header.frame_id = "mouse_front"
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    #odom.pose.pose = Pose(Point(abs_x, abs_y, 0.), Quaternion(*odom_quat))
    #odom.child_frame_id = "base_link"
    #odom.twist.twist = Twist(Vector3(mov_x, mov_y, 0), Vector3(0, 0, 0))
    #odom_pub.publish(odom)
    mov_x = 0
    mov_y = 0

device = evdev.InputDevice( "/dev/input/event1" );
rospy.init_node('mouse_front')
tfBuffer = tf2_ros.Buffer()
tf2_ros.TransformListener(tfBuffer)
print(device)
pub = rospy.Publisher('/mouse/front/rel', Vector3Stamped, queue_size=3)
odom_pub = rospy.Publisher("/mouse/front/odom", Odometry, queue_size=3)
r = rospy.Rate(10) # 10hz
rospy.Timer(rospy.Duration(0.1), my_callback)
for event in device.read_loop():
#    print(event)
    if event.type == evdev.ecodes.EV_REL:
        if event.code == 1:
            mov_x -= event.value
        if event.code == 0:
            mov_y-=event.value
