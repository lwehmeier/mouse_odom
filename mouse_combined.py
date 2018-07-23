#!/usr/bin/python
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
import struct
import tf2_ros
import tf2_geometry_msgs
BASE_FRAME= "base_footprint"
UPDATE_PERIOD = 0.1
global odom_pub
odom_pub=None
global last_front
global last_rear
last_front = Vector3Stamped()
last_rear = Vector3Stamped()
global estimated_position
global estimated_orientation
estimated_position = Vector3()
estimated_orientation = Vector3()
global tfBuffer

def callback_front(data):
    global last_front
    try:
        transf = tfBuffer.lookup_transform(BASE_FRAME, 'mouse_front', rospy.Time())
        data = tf2_geometry_msgs.do_transform_vector3(data, transf)
        last_front = data
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        print(ex)
        pass
def callback_rear(data):
        global last_rear
        try:
            transf = tfBuffer.lookup_transform(BASE_FRAME, 'mouse_rear', rospy.Time())
            data = tf2_geometry_msgs.do_transform_vector3(data, transf)
            last_rear = data
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print(ex)
            pass
def callback_odom(data):
    tgt = data.pose
    quaternion = [tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    global estimated_orientation
    estimated_orientation.z = yaw
def my_callback(event):
    try:
        t_front =  tfBuffer.lookup_transform(BASE_FRAME, 'mouse_front', rospy.Time())
        t_rear = tfBuffer.lookup_transform(BASE_FRAME, 'mouse_rear', rospy.Time())
        r = t_front.transform.translation.x
        v_front = last_front.vector
        v_rear = last_rear.vector
        print("("+str(v_front.x - v_rear.x)+","+str(v_front.y - v_rear.y)+")")
        #v = Vector3()
        #v.x = (v_front.x + v_rear.x)/2
        #v.y = (v_front.y + v_rear.y)/2
        #omega = 0 #((v_front.x -v.x) - (v_rear.x - v.x)) / 2
        travel_x = v_rear.x
        travel_y = v_rear.y
        travel_x = (travel_x * cos(estimated_orientation.z) + travel_y * sin(estimated_orientation.z));
        travel_y = (travel_x * sin(estimated_orientation.z) + travel_y * cos(estimated_orientation.z));
        estimated_position.x = estimated_position.x + travel_x
        estimated_position.y = estimated_position.y + travel_y
        #rot_travel = omega
        #yaw = rot_travel / r
        #estimated_orientation.z = estimated_orientation.z + yaw
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom_quat = tf.transformations.quaternion_from_euler(estimated_orientation.z,0,0,"rzyx")
        odom.pose.pose = Pose(Point(estimated_position.x, estimated_position.y, 0), Quaternion(*odom_quat))
        odom.child_frame_id = BASE_FRAME
        odom.twist.twist = Twist(Vector3(travel_x, travel_y, 0), Vector3(0,0,0))
        odom_pub.publish(odom)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        print(ex)
        pass

rospy.init_node('mouse_combined')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
odom_pub = rospy.Publisher("/mouse/odom", Odometry, queue_size=3)
r = rospy.Rate(5) # 10hz
rospy.Timer(rospy.Duration(0.2), my_callback)
rospy.Subscriber("/mouse/rear/rel", Vector3Stamped, callback_rear)
rospy.Subscriber("/mouse/front/rel", Vector3Stamped, callback_front)
rospy.Subscriber("/odom", Odometry, callback_odom)
rospy.spin()
