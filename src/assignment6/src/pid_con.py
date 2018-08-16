#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float64, Int16, Float64MultiArray

PI = 3.1415
set_point = PI

def angle_diff(angle1, angle2):
    diff = angle1 - angle2
    if diff > PI:
        diff -= 2*PI
    elif diff < -PI:
        diff += 2*PI
    return diff

def callback(msg):
    global set_point
    o = msg.pose.pose.orientation
    angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w]) 
    yaw = angles[2]

    error = angle_diff(set_point, yaw)

    pub_setpoint.publish(set_point)
    pub_measured.publish(yaw)
    pub_error.publish(error)

    error *= 150
    if error > 90:
        error = 90
    elif error < -90:
        error = -90

    if error < 10 and error>  -10:
        rospy.sleep(2)
        pub_speed.publish(0)
        localization.unregister()
    else:
        pub_steering.publish(error + 90)
        pub_speed.publish(150)

rospy.init_node('pid_con')
localization = rospy.Subscriber('/localization/odom/1', Odometry, callback)

pub_speed = rospy.Publisher('/AljoschaTim/speed',Int16,queue_size=100)
pub_steering = rospy.Publisher('/AljoschaTim/steering',UInt8, queue_size = 100)
pub_setpoint = rospy.Publisher('/stef/setpoint', Float32, queue_size = 1)
pub_measuered = rospy.Publisher('/stef/measured', Float32, queue_size = 1)
pub_error = rospy.Publisher('/stef/error', Float32, queue_size = 1)
rospy.spin()
