#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

marker = Marker()

def callback(msg):
    global marker

    marker.header.frame_id = "/neck"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.pose.orientation = msg.pose.pose.orientation
    marker.pose.position =msg.pose.pose.position
    pub_marker.publish(marker)

rospy.init_node('test')
localization = rospy.Subscriber('/localization/odom/1', Odometry, callback)

pub_marker = rospy.Publisher('/Schmami/marker', Marker, queue_size=100)

rospy.spin()


