#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Int16
from cv_bridge import CvBridge

bridge = CvBridge()
ns = ''

def callback(depth_msg):
    depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    depth_array = np.array(depth_image, dtype=np.uint16)
    image_np = cv2.imdecode(depth_array, cv2.IMREAD_COLOR)
    rows, cols = depth_array.shape
    center = depth_array[:, cols//3:2*cols//3]
    max_depth = np.max(center)
    print max_depth
    if max_depth > 0 and max_depth < 500:
       pub_obst.publish(1)
    else:
       if False and max_depth < 800:
           pub_lane.publish(1)
       pub_obst.publish(0)
    
rospy.init_node('depth')
rospy.Subscriber(ns+'/app/camera/depth/image_raw', Image, callback)

pub_obst = rospy.Publisher(ns+'/obstacle', UInt8, queue_size=100)
pub_lane = rospy.Publisher(ns+'/lane', Int16, queue_size = 100)
rospy.spin()
