#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int16
from cv_bridge import CvBridge

bridge = CvBridge()
ns = ''
np.set_printoptions(threshold='nan')

is_obstacle = False

detected_time, last_time = time.time(), time.time()

def nothing():
    return

def callback(depth_msg):
    global detected_time, is_obstacle

    depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    depth_array = np.array(depth_image, dtype=np.uint16)
    rows, cols = depth_array.shape
#    cut_array = depth_array[:, cols//3:2*cols//3]
#    max_depth = np.max(cut_array)
        
#    print max_depth
    is_obstacle = False
    for y in xrange(0, rows, rows//3):
        for x in xrange(0,cols,cols//3):  
            if depth_array[y,x] < 500 and depth_array[y,x] > 0: 
                is_obstacle = True
                detected_time = time.time()
                break
    pub_obst.publish(Bool(is_obstacle)) 

rospy.init_node('depth')
test = rospy.Subscriber(ns+'/app/camera/depth/image_raw', Image, callback)

pub_obst = rospy.Publisher(ns+'/obstacle', Bool, queue_size=100)
pub_lane = rospy.Publisher(ns+'/lane', Int16, queue_size = 100)
rospy.spin()
