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
ns = 'AljoschaTim'
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
    first_col = cols//2 - 25
    last_col = cols//2 + 25
    first_row = rows//2 - 25
    last_row = rows//2 + 25

    cut_array = depth_array[first_row:last_row,first_col:last_col]
    cut_array = cut_array[np.nonzero(cut_array)]
#    cut_array = depth_array[:, cols//3:2*cols//3]
#    max_depth = np.max(cut_array)
        
#    print max_depth
    is_obstacle = False
    if cut_array != []:
        max_depth = np.max(cut_array)
        #print 'Max dist:', max_depth
        if max_depth < 1500:
            is_obstacle = True
#        for y in xrange(cut_array.shape[0]):
#            for x in xrange(cut_array.shape[1]):  
#                if cut_array[y,x] < 700 and depth_array[y,x] > 0: 
#                    is_obstacle = True
#                    break
        if is_obstacle and time.time() - detected_time > 2:
            pub_lane.publish(Bool(True))
            first_col = 0
            last_cols = 2*cols//3
            detected_time = time.time()

rospy.init_node('depth')
test = rospy.Subscriber(ns+'/app/camera/depth/image_raw', Image, callback)

pub_obst = rospy.Publisher(ns+'/obstacle', Bool, queue_size=100)
pub_lane = rospy.Publisher(ns+'/lane', Bool, queue_size = 100)
rospy.spin()
