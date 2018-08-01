#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float64, Int16, Float64MultiArray
import rospkg

PI = 3.1415
set_point = PI
map_size_x=600 #cm
map_size_y=400 #cm
resolution = 10 # cm
lane=1
speed_value= 400
is_shutdown = False

rospack = rospkg.RosPack()
file_path=rospack.get_path('around_force')+'/src/'
matrix = np.load(file_path+'matrix100cm_lane2.npy')

def angle_diff(angle1, angle2):
    diff = angle1 - angle2
    if diff > np.pi:
        diff -= 2*np.pi
    elif diff < -np.pi:
        diff += 2*np.pi
    return diff

def callback(msg):
    global speed_value, matrix, is_shutdown

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    o = msg.pose.pose.orientation
    angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w]) 
    yaw = angles[2]

    x_ind = np.int(x*resolution)
    y_ind = np.int(y*resolution)

    if x_ind < 0:
        x_ind = 0
    if x_ind > map_size_x/resolution -1:
        x_ind = map_size_x/resolution -1

    if y_ind < 0:
        y_ind = 0
    if y_ind > map_size_y/resolution -1:
        y_ind = map_size_y/resolution -1

    x_map, y_map = matrix[x_ind, y_ind]
    x_car = np.cos(yaw)*x_map + np.sin(yaw)*y_map
    y_car = -np.sin(yaw)*x_map + np.cos(yaw)*y_map
    kp = 4

    steering = kp*np.arctan(y_car / (2.5*x_car))

    if (x_car<0):
        speed = -speed_value
#        if (y_car>0):
#        	steering = -np.pi/2
#        if (y_car<0):
#        	steering = np.pi/2
    else:
         speed = speed_value
#        if (y_car<0):
#        	steering = -np.pi/2
#        if (y_car>0):
#        	steering = np.pi/2

    if (steering>(np.pi)/2):
        steering = (np.pi)/2

    if (steering<-(np.pi)/2):
        steering = -(np.pi)/2

    if x_car > 0:
        speed = max(speed_value, speed * ((np.pi/3)/(abs(steering)+1)))
    steering = 90 + (180/np.pi)*steering 
  #  if steering > 90:
  #      steering = 90
  #  elif steering < -90:
  #      steering = -90
    if not is_shutdown:
        pub_steering.publish(steering)
        pub_speed.publish(speed)

def laneCallback(msg):
    global matrix
    print msg.data
    if msg.data == 1:
        matrix = np.load(file_path+'matrix100cm_lane1.npy')
    elif msg.data == 2:
        matrix = np.load(file_path+'matrix100cm_lane2.npy')

def shutdown():
    global is_shutdown
    pub_speed.publish(0)
    is_shutdown = True

rospy.init_node('around_force')
localization = rospy.Subscriber('/localization/odom/1', Odometry, callback)
rospy.Subscriber('/Schmami/lane', UInt8, laneCallback)

pub_speed = rospy.Publisher('/Schmami/speed',Int16,queue_size=100)
pub_steering = rospy.Publisher('/Schmami/steering',UInt8, queue_size = 100)
#pub_yaw = rospy.Publisher("/desired_yaw", Float32, queue_size=100, latch=True)
#sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size=1)

rospy.on_shutdown(shutdown)
rospy.spin()


