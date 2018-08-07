#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8, Float64, Int16, Float64MultiArray
import rospkg

PI = 3.1415
set_point = PI
map_size_x=600 #cm
map_size_y=400 #cm
resolution = 10 # cm
lane=1
speed_value= 900
is_shutdown = False
is_obstacle = False

last_time = time.time()
sample_time = 0.001
last_error = 0
windup_guard = 10
PTerm = 0
ITerm = 0
DTerm = 0


rospack = rospkg.RosPack()
file_path=rospack.get_path('around_force')+'/src/'
matrix = np.load(file_path+'matrix100cm_lane1.npy')

ns = ''

def drive(speed, angle):
    pub_steering.publish(angle)
    pub_speed.publish(speed)

def stop_driving():
    pub_speed.publish(0)

def angle_diff(angle1, angle2):
    diff = angle1 - angle2
    if diff > np.pi:
        diff -= 2*np.pi
    elif diff < -np.pi:
        diff += 2*np.pi
    return diff

def callback(msg):
    global speed_value, matrix, is_shutdown 
    global last_time, last_error, windup_guard, PTerm, ITerm, DTerm

    # get position and orientation
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
    kp = 1.7
    ki = 0.01
    kd = 0.01

    error = kp*np.arctan(y_car / (2.5*x_car))

    current_time = time.time()
    delta_time = current_time - last_time
    delta_error = error - last_error

    if (delta_time >= sample_time):
        PTerm = kp * error
        ITerm += error * delta_time

        if (ITerm < -windup_guard):
            ITerm = -windup_guard
        elif (ITerm > windup_guard):
            ITerm = windup_guard

        DTerm = 0.0
        if delta_time > 0:
            DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        last_time = current_time
        last_error = error

    steering = PTerm + (ki * ITerm) + (kd * DTerm)

#    print steering
    if (x_car<0):
        speed = -speed_value
    else:
         speed = speed_value

    if (steering>(np.pi)/2):
        steering = (np.pi)/2

    if (steering<-(np.pi)/2):
        steering = -(np.pi)/2

    if x_car > 0:
        speed = max(speed_value, speed * ((np.pi/3)/(abs(steering)+1)))
    steering = 90 + (180/np.pi)*steering 

    if not is_shutdown and not is_obstacle:
        drive(speed, steering)

def laneCallback(msg):
    global matrix, lane
    path = 'matrix100cm_lane'
    if lane == 1:
        lane = 2
    else:
        lane = 1
    print 'Switching to lane',lane 
    matrix = np.load(file_path+'matrix100cm_lane'+str(lane)+'.npy')

def maxSpeedCallback(msg):
    global speed_value
    speed_value = msg.data

def obstacleCallback(msg):
    global is_obstacle

    is_obstacle = msg.data

    if is_obstacle:
       stop_driving()

def shutdown():
    global is_shutdown
    stop_driving()
    is_shutdown = True

rospy.init_node('around_force')
localization = rospy.Subscriber('/localization/odom/4', Odometry, callback)
rospy.Subscriber(ns+'/lane', Bool, laneCallback)
rospy.Subscriber(ns+'/max_speed', Int16, maxSpeedCallback)
rospy.Subscriber(ns+'/obstacle', Bool, obstacleCallback)
pub_speed = rospy.Publisher(ns+'/speed',Int16,queue_size=100)
pub_steering = rospy.Publisher(ns+'/steering',UInt8, queue_size = 100)
#pub_yaw = rospy.Publisher("/desired_yaw", Float32, queue_size=100, latch=True)
#sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size=1)

rospy.on_shutdown(shutdown)
rospy.spin()


