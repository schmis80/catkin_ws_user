#!/usr/bin/env python
# SUBSCRIBER

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

def callback(raw_msg):
  publisher.publish(str(raw_msg))

# Initialize node
rospy.init_node("simple_node")

# Run subscriber
rospy.Subscriber("/yaw", Float32, callback)
publisher = rospy.Publisher("/assignment1_publisher_subscriber", String,queue_size=20)

rospy.spin() 
