#!/usr/bin/env python

# --- imports ---
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from math import sin, cos, radians

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    # set all values to "FREE"
    occupancy_grid.data = [0 for i in range(occupancy_grid.info.width*occupancy_grid.info.height)]
    
       

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = 100


def scanCallback(scan_msg):
    global occupancy_grid
    resetGrid()
    # convert scan measurements into an occupancy grid 
    for i in range(len(scan_msg.ranges)):
        r = scan_msg.ranges[i]
        if r != float('inf'):
            setCell(r*sin(radians(i)),r*cos(radians(i)))
    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.043

# width x height cells
occupancy_grid.info.width = 100
occupancy_grid.info.height = 100

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("/schmis80_grid", OccupancyGrid, queue_size=100)

rospy.spin()
