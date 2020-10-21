#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from gridworld import GridWorld
import numpy as np
from geometry_msgs.msg import Point

def find_position(index, m, res):
    a = (index%m)
    b = int(index/m)

    x = a*res
    y = b*res
    
    return x,y

def callbackfn(msg):
    data = np.array(msg.data)
    max_positions = np.where(data == max(data))
    global m        # This can be implemented as class attribute
    global res      # This can be implemented as class attribute
    global max_x
    global max_y
    max_x = 0
    max_y = 0

    for ind in max_positions[0]:
        x, y = find_position(ind, m, res)
        max_x += x
        max_y += y
    max_x /= len(max_positions[0])
    max_y /= len(max_positions[0])


rospy.init_node("source_from_map")
grid = GridWorld()
m = grid.m
res = grid.res
max_x = 0
max_y = 0
rospy.Subscriber("mapping_viz",OccupancyGrid, callbackfn)
pub = rospy.Publisher("max_probability", Point, queue_size=10)

max_prob = Point()
max_prob.z = 0

r = rospy.Rate(1)

while not rospy.is_shutdown():
    max_prob.x = max_x
    max_prob.y = max_y
    pub.publish(max_prob)
    r.sleep()