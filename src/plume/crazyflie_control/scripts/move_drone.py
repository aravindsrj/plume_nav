#! /usr/bin/env python

''' A server that moves the drone to a specified waypoint
'''

import math
import sys
import time

import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion

from crazyflie_control.msg import waypointAction, waypointGoal, waypointResult, waypointFeedback
        

def handle_shutdown():
    print("[COMPLETED] Reached. Shutting down")


class MoveDrone:
    def __init__(self):
        self.drone_curr_x = 0.0
        self.drone_curr_y = 0.0
        self.drone_curr_heading = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self._position_epsilon = 1e-2
        self._angle_epsilon = 1e-2

        self._velocity_gain = 2
        self._angular_gain = 3

        self.vel_msg = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.ground_truth_callback)


    def angular_difference(self, a, b):
        difference = 0
        if not a == b:
            diff = b-a
            dir = abs(diff)/(diff)

            if diff > math.pi or diff < -math.pi:
                dir *= -1

            v1x = math.cos(a)
            v1y = math.sin(a)
            v2x = math.cos(b)
            v2y = math.sin(b)
            try:
                difference = dir*math.acos(v1x*v2x + v1y*v2y)
            except ValueError:
                print(v1x*v2x + v1y*v2y)
                print(v1x, v2x, v1y, v2y)
                exit()

        return difference


    def publish_cmd_vel(self):

        goal_distance = math.sqrt((self.drone_curr_x - self.goal_x) ** 2 + (self.drone_curr_y - self.goal_y) ** 2)

        if goal_distance > self._position_epsilon:

            self.has_reached_goal = False

            angle_to_goal = math.atan2((self.goal_y - self.drone_curr_y), (self.goal_x - self.drone_curr_x))
        
            if abs(self.angular_difference(angle_to_goal, self.drone_curr_heading)) > self._angle_epsilon:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self._angular_gain * self.angular_difference(self.drone_curr_heading, angle_to_goal)
            else:
                self.vel_msg.linear.x = self._velocity_gain * goal_distance
                self.vel_msg.angular.z = 0

        else:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.has_reached_goal = True
        
        self.pub.publish(self.vel_msg)


    def waypoint_action_callback(self, goal):
        
        self.has_reached_goal = False

        self.goal_x = goal.points[0].x
        self.goal_y = goal.points[0].y

        result = waypointResult()

        while not self.has_reached_goal:            
            self.publish_cmd_vel()

        result.succ_msg = "Reached"
        server.set_succeeded(result)

        # handle failure to reach goal and/or preemption. Also, send feedback. Do this using has_reached_goal

    def ground_truth_callback(self, msg):
        self.drone_curr_x = msg.pose.pose.position.x
        self.drone_curr_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.drone_curr_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == "__main__":
    rospy.init_node("move_drone")
    rospy.on_shutdown(handle_shutdown)

    moveDrone = MoveDrone()

    server = actionlib.SimpleActionServer('waypoint', waypointAction, moveDrone.waypoint_action_callback, False)
    server.start()
    
    rospy.spin()