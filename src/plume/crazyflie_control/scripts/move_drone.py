#! /usr/bin/env python

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
        self._angular_gain = 0.5

        self.has_reached_goal = False

        self.actionlib_server = actionlib.SimpleActionServer('waypoint_heuristic', waypointAction, self.waypoint_callback, False)
        self.actionlib_server.start()

        self.vel_pub = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, moveDrone.ground_truth_callback)


    def angular_difference(self, a, b):
        normalized_angle = (a - b) % (2 * math.pi)
        difference = min(2 * math.pi - normalized_angle, normalized_angle)
        return difference


    def publish_cmd_vel(self):

        goal_distance = math.sqrt((self.drone_curr_x - self.goal_x) ** 2 + (self.drone_curr_y - self.goal_y) ** 2)

        if goal_distance > self._position_epsilon:

            self.has_reached_goal = False

            angle_to_goal = math.atan2((self.goal_y - self.drone_curr_y), (self.goal_x - self.drone_curr_x))
        
            if abs(angle_to_goal - self.drone_curr_heading) > self._angle_epsilon:
                vel_msg.linear.x = 0
                vel_msg.angular.z = self._velocity_gain * self.angular_difference(self.drone_curr_heading, angle_to_goal)
            else:
                vel_msg.linear.x = self._angular_gain * goal_distance
                vel_msg.angular.z = 0

        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.has_reached_goal = True
        
        self.pub.publish(vel_msg)


    def waypoint_action_callback(self, goal):
        # IMP!!!!!!!!!!!!!!!
        # Assign goal here from goal msg
        self.goal_x = goal.points[0].x
        self.goal_y = goal.points[0].y

        # handle failure to reach goal and/or preemption. Also, send feedback. Do this using has_reached_goal

    def ground_truth_callback(self, msg):
        self.drone_curr_x = msg.pose.pose.position.x
        self.drone_curr_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.position.orientation
        _, _, self.drone_curr_heading = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == "__main__":
    rospy.init_node("move_drone")
    rospy.on_shutdown(handle_shutdown)

    moveDrone = MoveDrone()

    server = actionlib.SimpleActionServer('waypoints', waypointAction, moveDrone.waypoint_action_callback, False)
    
    rospy.spin()