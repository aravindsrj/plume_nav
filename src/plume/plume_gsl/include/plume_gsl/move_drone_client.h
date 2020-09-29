#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "crazyflie_control/waypointAction.h"

#pragma once

struct Range
{
	double min, max;
};

class MoveDroneClient
{
	geometry_msgs::Point m_position;
	geometry_msgs::Point m_waypoint_prev;
	geometry_msgs::Point m_waypoint;

	ros::NodeHandle m_nh;
	ros::Subscriber m_pos_sub;

	actionlib::SimpleActionClient<crazyflie_control::waypointAction> m_action_client;

	Range m_xbounds, m_ybounds;

	double m_waypoint_resolution;
	double m_position_resolution;

	bool m_has_reached_waypoint;
	bool m_map_boundary_reached;

	void dronePositionCallback(const nav_msgs::Odometry::ConstPtr& msg);

	geometry_msgs::Point getDronePosition() const;

	void sendWaypoint(const geometry_msgs::Point &waypoint);

public:

	double drone_heading;
	
	MoveDroneClient();

	geometry_msgs::Point generateWayPoint(const double &waypoint_heading);

	void followDirection(const double &waypoint_heading, 
		const std::shared_ptr<double> &waypoint_res);

	void goToWaypoint(const geometry_msgs::Point &waypoint);

};