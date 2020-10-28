#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "crazyflie_control/waypointAction.h"
#include "crazyflie_control/follow_directionAction.h"

#include "tf/tf.h"

#pragma once

struct Range
{
	double min, max;
	void setRange(std::vector<double> range);
};

class MoveDroneClient
{
	std::shared_ptr<ros::NodeHandle> m_nh;

	ros::Subscriber m_pos_sub;

	geometry_msgs::Point m_waypoint_prev;
	geometry_msgs::Point m_waypoint;

	actionlib::SimpleActionClient<crazyflie_control::follow_directionAction> m_action_client;
	actionlib::SimpleActionClient<crazyflie_control::waypointAction> m_waypoint_client;
	crazyflie_control::follow_directionGoal m_goal;
	crazyflie_control::waypointGoal m_waypoint_goal;

	Range m_xbounds, m_ybounds;

	double m_waypoint_resolution;
	double m_position_resolution;
	double m_drone_heading;
	double m_goal_heading;
	double m_default_velocity;
	double m_epsilon_angle;

	bool m_reached_waypoint;
	bool m_position_initialized;
	
	void dronePositionCallback(const nav_msgs::Odometry::ConstPtr& msg);

	void waypointDoneCallback(const actionlib::SimpleClientGoalState&,
		const crazyflie_control::waypointResultConstPtr&);

	void sendWaypoint(const geometry_msgs::Point &waypoint);

public:

	geometry_msgs::Point position;

	bool map_boundary_reached;
	
	MoveDroneClient();

	MoveDroneClient(ros::NodeHandle&);

	/// \brief Clips the angle between -pi and pi
	static double normalizeAngle(double);

	static double euclideanDistance(const geometry_msgs::Point&,
		const geometry_msgs::Point&);

	geometry_msgs::Point generateWayPoint(const double &waypoint_heading);

	void followDirection(const double &waypoint_heading, 
		const std::shared_ptr<double> waypoint_res);

	void followDirection(const double &waypoint_heading);

	void goToWaypoint(const geometry_msgs::Point &waypoint);

	void stopMoving();

	bool reachedWaypoint() const;

	bool initialized() const;

};