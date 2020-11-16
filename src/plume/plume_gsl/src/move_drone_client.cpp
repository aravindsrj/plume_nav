#include "plume_gsl/move_drone_client.h"

void Range::setRange(const std::vector<double> ranges)
{
	min = ranges[0];
	max = ranges[1];
}

double MoveDroneClient::normalizeAngle(double angle)
{
	angle = fmod(angle, 2*M_PI);
	angle = fmod(angle + 2*M_PI, 2*M_PI);
	if (angle > M_PI)
		angle -= 2*M_PI;
  
  return angle;
}

double MoveDroneClient::euclideanDistance(const geometry_msgs::Point& point1,
		const geometry_msgs::Point& point2)
{
  return (sqrt(pow(point1.x - point2.x, 2) 
		+ pow(point1.y - point2.y, 2)));
}

double MoveDroneClient::angularDifference(const double& a, const double& b)
{
  if (a == b)
    return 0.0;

  double diff = b - a;
  double dir = fabs(diff)/diff;

  if (diff > M_PI or diff < -M_PI)
    dir *= -1;

  return dir * acos( cos(a)*cos(b) + sin(a)*sin(b) );

}

MoveDroneClient::MoveDroneClient():
m_action_client("follow_direction",true),
m_waypoint_client("waypoint", true),
m_reached_waypoint(true),
m_position_initialized(false)
{
	// Parameters
	m_default_velocity = 0.5;
	
	m_action_client.waitForServer(ros::Duration(10.0));
	if (!m_action_client.isServerConnected())
	{
		ROS_ERROR("Move_drone_client did not connect to follow_direction server");
	}

	m_waypoint_client.waitForServer(ros::Duration(10.0));
	if (!m_waypoint_client.isServerConnected())
	{
		ROS_ERROR("Move_drone_client did not connect to waypoint server");
	}
}

MoveDroneClient::MoveDroneClient(ros::NodeHandle& nh):
m_nh(std::make_shared<ros::NodeHandle>(nh)),
m_action_client(nh, "/follow_direction",true),
m_waypoint_client(nh, "/waypoint", true),
m_reached_waypoint(true),
m_position_initialized(false),
m_epsilon_angle(5e-2),
map_boundary_reached(false)
{
	// Parameters
	m_default_velocity = 0.5;
	
	m_action_client.waitForServer(ros::Duration(10.0));
	if (!m_action_client.isServerConnected())
	{
		ROS_ERROR("Move_drone_client did not connect to follow_direction server");
	}

	m_waypoint_client.waitForServer(ros::Duration(10.0));
	if (!m_waypoint_client.isServerConnected())
	{
		ROS_ERROR("Move_drone_client did not connect to waypoint server");
	}

	m_pos_sub = m_nh->subscribe("/base_pose_ground_truth", 1, 
		&MoveDroneClient::dronePositionCallback, this);

	m_goal.velocity = 0;
}

void MoveDroneClient::dronePositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	position = msg->pose.pose.position;
	m_drone_heading = tf::getYaw(msg->pose.pose.orientation);
	m_position_initialized = true;
	// ROS_WARN("In drone position callback");
}

void MoveDroneClient::waypointDoneCallback(
		const actionlib::SimpleClientGoalState& state,
		const crazyflie_control::waypointResultConstPtr& result)
{
	if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR("Waypoint complete - error");
		return;
	}
	m_reached_waypoint = true;
	ROS_INFO("Waypoint reached");
}

void MoveDroneClient::goToWaypoint(const geometry_msgs::Point &waypoint)
{
  // TODO if at map boundary while receiving waypoint, change map_boundary to false
	// if heading is away from boundary
	std::vector<geometry_msgs::Point> points{waypoint};
	m_waypoint_goal.points = points;

	m_waypoint_client.sendGoal(m_waypoint_goal, 
		boost::bind(&MoveDroneClient::waypointDoneCallback, this, _1, _2));
	m_reached_waypoint = false;
}

void MoveDroneClient::followDirection(const double& heading)
{
	if (fabs(MoveDroneClient::angularDifference(heading,m_drone_heading)) < m_epsilon_angle 
		&& m_goal.velocity == m_default_velocity)
		return;

	m_goal_heading = heading;
	m_goal.heading = m_goal_heading;
	m_goal.velocity = m_default_velocity;

	// Send goal and check if successful
	// if (m_action_client.sendGoalAndWait(m_goal, ros::Duration(1.0)) 
	// 		!= actionlib::SimpleClientGoalState::SUCCEEDED)
	// 	ROS_ERROR("Follow_direction action not complete");
	m_action_client.sendGoal(m_goal);
	return;
}

void MoveDroneClient::stopMoving()
{
	if (m_goal.velocity == 0)
		return;
	m_goal.heading = m_goal_heading;
	m_goal.velocity = 0;
	if (m_action_client.sendGoalAndWait(m_goal, ros::Duration(1.0)) 
			!= actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_ERROR("Stop action not complete");

}

bool MoveDroneClient::reachedWaypoint() const
{
  return m_reached_waypoint;
}

bool MoveDroneClient::initialized() const
{
	return m_position_initialized;
}

bool MoveDroneClient::isMoving() const
{
	if (m_goal.velocity == 0)
		return false;
	else
		return true;	
}