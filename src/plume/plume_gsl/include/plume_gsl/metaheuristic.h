#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "plume_gsl/move_drone_client.h"
#include "crazyflie_control/waypointAction.h"
#include "plume_gsl/rasterScanAction.h"

#include "tf/transform_listener.h"

#pragma once

enum Algorithm
{
	FOLLOW_WIND,
	UPWIND,
	ZIGZAG,
	METAHEURISTIC
};
class Localization
{
	Algorithm m_algorithm;

	std::shared_ptr<double> m_waypoint_res;

	actionlib::SimpleActionClient<crazyflie_control::waypointAction> m_waypoint_client;
	actionlib::SimpleActionClient<plume_gsl::rasterScanAction> m_raster_client;

	Range m_resolution_range;
	Range m_concentration_range;

	double m_waypoint_heading;
	double m_alpha;
	double m_position_epsilon;
	double m_conc_grad_epsilon;
	double m_concentration_epsilon;
	double m_probability_threshold;
	double m_maintain_dir_prob;
	double m_max_conc_value;

	bool m_max_prob_message_received;

	tf::TransformListener m_tf;

	std::string m_fixed_frame;
	std::string m_anemo_frame;

	
};