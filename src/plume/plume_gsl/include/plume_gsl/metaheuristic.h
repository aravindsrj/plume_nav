#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "plume_gsl/move_drone_client.h"
#include "plume_gsl/reading_history.h"
#include "plume_gsl/rasterScanAction.h"

#include "crazyflie_control/waypointAction.h"
#include "olfaction_msgs/anemometer.h"
#include "olfaction_msgs/gas_sensor.h"

#include "tf/transform_listener.h"

#include <queue>
#include <deque>

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
	ros::NodeHandle m_nh;

	ros::Subscriber m_wind_sub;
	ros::Subscriber m_gas_sub;
	ros::Subscriber m_prob_sub;
	
	Algorithm m_algorithm;
	MoveDroneClient m_drone;

	std::shared_ptr<double> m_waypoint_res;

	actionlib::SimpleActionClient<crazyflie_control::waypointAction> m_waypoint_client;
	actionlib::SimpleActionClient<plume_gsl::rasterScanAction> m_raster_client;

	Range m_resolution_range;
	Range m_concentration_range;

	geometry_msgs::Point m_max_source_probability;
	geometry_msgs::Point m_max_concentration_at;
	geometry_msgs::Point m_previous_position;
	olfaction_msgs::gas_sensor m_current_gas_concentration;
	
	unsigned int m_plume_lost_counter;
	double m_waypoint_heading;

	/// \brief Zigzag angle to be used in zigzag algorithm
	double m_alpha;
	double m_epsilon_position;
	double m_epsilon_conc_grad;
	double m_min_concentration;
	double m_probability_threshold;
	double m_maintain_dir_prob;
	double m_max_concentration_value;
	double m_distance_from_waypoint;

	bool m_max_prob_message_received;
	bool m_moving_to_source;
	bool m_lost_plume;
	bool m_raster_scan_complete;
	bool m_initial_scan_complete;

	tf::TransformListener m_tf;

	std::string m_fixed_frame;
	std::string m_anemo_frame;

	double m_Temperature;
	double m_delta_temp;
	double m_meta_std;
	double m_lost_distance;

	ReadingHistory<std::pair<double, geometry_msgs::Point>> m_concentration_points;
	ReadingHistory<double> m_concentration_history;
	ReadingHistory<double> m_wind_dir_history;

	void calcWaypointSlopeIntercept();

	bool callRasterScan(const double &distance);

	void changeTemperature();

	double checkGradient() const;

	void concentrationCallback(const olfaction_msgs::gas_sensor::ConstPtr &msg);

	void declareSourceCondition();

	double euclideanDistance(const geometry_msgs::Point&,
		const geometry_msgs::Point&) const;

	void getHeuristicMeta();

	bool getInitialHeuristic();

	void getNewHeuristic();

	void getNormalHeuristic();

	void goToMaxConcentration();

	void maxSourceProbabilityCallback(const geometry_msgs::Point::ConstPtr &msg);

	/// \brief Clips the angle between -pi and pi
	void normalize_angle(double &angle) const;

	void rasterDone();

	/// \brief Calculate resolution of waypoint based on concentration readings
	void waypointResCalc();

	void windCallback(const olfaction_msgs::anemometer::ConstPtr &msg);

public:
	Localization();

	void run();
	
};