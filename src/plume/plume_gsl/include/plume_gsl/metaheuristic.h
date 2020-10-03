#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "plume_gsl/move_drone_client.h"
#include "crazyflie_control/waypointAction.h"
#include "plume_gsl/rasterScanAction.h"
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

template<typename T>
class ReadingHistory
{
	std::deque<T> array;
	int m_size;
	double m_sum;

public:
	
	ReadingHistory();
	void setSize(const int& size);
	void append(const std::pair<double, geometry_msgs::Point>);
	void append(const double);
	void pop();

	/// \brief Find mean of the concentrations in the vector of pairs
	double mean() const;

	int size() const;

	/// \brief Find mean value between two positions in a double array
	double mean(const int& begin, const int& end) const;
	void clear();
	geometry_msgs::Point getPoint() const;
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
	olfaction_msgs::gas_sensor m_current_gas_concentration;
	
	unsigned int m_plume_lost_counter;
	double m_waypoint_heading;
	double m_alpha;
	double m_position_epsilon;
	double m_conc_grad_epsilon;
	double m_min_concentration;
	double m_probability_threshold;
	double m_maintain_dir_prob;
	double m_max_concentration_value;

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