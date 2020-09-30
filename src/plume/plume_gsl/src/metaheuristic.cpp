// #include "ros/ros.h"
#include "plume_gsl/metaheuristic.h"

GasHistory::GasHistory():
m_sum(0.0)
{
}

void GasHistory::setSize(const int& size)
{
	m_size = size;
}

void GasHistory::append(const std::pair<double,geometry_msgs::Point> data)
{
	m_sum += data.first;
	array.push_back(data);
	if (array.size() > m_size)
	{
		m_sum -= array[0].first;
		array.erase(array.begin());
	}
	assert(m_sum >= 0);
}

double GasHistory::mean() const
{
	return m_sum/array.size();
}

geometry_msgs::Point GasHistory::getPoint() const
{
	assert(array.size() > 0);
	return array[array.size()/2].second;
}

Localization::Localization() :
m_waypoint_client("waypoint"),
m_raster_client("rasterScan"),
m_nh("~"),
m_max_prob_message_received(false),
m_lost_plume(false),
m_moving_to_source(false),
m_raster_scan_complete(false),
m_initial_scan_complete(false),
m_plume_lost_counter(0),
m_max_concentration_value(-1)
{
	std::vector<double> temp_ranges;
	double d_temp;
	int i_temp;
	
	m_nh.param("waypoint_resolution_range", temp_ranges, std::vector<double>(1, 2.5));
	m_resolution_range.setRange(temp_ranges);

	m_nh.param("concentration_range", temp_ranges, std::vector<double>(10.0, 200.0));
	m_concentration_range.setRange(temp_ranges);

	m_nh.param("waypoint_resolution", d_temp, 0.5);
	m_waypoint_res = std::make_shared<double>(d_temp);

	m_nh.param("fixed_frame", m_fixed_frame, std::string("map"));
	m_nh.param("anemometer_frame", m_anemo_frame, std::string("anemometer_frame"));
	m_nh.param("Initial_temperature", m_Temperature, 14.0);
	m_nh.param("delta_temperature", m_delta_temp, 0.5);
	m_nh.param("meta_standard_deviation", m_meta_std, 0.2);
	m_nh.param("initial_zigzag_angle", m_alpha, 35.0);
	m_alpha = m_alpha * M_PI / 180;

	m_nh.param("position_epsilon", m_position_epsilon, 1e-4);
	m_nh.param("conc_grad_epsilon", m_conc_grad_epsilon, 0.0);
	m_nh.param("minimum_detectable_concentration", m_min_concentration, m_concentration_range.min);
	m_nh.param("probability_threshold", m_probability_threshold, 1e-4);
	m_nh.param("probability_to_maintain_dir", m_maintain_dir_prob, 0.4);
	m_nh.param("lost_distance", m_lost_distance, 1.0);
	m_nh.param("wind_history_size", m_wind_history_size, 15);
	m_nh.param("recent_concentration_queue_size", i_temp, 5);
	m_concentration_points.setSize(i_temp);

}

void Localization::concentrationCallback(const olfaction_msgs::gas_sensor::ConstPtr& msg)
{
	m_current_gas_concentration = *msg;

	// Updating the vector for recent concentrations
	m_concentration_points.append(std::make_pair(msg->raw, m_drone.position));

	if (m_concentration_points.mean() > m_max_concentration_value)
	{
		m_max_concentration_value = m_concentration_points.mean();
		m_max_concentration_at = m_concentration_points.getPoint();
	}

	// Updating the concentration history queue
	m_concentration_history.push_back(msg->raw);
}

double Localization::findMean(const std::vector<double>& array) const
{

}

void Localization::maxSourceProbabilityCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	m_max_source_probability = *msg;

	if (!m_max_prob_message_received)
		m_max_prob_message_received = true;
}

void Localization::normalize_angle(double& angle) const
{
	angle = fmod(angle, 2*M_PI);
	angle = fmod(angle + 2*M_PI, 2*M_PI);
	if (angle > M_PI)
		angle -= 2*M_PI;
}

void Localization::waypointResCalc()
{

}

void Localization::windCallback(const olfaction_msgs::anemometer::ConstPtr &msg)
{
	m_tf.waitForTransform(m_anemo_frame, m_fixed_frame, 
		ros::Time(0), ros::Duration(3.0));

	tf::StampedTransform transform;
	m_tf.lookupTransform(m_fixed_frame, m_anemo_frame, ros::Time(0), transform);

	double yaw = tf::getYaw(transform.getRotation());

	double wind_dir = yaw + msg->wind_direction - M_PI;

	normalize_angle(wind_dir);

	m_wind_dir_history.push(wind_dir);

	// Delete old wind data
	if (m_wind_dir_history.size() > m_wind_history_size)
		m_wind_dir_history.pop();

}

void Localization::run()
{
	
	// Last step
	m_concentration_history.clear();
}

int main()
{
	return 0;
}