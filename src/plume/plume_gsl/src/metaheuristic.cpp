#include "plume_gsl/metaheuristic.h"
#include <random>


Localization::Localization() :
m_waypoint_client("waypoint"),
m_raster_client("rasterScan"),
m_nh("~"),
m_max_prob_message_received(false),
m_lost_plume(false),
m_moving_to_source(false),
m_raster_scan_complete(false),
m_initial_scan_complete(false),
m_got_initial_heuristic(false),
m_source_reached(false),
m_lost_plume_counter(0),
m_max_concentration_value(-1),
m_lost_plume_counter_maxlimit(3),
m_distance_from_waypoint(0.0),
m_maintain_dir_prob(0.4)
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

	m_nh.param("position_epsilon", m_epsilon_position, 3e-3);
	m_nh.param("conc_grad_epsilon", m_epsilon_conc_grad, 0.0);
	m_nh.param("minimum_detectable_concentration", m_epsilon_concentration, m_concentration_range.min);
	m_nh.param("probability_threshold", m_probability_threshold, 1e-4);
	m_nh.param("probability_to_maintain_dir", m_maintain_dir_prob, 0.4);
	m_nh.param("lost_distance", m_lost_distance, 1.0);

	m_nh.param("wind_history_size", i_temp, 15);
	m_wind_dir_history.setSize(i_temp);

	m_nh.param("recent_concentration_queue_size", i_temp, 5);
	m_concentration_points.setSize(i_temp);

	m_wind_sub = m_nh.subscribe("/Anemometer/WindSensor_reading", 
		5, &Localization::windCallback, this);
	m_gas_sub = m_nh.subscribe("/PID/Sensor_reading", 
		5, &Localization::concentrationCallback, this);
	m_prob_sub = m_nh.subscribe("/max_probability", 
		10, &Localization::maxSourceProbabilityCallback, this);

	// Seed random number generator
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

}

void Localization::callRasterScan(const double& distance)
{

}

void Localization::changeTemperature()
{
	if (m_algorithm == METAHEURISTIC)
	{
		m_Temperature -= m_delta_temp;
		if (m_Temperature < 0)
			ROS_ERROR("Temperature parameter has gone below zero");
		ROS_INFO("New Temperature: %.3lf", m_Temperature);
	}
}

double Localization::checkGradient() const
{
	int L = m_concentration_history.size();

	assert(L > 0);
	/// \todo Error if L == 0
	if (L < 2)
	{
		ROS_WARN("Concentration data not long enough");
	}

	double gradient = m_concentration_history.mean(L/2, L) - m_concentration_history.mean(0,L/2);

	return gradient;
}

void Localization::concentrationCallback(const olfaction_msgs::gas_sensor::ConstPtr& msg)
{
	m_current_gas_concentration = *msg;

	// Updating the vector for recent concentrations. Automatically pops when size exceeds
	m_concentration_points.append(std::make_pair(msg->raw, m_drone.position));

	if (m_concentration_points.mean() > m_max_concentration_value)
	{
		m_max_concentration_value = m_concentration_points.mean();
		m_max_concentration_at = m_concentration_points.getPoint();
	}

	// Updating the concentration history queue. This will be subsequently cleared in run()
	// TODO The following append must happen only when raster and initial scans are complete
	m_concentration_history.append(msg->raw);
}

void Localization::declareSourceCondition()
{

}

double Localization::euclideanDistance(const geometry_msgs::Point& point1,
	const geometry_msgs::Point& point2) const
{
	return (sqrt(pow(point1.x - point2.x, 2) 
		+ pow(point1.y - point2.y, 2)));
}

bool Localization::getInitialHeuristic()
{
	if (m_wind_dir_history.size() != m_wind_dir_history.maxSize())
		return false;

	ROS_INFO("Getting Initial Heuristic");

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, m_meta_std);

	switch (m_algorithm)
	{
		case UPWIND:
		case FOLLOW_WIND:
			ROS_INFO("Following wind");
			m_heading_angle = M_PI + m_wind_dir_history.mean();
			break;

		case ZIGZAG:
			ROS_INFO("Choosing zigzag direction");
			if ((float) rand()/RAND_MAX >= 0.5)
				m_alpha *= -1;
			m_heading_angle = M_PI + m_wind_dir_history.mean() + m_alpha;
			break;

		case METAHEURISTIC:
			m_heading_angle = atan2(m_max_source_probability.y - m_drone.position.y,
				m_max_source_probability.x - m_drone.position.x);
			m_heading_angle += distribution(generator);
			ROS_INFO("Choosing direction with respect to max probability. Angle = %.3lf degrees",
				m_heading_angle * 180 / M_PI);
			break;

		default:
			ROS_ERROR("Unable to get initial heuristic since no algorithm chosen");
			return false;
	}
	m_got_initial_heuristic = true;
	return true;
}

void Localization::getNewHeuristic()
{

}

void Localization::goToMaxConcentration()
{
	m_drone.goToWaypoint(m_max_concentration_at);
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

	m_wind_dir_history.append(wind_dir);

}

void Localization::run()
{

	// First Steps ########################################################

	// TODO Raster scan implementation

	if (!m_got_initial_heuristic)
	{
		if (!getInitialHeuristic())
		{
			ROS_INFO_ONCE("Waiting to get initial heuristic");
			return;
		}
		waypointResCalc();
		m_drone.followDirection(m_heading_angle);
		m_reached_waypoint = false;
	}

	// End of First steps #################################################

	// TODO Give initial value for m_previous_position

	m_distance_from_waypoint += euclideanDistance(m_drone.position, m_previous_position);
	m_previous_position = m_drone.position;

	// New way of checking if waypoint is reached
	if ((*m_waypoint_res - m_distance_from_waypoint) < m_epsilon_position)
	{
		m_distance_from_waypoint = 0.0;
		m_reached_waypoint = true;
	}
	else
	{
		return; // wait to reach the waypoint
	}

	// This gets executed each time a waypoint is reached
	if (m_reached_waypoint and !m_source_reached)
	{
		if (m_lost_plume)
		{
			if (m_concentration_history.front() > m_epsilon_concentration)
			{
				declareSourceCondition();
				
				// This may have to be changed
				m_lost_plume = false;
			}
			else
			{
				// TODO Call raster scan or any other recovery procedure
				
				return;
			}
		}

		if (m_source_reached)
			return;

		// This may have to be moved from here
		m_reached_waypoint = false;

		double gradient = checkGradient();

		if (gradient > m_epsilon_conc_grad)
		{
			// Continue with same heading
			m_lost_plume_counter = 0;
			changeTemperature();
		}
		else // Decreasing gradient
		{
			
			// Change the maintain_dir_prob (lambda) value
			if (m_algorithm == METAHEURISTIC)
				m_maintain_dir_prob = pow(M_Ef32, (gradient-m_epsilon_conc_grad)/m_Temperature);

			if (m_concentration_history.mean() < m_epsilon_concentration)
			{
				++m_lost_plume_counter;
				if (m_lost_plume_counter < m_lost_plume_counter_maxlimit)
				{
					ROS_INFO("Concentration too low. Getting new heuristic");
					getNewHeuristic();
				}
				else
				{
					ROS_WARN("Plume lost due to low concentration");
					m_lost_plume = true;
					m_lost_plume_counter = 0;

					// TODO Recovery procedure
					
					return;
				}
			}
			else if (m_maintain_dir_prob > (double)rand()/RAND_MAX)
			{
				m_lost_plume_counter = 0;

				ROS_INFO("Maintaining direction probability");
				if (m_algorithm == METAHEURISTIC)
					ROS_INFO("Direction prob: %.3lf, Gradient: %.3lf", 
						m_maintain_dir_prob, gradient);
				
				// keep same heading angle
				changeTemperature();
			}
			else
			{
				m_lost_plume_counter = 0;

				ROS_INFO("Low gradient. Getting new heuristic");
				if (m_algorithm == METAHEURISTIC)
					ROS_INFO("Direction prob: %.3lf, Gradient: %.3lf", 
						m_maintain_dir_prob, gradient);
				
				getNewHeuristic();
			}
		}

		// Calculate waypoint resolution and move
		waypointResCalc();
		m_drone.followDirection(m_heading_angle);
		
		m_concentration_history.clear();

		if (m_drone.map_boundary_reached)
		{
			ROS_INFO("Map boundary reached. Plume lost.");
			m_lost_plume = true;

			// This is the recovery step if drone reaches map boundary.
			// This step can be changed
			ROS_INFO("Going to max concentration point");
			goToMaxConcentration();
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Metaheuristic");
	ros::Rate r(30);
	Localization obj;
	while(ros::ok())
	{
		obj.run();
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}