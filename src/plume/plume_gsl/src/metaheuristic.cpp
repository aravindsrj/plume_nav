#include "plume_gsl/metaheuristic.h"
#include <random>

#define DEBUG 0

Localization::Localization() :
m_waypoint_client("waypoint"),
m_raster_client("/raster_scan"),
m_nh("~"),
m_max_prob_message_received(false),
m_lost_plume(false),
m_moving_to_source(false),
m_in_raster_scan(false),
m_initial_scan_complete(false),
m_got_initial_heuristic(false),
m_source_reached(false),
m_goal_point_sent(false),
m_losing_plume_counter(0),
m_lost_plume_counter(0),
m_max_concentration_value(-1),
m_losing_plume_counter_maxlimit(3),
m_lost_plume_counter_maxlimit(3),
m_distance_from_waypoint(0.0),
m_maintain_dir_prob(0.4),
m_direction_noise(0),
m_drone(m_nh)
{
	std::vector<double> temp_ranges;
	int i_temp;
	
	m_nh.getParam("waypoint_resolution_range", temp_ranges);
	m_resolution_range.setRange(temp_ranges);

	m_nh.getParam("concentration_range", temp_ranges);
	m_concentration_range.setRange(temp_ranges);

	m_nh.param("waypoint_resolution", m_waypoint_res, 0.5);
	#if DEBUG
	ROS_INFO("Waypoint resolution default: %.2lf", m_waypoint_res);
	#endif

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

	m_nh.param("min_number_of_conc_readings", m_min_concentration_readings, 5);
	ROS_ASSERT(m_min_concentration_readings > 0);

	m_concentration_history.setSize(10);

	m_wind_sub = m_nh.subscribe("/Anemometer/WindSensor_reading", 
		5, &Localization::windCallback, this);
	m_gas_sub = m_nh.subscribe("/PID/Sensor_reading", 
		5, &Localization::concentrationCallback, this);
	m_prob_sub = m_nh.subscribe("/max_probability", 
		10, &Localization::maxSourceProbabilityCallback, this);

	m_raster_client.waitForServer(ros::Duration(3.0));
	ROS_ERROR_COND(!m_raster_client.isServerConnected(),
		"Metaheuristic not connected to raster server");

	m_algorithm = METAHEURISTIC;

	m_status = START;
}

void Localization::callRasterScan(const double& distance)
{
	m_drone.stopMoving();

	plume_gsl::rasterScanGoal goal;
	goal.scan_distance = distance;
	m_raster_client.sendGoal(goal, 
		boost::bind(&Localization::rasterDone, this, _1, _2));
	m_in_raster_scan = true;
	m_got_initial_heuristic = false;
	m_concentration_history.clear();
}

void Localization::changeTemperature()
{
	if (m_algorithm == METAHEURISTIC)
	{
		m_Temperature -= m_delta_temp;
		if (m_Temperature < 0)
			ROS_ERROR("[Metaheuristic]: Temperature parameter has gone below zero");
		ROS_INFO("[Metaheuristic]: New Temperature: %.3lf", m_Temperature);
	}
}

double Localization::checkGradient() const
{
	int L = m_concentration_history.size();

	assert(L > 0);

	if (L < 2)
	{
		ROS_WARN("[Metaheuristic]: Concentration data not long enough");
	}

	#if DEBUG
		// m_concentration_history.printData();
	#endif

	double gradient = m_concentration_history.mean(L/2, L) - m_concentration_history.mean(0,L/2);

	return gradient;
}

void Localization::concentrationCallback(const olfaction_msgs::gas_sensor::ConstPtr& msg)
{
	m_current_gas_concentration = *msg;

	// Updating the position of the last detected point
	if (m_current_gas_concentration.raw > m_epsilon_concentration
			and !m_drone.mapBoundaryReached())
		m_last_detected_point = m_drone.position;
	
	// Updating the vector for recent concentrations. Automatically pops when size exceeds
	m_concentration_points.append(std::make_pair(msg->raw, m_drone.position));

	if (m_concentration_points.mean() > m_max_concentration_value)
	{
		m_max_concentration_value = m_concentration_points.mean();
		m_max_concentration_at = m_concentration_points.getPoint();
	}

	// Updating the concentration history queue. This will be subsequently cleared in run()

	m_concentration_history.append(msg->raw);
}

void Localization::declareSourceCondition()
{
	ROS_INFO("At declareSourceCondition");
	m_drone.stopMoving();
}

void Localization::getHeuristicMeta()
{
	std::random_device rd;
	std::mt19937_64 generator(rd());
	std::normal_distribution<double> distribution(0.0, m_meta_std);
	m_heading_angle = atan2(m_max_source_probability.y - m_drone.position.y,
		m_max_source_probability.x - m_drone.position.x);
	double noise = distribution(generator);
	if (m_direction_noise == 0)
	{
		m_direction_noise = noise > 0? 1 : -1;
	}
	else
	{
		m_direction_noise *= -1;

		if ((m_direction_noise > 0 and noise < 0) or (m_direction_noise < 0 and noise > 0))
		{
			noise *= -1;
		}
	}
	m_heading_angle += noise;
	m_heading_angle = MoveDroneClient::normalizeAngle(m_heading_angle);
	ROS_INFO("Choosing direction with respect to max probability. Angle = %.3lf degrees. Noise = %.2f",
		m_heading_angle * 180 / M_PI, noise * 180 / M_PI);
	#if DEBUG
		// ROS_WARN("End of getHeuristicMeta");
	#endif
	return;
}

bool Localization::getInitialHeuristic()
{
	if (m_wind_dir_history.size() != m_wind_dir_history.maxSize())
		return false;

	ROS_INFO("Getting Initial Heuristic");

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
			getHeuristicMeta();
			break;

		default:
			ROS_ERROR("Unable to get initial heuristic since no algorithm chosen");
			return false;
	}
	m_got_initial_heuristic = true;
	#if DEBUG
		// ROS_WARN("Got new heuristic");
	#endif
	return true;
}

void Localization::getNewHeuristic()
{
	switch (m_algorithm)
	{
		case UPWIND:
			// Choose a random perpendicular-to-wind direction
			if ((float) rand()/RAND_MAX >= 0.5)
				m_heading_angle = (M_PI/2) + m_wind_dir_history.mean();
			else
				m_heading_angle = -(M_PI/2) + m_wind_dir_history.mean();
			
		case FOLLOW_WIND:
			// Do nothing. Previous heading angle is used
			break;

		case ZIGZAG:
			ROS_INFO("Changing zigzag direction");
			m_alpha *= -1;
			m_heading_angle = M_PI + m_wind_dir_history.mean() + m_alpha;
			break;

		case METAHEURISTIC:
			getHeuristicMeta();
			break;

		default:
			ROS_ERROR("Unable to get initial heuristic since no algorithm chosen");
	}
	#if DEBUG
		// ROS_WARN("Got new heuristic");
	#endif
	return;
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

void Localization::rasterDone(const actionlib::SimpleClientGoalState& state,
		const plume_gsl::rasterScanResultConstPtr& result)
{
	if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Raster scan complete");
		m_in_raster_scan = false;
	}
	else
	{
		ROS_ERROR("Raster done, but not successful");
	}
}

void Localization::waypointResCalc()
{
	// TODO
	return;
}

void Localization::windCallback(const olfaction_msgs::anemometer::ConstPtr &msg)
{
	m_tf.waitForTransform(m_anemo_frame, m_fixed_frame, 
		ros::Time(0), ros::Duration(3.0));

	tf::StampedTransform transform;
	m_tf.lookupTransform(m_fixed_frame, m_anemo_frame, ros::Time(0), transform);

	double yaw = tf::getYaw(transform.getRotation());

	double wind_dir = yaw + msg->wind_direction - M_PI;

	MoveDroneClient::normalizeAngle(wind_dir);

	m_wind_dir_history.append(wind_dir);

}

void Localization::run()
{
	if (m_drone.mapBoundaryReached() and m_status != REACHED_SOURCE)
		m_status = LOST_PLUME;

	switch(m_status)
	{
		case REACHED_SOURCE:
			ROS_INFO_ONCE("Reached Source");
			m_status = REACHED_SOURCE;
			m_drone.stopMoving();
			break;

		case START:
			ROS_INFO("[Metaheuristic]: At START");

			m_status = INCOMPLETE_INITIAL_HEURISTIC;

			#if DEBUG
			break; // Skip initial raster scan
			#endif

			if (m_algorithm == ZIGZAG)
			{
				// TODO this might have to be changed
				if (m_current_gas_concentration.raw > m_epsilon_concentration)
				{
					// Zigzag algorithm does not need initial scan in this version
					m_initial_scan_complete = true;
					m_status = INCOMPLETE_INITIAL_HEURISTIC;
				}
				else
				{
					ROS_INFO_ONCE("Concentration less than threshold");
					break;
				}
			}
			else
			{
				callRasterScan(3.0);
				m_status = IN_RASTER_SCAN;
			}
		
		case IN_RASTER_SCAN:
			ROS_INFO_ONCE("[Metaheuristic]: At IN RASTER SCAN");

			if (m_in_raster_scan)
				break; // Ongoing raster scan; do nothing

			m_status = INCOMPLETE_INITIAL_HEURISTIC;

		case INCOMPLETE_INITIAL_HEURISTIC:
			ROS_INFO_ONCE("[Metaheuristic]: At INCOMPLETE INITIAL HEURISTIC");

			if (!getInitialHeuristic())
			{
				ROS_INFO_ONCE("[Metaheuristic]: Waiting to get initial heuristic");
				break;
			}
			ROS_INFO("Got initial heuristic");

			if (m_concentration_history.size() < 2)
			{
				ROS_INFO("[Metaheuristic]: Concentration data not long enough");
				break;
			}
			waypointResCalc();
			m_drone.followDirection(m_heading_angle);
			m_reached_waypoint = false;

			m_previous_position = m_drone.position;

			m_status = MOVING_TO_WAYPOINT;

		case MOVING_TO_WAYPOINT:
			if (m_status != MOVING_TO_WAYPOINT)
					break;

			m_distance_from_waypoint = 
				MoveDroneClient::euclideanDistance(m_drone.position, m_previous_position);

			// New way of checking if waypoint is reached
			if ((m_waypoint_res - m_distance_from_waypoint) < m_epsilon_position)
			{
				m_distance_from_waypoint = 0.0;
				m_previous_position = m_drone.position;
				m_reached_waypoint = true;
				m_status = REACHED_WAYPOINT;
			}
			else
			{
				break; // wait to reach the waypoint
			}

		case REACHED_WAYPOINT:
			if (m_status != REACHED_WAYPOINT)
				break;

			if (m_concentration_history.size() < m_min_concentration_readings)
			{
				m_drone.stopMoving();
				break;
			}

			ROS_INFO("[Metaheuristic]: At REACHED_WAYPOINT. Conc = %.3lf", 
				m_concentration_history.back());

			if (m_reached_waypoint and !m_source_reached)
			{

				// TODO This if condition might have to be eliminated
				if (m_lost_plume)
				{
					if (m_concentration_history.back() > m_epsilon_concentration)
					{
						declareSourceCondition();
						
						// This may have to be changed
						m_lost_plume = false;
					}
					else
					{
						// TODO Call raster scan or any other recovery procedure
						callRasterScan(2.0);
						m_status = IN_RASTER_SCAN;
						break;
					}
				}

				if (m_source_reached)
				{
					m_status = REACHED_SOURCE;
					break;
				}
					
				// This may have to be moved from here
				m_reached_waypoint = false;

				double gradient = checkGradient();

				if (gradient > m_epsilon_conc_grad)
				{
					// Continue with same heading
					ROS_INFO("[Metaheuristic]: Gradient > epsilon");
					m_losing_plume_counter = 0;
					changeTemperature();
				}
				else // Decreasing gradient
				{
					
					// Change the maintain_dir_prob (lambda) value
					if (m_algorithm == METAHEURISTIC)
						m_maintain_dir_prob = pow(M_Ef32, (gradient-m_epsilon_conc_grad)/m_Temperature);

					if (m_concentration_history.mean() < m_epsilon_concentration)
					{
						++m_losing_plume_counter;
						if (m_losing_plume_counter < m_losing_plume_counter_maxlimit)
						{
							ROS_WARN("[Metaheuristic]: Concentration too low. Getting new heuristic");
							getNewHeuristic();
						}
						else
						{
							ROS_WARN("[Metaheuristic]: Plume lost due to low concentration");
							m_lost_plume = true;
							m_losing_plume_counter = 0;
							m_drone.stopMoving();
							m_status = LOST_PLUME;
							break;
						}
					}
					else if (m_maintain_dir_prob > (double)rand()/RAND_MAX)
					{
						m_losing_plume_counter = 0;

						ROS_INFO("[Metaheuristic]: Maintaining direction probability");
						if (m_algorithm == METAHEURISTIC)
							ROS_INFO("[Metaheuristic]: Direction prob: %.3lf, Gradient: %.3lf", 
								m_maintain_dir_prob, gradient);
						
						// keep same heading angle
						changeTemperature();
					}
					else
					{
						m_losing_plume_counter = 0;

						ROS_WARN("[Metaheuristic]: Low gradient. Getting new heuristic");
						if (m_algorithm == METAHEURISTIC)
							ROS_INFO("[Metaheuristic]: Direction prob: %.3lf, Gradient: %.3lf", 
								m_maintain_dir_prob, gradient);
						
						getNewHeuristic();
					}
				}

				// Calculate waypoint resolution and move
				waypointResCalc();
				m_drone.followDirection(m_heading_angle);

				ROS_ASSERT(m_drone.isMoving());
				
				m_concentration_history.clear();
				#if DEBUG
				ROS_WARN("Concentration history cleared");
				#endif
			}
			else // Source reached
			{
				// Should not reach here
			}

			m_status = MOVING_TO_WAYPOINT;
			break;

		case LOST_PLUME:

			// TODO This variable can be eliminated?
			m_lost_plume = false;

			if (!m_goal_point_sent and m_drone.reachedWaypoint())
			{
				++m_lost_plume_counter;
				ROS_WARN("[Metaheuristic]: Lost Plume. Number of times lost plume: %d/%d", 
					m_lost_plume_counter, m_lost_plume_counter_maxlimit);
			}

			if (m_lost_plume_counter >= m_lost_plume_counter_maxlimit)
			{
				ROS_WARN_ONCE("Moving to source conditions");
				if (!m_goal_point_sent)
				{
					m_drone.goToWaypoint(m_last_detected_point);
					m_goal_point_sent = true;
				}
				if (m_drone.reachedWaypoint() and m_goal_point_sent)
				{
					m_goal_point_sent = false;
					
					// TODO Source declaration after doing raster scan
					m_drone.stopMoving();
					m_status = REACHED_SOURCE;
					ROS_INFO("Changed status to REACHED_SOURCE");
				}
				
				break;
			}

			if (!m_goal_point_sent)
			{
				m_drone.goToWaypoint(m_last_detected_point);
				m_goal_point_sent = true;
			}
			if (m_drone.reachedWaypoint() and m_goal_point_sent)
			{
				m_goal_point_sent = false;
				callRasterScan(2.0);
				m_status = IN_RASTER_SCAN;
			}
			break;

		case AT_MAP_BOUNDARY:

			ROS_WARN("[Metaheuristic]: Map boundary reached. Plume lost.");
			m_lost_plume = true;

			// This is the recovery step if drone reaches map boundary.
			// This step can be changed
			ROS_INFO("[Metaheuristic]: Going to max concentration point");
			goToMaxConcentration();

			// TODO This might have to be changed
			m_status = MOVING_TO_WAYPOINT;

			break;

		default:
			ROS_ERROR("[Metaheuristic]: No valid case for algorithm status");

	}

	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Metaheuristic");
	Localization obj;
	ros::Rate r(30);
	while(ros::ok())
	{
		obj.run();
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}