#include "plume_gsl/move_drone_client.h"
#include "plume_gsl/reading_history.h"

#include "actionlib/server/simple_action_server.h"

#include "plume_gsl/rasterScanAction.h"

#include "olfaction_msgs/gas_sensor.h"
#include "olfaction_msgs/anemometer.h"

#include "tf/transform_listener.h"

enum Movement
{
  NOT_STARTED,
  FIRST_FLANK,
  SECOND_FLANK,
  MAX_CONCENTRATION,
  STOPPED,
  COMPLETED
};

class RasterSearch
{
  ros::NodeHandle m_nh;

  actionlib::SimpleActionServer<plume_gsl::rasterScanAction> m_server;
  plume_gsl::rasterScanResult m_action_result;

  ros::Subscriber sub_sensor;
  ros::Subscriber sub_wind;
  ros::Subscriber pos_sub;

  void concCallback(const olfaction_msgs::gas_sensor::ConstPtr&);
  void windCallback(const olfaction_msgs::anemometer::ConstPtr&);

  ReadingHistory<std::pair<double, geometry_msgs::Point>> m_concentration_points;
  ReadingHistory<double> m_wind_dir_history{true};

  /// \brief Shows the stage of raster scan
  Movement m_movement;

  std::string m_fixed_frame;
  std::string m_anemometer_frame;

  tf::TransformListener m_tf;
  tf::StampedTransform m_transform;

  geometry_msgs::Point m_max_concentration_at;

  /// \brief Start position of the raster scan
  geometry_msgs::Point m_start_position;

  double m_wind_direction;
  double m_max_concentration_value;
  double m_goal_distance;
  double m_distance_from_start;
  double m_epsilon_position;
  double m_heading;

  int m_wind_history_size;
  
  /// \brief Direction of raster scan. A value of +/- 1
  int m_alpha;

  /// \brief A flag to check if enough wind data is collected
  bool m_wind_history_full;
  bool m_goal_point_sent;
  bool m_drone_initialized;

  /// \brief A flag to know if the drone reached the extreme end of the flank,
  /// and is ready to move back to the start point
  bool m_end_reached;

  MoveDroneClient m_drone;

  void goalCallback();

  /// \brief This conducts one-half of a raster scan (moves to one direction and
  /// returns to start). Function returns true when this operation is complete
  bool flankScan();

public:
  RasterSearch();
  void run();
};

RasterSearch::RasterSearch():
m_max_concentration_value(0.0),
m_wind_history_full(false),
m_end_reached(false),
m_goal_point_sent(false),
m_drone_initialized(false),
m_server(m_nh, "raster_scan", false),
m_drone(m_nh)
{
  sub_sensor = m_nh.subscribe("/PID/Sensor_reading",5, 
    &RasterSearch::concCallback, this);

  sub_wind = m_nh.subscribe("/Anemometer/WindSensor_reading",5, 
    &RasterSearch::windCallback, this);

  m_nh.getParam("/fixed_frame", m_fixed_frame);
  m_nh.getParam("/anemometer_frame", m_anemometer_frame);

  int i_temp;
  m_nh.param("/Metaheuristic/recent_concentration_queue_size", i_temp, 5);
	m_concentration_points.setSize(i_temp);

  m_nh.param("/Metaheuristic/wind_history_size", m_wind_history_size, 15);
	m_wind_dir_history.setSize(m_wind_history_size);

  m_nh.param("/Metaheuristic/position_epsilon", m_epsilon_position, 3e-3);

  m_server.registerGoalCallback(boost::bind(&RasterSearch::goalCallback, this));

  // Initialize result with zeros
  m_action_result.max_concentration_point.assign(3, 0.0);

  // TODO this parameter should be sampled from a uniform distribution
  m_alpha = 1;

  // sub = m_nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth",1, boost::bind(&MoveDroneClient::dronePositionCallback, this, _1));

  m_server.start();
}

void RasterSearch::concCallback(const olfaction_msgs::gas_sensor::ConstPtr& msg)
{
	// We have to process concentration data only if the server is active
  if (!m_server.isActive())
    return;
  
  // Updating the vector for recent concentrations. Automatically pops when size exceeds
	m_concentration_points.append(std::make_pair(msg->raw, m_drone.position));

	if (m_concentration_points.mean() > m_max_concentration_value)
	{
		m_max_concentration_value = m_concentration_points.mean();
		m_max_concentration_at = m_concentration_points.getPoint();
	}
}

void RasterSearch::goalCallback()
{
  plume_gsl::rasterScanGoalConstPtr goal;
  goal = m_server.acceptNewGoal();
  m_goal_distance = goal->scan_distance;

  // Clear previous data
  m_wind_dir_history.clear();
  m_concentration_points.clear();

  // Re-initialize some variables
  m_wind_history_full = false;
  if (m_drone.initialized())
  {
    m_drone_initialized = true;
    m_start_position = m_drone.position;
  }
  m_movement = NOT_STARTED;
  m_max_concentration_value = 0.0;
}

void RasterSearch::windCallback(const olfaction_msgs::anemometer::ConstPtr& msg)
{
  // We have to process wind data only if the server is active
  if (!m_server.isActive())
    return;
  
  m_tf.waitForTransform(m_anemometer_frame, m_fixed_frame, 
		ros::Time(0), ros::Duration(3.0));
	m_tf.lookupTransform(m_fixed_frame, m_anemometer_frame, ros::Time(0), m_transform);

	double yaw = tf::getYaw(m_transform.getRotation());
	m_wind_direction = yaw + msg->wind_direction - M_PI;

	MoveDroneClient::normalizeAngle(m_wind_direction);
  m_wind_dir_history.append(m_wind_direction);

  if(!m_wind_history_full)
  {
    if (m_wind_dir_history.size() == m_wind_history_size)
      m_wind_history_full = true;
  }
}

bool RasterSearch::flankScan()
{
  m_distance_from_start = 
    MoveDroneClient::euclideanDistance(m_drone.position, m_start_position);
  // ROS_WARN("Distance from start = %.2lf", m_distance_from_start);
  if (!m_end_reached) 
  { // Moving away from start point
    // ROS_INFO("Not end reached. Distance from start = %.2lf", m_distance_from_start);
    m_drone.followDirection(m_heading);
    if (m_distance_from_start >= m_goal_distance)
      m_end_reached = true;
  }

  if (m_end_reached)  
  { // Moving towards start point
    // ROS_INFO("End reached");
    if (!m_goal_point_sent)
    {
      // This is where the start points are given as the waypoint
      m_drone.goToWaypoint(m_start_position);
      m_goal_point_sent = true;
    }

    if (m_drone.reachedWaypoint() and m_goal_point_sent)
    { // Reached start point

      // Reset flags
      m_end_reached = false;
      m_goal_point_sent = false;
      
      return true; // Reached waypoint and end flank scan
    }
  }
  return false; // Did not reach start point
}

void RasterSearch::run()
{
  // Proceed only if server is active
  if (!m_server.isActive())
    return;

  switch (m_movement)
  {
    case NOT_STARTED:
      if (!m_wind_history_full)
      {
        ROS_INFO_ONCE("[Raster] Collecting wind data");
        return;
      }

      if (!m_drone_initialized)
      {
        if (m_drone.initialized())
        {
          m_start_position = m_drone.position;
          m_drone_initialized = true;
        }
        else
        {
          ROS_INFO_ONCE("[Raster] Waiting for drone to initialize");
          return;
        }
      }

      // Get heading direction perpendicular to wind
      m_heading = m_wind_dir_history.mean() + m_alpha * M_PI/2;
      m_movement = FIRST_FLANK;

    case FIRST_FLANK:
      ROS_INFO_ONCE("[Raster] First flank");
      if (flankScan())
      {
        // Go to next flank if first one is complete
        m_movement = SECOND_FLANK;

        // Change heading direction by 180 degrees for next flank
        m_heading = MoveDroneClient::normalizeAngle(m_heading - M_PI);
      }
    case SECOND_FLANK:
      if (m_movement != SECOND_FLANK)
        break;
      
      ROS_INFO_ONCE("[Raster] Second flank");
      if (flankScan())
      {
        // Stop raster scan if both flanks are complete
        m_movement = MAX_CONCENTRATION;
      }

    case MAX_CONCENTRATION:
      if (m_movement != MAX_CONCENTRATION)
        break;
      ROS_INFO_ONCE("Moving to max concentration");
      if (!m_goal_point_sent)
      {
        m_drone.goToWaypoint(m_max_concentration_at);
        m_goal_point_sent = true;
      }
      if (m_drone.reachedWaypoint() and m_goal_point_sent)
      {
        m_goal_point_sent = false;
        m_movement = STOPPED;
      }
      break;
    case STOPPED:
      if (m_movement != STOPPED)
        break;
      ROS_INFO("[Raster] Stopped");
      m_action_result.max_concentration = m_max_concentration_value;
      
      m_action_result.max_concentration_point[0] = m_max_concentration_at.x;
      m_action_result.max_concentration_point[1] = m_max_concentration_at.y;
      m_action_result.max_concentration_point[2] = m_max_concentration_at.z;

      m_drone.stopMoving();
      m_movement = COMPLETED;
      m_server.setSucceeded(m_action_result);

    case COMPLETED:
      break;

    default:
      ROS_ERROR("[Raster]: Invalid argument for switch case");
      break;
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raster_search");
  RasterSearch R;
  ros::Rate r(50);
  while(ros::ok())
  {
    R.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}