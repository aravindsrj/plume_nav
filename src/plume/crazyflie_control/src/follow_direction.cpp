#include <ros/ros.h>

#include "actionlib/server/simple_action_server.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "crazyflie_control/follow_directionAction.h"

struct States
{
  geometry_msgs::Point position;
  double theta;
};

class FollowDirection
{
  ros::NodeHandle nh;

  ros::Publisher m_vel_pub;
  ros::Subscriber m_pos_sub;

  actionlib::SimpleActionServer<crazyflie_control::follow_directionAction> m_server;
  crazyflie_control::follow_directionResult m_action_result;

  States m_states;
  geometry_msgs::Twist m_vel_msg;

  bool m_action_successful;

  double m_epslion_angle;
  double m_goal_velocity;
  double m_goal_heading;
  double m_gain_angular;

  void posCallback(const nav_msgs::Odometry::ConstPtr& msg);
  double angularDifference(const double&, const double&) const;
  void goalCallback();

public:
  FollowDirection();

  void publishVelocity();
};

FollowDirection::FollowDirection():
m_goal_velocity(0.0),
m_gain_angular(3.0),
m_epslion_angle(5e-3),
m_action_successful(false),
m_server(nh, "follow_direction", false)
{
  m_pos_sub = nh.subscribe("/base_pose_ground_truth", 10, &FollowDirection::posCallback, this);
  m_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  m_server.registerGoalCallback(boost::bind(&FollowDirection::goalCallback, this));

  m_server.start();
}

void FollowDirection::goalCallback()
{
  crazyflie_control::follow_directionGoalConstPtr goal;
  goal = m_server.acceptNewGoal();
  
  m_goal_velocity = goal->velocity;
  m_goal_heading = goal->heading;

  m_action_successful = false;
}

void FollowDirection::posCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_states.position = msg->pose.pose.position;
  m_states.theta = tf::getYaw(msg->pose.pose.orientation);
}

double FollowDirection::angularDifference(const double& a, const double& b) const
{
  if (a == b)
    return 0.0;

  double diff = b - a;
  double dir = fabs(diff)/diff;

  if (diff > M_PI or diff < -M_PI)
    dir *= -1;

  return dir * acos( cos(a)*cos(b) + sin(a)*sin(b) );

}

void FollowDirection::publishVelocity()
{
  // If goal velocity is 0, we must make sure that 0 velocity is always published irrespective
  // of heading
  if (m_goal_velocity == 0)
  {
    m_vel_msg.linear.x = 0;
    m_vel_msg.angular.z = 0;
    m_vel_pub.publish(m_vel_msg);

    // If action is waiting to be completed
    if (!m_action_successful)
    {
      m_action_successful = true;
      m_server.setSucceeded();
    }
    
    return;
  }

  // If there is angle difference, stop moving, and start turning
  if (fabs(m_goal_heading - m_states.theta) > m_epslion_angle)
  {
    m_vel_msg.linear.x = 0;
    m_vel_msg.angular.z = m_gain_angular * angularDifference(m_states.theta, m_goal_heading);
  }
  else // If no difference in angle, move with desired velocity
  {
    m_vel_msg.linear.x = m_goal_velocity;
    m_vel_msg.angular.z = 0;
  }

  m_vel_pub.publish(m_vel_msg);

  // If action is waiting to be completed
  if (!m_action_successful)
  {
    if (fabs(m_goal_heading - m_states.theta) <= m_epslion_angle 
      and m_vel_msg.linear.x == m_goal_velocity)
    {
      m_action_successful = true;
      m_server.setSucceeded();
    }
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_direction");
  FollowDirection F;
  ros::Rate r(100);
  while (ros::ok())
  {
    F.publishVelocity();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}