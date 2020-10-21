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

MoveDroneClient::MoveDroneClient() :
m_action_client("waypoint")
{

}

void MoveDroneClient::goToWaypoint(const geometry_msgs::Point &waypoint)
{
  // TODO if at map boundary while receiving waypoint, change map_boundary to false
	// if heading is away from boundary
}

void MoveDroneClient::followDirection(const double& heading)
{
	// TODO If same heading angle as before, don't send new goal
}

void MoveDroneClient::stopMoving()
{

}

// int main()
// {
//     return 0;
// }