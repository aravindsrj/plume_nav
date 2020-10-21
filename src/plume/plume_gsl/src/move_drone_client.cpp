#include "plume_gsl/move_drone_client.h"

void Range::setRange(const std::vector<double> ranges)
{
    min = ranges[0];
    max = ranges[1];
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