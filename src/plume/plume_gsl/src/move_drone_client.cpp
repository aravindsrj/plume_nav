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


// int main()
// {
//     return 0;
// }