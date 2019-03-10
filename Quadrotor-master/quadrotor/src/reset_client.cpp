#include "ros/ros.h"
#include "RESL/Reset.h"
#include <cstdlib>
#include "geometry_msgs/Twist.h"

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<RESL::Reset>("reset_service");
  RESL::Reset srv;
  
  if (client.call(srv))
  {
     cout<<srv.response<<endl;
  }
  else
  {
    ROS_ERROR("Failed to call Reset Service");
    return 1;
  }

  return 0;
}