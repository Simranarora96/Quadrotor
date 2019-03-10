#include "ros/ros.h"
#include "RESL/Mellinger.h"
#include <cstdlib>

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mellinger_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<RESL::Mellinger>("mellinger_service");
  RESL::Mellinger srv;
  
  if (client.call(srv))
  {
     cout<<srv.response<<endl;
  }
  else
  {
    ROS_ERROR("Failed to call Mellinger Service");
    return 1;
  }

  return 0;
}