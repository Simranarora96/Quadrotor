#include "ros/ros.h"
#include "RESL/Step.h"
#include <cstdlib>
#include "geometry_msgs/Twist.h"

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "step_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<RESL::Step>("step_service");
  RESL::Step srv;
  srv.request.w1 = -4479;
  srv.request.w2 = 4479;
  srv.request.w3 = -4479;
  srv.request.w4 = 4479;
  srv.request.N = 800;
  
  if (client.call(srv))
  {
    //cout<<srv.response.rotation_matrix.rotation[0].R_00<<endl;
    cout<<srv.response<<endl;
  }
  else
  {
    ROS_ERROR("Failed to call service step");
    return 1;
  }

  return 0;
}