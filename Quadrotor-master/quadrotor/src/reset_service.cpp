#include "ros/ros.h"
#include "RESL/Reset.h"
#include "eigen3/Eigen/Dense"
#include "headers/quad.h"
#include "geometry_msgs/Vector3.h"
#include "yaml-cpp/yaml.h"

using namespace Eigen;
using namespace std;
using namespace quad_dynamics;
Quadynamics::Quadynamics(){
  }

bool Quadynamics::reset(RESL::Reset::Request  &req,
         RESL::Reset::Response &res)
{
YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
  
	x << config["x"].as<double>(), config["y"].as<double>(),config["z"].as<double>();
  xdot << config["x_dot"].as<double>(), config["y_dot"].as<double>(),config["z_dot"].as<double>();
  theta << config["roll"].as<double>(), config["pitch"].as<double>(),config["yaw"].as<double>(); // Assuming the values are in radians
  thetadot << config["roll_dot"].as<double>(), config["pitch_dot"].as<double>(),config["yaw_dot"].as<double>();   
  geometry_msgs::Vector3 position;
  geometry_msgs::Vector3 linear_velocity;
  geometry_msgs::Vector3 angular_position;
  geometry_msgs::Vector3 angular_velocity;
  
    position.x = x(0,0);
  position.y = x(1,0);
  position.z = x(2,0);

  linear_velocity.x = xdot(0,0);
  linear_velocity.y = xdot(1,0);
  linear_velocity.z = xdot(2,0);

  angular_position.x = theta(
    0,0);
  angular_position.y = theta(
    1,0);
  angular_position.z = theta(
    2,0);

  angular_velocity.x = thetadot(0,0);
  angular_velocity.y = thetadot(1,0);
  angular_velocity.z = thetadot(2,0);
res.position = position;
  res.linear_velocity = linear_velocity;
  res.angular_velocity = angular_velocity;
  res.angular_position = angular_position;
	return true;
}
bool reset(RESL::Reset::Request  &req,
         RESL::Reset::Response &res)
{
quad_dynamics::Quadynamics qd;
qd.reset(req,res);
return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_server");
  YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
  string servicename  = config["reset"].as<string>();
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService(servicename, reset);
  ROS_INFO("Reset Service Ready");
  ros::spin();

  return 0;
}
