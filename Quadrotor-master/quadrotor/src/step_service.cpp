#include "ros/ros.h"
#include "RESL/Step.h"
#include "eigen3/Eigen/Dense"
#include "headers/quad.h"
#include "geometry_msgs/Vector3.h"
#include "RESL/Rotation.h"
#include "RESL/RotationData.h"
#include "yaml-cpp/yaml.h"
#include  "string.h"
using namespace Eigen;
using namespace std;
using namespace quad_dynamics;

// Method for getting angular velocity vector
Quadynamics::Quadynamics(){
  // should be picked from a config file //
  YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
  m = config["m"].as<double>();
  g = config["g"].as<double>();
  kd = config["kd"].as<double>();
  Ixx = config["Ixx"].as<double>();
  Iyy = config["Iyy"].as<double>();
  Izz = config["Izz"].as<double>();
  k = config["kf"].as<double>();
  L = config["L"].as<double>();
  b = config["km"].as<double>();
  dt = config["dt"].as<double>();
  x << config["x"].as<double>(), config["y"].as<double>(),config["z"].as<double>();
  xdot << config["x_dot"].as<double>(), config["y_dot"].as<double>(),config["z_dot"].as<double>();
  theta << config["roll"].as<double>(), config["pitch"].as<double>(),config["yaw"].as<double>(); // Assuming the values are in radians
  thetadot << config["roll_dot"].as<double>(), config["pitch_dot"].as<double>(),config["yaw_dot"].as<double>();   
    I << Ixx,0,0,
    0,Iyy,0,
    0,0, Izz;
}
MatrixXd Quadynamics::getOmega(MatrixXd theta, MatrixXd thetadot) {
    Matrix<double,3,3> wv;// this is the matrix that is involved in computation of omega
    double phi = theta(0,0);
    double tht = theta(1,0);
    wv << 1,0,(-1)*sin(tht),
    0,cos(phi),cos(tht)*sin(phi),
    0,(-1)*sin(phi),cos(tht)*cos(phi);
    return wv*thetadot;
}

MatrixXd Quadynamics::getThetadot(Matrix<double,3,1> omega, MatrixXd theta) {
    Matrix<double,3,3> wv;// this is the matrix that is involved in computation of omega
    double phi = theta(0,0);
    double tht = theta(1,0);
    wv << 1,0,(-1)*sin(tht),
    0,cos(phi),cos(tht)*sin(phi),
    0,(-1)*sin(phi),cos(tht)*cos(phi);
    return wv.inverse()*omega;
}
// Method for calculating the Thrust
MatrixXd Quadynamics::getThrust( MatrixXd inputs, double k)
{
    Matrix<double,3,1> t;
    t << 0,0,k*(inputs.array().square().sum());
    return t;
    
}
//Method to Calculate the R mattrix i.e Rotaion of the Body frame w.r.t inertial frame.
MatrixXd Quadynamics::getRotation(MatrixXd theta)
{
    Matrix<double, 3,3> R;
    double phi = theta(0,0);
    double tht = theta(1,0);
    double omg = theta (2,0);
    R << (cos(phi)*cos(omg))-(cos(tht)*sin(phi)*sin(omg)), (-1)*(cos(omg)*sin(phi)) - (cos(phi)*cos(tht)*sin(omg)), sin(tht)*sin(omg),
    (cos(tht)*cos(omg)*sin(phi)) - cos(phi)*sin(omg), cos(phi)*cos(tht)*cos(omg) - sin(phi)*sin(omg), (-1)*cos(omg)*sin(tht),
    sin(phi)*sin(tht), cos(phi)*sin(tht), cos(tht);
    return R;
}
// Method to calculate acceleration in linear frame.
MatrixXd Quadynamics::getAcc(MatrixXd inputs, MatrixXd theta, MatrixXd xdot, double m, double g, double k, double kd)
{
    Matrix<double, 3, 1> gravity;
    gravity<<0,0,(-1)*g;
    MatrixXd R = getRotation(theta);
    MatrixXd Tb = getThrust(inputs, k);
    MatrixXd T = R*Tb;
    MatrixXd Fd = xdot*kd*(-1);
    return gravity + (1/m) * T + Fd;
}
//Method to calculate the torque.
MatrixXd Quadynamics::getTorque(MatrixXd inputs, double L, double b, double k)
{
    Matrix<double, 3, 1> torque;
    torque << L*k*(inputs(0,0)- inputs(2, 0)),
    L*k*(inputs(1, 0)- inputs(3, 0)),
    b*(inputs(0, 0) - inputs(1, 0) + inputs(2, 0) - inputs(3, 0));
    return torque;
}
// Method to calculate angular accel;eration.
 MatrixXd Quadynamics::getAngularAcc(MatrixXd inputs, Matrix<double,3,1> omega, MatrixXd I, double L, double b, double k)
 {
 MatrixXd Tau = getTorque(inputs, L, b, k);
 Matrix<double,3,1> invt = I * omega;
 Matrix<double,3,1> O = omega;
return I.inverse()*(Tau-O.cross(invt));
 }

bool Quadynamics::step(RESL::Step::Request  &req,
         RESL::Step::Response &res)
{
  Matrix<double, 4, 1> inputs;
  Matrix<double, 3,3> R;

  inputs << req.w1,req.w2,req.w3,req.w4;
  geometry_msgs::Vector3 position;
  geometry_msgs::Vector3 linear_velocity;
  geometry_msgs::Vector3 angular_position;
  geometry_msgs::Vector3 angular_velocity;
  geometry_msgs::Vector3 linear_acceleration;
  geometry_msgs::Vector3 angular_acceleration;
  RESL::Rotation msg;
  RESL::RotationData data;
  
  for (int i=0;i<req.N;i++)
    {
     omega = getOmega(theta,thetadot);
     a = getAcc(inputs, theta, xdot, m,g,k,kd);
     omegadot = getAngularAcc(inputs, omega, I, L, b, k);
     // Calulating the new velocities, postions w.r.t to the input.
     omega = omega + dt * omegadot;
     thetadot = getOmega(omega, theta);
     theta = theta + dt * thetadot;
     xdot = xdot + dt * a;
     x = x + dt * xdot;
    
    }
    R = getRotation(theta);//Getting the current orientation of the robot

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

  linear_acceleration.x = a(0,0);
  linear_acceleration.y = a(1,0);
  linear_acceleration.z = a(2,0);

  angular_acceleration.x = omegadot(0,0);
  angular_acceleration.y = omegadot(1,0);
  angular_acceleration.z = omegadot(2,0);
  
  data.R_00 = R(0,0);
  data.R_10 = R(1,0);
  data.R_20 = R(2,0);
  data.R_01 = R(0,1);
  data.R_02 = R(0,2);
  data.R_11 = R(1,1);
  data.R_12 = R(1,2);
  data.R_21 = R(2,1);
  data.R_22 = R(2,2);
  msg.rotation.push_back(data);

  res.position = position;
  res.linear_velocity = linear_velocity;
  res.angular_velocity = angular_velocity;
  res.angular_position = angular_position;
  res.linear_acceleration = linear_acceleration;
  res.angular_acceleration = angular_acceleration;
  res.rotation_matrix = msg;
   return true;
}


bool step(RESL::Step::Request  &req,
         RESL::Step::Response &res)
{
quad_dynamics::Quadynamics qd;
qd.step(req,res);
return true;
}


int main(int argc, char **argv)
{
  YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
  string servicename = config["step"].as<string>();
  ros::init(argc, argv, "step_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService(servicename, step);
  ROS_INFO("Step Service Ready");
  ros::spin();

  return 0;
}